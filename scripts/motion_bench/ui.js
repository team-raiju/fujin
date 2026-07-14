/* =========================================================
   UI STATE + WIRING
   ========================================================= */
const CH_META = {
  trap: { label: 'TRAPEZOIDAL', color: '#ff9d52', fn: simulateTrapezoidal },
  ideal: { label: 'MOTOR CURVE', color: '#57d6ff', fn: simulateIdealCurve },
  jerk: { label: 'JERK S-CURVE', color: '#79ff9e', fn: simulateJerk },
  arc: { label: 'ARC CURVE', color: '#d386ff', fn: simulateArc },
};

const fields = [
  ['turnAngle', 'turnAngleDeg', 0],
  ['linSpeed', 'linearSpeed', 3],
  ['mmBefore', 'mmBeforeTurn', 1],
  ['mmAfter', 'mmAfterTurn', 1],
  ['trapAccel', 'trapAccel', 3],
  ['trapDecel', 'trapDecel', 3],
  ['trapOmega', 'trapOmega', 3],
  ['idealAbsMax', 'idealAbsMax', 3],
  ['idealRefOmega', 'idealRefOmega', 3],
  ['idealRefAccel', 'idealRefAccel', 3],
  ['idealMaxTarget', 'idealMaxTarget', 3],
  ['jerkJerk', 'jerkJerk', 0],
  ['jerkJerkAfter', 'jerkJerkAfter', 0],
  ['jerkAccel', 'jerkAccel', 3],
  ['jerkOmega', 'jerkOmega', 3],
  ['arcTransition', 'arcTransition', 4],
  ['arcArc', 'arcArc', 4],
  ['arcOmega', 'arcOmega', 4],
];

const state = {};
fields.forEach(([id, key]) => {
  const r = document.getElementById('r-' + id);
  state[key] = parseFloat(r.value);
});

function syncFieldUI(id, decimals, key) {
  const v = state[key];
  // Show at least 'decimals' digits, but up to 6 if the value has more precision
  const strVal = String(v);
  const dotIdx = strVal.indexOf('.');
  const actualDecimals = dotIdx >= 0 ? strVal.length - dotIdx - 1 : 0;
  const showDecimals = Math.min(Math.max(decimals, actualDecimals), 6);
  document.getElementById('v-' + id).textContent = v.toFixed(showDecimals);
  document.getElementById('r-' + id).value = v;
  document.getElementById('n-' + id).value = v;
}

fields.forEach(([id, key, decimals]) => {
  const r = document.getElementById('r-' + id);
  const n = document.getElementById('n-' + id);
  r.addEventListener('input', () => { state[key] = parseFloat(r.value); syncFieldUI(id, decimals, key); recomputeAndRender(); });
  n.addEventListener('input', () => {
    const val = parseFloat(n.value);
    if (!isNaN(val)) {
      state[key] = val;
      // Update label and range slider only — don't touch n.value to preserve cursor position during typing
      const strVal = String(val);
      const dotIdx = strVal.indexOf('.');
      const actualDecimals = dotIdx >= 0 ? strVal.length - dotIdx - 1 : 0;
      const showDecimals = Math.min(Math.max(decimals, actualDecimals), 6);
      document.getElementById('v-' + id).textContent = val.toFixed(showDecimals);
      document.getElementById('r-' + id).value = val;
      recomputeAndRender();
    }
  });
});

const activeChannels = { trap: true, ideal: false, jerk: false, arc: false };
['trap', 'ideal', 'jerk', 'arc'].forEach(ch => {
  const row = document.querySelector(`.channel-row[data-ch="${ch}"]`);
  const chk = document.getElementById('chk-' + ch);
  const group = document.querySelector(`.group[data-owner="${ch}"]`);
  function apply() {
    activeChannels[ch] = chk.checked;
    row.classList.toggle('active', chk.checked);
    group.classList.toggle('on', chk.checked);
    recomputeAndRender();
  }
  row.addEventListener('click', (e) => {
    if (e.target === chk) return;
    chk.checked = !chk.checked;
    apply();
  });
  chk.addEventListener('change', apply);
  group.classList.toggle('on', chk.checked);
});

/* =========================================================
   CHARTS
   ========================================================= */
Chart.defaults.font.family = "'IBM Plex Mono', monospace";
Chart.defaults.font.size = 10.5;
Chart.defaults.color = '#7d9188';

function baseChartOptions(yLabel) {
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: { duration: 200 },
    parsing: false,
    interaction: { mode: 'index', intersect: false },
    scales: {
      x: {
        type: 'linear',
        title: { display: true, text: 'time (ms)', color: '#526059' },
        grid: { color: '#1c2622' },
        ticks: { color: '#7d9188' }
      },
      y: {
        title: { display: true, text: yLabel, color: '#526059' },
        grid: { color: '#1c2622' },
        ticks: { color: '#7d9188' }
      }
    },
    plugins: {
      legend: { display: true, labels: { boxWidth: 12, boxHeight: 2, color: '#a9bab1' } },
      tooltip: { backgroundColor: '#0b100e', borderColor: '#24322c', borderWidth: 1, titleColor: '#dfe9e3', bodyColor: '#dfe9e3' }
    },
    elements: { point: { radius: 0 }, line: { borderWidth: 2, tension: 0 } }
  };
}

const omegaChart = new Chart(document.getElementById('chart-omega'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('rad/s')
});

const alphaChart = new Chart(document.getElementById('chart-alpha'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('rad/s²')
});

function downsample(times, values, maxPoints = 400) {
  const n = times.length;
  if (n <= maxPoints) return times.map((t, i) => ({ x: t, y: values[i] }));
  const step = Math.ceil(n / maxPoints);
  const out = [];
  for (let i = 0; i < n; i += step) out.push({ x: times[i], y: values[i] });
  out.push({ x: times[n - 1], y: values[n - 1] });
  return out;
}

/* =========================================================
   TRAJECTORY SVG
   ========================================================= */
const SVG_SIZE = 420;
const DATA_MIN = -250, DATA_MAX = 540;
const DATA_SPAN = DATA_MAX - DATA_MIN;
function mapX(x) { return ((x - DATA_MIN) / DATA_SPAN) * SVG_SIZE; }
function mapY(y) { return SVG_SIZE - ((y - DATA_MIN) / DATA_SPAN) * SVG_SIZE; }

function wallSegments() {
  const segs = [];
  segs.push([0, 0, 0, 180]);
  segs.push([0, 0, 0, -180]);
  segs.push([0, 180, 180, 180]);
  segs.push([180, 0, 180, -180]);
  segs.push([180, 180, 360, 180]);
  segs.push([360, 180, 360, 0]);
  segs.push([360, 0, 540, 0]);
  for (let i = 180; i < 3600; i += 180) {
    segs.push([i, 180 - i, i, -i]);
    segs.push([i, -i, i + 180, -i]);
    segs.push([i + 360, 180 - i, i + 360, -i]);
    segs.push([i + 360, -i, i + 360 + 180, -i]);
  }
  return segs.filter(([x1, y1, x2, y2]) =>
    (x1 >= DATA_MIN && x1 <= DATA_MAX && y1 >= DATA_MIN && y1 <= DATA_MAX) ||
    (x2 >= DATA_MIN && x2 <= DATA_MAX && y2 >= DATA_MIN && y2 <= DATA_MAX)
  );
}
const WALLS = wallSegments();

function buildTrajectorySVG(runs) {
  let gridLines = '';
  // grid lines every 90mm
  for (let v = Math.ceil(DATA_MIN / 90) * 90; v <= DATA_MAX; v += 90) {
    const isMajor = (v % 180 === 0);
    const color = isMajor ? '#24322c' : '#1c2622';
    const width = isMajor ? 1 : 0.7;
    gridLines += `<line x1="${mapX(v)}" y1="0" x2="${mapX(v)}" y2="${SVG_SIZE}" stroke="${color}" stroke-width="${width}"/>`;
    gridLines += `<line x1="0" y1="${mapY(v)}" x2="${SVG_SIZE}" y2="${mapY(v)}" stroke="${color}" stroke-width="${width}"/>`;
  }

  let wallLines = '';
  WALLS.forEach(([x1, y1, x2, y2]) => {
    wallLines += `<line x1="${mapX(x1)}" y1="${mapY(y1)}" x2="${mapX(x2)}" y2="${mapY(y2)}" stroke="#3a4a41" stroke-width="4" stroke-linecap="square"/>`;
  });

  let paths = '';
  runs.forEach(r => {
    const pts = r.positions;
    const step = Math.max(1, Math.floor(pts.length / 500));
    let d = '';
    for (let i = 0; i < pts.length; i += step) {
      const px = mapX(pts[i].x), py = mapY(pts[i].y);
      d += (i === 0 ? 'M' : 'L') + px.toFixed(2) + ',' + py.toFixed(2) + ' ';
    }
    const last = pts[pts.length - 1];
    d += 'L' + mapX(last.x).toFixed(2) + ',' + mapY(last.y).toFixed(2);
    paths += `<path d="${d}" fill="none" stroke="${r.color}" stroke-width="2.2" opacity="0.95"/>`;
    paths += `<circle cx="${mapX(last.x)}" cy="${mapY(last.y)}" r="3.5" fill="${r.color}"/>`;
  });

  const startX = (runs.length > 0 && runs[0].positions.length > 0) ? runs[0].positions[0].x : 90;
  const startY = (runs.length > 0 && runs[0].positions.length > 0) ? runs[0].positions[0].y : 0;
  const startPt = `<circle cx="${mapX(startX)}" cy="${mapY(startY)}" r="3" fill="#dfe9e3"/>`;

  return `<svg width="${SVG_SIZE}" height="${SVG_SIZE}" viewBox="0 0 ${SVG_SIZE} ${SVG_SIZE}" style="background:#0b100e; border:1px solid #24322c; border-radius:3px;">
${gridLines}${wallLines}${paths}${startPt}
  </svg>`;
}

/* =========================================================
   RESULTS CARDS
   ========================================================= */
function formatMs(v) { return v.toFixed(0); }

function buildReadoutCard(ch, res) {
  const meta = CH_META[ch];
  return `<div class="readout-card" data-ch="${ch}">
<div class="rc-head"><div class="led"></div><div class="rc-name">${meta.label}</div></div>
<div class="rc-grid">
  <div class="rc-item"><span class="rc-label">t1 accel</span><span class="rc-value">${formatMs(res.t1)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">t2 cruise</span><span class="rc-value">${formatMs(res.t2)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">t3 decel</span><span class="rc-value">${formatMs(res.t3)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">T total</span><span class="rc-value">${formatMs(res.T)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">t1+t2</span><span class="rc-value">${formatMs(res.t1 + res.t2)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">peak accel</span><span class="rc-value">${res.peakAccel.toFixed(1)}<span class="rc-unit">rad/s&sup2;</span></span></div>
  <div class="rc-item"><span class="rc-label">peak speed</span><span class="rc-value">${res.peakOmega.toFixed(3)}<span class="rc-unit">rad/s</span></span></div>
  <div class="rc-item"><span class="rc-label">final position</span><span class="rc-value">${res.final.x.toFixed(1)}, ${res.final.y.toFixed(1)}, ${(res.final.theta * R2D).toFixed(2)}&#176;</span></div>
  <div class="rc-item"><span class="rc-label">t4 jerk decel</span><span class="rc-value">${formatMs(res.t4 || 0)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">t5 jerk accel</span><span class="rc-value">${formatMs(res.t5 || 0)}<span class="rc-unit">ms</span></span></div>
</div>
  </div>`;
}

/* =========================================================
   MAIN RENDER
   ========================================================= */
function applyBeforeAfterTurn(positions, mmBefore, mmAfter) {
  // mmBefore simply shifts the entire turn along y:
  //   negative → turn starts before y=0 (shifted down)
  //   positive → turn starts after y=0 (shifted up)
  const result = [];

  for (const pt of positions) {
    result.push({ x: pt.x, y: pt.y + mmBefore, theta: pt.theta });
  }

  // Append segment: robot continuing straight after the turn
  if (mmAfter > 0) {
    const last = result[result.length - 1];
    const finalTheta = last.theta;
    result.push({
      x: last.x + mmAfter * Math.sin(finalTheta),
      y: last.y + mmAfter * Math.cos(finalTheta),
      theta: finalTheta
    });
  }

  return result;
}

function recomputeAndRender() {
  const runs = [];
  const mmBefore = state.mmBeforeTurn || 0;
  const mmAfter = Math.max(0, state.mmAfterTurn || 0);
  ['trap', 'ideal', 'jerk', 'arc'].forEach(ch => {
    if (!activeChannels[ch]) return;
    let out;
    try {
      out = CH_META[ch].fn(state);
    } catch (e) {
      console.error(ch, e);
      return;
    }
    // Apply before/after turn offsets to trajectory positions
    out.positions = applyBeforeAfterTurn(out.positions, mmBefore, mmAfter);
    // Update final position in results to reflect trajectory modification
    out.results.final = out.positions[out.positions.length - 1];
    runs.push({ ch, color: CH_META[ch].color, ...out });
  });

  // readouts
  const readoutsEl = document.getElementById('readouts');
  const emptyEl = document.getElementById('empty-readouts');
  if (runs.length === 0) {
    readoutsEl.innerHTML = '';
    emptyEl.style.display = 'block';
  } else {
    emptyEl.style.display = 'none';
    readoutsEl.innerHTML = runs.map(r => buildReadoutCard(r.ch, r.results)).join('');
  }

  // charts
  omegaChart.data.datasets = runs.map(r => ({
    label: CH_META[r.ch].label,
    data: downsample(r.times, r.omegaArr),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  omegaChart.update('none');

  alphaChart.data.datasets = runs.map(r => ({
    label: CH_META[r.ch].label,
    data: downsample(r.times, r.alphaArr),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  alphaChart.update('none');

  // trajectory
  document.getElementById('traj-svg-wrap').innerHTML = buildTrajectorySVG(runs);
  document.getElementById('traj-legend').innerHTML = runs.map(r => `
<div class="legend-item"><div class="swatch" style="background:${r.color}"></div>${CH_META[r.ch].label}</div>
  `).join('') + `<div class="legend-note">Walls in dim green. Path traced from turn entry (white dot) to exit. Endpoint marked with filled dot per channel.</div>`;
}

// init field displays
fields.forEach(([id, key, decimals]) => syncFieldUI(id, decimals, key));
recomputeAndRender();
