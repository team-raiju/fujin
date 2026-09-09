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
  ['initialAngle', 'initialAngleDeg', 0],
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

// Bind plotTemplate select
state.plotTemplate = 'corner';
const selTemplate = document.getElementById('s-plotTemplate');
if (selTemplate) {
  state.plotTemplate = selTemplate.value;
  selTemplate.addEventListener('change', () => {
    state.plotTemplate = selTemplate.value;
    recomputeAndRender();
  });
}

// Bind simStep select
state.simStep = 0.001;
const selSimStep = document.getElementById('s-simStep');
if (selSimStep) {
  state.simStep = parseFloat(selSimStep.value);
  selSimStep.addEventListener('change', () => {
    state.simStep = parseFloat(selSimStep.value);
    recomputeAndRender();
  });
}

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

function baseChartOptions(yLabel, xLabel = 'time (ms)') {
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: { duration: 150 },
    parsing: false,
    interaction: { mode: 'index', intersect: false },
    scales: {
      x: {
        type: 'linear',
        title: { display: true, text: xLabel, color: '#526059' },
        grid: { color: '#1c2622' },
        ticks: {
          color: '#7d9188',
          callback: function(value) {
            return Number(value).toFixed(0);
          }
        }
      },
      y: {
        title: { display: true, text: yLabel, color: '#526059' },
        grid: { color: '#1c2622' },
        ticks: { color: '#7d9188' }
      }
    },
    plugins: {
      legend: { display: true, labels: { boxWidth: 12, boxHeight: 2, color: '#a9bab1' } },
      tooltip: {
        backgroundColor: '#0b100e',
        borderColor: '#24322c',
        borderWidth: 1,
        titleColor: '#dfe9e3',
        bodyColor: '#dfe9e3',
        callbacks: {
          title: function(context) {
            const val = context[0].parsed.x;
            return (xLabel.includes('time') ? 'Time: ' + val.toFixed(1) + ' ms' : 'Dist: ' + val.toFixed(1) + ' mm');
          }
        }
      }
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

function downsample(times, values, maxPoints = 2000) {
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

function wallSegments(template) {
  const segs = [];
  if (template === 'diagonal') {
    // Generate S-curve diagonal zigzag walls
    segs.push([-180, -180, -180, 0]);
    segs.push([0, 0, 0, 180]);
    segs.push([-180, 0, 0, 0]);
    segs.push([0, -180, 180, -180]);

    for (let k = 1; k < 20; k++) {
      const offset = 180 * k;
      segs.push([offset, offset - 360, offset, offset - 180]);
      segs.push([offset, offset, offset, offset + 180]);
      segs.push([offset - 180, offset, offset, offset]);
      segs.push([offset, offset - 180, offset + 180, offset - 180]);
    }
  } else if (template === 'multiple') {
    // V90 / S-Turn walls
    segs.push([-360, -180, -180, -180]);
    segs.push([-180, -180, -180, 0]);
    segs.push([-180, 0, 0, 0]);
    segs.push([0, 0, 0, 180]);
    segs.push([0, 180, 360, 180]);
    segs.push([360, 180, 540, 180]);
    segs.push([360, 0, 540, 0]);
    segs.push([540, 0, 540, -180]);
    segs.push([540, -180, 720, -180]);
    segs.push([180, -180, 180, 0]);
    segs.push([0, -250, 0, -180]);
    segs.push([360, -250, 360, -180]);
  } else {
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
  }
  return segs.filter(([x1, y1, x2, y2]) =>
    (x1 >= DATA_MIN && x1 <= DATA_MAX && y1 >= DATA_MIN && y1 <= DATA_MAX) ||
    (x2 >= DATA_MIN && x2 <= DATA_MAX && y2 >= DATA_MIN && y2 <= DATA_MAX)
  );
}

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
  const walls = wallSegments(state.plotTemplate);
  walls.forEach(([x1, y1, x2, y2]) => {
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
  const startTheta = (runs.length > 0 && runs[0].positions.length > 0) ? runs[0].positions[0].theta : 0;
  const appX = startX - 200 * Math.sin(startTheta);
  const appY = startY - 200 * Math.cos(startTheta);
  const approachLine = `<line x1="${mapX(appX)}" y1="${mapY(appY)}" x2="${mapX(startX)}" y2="${mapY(startY)}" stroke="#dfe9e3" stroke-width="1.5" stroke-dasharray="3,3" opacity="0.35"/>`;
  const startPt = `<circle cx="${mapX(startX)}" cy="${mapY(startY)}" r="3" fill="#dfe9e3"/>`;

  return `<svg width="${SVG_SIZE}" height="${SVG_SIZE}" viewBox="0 0 ${SVG_SIZE} ${SVG_SIZE}" style="background:#0b100e; border:1px solid #24322c; border-radius:3px;">
${gridLines}${wallLines}${approachLine}${paths}${startPt}
  </svg>`;
}

/* =========================================================
   RESULTS CARDS
   ========================================================= */
function formatMs(v) {
  const step = state.simStep || 0.001;
  return step < 0.001 ? v.toFixed(1) : v.toFixed(0);
}

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
  <div class="rc-item"><span class="rc-label">final position</span><span class="rc-value">${res.final.x.toFixed(2)}, ${res.final.y.toFixed(2)}, ${(res.final.theta * R2D).toFixed(2)}&#176;</span></div>
  <div class="rc-item"><span class="rc-label">t4 jerk decel</span><span class="rc-value">${formatMs(res.t4 || 0)}<span class="rc-unit">ms</span></span></div>
  <div class="rc-item"><span class="rc-label">t5 jerk accel</span><span class="rc-value">${formatMs(res.t5 || 0)}<span class="rc-unit">ms</span></span></div>
</div>
  </div>`;
}

/* =========================================================
   MAIN RENDER
   ========================================================= */
function applyBeforeAfterTurn(positions, mmBefore, mmAfter) {
  const result = [];
  if (positions.length === 0) return result;

  const startTheta = positions[0].theta;
  const dx = mmBefore * Math.sin(startTheta);
  const dy = mmBefore * Math.cos(startTheta);

  for (const pt of positions) {
    result.push({ x: pt.x + dx, y: pt.y + dy, theta: pt.theta });
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

/* =========================================================
   TAB SWITCHING
   ========================================================= */
const tabButtons = document.querySelectorAll('.tab-btn');
const tabViews = document.querySelectorAll('.tab-view');
let currentTab = 'angular';

tabButtons.forEach(btn => {
  btn.addEventListener('click', () => {
    const targetTab = btn.dataset.tab;
    if (targetTab === currentTab) return;
    currentTab = targetTab;
    tabButtons.forEach(b => b.classList.toggle('active', b === btn));
    tabViews.forEach(v => v.classList.toggle('active', v.id === `tab-${targetTab}`));

    if (targetTab === 'linear') {
      recomputeAndRenderLinear();
      linSpeedTimeChart.resize();
      linSpeedDistChart.resize();
      linAccelTimeChart.resize();
      linJerkTimeChart.resize();
    } else if (targetTab === 'angular') {
      recomputeAndRender();
      omegaChart.resize();
      alphaChart.resize();
    }
  });
});

/* =========================================================
   LINEAR MOTION UI STATE + WIRING
   ========================================================= */
const CH_META_LIN = {
  'lin-trap': { label: 'TRAPEZOIDAL', color: '#ff9d52', fn: simulateLinearTrapezoidal },
  'lin-scurve': { label: 'JERK S-CURVE', color: '#79ff9e', fn: simulateLinearSCurve },
};

const linFields = [
  ['linForwardCells', 'forwardCells', 0],
  ['linCellSize', 'cellSizeMm', 0],
  ['linStartDist', 'startDistMm', 1],
  ['linStopDist', 'stopDistMm', 1],
  ['linTrapSpeed', 'trapMaxSpeed', 3],
  ['linTrapAccel', 'trapAccel', 3],
  ['linTrapDecel', 'trapDecel', 3],
  ['linSCurveSpeed', 'sCurveMaxSpeed', 3],
  ['linSCurveAccel', 'sCurveAccel', 3],
  ['linSCurveDecel', 'sCurveDecel', 3],
  ['linSCurveJerkAcc', 'sCurveJerkAcc', 0],
  ['linSCurveJerkBrake', 'sCurveJerkBrake', 0],
  ['linBrakeMargin', 'brakeMarginMm', 1],
  ['linAccelMargin', 'accelMarginMm', 1],
  ['linStartSpeed', 'startMaxSpeed', 3],
  ['linStartAccel', 'startAccel', 1],
  ['linStopDecel', 'stopDecel', 1],
];

const stateLin = {};
linFields.forEach(([id, key]) => {
  const r = document.getElementById('r-' + id);
  if (r) stateLin[key] = parseFloat(r.value);
});

function syncLinFieldUI(id, decimals, key) {
  const v = stateLin[key];
  if (v === undefined) return;
  const strVal = String(v);
  const dotIdx = strVal.indexOf('.');
  const actualDecimals = dotIdx >= 0 ? strVal.length - dotIdx - 1 : 0;
  const showDecimals = Math.min(Math.max(decimals, actualDecimals), 6);
  const valEl = document.getElementById('v-' + id);
  const r = document.getElementById('r-' + id);
  const n = document.getElementById('n-' + id);
  if (valEl) valEl.textContent = v.toFixed(showDecimals);
  if (r) r.value = v;
  if (n) n.value = v;
}

linFields.forEach(([id, key, decimals]) => {
  const r = document.getElementById('r-' + id);
  const n = document.getElementById('n-' + id);
  if (r) {
    r.addEventListener('input', () => {
      stateLin[key] = parseFloat(r.value);
      syncLinFieldUI(id, decimals, key);
      recomputeAndRenderLinear();
    });
  }
  if (n) {
    n.addEventListener('input', () => {
      const val = parseFloat(n.value);
      if (!isNaN(val)) {
        stateLin[key] = val;
        const strVal = String(val);
        const dotIdx = strVal.indexOf('.');
        const actualDecimals = dotIdx >= 0 ? strVal.length - dotIdx - 1 : 0;
        const showDecimals = Math.min(Math.max(decimals, actualDecimals), 6);
        const valEl = document.getElementById('v-' + id);
        if (valEl) valEl.textContent = val.toFixed(showDecimals);
        if (r) r.value = val;
        recomputeAndRenderLinear();
      }
    });
  }
});

// Frequency & Derating controls
stateLin.simFrequency = 2000;
const selLinFreq = document.getElementById('s-linFrequency');
if (selLinFreq) {
  stateLin.simFrequency = parseFloat(selLinFreq.value);
  selLinFreq.addEventListener('change', () => {
    stateLin.simFrequency = parseFloat(selLinFreq.value);
    recomputeAndRenderLinear();
  });
}

stateLin.useDerating = true;
const chkLinDerating = document.getElementById('chk-linDerating');
if (chkLinDerating) {
  stateLin.useDerating = chkLinDerating.checked;
  chkLinDerating.addEventListener('change', () => {
    stateLin.useDerating = chkLinDerating.checked;
    recomputeAndRenderLinear();
  });
}

// Active channels for linear
const activeChannelsLin = { 'lin-trap': true, 'lin-scurve': true };
['lin-trap', 'lin-scurve'].forEach(ch => {
  const row = document.querySelector(`.channel-row[data-ch="${ch}"]`);
  const chk = document.getElementById('chk-' + ch);
  const group = document.querySelector(`.group[data-owner="${ch}"]`);
  function apply() {
    activeChannelsLin[ch] = chk.checked;
    row.classList.toggle('active', chk.checked);
    if (group) group.classList.toggle('on', chk.checked);
    recomputeAndRenderLinear();
  }
  if (row && chk) {
    row.addEventListener('click', (e) => {
      if (e.target === chk) return;
      chk.checked = !chk.checked;
      apply();
    });
    chk.addEventListener('change', apply);
    if (group) group.classList.toggle('on', chk.checked);
  }
});

/* =========================================================
   LINEAR CHARTS
   ========================================================= */
const linSpeedTimeChart = new Chart(document.getElementById('chart-lin-speed-time'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('m/s', 'time (ms)')
});

const linSpeedDistChart = new Chart(document.getElementById('chart-lin-speed-dist'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('m/s', 'distance (mm)')
});

const linAccelTimeChart = new Chart(document.getElementById('chart-lin-accel-time'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('m/s²', 'time (ms)')
});

const linJerkTimeChart = new Chart(document.getElementById('chart-lin-jerk-time'), {
  type: 'line',
  data: { datasets: [] },
  options: baseChartOptions('m/s³', 'time (ms)')
});

function downsamplePair(xArr, yArr, maxPoints = 2000) {
  const n = xArr.length;
  if (n <= maxPoints) return xArr.map((x, i) => ({ x, y: yArr[i] }));
  const step = Math.ceil(n / maxPoints);
  const out = [];
  for (let i = 0; i < n; i += step) out.push({ x: xArr[i], y: yArr[i] });
  out.push({ x: xArr[n - 1], y: yArr[n - 1] });
  return out;
}

/* =========================================================
   LINEAR READOUT CARDS & COMPARISON BANNER
   ========================================================= */
function buildLinearReadoutCard(ch, res) {
  const meta = CH_META_LIN[ch];
  const timeS = (res.totalTimeMs / 1000.0).toFixed(3);
  return `<div class="readout-card" data-ch="${ch}">
    <div class="rc-head"><div class="led"></div><div class="rc-name">${meta.label}</div></div>
    <div class="rc-grid">
      <div class="rc-item"><span class="rc-label">Total Time</span><span class="rc-value">${res.totalTimeMs.toFixed(1)}<span class="rc-unit">ms</span> <span style="font-size:10px; color:var(--muted)">(${timeS}s)</span></span></div>
      <div class="rc-item"><span class="rc-label">Total Distance</span><span class="rc-value">${res.totalDistMm.toFixed(1)}<span class="rc-unit">mm</span></span></div>
      <div class="rc-item"><span class="rc-label">Peak Speed</span><span class="rc-value">${res.peakSpeed.toFixed(3)}<span class="rc-unit">m/s</span></span></div>
      <div class="rc-item"><span class="rc-label">Peak Accel</span><span class="rc-value">${res.peakAccel.toFixed(1)}<span class="rc-unit">m/s&sup2;</span></span></div>
      <div class="rc-item"><span class="rc-label">Peak Brake Decel</span><span class="rc-value">${res.peakDecel.toFixed(1)}<span class="rc-unit">m/s&sup2;</span></span></div>
      <div class="rc-item"><span class="rc-label">Peak Jerk</span><span class="rc-value">${Math.round(res.peakJerk).toLocaleString()}<span class="rc-unit">m/s&sup3;</span></span></div>
      <div class="rc-item"><span class="rc-label">Cruise Time</span><span class="rc-value">${res.cruiseTimeMs.toFixed(1)}<span class="rc-unit">ms</span></span></div>
      <div class="rc-item"><span class="rc-label">Cruise Dist</span><span class="rc-value">${res.cruiseDistMm.toFixed(1)}<span class="rc-unit">mm</span></span></div>
    </div>
  </div>`;
}

function buildLinearComparisonBanner(runs) {
  const trapRun = runs.find(r => r.ch === 'lin-trap');
  const scurveRun = runs.find(r => r.ch === 'lin-scurve');
  if (!trapRun || !scurveRun) return '';

  const tTrap = trapRun.results.totalTimeMs;
  const tSCurve = scurveRun.results.totalTimeMs;
  const deltaT = tSCurve - tTrap;
  const pct = ((deltaT / tTrap) * 100).toFixed(1);
  const sign = deltaT >= 0 ? '+' : '';

  const jerkTrap = trapRun.results.peakJerk;
  const jerkSCurve = scurveRun.results.peakJerk;
  const jerkReduction = jerkSCurve > 0 ? (jerkTrap / jerkSCurve).toFixed(0) : 'N/A';

  return `<div class="comparison-banner">
    <div class="cb-title">
      <span>⚖️ PROFILE COMPARISON</span>
    </div>
    <div class="cb-metrics">
      <div class="cb-item">
        <span class="cb-label">Time Difference:</span>
        <span class="cb-val" style="color:${deltaT > 0 ? 'var(--trap)' : 'var(--jerk)'};">${sign}${deltaT.toFixed(1)} ms (${sign}${pct}%)</span>
      </div>
      <div class="cb-item">
        <span class="cb-label">Jerk Spike:</span>
        <span class="cb-val" style="color:var(--jerk);">${jerkReduction}x lower shock in S-Curve</span>
      </div>
      <div class="cb-item">
        <span class="cb-label">Achieved Speed Diff:</span>
        <span class="cb-val">${(scurveRun.results.peakSpeed - trapRun.results.peakSpeed).toFixed(3)} m/s</span>
      </div>
    </div>
  </div>`;
}

/* =========================================================
   LINEAR TRACK SVG
   ========================================================= */
function buildLinearTrackSVG(runs, p) {
  const startDist = p.startDistMm ?? 111.0;
  const forwardCells = Math.max(0, Math.min(16, Math.round(p.forwardCells ?? 5)));
  const cellSize = p.cellSizeMm ?? 180.0;
  const stopDist = p.stopDistMm ?? 80.0;
  const totalTrackMm = startDist + forwardCells * cellSize + stopDist;

  const svgW = 1000;
  const svgH = 130;
  const padLeft = 45;
  const padRight = 45;
  const padTop = 22;
  const padBottom = 26;
  const plotW = svgW - padLeft - padRight;
  const plotH = svgH - padTop - padBottom;

  function toX(dMm) {
    return padLeft + (Math.max(0, Math.min(totalTrackMm, dMm)) / totalTrackMm) * plotW;
  }

  let cellRects = '';
  let cellLabels = '';
  let gridLines = '';

  // 1. START cell
  const startX1 = toX(0);
  const startX2 = toX(startDist);
  cellRects += `<rect x="${startX1}" y="${padTop}" width="${startX2 - startX1}" height="${plotH}" fill="#16201d" stroke="#24322c" stroke-width="1.2"/>`;
  cellLabels += `<text x="${(startX1 + startX2) / 2}" y="${padTop - 6}" fill="#7d9188" font-family="'IBM Plex Mono', monospace" font-size="10" text-anchor="middle">START (${startDist}mm)</text>`;

  // 2. FORWARD cells
  for (let c = 0; c < forwardCells; c++) {
    const d1 = startDist + c * cellSize;
    const d2 = d1 + cellSize;
    const x1 = toX(d1);
    const x2 = toX(d2);
    const bgFill = (c % 2 === 0) ? 'rgba(255,255,255,0.015)' : 'rgba(255,255,255,0.03)';
    cellRects += `<rect x="${x1}" y="${padTop}" width="${x2 - x1}" height="${plotH}" fill="${bgFill}" stroke="#24322c" stroke-width="1"/>`;
    cellLabels += `<text x="${(x1 + x2) / 2}" y="${padTop - 6}" fill="#526059" font-family="'IBM Plex Mono', monospace" font-size="10" text-anchor="middle">Cell ${c + 1}</text>`;
    gridLines += `<text x="${x2}" y="${padTop + plotH + 15}" fill="#526059" font-family="'IBM Plex Mono', monospace" font-size="9" text-anchor="middle">${d2.toFixed(0)}</text>`;
  }

  // 3. STOP cell
  const stopD1 = startDist + forwardCells * cellSize;
  const stopD2 = totalTrackMm;
  const stopX1 = toX(stopD1);
  const stopX2 = toX(stopD2);
  cellRects += `<rect x="${stopX1}" y="${padTop}" width="${stopX2 - stopX1}" height="${plotH}" fill="#16201d" stroke="#24322c" stroke-width="1.2"/>`;
  cellLabels += `<text x="${(stopX1 + stopX2) / 2}" y="${padTop - 6}" fill="#7d9188" font-family="'IBM Plex Mono', monospace" font-size="10" text-anchor="middle">STOP (${stopDist}mm)</text>`;
  gridLines += `<text x="${stopX2}" y="${padTop + plotH + 15}" fill="#7d9188" font-family="'IBM Plex Mono', monospace" font-size="9" text-anchor="middle">${totalTrackMm.toFixed(0)} mm</text>`;
  gridLines += `<text x="${startX1}" y="${padTop + plotH + 15}" fill="#7d9188" font-family="'IBM Plex Mono', monospace" font-size="9" text-anchor="middle">0</text>`;

  // Track profile curves
  let pathLines = '';
  let transitionMarkers = '';

  const maxSpeed = Math.max(0.1, ...runs.map(r => r.results.peakSpeed));
  function toY(v) {
    return padTop + plotH - (v / maxSpeed) * (plotH - 12) - 4;
  }

  runs.forEach(r => {
    const pts = r.distances;
    const vArr = r.speeds;
    const step = Math.max(1, Math.floor(pts.length / 400));
    let d = '';
    for (let i = 0; i < pts.length; i += step) {
      const px = toX(pts[i]).toFixed(1);
      const py = toY(vArr[i]).toFixed(1);
      d += (i === 0 ? 'M' : 'L') + px + ',' + py + ' ';
    }
    const lastIdx = pts.length - 1;
    d += 'L' + toX(pts[lastIdx]).toFixed(1) + ',' + toY(vArr[lastIdx]).toFixed(1);
    pathLines += `<path d="${d}" fill="none" stroke="${r.color}" stroke-width="2.2" opacity="0.95"/>`;

    // Stopping point dot
    const stopX = toX(r.results.totalDistMm);
    const stopY = toY(0);
    pathLines += `<circle cx="${stopX}" cy="${stopY}" r="4" fill="${r.color}"/>`;

    // Transitions
    if (r.transitions) {
      r.transitions.forEach(tr => {
        const tx = toX(tr.distMm);
        const ty = toY(tr.speed);
        transitionMarkers += `<line x1="${tx}" y1="${padTop}" x2="${tx}" y2="${padTop + plotH}" stroke="${r.color}" stroke-width="1" stroke-dasharray="3,3" opacity="0.4"/>`;
        transitionMarkers += `<circle cx="${tx}" cy="${ty}" r="3" fill="${r.color}" stroke="#0b100e" stroke-width="1"/>`;
      });
    }
  });

  return `<svg id="lin-track-svg" viewBox="0 0 ${svgW} ${svgH}" width="100%" height="${svgH}" style="background:#0b100e; border:1px solid #24322c; border-radius:3px;">
    ${cellRects}
    ${cellLabels}
    ${gridLines}
    ${transitionMarkers}
    ${pathLines}
  </svg>`;
}

/* =========================================================
   LINEAR MAIN RENDER
   ========================================================= */
function recomputeAndRenderLinear() {
  const runs = [];
  ['lin-trap', 'lin-scurve'].forEach(ch => {
    if (!activeChannelsLin[ch]) return;
    let out;
    try {
      out = CH_META_LIN[ch].fn(stateLin);
    } catch (e) {
      console.error(ch, e);
      return;
    }
    runs.push({ ch, color: CH_META_LIN[ch].color, ...out });
  });

  // Readouts
  const readoutsEl = document.getElementById('lin-readouts');
  const emptyEl = document.getElementById('lin-empty-readouts');
  const bannerEl = document.getElementById('lin-comparison-banner');

  if (runs.length === 0) {
    readoutsEl.innerHTML = '';
    emptyEl.style.display = 'block';
    bannerEl.innerHTML = '';
  } else {
    emptyEl.style.display = 'none';
    readoutsEl.innerHTML = runs.map(r => buildLinearReadoutCard(r.ch, r.results)).join('');
    bannerEl.innerHTML = buildLinearComparisonBanner(runs);
  }

  // Update Chart 1: Speed vs Time
  linSpeedTimeChart.data.datasets = runs.map(r => ({
    label: CH_META_LIN[r.ch].label,
    data: downsamplePair(r.times, r.speeds),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  linSpeedTimeChart.update('none');

  // Update Chart 2: Speed vs Distance
  linSpeedDistChart.data.datasets = runs.map(r => ({
    label: CH_META_LIN[r.ch].label,
    data: downsamplePair(r.distances, r.speeds),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  linSpeedDistChart.update('none');

  // Update Chart 3: Accel vs Time
  linAccelTimeChart.data.datasets = runs.map(r => ({
    label: CH_META_LIN[r.ch].label,
    data: downsamplePair(r.times, r.accels),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  linAccelTimeChart.update('none');

  // Update Chart 4: Jerk vs Time
  linJerkTimeChart.data.datasets = runs.map(r => ({
    label: CH_META_LIN[r.ch].label,
    data: downsamplePair(r.times, r.jerks),
    borderColor: r.color,
    backgroundColor: r.color,
    fill: false
  }));
  linJerkTimeChart.update('none');

  // Track SVG
  const trackWrap = document.getElementById('lin-track-wrap');
  const trackLegend = document.getElementById('lin-track-legend');
  if (trackWrap) {
    trackWrap.innerHTML = buildLinearTrackSVG(runs, stateLin);
  }
  if (trackLegend) {
    trackLegend.innerHTML = runs.map(r => `
      <div class="legend-item"><div class="swatch" style="background:${r.color}"></div>${CH_META_LIN[r.ch].label}</div>
    `).join('') + `<div class="legend-note">Cell borders every 180mm. Dashed markers denote START, FORWARD and STOP transitions. Dots indicate stopping positions.</div>`;
  }
}

// Init linear field displays & initial render
linFields.forEach(([id, key, decimals]) => syncLinFieldUI(id, decimals, key));
recomputeAndRenderLinear();

