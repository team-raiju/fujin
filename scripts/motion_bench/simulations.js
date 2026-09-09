/* =========================================================
   MATH — three motion profiles
   ========================================================= */
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const DT = 0.001;

function simulateTrapezoidal(p) {
  const turnAngleRad = p.turnAngleDeg * D2R;
  const maxAlpha = p.trapAccel, maxDecel = p.trapDecel, maxOmega = p.trapOmega;

  let t1 = (maxOmega / maxAlpha) * 1000;
  let t3 = (maxOmega / maxDecel) * 1000;
  let t2 = (turnAngleRad / maxOmega) * 1000 - t1 / 2 - t3 / 2;

  if (t2 < 0) {
    t2 = 0;
    t1 = Math.sqrt((2 * turnAngleRad) / ((1 + maxAlpha / maxDecel) * maxAlpha)) * 1000;
    t3 = t1 * (maxAlpha / maxDecel);
  }
  const T = t1 + t2 + t3;

  const dt = p.simStep || 0.001;
  const dtMs = dt * 1000;

  const initialAngleRad = (p.initialAngleDeg || 0) * D2R;
  const times = [0], omegaArr = [0], alphaArr = [0];
  const positions = [{ x: 90, y: 0, theta: initialAngleRad }];
  let omega = 0, theta = initialAngleRad;
  const totalSteps = Math.max(1, Math.round(T / dtMs));

  for (let step = 1; step <= totalSteps; step++) {
    const tMs = step * dtMs;
    let alpha;
    if (tMs <= t1) alpha = maxAlpha;
    else if (tMs <= t1 + t2) alpha = 0;
    else if (tMs <= T) alpha = -maxDecel;
    else alpha = 0;

    omega += alpha * dt;
    if (omega > maxOmega) omega = maxOmega;
    if (omega < 0) omega = 0;
    theta += omega * dt;

    const last = positions[positions.length - 1];
    const x = last.x + (p.linearSpeed * 1000 * dt) * Math.sin(theta);
    const y = last.y + (p.linearSpeed * 1000 * dt) * Math.cos(theta);
    positions.push({ x, y, theta });
    times.push(tMs);
    omegaArr.push(omega);
    alphaArr.push(alpha);
  }

  return {
    times, omegaArr, alphaArr, positions,
    results: {
      t1: Math.round(t1 * 10) / 10, t2: Math.round(t2 * 10) / 10, t3: Math.round(t3 * 10) / 10, T: Math.round(T * 10) / 10,
      t4: 0, t5: 0,
      peakAccel: maxAlpha, peakOmega: Math.max(...omegaArr),
      final: positions[positions.length - 1]
    }
  };
}

function simulateIdealCurve(p) {
  const dt = p.simStep || 0.001;
  const dtMs = dt * 1000;
  const turnAngleRad = p.turnAngleDeg * D2R;
  const { idealAbsMax: absMaxOmega, idealRefOmega: refOmega, idealRefAccel: refAccel, idealMaxTarget: maxTargetOmega } = p;
  const alpha0 = refAccel / (1 - refOmega / absMaxOmega);

  let tempOmega = 0, thetaAccel = 0;
  const accelArr = [], omegaPreArr = [];
  let guard = 0;
  while (tempOmega < maxTargetOmega && guard < 200000) {
    guard++;
    const alpha = alpha0 * (1 - tempOmega / absMaxOmega);
    accelArr.push(alpha);
    omegaPreArr.push(tempOmega);
    thetaAccel += tempOmega * dt;
    tempOmega += alpha * dt;
    if (alpha <= 0) break;
  }

  let fullOmega = [], fullAlpha = [];

  if (omegaPreArr.length === 0) {
    fullOmega = [0]; fullAlpha = [0];
  } else if (2 * thetaAccel <= turnAngleRad) {
    const cruiseAngle = turnAngleRad - 2 * thetaAccel;
    const cruiseSteps = Math.max(0, Math.round(cruiseAngle / (maxTargetOmega * dt)));
    fullOmega = fullOmega.concat(omegaPreArr);
    fullAlpha = fullAlpha.concat(accelArr);
    for (let i = 0; i < cruiseSteps; i++) { fullOmega.push(maxTargetOmega); fullAlpha.push(0); }
    fullOmega = fullOmega.concat([...omegaPreArr].reverse());
    fullAlpha = fullAlpha.concat([...accelArr].reverse().map(a => -a));
  } else {
    const targetHalf = turnAngleRad / 2;
    let currentAngle = 0, idx = omegaPreArr.length - 1;
    for (let i = 0; i < omegaPreArr.length; i++) {
      currentAngle += omegaPreArr[i] * dt;
      if (currentAngle >= targetHalf) { idx = i; break; }
    }
    fullOmega = fullOmega.concat(omegaPreArr.slice(0, idx));
    fullAlpha = fullAlpha.concat(accelArr.slice(0, idx));
    fullOmega = fullOmega.concat([...omegaPreArr.slice(0, idx)].reverse());
    fullAlpha = fullAlpha.concat([...accelArr.slice(0, idx)].reverse().map(a => -a));
  }

  const initialAngleRad = (p.initialAngleDeg || 0) * D2R;
  const times = [0], omegaArr = [0], alphaArr = [0];
  const positions = [{ x: 90, y: 0, theta: initialAngleRad }];
  let theta = initialAngleRad;
  for (let i = 0; i < fullOmega.length; i++) {
    const omega = fullOmega[i];
    theta += omega * dt;
    const last = positions[positions.length - 1];
    const x = last.x + (p.linearSpeed * 1000 * dt) * Math.sin(theta);
    const y = last.y + (p.linearSpeed * 1000 * dt) * Math.cos(theta);
    positions.push({ x, y, theta });
    times.push((i + 1) * dtMs);
    omegaArr.push(omega);
    alphaArr.push(fullAlpha[i]);
  }

  const t1 = omegaPreArr.length * dtMs;
  const T = fullOmega.length * dtMs;
  const t2 = T - 2 * t1 > 0 ? T - 2 * t1 : 0;
  return {
    times, omegaArr, alphaArr, positions,
    results: {
      t1: Math.round(t1 * 10) / 10, t2: Math.round(t2 * 10) / 10, t3: Math.round(t1 * 10) / 10, T: Math.round(T * 10) / 10,
      t4: 0, t5: 0,
      peakAccel: fullAlpha.length ? Math.max(...fullAlpha) : 0, peakOmega: fullOmega.length ? Math.max(...fullOmega) : 0,
      final: positions[positions.length - 1]
    }
  };
}

function simulateJerk(p) {
  const turnAngleRad = p.turnAngleDeg * D2R;
  const j1 = p.jerkJerk;
  const j2 = p.jerkJerkAfter;
  const maxAlpha = p.jerkAccel;
  const maxOmega = p.jerkOmega;
  const dt = p.simStep || 0.001;

  const theta_half = turnAngleRad / 2.0;

  // Calculate what speed we would reach if we didn't have enough room to hit max acceleration
  const K = 1.0 / (6.0 * j1 * j1) + 1.0 / (2.0 * j1 * j2) + 1.0 / (3.0 * j2 * j2);
  const A_peak_dist = Math.pow(theta_half / K, 1.0 / 3.0);

  let achievable_omega;
  if (A_peak_dist <= maxAlpha) {
    // Turn is very short, we don't even reach maxAlpha
    achievable_omega = 0.5 * (A_peak_dist * A_peak_dist) * (1.0 / j1 + 1.0 / j2);
  } else {
    // We reach maxAlpha
    const achieved_alpha = maxAlpha;
    const tau1 = achieved_alpha / j1;
    const tau3 = achieved_alpha / j2;
    const omega_1 = 0.5 * j1 * (tau1 * tau1);

    const C_quad = (1.0 / 6.0) * (achieved_alpha * achieved_alpha * achieved_alpha) / (j1 * j1) + omega_1 * tau3 + (1.0 / 3.0) * (achieved_alpha * achieved_alpha * achieved_alpha) / (j2 * j2) - theta_half;
    const B_quad = omega_1 + achieved_alpha * tau3;
    const A_quad = 0.5 * achieved_alpha;

    const discriminant = B_quad * B_quad - 4 * A_quad * C_quad;
    const tau2 = discriminant >= 0 ? (-B_quad + Math.sqrt(discriminant)) / (2 * A_quad) : 0.0;
    achievable_omega = omega_1 + achieved_alpha * tau2 + 0.5 * j2 * (tau3 * tau3);
  }

  // The actual peak omega is restricted by our physical speed cap
  const peak_omega = Math.min(maxOmega, achievable_omega);

  // Now recalculate phase times based on the locked-in peak_omega
  const A_req = Math.sqrt(2.0 * peak_omega / (1.0 / j1 + 1.0 / j2));

  let achieved_alpha, tau1, tau2, tau3;
  if (A_req <= maxAlpha) {
    // Triangular acceleration profile
    achieved_alpha = A_req;
    tau1 = achieved_alpha / j1;
    tau2 = 0.0;
    tau3 = achieved_alpha / j2;
  } else {
    // Trapezoidal acceleration profile
    achieved_alpha = maxAlpha;
    tau1 = achieved_alpha / j1;
    tau3 = achieved_alpha / j2;
    const omega_ramp = 0.5 * (achieved_alpha * achieved_alpha) * (1.0 / j1 + 1.0 / j2);
    tau2 = (peak_omega - omega_ramp) / achieved_alpha;
  }

  // Define alpha functions for simulation
  const alpha_phase1 = (t) => tau1 > 0 ? Math.min(achieved_alpha, achieved_alpha * (t / tau1)) : 0.0;
  const alpha_phase2 = (t) => achieved_alpha;
  const alpha_phase3 = (t) => tau3 > 0 ? Math.max(0.0, achieved_alpha * (1.0 - t / tau3)) : 0.0;

  // Compute Euler-simulated theta to match the exact discretised simulation steps
  function compute_simulated_theta_1() {
    let theta = 0, omega = 0;
    const phases = [
      { fn: alpha_phase1, dur: tau1 },
      { fn: alpha_phase2, dur: tau2 },
      { fn: alpha_phase3, dur: tau3 }
    ];
    for (const p of phases) {
      const n = Math.round(p.dur / dt);
      for (let i = 0; i < n; i++) {
        const alpha = p.fn((i + 1) * dt);
        omega += alpha * dt;
        omega = Math.min(Math.max(omega, 0.0), maxOmega);
        theta += omega * dt;
      }
    }
    return { theta, omega };
  }

  const { theta: sim_theta_1, omega: sim_omega_peak } = compute_simulated_theta_1();

  // Define alpha functions for deceleration
  const alpha_decelA = (t) => tau3 > 0 ? Math.max(-achieved_alpha, -achieved_alpha * (t / tau3)) : 0.0;
  const alpha_decelB = (t) => -achieved_alpha;
  const alpha_decelC = (t) => tau1 > 0 ? Math.min(0.0, -achieved_alpha * (1.0 - t / tau1)) : 0.0;

  // Compute Euler-simulated theta to match the exact discretised simulation steps for deceleration
  function compute_simulated_theta_2() {
    let theta = 0, omega = sim_omega_peak;
    const phases = [
      { fn: alpha_decelA, dur: tau3 },
      { fn: alpha_decelB, dur: tau2 },
      { fn: alpha_decelC, dur: tau1 }
    ];
    for (const p of phases) {
      const n = Math.round(p.dur / dt);
      for (let i = 0; i < n; i++) {
        const alpha = p.fn((i + 1) * dt);
        omega += alpha * dt;
        omega = Math.min(Math.max(omega, 0.0), maxOmega);
        theta += omega * dt;
      }
    }
    return theta;
  }

  const sim_theta_2 = compute_simulated_theta_2();

  // Calculate cruise phase based on the simulated acceleration and deceleration angles to avoid rounding errors
  let cruise_angle_rad = turnAngleRad - (sim_theta_1 + sim_theta_2);
  let cruise_time_s = 0.0;
  if (cruise_angle_rad < 1e-9) {
    cruise_angle_rad = 0.0;
    cruise_time_s = 0.0;
  } else {
    cruise_time_s = cruise_angle_rad / sim_omega_peak;
  }

  // Full simulation
  const initialAngleRad = (p.initialAngleDeg || 0) * D2R;
  const times = [0], omegaArr = [0], alphaArr = [0];
  const positions = [{ x: 90, y: 0, theta: initialAngleRad }];
  let theta = initialAngleRad, omega = 0, tMs = 0;
  const dtMs = dt * 1000;

  function step(alpha) {
    omega += alpha * dt;
    omega = Math.min(Math.max(omega, 0.0), maxOmega);
    theta += omega * dt;
    const last = positions[positions.length - 1];
    const x = last.x + (p.linearSpeed * 1000 * dt) * Math.sin(theta);
    const y = last.y + (p.linearSpeed * 1000 * dt) * Math.cos(theta);
    positions.push({ x, y, theta });
    tMs += dtMs;
    times.push(tMs);
    omegaArr.push(omega);
    alphaArr.push(alpha);
  }

  // --- Phase 1: jerk ramp-up ---
  const n1 = Math.max(0, Math.round(tau1 / dt));
  for (let i = 0; i < n1; i++) {
    step(alpha_phase1((i + 1) * dt));
  }

  // --- Phase 2: constant max acceleration ---
  const n2 = Math.max(0, Math.round(tau2 / dt));
  for (let i = 0; i < n2; i++) {
    step(alpha_phase2((i + 1) * dt));
  }

  // --- Phase 3: jerk ramp-down ---
  const n3 = Math.max(0, Math.round(tau3 / dt));
  for (let i = 0; i < n3; i++) {
    step(alpha_phase3((i + 1) * dt));
  }

  // --- Cruise phase ---
  const nCruise = Math.max(0, Math.round(cruise_time_s / dt));
  for (let i = 0; i < nCruise; i++) {
    step(0.0);
  }

  // --- Decel A: jerk ramp-up (deceleration grows 0 -> -max_alpha) ---
  const nDecelA = Math.max(0, Math.round(tau3 / dt));
  for (let i = 0; i < nDecelA; i++) {
    step(alpha_decelA((i + 1) * dt));
  }

  // --- Decel B: constant max deceleration ---
  const nDecelB = Math.max(0, Math.round(tau2 / dt));
  for (let i = 0; i < nDecelB; i++) {
    step(alpha_decelB((i + 1) * dt));
  }

  // --- Decel C: jerk ramp-down (deceleration eases back to 0) ---
  const nDecelC = Math.max(0, Math.round(tau1 / dt));
  for (let i = 0; i < nDecelC; i++) {
    step(alpha_decelC((i + 1) * dt));
  }

  const t1 = (n1 + n2 + n3) * dtMs;
  const t2 = nCruise * dtMs;

  return {
    times, omegaArr, alphaArr, positions,
    results: {
      t1: Math.round(t1 * 10) / 10, t2: Math.round(t2 * 10) / 10, t3: Math.round(t1 * 10) / 10, T: Math.round((2 * t1 + t2) * 10) / 10,
      t4: Math.round((n1 + n2) * dtMs * 10) / 10,
      t5: Math.round((n1 + n2 + n3 + nCruise + nDecelA + nDecelB) * dtMs * 10) / 10,
      peakAccel: achieved_alpha, peakOmega: sim_omega_peak,
      final: positions[positions.length - 1]
    }
  };
}

function simulateArc(p) {
  const dt = p.simStep || 0.001;
  const dtMs = dt * 1000;
  const linear_speed_mm_ms = p.linearSpeed;
  const turn_sign = 1;
  const total_turn_distance_mm = 2.0 * p.arcTransition + p.arcArc;

  if (linear_speed_mm_ms <= 0.0001 || total_turn_distance_mm <= 0) {
    const initialAngleRad = (p.initialAngleDeg || 0) * D2R;
    return {
      times: [0], omegaArr: [0], alphaArr: [0],
      positions: [{ x: 90, y: 0, theta: initialAngleRad }],
      results: { t1: 0, t2: 0, t3: 0, T: 0, peakAccel: 0, peakOmega: 0, final: { x: 90, y: 0, theta: initialAngleRad } }
    };
  }

  let travelled_mm = 0.0;
  let tMs = 0;
  let last_angular_speed_rad_s = 0.0;
  let max_measured_acceleration_rad_s2 = 0.0;
  let t1_ms = null;
  let t_decel_start = null;

  const initialAngleRad = (p.initialAngleDeg || 0) * D2R;
  const times = [0];
  const omegaArr = [0.0];
  const alphaArr = [0.0];
  const positions = [{ x: 90, y: 0, theta: initialAngleRad }];

  while (travelled_mm < total_turn_distance_mm && tMs < 20000) {
    tMs += dtMs;
    // Increment travelled_mm first as we move forward during this step
    travelled_mm += linear_speed_mm_ms * dtMs;

    let angular_speed_rad_s = turn_sign * p.arcOmega;

    if (travelled_mm < p.arcTransition) {
      const factor = travelled_mm / p.arcTransition;
      angular_speed_rad_s *= Math.sin(factor * Math.PI / 2.0);
    } else if (travelled_mm >= p.arcTransition + p.arcArc) {
      const factor = (travelled_mm - p.arcArc) / p.arcTransition;
      angular_speed_rad_s *= Math.sin(factor * Math.PI / 2.0);
    }

    if (t1_ms === null && travelled_mm >= p.arcTransition) {
      t1_ms = tMs;
    }
    if (t_decel_start === null && travelled_mm >= p.arcTransition + p.arcArc) {
      t_decel_start = tMs;
    }

    const current_accel = (angular_speed_rad_s - last_angular_speed_rad_s) / dt;
    const measured_accel = Math.abs(current_accel);
    if (measured_accel > max_measured_acceleration_rad_s2) {
      max_measured_acceleration_rad_s2 = measured_accel;
    }

    const last = positions[positions.length - 1];
    const theta = last.theta + (angular_speed_rad_s * dt);
    const x = last.x + linear_speed_mm_ms * dtMs * Math.sin(theta);
    const y = last.y + linear_speed_mm_ms * dtMs * Math.cos(theta);

    positions.push({ x, y, theta });
    omegaArr.push(angular_speed_rad_s);
    alphaArr.push(current_accel);
    times.push(tMs);

    last_angular_speed_rad_s = angular_speed_rad_s;
  }

  t1_ms = t1_ms ?? 0;
  t_decel_start = t_decel_start ?? tMs;

  return {
    times,
    omegaArr,
    alphaArr,
    positions,
    results: {
      t1: Math.round(t1_ms * 10) / 10,
      t2: Math.round((t_decel_start - t1_ms) * 10) / 10,
      t3: Math.round((tMs - t_decel_start) * 10) / 10,
      T: Math.round(tMs * 10) / 10,
      t4: 0, t5: 0,
      peakAccel: max_measured_acceleration_rad_s2,
      peakOmega: Math.max(...omegaArr.map(Math.abs)),
      final: positions[positions.length - 1]
    }
  };
}

/* =========================================================
   LINEAR MOTION SIMULATIONS (TRAPEZOIDAL & JERK S-CURVE)
   Based on simulate_s_curve.py and firmware/src/services/navigation.cpp
   ========================================================= */

function getTorricelliDistance(finalSpeed, initialSpeed, acceleration) {
  if (acceleration === 0.0) return 0.0;
  return (finalSpeed * finalSpeed - initialSpeed * initialSpeed) / (2.0 * acceleration);
}

function getSCurveBrakeDistance(initialSpeed, finalSpeed, deceleration, jerk) {
  if (initialSpeed <= finalSpeed || deceleration <= 0.0 || jerk <= 0.0) {
    return 0.0;
  }
  const deltaV = initialSpeed - finalSpeed;
  const vThreshold = (deceleration * deceleration) / jerk;

  if (deltaV >= vThreshold) {
    return ((initialSpeed * initialSpeed - finalSpeed * finalSpeed) / (2.0 * deceleration)) +
           (((initialSpeed + finalSpeed) * deceleration) / (2.0 * jerk));
  } else {
    return (initialSpeed + finalSpeed) * Math.sqrt(deltaV / jerk);
  }
}

function startAccelRampDown(currentSpeed, currentAccel, maxSpeed, jerk) {
  if (currentAccel <= 0.0 || jerk <= 0.0) return false;
  const predSpeed = currentSpeed + (currentAccel * currentAccel) / (2.0 * jerk);
  return predSpeed >= maxSpeed;
}

function startBrakeRampUp(currentSpeed, currentAccel, finalSpeed, jerk) {
  if (currentAccel > 0.0 || jerk <= 0.0) return false;
  const predSpeed = currentSpeed - (currentAccel * currentAccel) / (2.0 * jerk);
  return predSpeed <= finalSpeed;
}

function getEffectiveMaxAcceleration(currentSpeed, baseMaxAccel) {
  if (currentSpeed > 7.0) return 0.50 * baseMaxAccel;
  if (currentSpeed > 6.0) return 0.70 * baseMaxAccel;
  if (currentSpeed > 5.0) return 0.85 * baseMaxAccel;
  return baseMaxAccel;
}

/**
 * Builds the movement sequence array: START -> FORWARD (0..16 cells) -> STOP
 */
function buildLinearMovementSequence(p, profileKey) {
  const isSCurve = (profileKey === 'sCurve');
  const maxSpeed = isSCurve ? (p.sCurveMaxSpeed ?? 7.5) : (p.trapMaxSpeed ?? 7.5);
  const accel = isSCurve ? (p.sCurveAccel ?? 35.0) : (p.trapAccel ?? 35.0);
  const decel = isSCurve ? (p.sCurveDecel ?? 35.0) : (p.trapDecel ?? 35.0);

  const cellSizeMm = p.cellSizeMm ?? 180.0;
  const startDistMm = p.startDistMm ?? 111.0;
  const stopDistMm = p.stopDistMm ?? 80.0;
  const startMaxSpeed = p.startMaxSpeed ?? 0.5;
  const startAccel = p.startAccel ?? 12.0;
  const startDecel = p.startDecel ?? 12.0;
  const stopMaxSpeed = p.stopMaxSpeed ?? 0.5;
  const stopAccel = p.stopAccel ?? 20.0;
  const stopDecel = p.stopDecel ?? 20.0;
  const forwardCells = Math.max(0, Math.min(16, Math.round(p.forwardCells ?? 5)));
  const accJerk = p.sCurveJerkAcc ?? 625.0;

  // Calculate distance to accelerate from 0 to startMaxSpeed and ramp accel back to 0:
  let dAccelToZeroMm = 0.0;
  if (isSCurve) {
    dAccelToZeroMm = 1000.0 * getSCurveBrakeDistance(startMaxSpeed, 0.0, startAccel, accJerk);
  } else {
    dAccelToZeroMm = 1000.0 * ((startMaxSpeed * startMaxSpeed) / (2.0 * startAccel));
  }

  // If next move is FORWARD and START has no significant cruise phase at startMaxSpeed:
  const cruiseMarginMm = 15.0;
  const startCruises = (startDistMm - dAccelToZeroMm) > cruiseMarginMm;
  const seamlessToForward = (forwardCells > 0) && !startCruises;

  const seq = [];

  // 1. START movement
  seq.push({
    name: 'START',
    count: 1,
    targetTravelMm: startDistMm,
    maxSpeed: seamlessToForward ? maxSpeed : startMaxSpeed,
    accel: startAccel,
    decel: startDecel,
    endSpeed: seamlessToForward ? maxSpeed : startMaxSpeed,
    seamlessToForward
  });

  // 2. FORWARD movement (if forwardCells > 0)
  if (forwardCells > 0) {
    seq.push({
      name: 'FORWARD',
      count: forwardCells,
      targetTravelMm: cellSizeMm * forwardCells,
      maxSpeed: maxSpeed,
      accel: accel,
      decel: decel,
      endSpeed: stopMaxSpeed
    });
  }

  // 3. STOP movement
  seq.push({
    name: 'STOP',
    count: 1,
    targetTravelMm: stopDistMm,
    maxSpeed: stopMaxSpeed,
    accel: stopAccel,
    decel: stopDecel,
    endSpeed: 0.0
  });

  return seq;
}

function simulateLinearSCurve(p) {
  const frequency = p.simFrequency ?? 2000.0;
  const dt = 1.0 / frequency;
  const dtMs = dt * 1000.0;
  const accJerk = p.sCurveJerkAcc ?? 625.0;
  const brakeJerk = p.sCurveJerkBrake ?? 625.0;
  const brakeMarginMm = p.brakeMarginMm ?? 20.0;
  const accelMarginMm = p.accelMarginMm ?? 20.0;
  const minMoveSpeed = p.minMoveSpeed ?? 0.2;
  const useDerating = p.useDerating ?? true;

  const sequence = buildLinearMovementSequence(p, 'sCurve');

  const times = [];
  const distances = [];
  const speeds = [];
  const accels = [];
  const jerks = [];
  const transitions = [];

  let controlLinearSpeed = 0.0;
  let totalTraveledDistMm = 0.0;
  let totalTimeS = 0.0;
  let currentLinearAccel = 0.0;
  let cruiseTimeS = 0.0;
  let cruiseDistMm = 0.0;
  let prevAccel = 0.0;

  for (let i = 0; i < sequence.length; i++) {
    const move = sequence[i];
    let traveledDistMm = 0.0;
    let isBraking = false;
    let isFinished = false;

    const label = move.count > 1 ? `${move.name} x${move.count}` : move.name;
    transitions.push({
      timeMs: totalTimeS * 1000.0,
      distMm: totalTraveledDistMm,
      label,
      speed: controlLinearSpeed
    });

    const maxTicks = Math.round(30 * frequency);
    let ticks = 0;

    while (!isFinished && ticks < maxTicks) {
      ticks++;

      // Log current state
      const tMs = totalTimeS * 1000.0;
      times.push(tMs);
      distances.push(totalTraveledDistMm);
      speeds.push(controlLinearSpeed);
      accels.push(currentLinearAccel);

      const currentJerk = (currentLinearAccel - prevAccel) * frequency;
      jerks.push(currentJerk);
      prevAccel = currentLinearAccel;

      // Calculate brake distance including ramp-down of positive acceleration to 0
      let dRampM = 0.0;
      let vPeak = controlLinearSpeed;
      if (currentLinearAccel > 0.0 && brakeJerk > 0.0) {
        const tRamp = currentLinearAccel / brakeJerk;
        const deltaV = (currentLinearAccel * currentLinearAccel) / (2.0 * brakeJerk);
        vPeak = controlLinearSpeed + deltaV;
        dRampM = (controlLinearSpeed * tRamp) + Math.pow(currentLinearAccel, 3) / (3.0 * brakeJerk * brakeJerk);
      }

      const reqBrakeDist = move.seamlessToForward ? 0.0 : ((1000.0 * (dRampM + getSCurveBrakeDistance(
        vPeak, move.endSpeed, move.decel, brakeJerk
      ))) + brakeMarginMm);

      const isPostStraightStart = (i > 0 && sequence[i - 1].seamlessToForward);
      const canAccelerate = isPostStraightStart || controlLinearSpeed < 1.0 || Math.abs(traveledDistMm) > accelMarginMm;

      if (!isBraking && (Math.abs(traveledDistMm) < (move.targetTravelMm - reqBrakeDist))) {
        if (canAccelerate) {
          if (controlLinearSpeed >= move.maxSpeed && currentLinearAccel <= 0.0) {
            currentLinearAccel = 0.0;
            controlLinearSpeed = move.maxSpeed;
            cruiseTimeS += dt;
            cruiseDistMm += (controlLinearSpeed * 1000.0) / frequency;
          } else if (controlLinearSpeed >= move.maxSpeed || startAccelRampDown(controlLinearSpeed, currentLinearAccel, move.maxSpeed, accJerk)) {
            currentLinearAccel -= (accJerk / frequency);
            currentLinearAccel = Math.max(currentLinearAccel, 0.0);
            controlLinearSpeed += currentLinearAccel / frequency;
            if (currentLinearAccel === 0.0) {
              controlLinearSpeed = move.maxSpeed;
            }
          } else {
            const effMaxAccel = useDerating ? getEffectiveMaxAcceleration(controlLinearSpeed, move.accel) : move.accel;
            if (currentLinearAccel < effMaxAccel) {
              currentLinearAccel += (accJerk / frequency);
              currentLinearAccel = Math.min(currentLinearAccel, effMaxAccel);
            } else if (currentLinearAccel > effMaxAccel) {
              currentLinearAccel -= (accJerk / frequency);
              currentLinearAccel = Math.max(currentLinearAccel, effMaxAccel);
            }
            controlLinearSpeed += currentLinearAccel / frequency;
            controlLinearSpeed = Math.min(controlLinearSpeed, move.maxSpeed);
          }
        }
      } else if (Math.abs(traveledDistMm) > accelMarginMm) {
        isBraking = true;
        if (controlLinearSpeed > move.endSpeed || currentLinearAccel < 0.0) {
          if (startBrakeRampUp(controlLinearSpeed, currentLinearAccel, move.endSpeed, brakeJerk) || controlLinearSpeed <= move.endSpeed) {
            currentLinearAccel += (brakeJerk / frequency);
            currentLinearAccel = Math.min(currentLinearAccel, 0.0);
          } else {
            currentLinearAccel -= (brakeJerk / frequency);
            currentLinearAccel = Math.max(currentLinearAccel, -move.decel);
          }

          controlLinearSpeed += currentLinearAccel / frequency;
          if (currentLinearAccel === 0.0 && controlLinearSpeed < move.endSpeed) {
            controlLinearSpeed = move.endSpeed;
          }
          if (move.endSpeed > 0.0) {
            controlLinearSpeed = Math.max(controlLinearSpeed, minMoveSpeed);
          }
        } else if (currentLinearAccel > 0.0) {
          currentLinearAccel -= (brakeJerk / frequency);
          currentLinearAccel = Math.max(currentLinearAccel, 0.0);
          controlLinearSpeed += currentLinearAccel / frequency;
        } else {
          currentLinearAccel = 0.0;
          controlLinearSpeed = move.endSpeed;
          if (move.endSpeed > 0.0) {
            controlLinearSpeed = Math.max(controlLinearSpeed, minMoveSpeed);
          }
        }
      }

      if ((Math.abs(traveledDistMm) >= move.targetTravelMm) ||
          (move.name === 'STOP' && isBraking && controlLinearSpeed <= 0.0)) {
        isFinished = true;
      }

      const deltaXMm = (controlLinearSpeed * 1000.0) / frequency;
      traveledDistMm += deltaXMm;
      totalTraveledDistMm += deltaXMm;
      totalTimeS += dt;
    }
  }

  const peakSpeed = speeds.length > 0 ? Math.max(...speeds) : 0;
  const peakAccel = accels.length > 0 ? Math.max(...accels) : 0;
  const peakDecel = accels.length > 0 ? Math.abs(Math.min(...accels)) : 0;
  const peakJerk = jerks.length > 0 ? Math.max(...jerks.map(Math.abs)) : 0;

  return {
    times,
    distances,
    speeds,
    accels,
    jerks,
    transitions,
    results: {
      totalTimeMs: Math.round(totalTimeS * 10000) / 10,
      totalDistMm: Math.round(totalTraveledDistMm * 100) / 100,
      peakSpeed: Math.round(peakSpeed * 1000) / 1000,
      peakAccel: Math.round(peakAccel * 100) / 100,
      peakDecel: Math.round(peakDecel * 100) / 100,
      peakJerk: Math.round(peakJerk * 10) / 10,
      cruiseTimeMs: Math.round(cruiseTimeS * 10000) / 10,
      cruiseDistMm: Math.round(cruiseDistMm * 10) / 10
    }
  };
}

function simulateLinearTrapezoidal(p) {
  const frequency = p.simFrequency ?? 2000.0;
  const dt = 1.0 / frequency;
  const brakeMarginMm = p.brakeMarginMm ?? 20.0;
  const accelMarginMm = p.accelMarginMm ?? 20.0;
  const minMoveSpeed = p.minMoveSpeed ?? 0.2;
  const useDerating = p.useDerating ?? true;

  const sequence = buildLinearMovementSequence(p, 'trap');

  const times = [];
  const distances = [];
  const speeds = [];
  const accels = [];
  const jerks = [];
  const transitions = [];

  let controlLinearSpeed = 0.0;
  let totalTraveledDistMm = 0.0;
  let totalTimeS = 0.0;
  let currentLinearAccel = 0.0;
  let cruiseTimeS = 0.0;
  let cruiseDistMm = 0.0;
  let prevAccel = 0.0;

  for (let i = 0; i < sequence.length; i++) {
    const move = sequence[i];
    let traveledDistMm = 0.0;
    let isBraking = false;
    let isFinished = false;

    const label = move.count > 1 ? `${move.name} x${move.count}` : move.name;
    transitions.push({
      timeMs: totalTimeS * 1000.0,
      distMm: totalTraveledDistMm,
      label,
      speed: controlLinearSpeed
    });

    const maxTicks = Math.round(30 * frequency);
    let ticks = 0;

    while (!isFinished && ticks < maxTicks) {
      ticks++;

      const tMs = totalTimeS * 1000.0;
      times.push(tMs);
      distances.push(totalTraveledDistMm);
      speeds.push(controlLinearSpeed);
      accels.push(currentLinearAccel);

      const currentJerk = (currentLinearAccel - prevAccel) * frequency;
      jerks.push(currentJerk);
      prevAccel = currentLinearAccel;

      // Torricelli brake distance
      let reqBrakeDist = brakeMarginMm;
      if (controlLinearSpeed > move.endSpeed && move.decel > 0.0) {
        reqBrakeDist += 1000.0 * getTorricelliDistance(move.endSpeed, controlLinearSpeed, -move.decel);
      }
      if (move.seamlessToForward) {
        reqBrakeDist = 0.0;
      }

      const isPostStraightStart = (i > 0 && sequence[i - 1].seamlessToForward);
      const canAccelerate = isPostStraightStart || controlLinearSpeed < 1.0 || Math.abs(traveledDistMm) > accelMarginMm;

      if (!isBraking && (Math.abs(traveledDistMm) < (move.targetTravelMm - reqBrakeDist))) {
        if (canAccelerate) {
          if (controlLinearSpeed >= move.maxSpeed) {
            currentLinearAccel = 0.0;
            controlLinearSpeed = move.maxSpeed;
            cruiseTimeS += dt;
            cruiseDistMm += (controlLinearSpeed * 1000.0) / frequency;
          } else {
            const effMaxAccel = useDerating ? getEffectiveMaxAcceleration(controlLinearSpeed, move.accel) : move.accel;
            currentLinearAccel = effMaxAccel;
            controlLinearSpeed += effMaxAccel / frequency;
            if (controlLinearSpeed >= move.maxSpeed) {
              controlLinearSpeed = move.maxSpeed;
              currentLinearAccel = 0.0;
            }
          }
        }
      } else if (Math.abs(traveledDistMm) > accelMarginMm) {
        isBraking = true;
        if (controlLinearSpeed > move.endSpeed) {
          currentLinearAccel = -move.decel;
          controlLinearSpeed -= move.decel / frequency;
          if (controlLinearSpeed <= move.endSpeed) {
            controlLinearSpeed = move.endSpeed;
            currentLinearAccel = 0.0;
          }
          if (move.endSpeed > 0.0) {
            controlLinearSpeed = Math.max(controlLinearSpeed, minMoveSpeed);
          }
        } else {
          currentLinearAccel = 0.0;
          controlLinearSpeed = Math.min(controlLinearSpeed, move.endSpeed);
          if (move.endSpeed > 0.0) {
            controlLinearSpeed = Math.max(controlLinearSpeed, minMoveSpeed);
          }
        }
      }

      if ((Math.abs(traveledDistMm) >= move.targetTravelMm) ||
          (move.name === 'STOP' && isBraking && controlLinearSpeed <= 0.0)) {
        isFinished = true;
      }

      const deltaXMm = (controlLinearSpeed * 1000.0) / frequency;
      traveledDistMm += deltaXMm;
      totalTraveledDistMm += deltaXMm;
      totalTimeS += dt;
    }
  }

  const peakSpeed = speeds.length > 0 ? Math.max(...speeds) : 0;
  const peakAccel = accels.length > 0 ? Math.max(...accels) : 0;
  const peakDecel = accels.length > 0 ? Math.abs(Math.min(...accels)) : 0;
  const peakJerk = jerks.length > 0 ? Math.max(...jerks.map(Math.abs)) : 0;

  return {
    times,
    distances,
    speeds,
    accels,
    jerks,
    transitions,
    results: {
      totalTimeMs: Math.round(totalTimeS * 10000) / 10,
      totalDistMm: Math.round(totalTraveledDistMm * 100) / 100,
      peakSpeed: Math.round(peakSpeed * 1000) / 1000,
      peakAccel: Math.round(peakAccel * 100) / 100,
      peakDecel: Math.round(peakDecel * 100) / 100,
      peakJerk: Math.round(peakJerk * 10) / 10,
      cruiseTimeMs: Math.round(cruiseTimeS * 10000) / 10,
      cruiseDistMm: Math.round(cruiseDistMm * 10) / 10
    }
  };
}

