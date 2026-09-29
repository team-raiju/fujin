/**
 * SEQUENCE SIMULATOR FOR MOTION BENCH
 * Replicates firmware navigation FSM, jerk S-curves, and turn kinematics.
 */

// Clone FIRMWARE_PRESETS.CUSTOM or initialize from MEDIUM
let customPreset = JSON.parse(JSON.stringify(FIRMWARE_PRESETS.CUSTOM || FIRMWARE_PRESETS.MEDIUM));

function getActivePreset(presetName) {
  if (presetName === 'CUSTOM') {
    return customPreset;
  }
  return FIRMWARE_PRESETS[presetName] || FIRMWARE_PRESETS.MEDIUM;
}

function copyPresetToCustom(sourcePresetName) {
  const src = FIRMWARE_PRESETS[sourcePresetName] || FIRMWARE_PRESETS.MEDIUM;
  customPreset = JSON.parse(JSON.stringify(src));
  return customPreset;
}

const ALL_MOVEMENTS = [
  'START',
  'FORWARD',
  'DIAGONAL',
  'STOP',
  'TURN_RIGHT_90',
  'TURN_LEFT_90',
  'TURN_RIGHT_45',
  'TURN_LEFT_45',
  'TURN_RIGHT_135',
  'TURN_LEFT_135',
  'TURN_RIGHT_180',
  'TURN_LEFT_180',
  'TURN_RIGHT_45_FROM_45',
  'TURN_LEFT_45_FROM_45',
  'TURN_RIGHT_90_FROM_45',
  'TURN_LEFT_90_FROM_45',
  'TURN_RIGHT_135_FROM_45',
  'TURN_LEFT_135_FROM_45',
  'TURN_AROUND',
  'TURN_AROUND_INPLACE',
  'TURN_RIGHT_90_SEARCH_MODE',
  'TURN_LEFT_90_SEARCH_MODE'
];

function isLinearMovement(name) {
  return name === 'START' || name === 'FORWARD' || name === 'DIAGONAL' || name === 'STOP';
}

/**
 * Simulates a full sequence of movements.
 * @param {Array<{name: string, count: number}>} steps 
 * @param {string} presetName 
 * @param {{ brakeMarginMm?: number, accelMarginMm?: number }} [options]
 * @returns {{ data: Array<{x: number, yLinVel: number, yAngVel: number, yLinAcc: number, yAngAcc: number}>, summary: {totalTimeMs: number, maxLinSpeed: number, maxAngSpeed: number, stepCount: number} }}
 */
function simulateSequence(steps, presetName, options = {}) {
  if (!steps || steps.length === 0) {
    return { data: [], summary: { totalTimeMs: 0, maxLinSpeed: 0, maxAngSpeed: 0, stepCount: 0 } };
  }

  const preset = getActivePreset(presetName);
  const genParams = preset.general || {
    max_linear_acc_jerk: 625.0,
    max_linear_brake_jerk: 625.0,
    accel_margin_mm: 20.0,
    brake_margin_mm: 20.0
  };

  const brakeMarginMm = (options && options.brakeMarginMm !== undefined) ? options.brakeMarginMm : (genParams.brake_margin_mm ?? 20.0);
  const accelMarginMm = (options && options.accelMarginMm !== undefined) ? options.accelMarginMm : (genParams.accel_margin_mm ?? 20.0);

  const Hz = 2000.0;
  const dt = 1.0 / Hz;
  const rawData = [];

  let controlLinearSpeed = 0.0;
  let currentLinearAccel = 0.0;
  let controlAngularSpeed = 0.0;
  let currentAngularAccel = 0.0;
  let totalTimeS = 0.0;
  let completePrevMoveTravel = 0.0;

  let maxLinSpeedObserved = 0.0;
  let maxAngSpeedObserved = 0.0;

  for (let i = 0; i < steps.length; i++) {
    const move = steps[i];
    const movement = move.name;
    const count = Math.max(1, move.count || 1);
    const prevMovement = (i > 0) ? steps[i - 1].name : 'NONE';
    const nextMovement = (i + 1 < steps.length) ? steps[i + 1].name : 'STOP';
    const nextMoveCount = (i + 1 < steps.length) ? Math.max(1, steps[i + 1].count || 1) : 1;

    // Movement configuration matching Navigation::set_movement
    const prevTurn = preset.turn[prevMovement] || { end: 0 };
    completePrevMoveTravel = -1.0 * (prevTurn.end || 0.0);

    const isLinear = isLinearMovement(movement);
    const fwdDef = preset.forward[movement] || (isLinear
      ? { max_speed: 1.0, acceleration: 10.0, deceleration: 10.0, target_travel_mm: 180.0 }
      : { max_speed: 0.0, acceleration: 0.0, deceleration: 0.0, target_travel_mm: 0.0 });
    const nextTurn = preset.turn[nextMovement] || { start: 0.0 };

    let targetTravelMm = 0.0;
    if (movement === 'FORWARD' || movement === 'DIAGONAL') {
      targetTravelMm = completePrevMoveTravel + (fwdDef.target_travel_mm * count) + (nextTurn.start || 0.0);
    } else if (movement === 'START') {
      targetTravelMm = fwdDef.target_travel_mm + (nextTurn.start || 0.0);
    } else {
      targetTravelMm = completePrevMoveTravel + fwdDef.target_travel_mm;
    }

    const preserveAccelFromStart =
      (prevMovement === 'START' && (movement === 'FORWARD' || movement === 'DIAGONAL') && count >= 3);
    const continuousStartToForward =
      (movement === 'START' && (nextMovement === 'FORWARD' || nextMovement === 'DIAGONAL') && nextMoveCount >= 3);
    const resetLinearAccel = !preserveAccelFromStart;

    let forwardEndSpeed = 0.0;
    if (movement === 'STOP') {
      forwardEndSpeed = 0.0;
    } else if (movement === 'START') {
      if (continuousStartToForward) {
        forwardEndSpeed = preset.forward[nextMovement] ? preset.forward[nextMovement].max_speed : 0.0;
      } else if (preset.turn && preset.turn[nextMovement]) {
        forwardEndSpeed = preset.turn[nextMovement].turn_linear_speed;
      } else if (nextMovement === 'STOP') {
        forwardEndSpeed = 0.0;
      } else {
        forwardEndSpeed = preset.forward['START'] ? preset.forward['START'].max_speed : 0.0;
      }
    } else if (nextMovement === 'FORWARD' || nextMovement === 'DIAGONAL') {
      forwardEndSpeed = preset.forward[nextMovement] ? preset.forward[nextMovement].max_speed : 0.0;
    } else if (nextMovement === 'STOP') {
      forwardEndSpeed = preset.forward['STOP'] ? preset.forward['STOP'].max_speed : 0.0;
    } else {
      const turnDef = preset.turn[nextMovement];
      forwardEndSpeed = turnDef ? turnDef.turn_linear_speed : 0.0;
    }

    let traveledDistMm = 0.0;
    let isBraking = false;
    let isFinished = false;
    let miniFsmState = 'FORWARD_1';
    let turnTickCounter = 0;

    if (resetLinearAccel) {
      currentLinearAccel = 0.0;
    }

    if (isLinearMovement(movement)) {
      // LINEAR EXECUTION
      const maxSpeed = (movement === 'START' && continuousStartToForward)
        ? forwardEndSpeed
        : ((movement === 'START' && forwardEndSpeed > 0.0) ? Math.min(fwdDef.max_speed, forwardEndSpeed) : fwdDef.max_speed);
      const maxAcceleration = fwdDef.acceleration;
      const deceleration = fwdDef.deceleration;
      const isStartMove = (movement === 'START');
      const isForwardAfterStart = (movement === 'FORWARD' || movement === 'DIAGONAL') && (prevMovement === 'START');
      const moveBrakeMarginMm = isStartMove ? 0.0 : brakeMarginMm;
      const moveAccelMarginMm = (isStartMove || isForwardAfterStart) ? 0.0 : accelMarginMm;

      const stateObj = {
        controlLinearSpeed,
        currentLinearAccel,
        traveledDistMm,
        isBraking,
        prevMovement
      };

      const maxTicks = Math.round(60 * Hz);
      let ticks = 0;

      while (!isFinished && ticks < maxTicks) {
        ticks++;

        // Record point with 1 decimal digit on time
        const tMs = Math.round(totalTimeS * 10000.0) / 10.0;
        rawData.push({
          x: tMs,
          yLinVel: stateObj.controlLinearSpeed,
          yAngVel: 0.0,
          yLinAcc: stateObj.currentLinearAccel,
          yAngAcc: 0.0
        });

        maxLinSpeedObserved = Math.max(maxLinSpeedObserved, Math.abs(stateObj.controlLinearSpeed));

      const stepGenParams = (movement === 'STOP'
        ? { ...genParams, max_linear_acc_jerk: (genParams.max_linear_acc_jerk || 625.0) * 2.0, max_linear_brake_jerk: (genParams.max_linear_brake_jerk || 625.0) * 2.0 }
        : genParams);

      // Update linear speed via updated S-curve function
      updateLinearTargetSpeedStep(stateObj, maxSpeed, maxAcceleration, deceleration, continuousStartToForward, forwardEndSpeed, targetTravelMm, stepGenParams, moveBrakeMarginMm, moveAccelMarginMm);

        // Distance & time integration
        stateObj.traveledDistMm += (stateObj.controlLinearSpeed * 1000.0) / Hz;
        totalTimeS += dt;

        // Finish condition matching finish_linear_movement
        const reachedTarget = Math.abs(stateObj.traveledDistMm) >= targetTravelMm;
        const finishedStop = (movement === 'STOP' && stateObj.isBraking && stateObj.controlLinearSpeed <= 0.0);
        if (reachedTarget || finishedStop) {
          isFinished = true;
        }
      }

      controlLinearSpeed = stateObj.controlLinearSpeed;
      currentLinearAccel = stateObj.currentLinearAccel;
      controlAngularSpeed = 0.0;
      currentAngularAccel = 0.0;

    } else {
      // TURN EXECUTION
      const turnParams = preset.turn[movement];
      if (!turnParams) {
        continue;
      }

      if (targetTravelMm <= 0.0) {
        miniFsmState = 'TURN';
        currentAngularAccel = 0.0;
        turnTickCounter = 0;
      } else {
        miniFsmState = 'FORWARD_1';
      }

      let turnFinished = false;
      const maxTicks = Math.round(30 * Hz);
      let ticks = 0;

      while (!turnFinished && ticks < maxTicks) {
        ticks++;
        const tMs = Math.round(totalTimeS * 10000.0) / 10.0;

        if (miniFsmState === 'FORWARD_1') {
          rawData.push({
            x: tMs,
            yLinVel: controlLinearSpeed,
            yAngVel: 0.0,
            yLinAcc: 0.0,
            yAngAcc: 0.0
          });

          // update_turn_linear_speed
          const maxSpeed = fwdDef.max_speed;
          const acceleration = fwdDef.acceleration;
          const deceleration = fwdDef.deceleration;
          const isTurnAround = (movement === 'TURN_AROUND' || movement === 'TURN_AROUND_INPLACE');
          const finalSpeed = isTurnAround ? 0.0 : fwdDef.max_speed;

          const brakingDistMm = 1000.0 * getTorricelliDistance(finalSpeed, controlLinearSpeed, -deceleration);
          const beforeBrakingPoint = !isBraking && (Math.abs(traveledDistMm) < (targetTravelMm - brakingDistMm));

          if (beforeBrakingPoint) {
            if (controlLinearSpeed < maxSpeed) {
              controlLinearSpeed += acceleration / Hz;
              controlLinearSpeed = Math.min(controlLinearSpeed, maxSpeed);
            }
          } else if (controlLinearSpeed > finalSpeed) {
            isBraking = true;
            controlLinearSpeed -= deceleration / Hz;
            controlLinearSpeed = Math.max(controlLinearSpeed, finalSpeed);
            if (finalSpeed > 0.0) {
              controlLinearSpeed = Math.max(controlLinearSpeed, 0.2);
            }
          }

          traveledDistMm += (controlLinearSpeed * 1000.0) / Hz;
          totalTimeS += dt;

          const reachedTarget = Math.abs(traveledDistMm) >= targetTravelMm;
          const shouldTransition = isTurnAround ? (isBraking && controlLinearSpeed <= 0.0) : reachedTarget;
          if (shouldTransition) {
            isBraking = false;
            miniFsmState = 'TURN';
            turnTickCounter = 0;
            currentAngularAccel = 0.0;
          }

        } else if (miniFsmState === 'TURN') {
          // step_turn_rotation
          const tStartDecelTicks = Math.round(turnParams.t_start_deccel * 2.0);
          const tStopTicks = Math.round(turnParams.t_stop * 2.0);
          const tJerk1Ticks = Math.round((turnParams.time_to_decrease_jerk_1 || 0.0) * 2.0);
          const tJerk2Ticks = Math.round((turnParams.time_to_decrease_jerk_2 || 0.0) * 2.0);

          const maxAngAccel = turnParams.angular_accel;
          const maxAngDecel = -turnParams.angular_accel;

          // update_turn_angular_acceleration
          if (turnTickCounter < tStartDecelTicks) {
            if (turnParams.accel_ramp_up_jerk === 0 || tJerk1Ticks === 0) {
              currentAngularAccel = maxAngAccel;
            } else if (turnTickCounter < tJerk1Ticks) {
              currentAngularAccel += turnParams.accel_ramp_up_jerk / Hz;
              currentAngularAccel = Math.min(currentAngularAccel, maxAngAccel);
            } else {
              currentAngularAccel -= turnParams.accel_ramp_down_jerk / Hz;
              currentAngularAccel = Math.max(currentAngularAccel, 0.0);
            }
          } else {
            if (turnParams.accel_ramp_down_jerk === 0 || tJerk2Ticks === 0) {
              currentAngularAccel = maxAngDecel;
            } else if (turnTickCounter < tJerk2Ticks) {
              currentAngularAccel -= turnParams.accel_ramp_down_jerk / Hz;
              currentAngularAccel = Math.max(currentAngularAccel, maxAngDecel);
            } else {
              currentAngularAccel += turnParams.accel_ramp_up_jerk / Hz;
              currentAngularAccel = Math.min(currentAngularAccel, 0.0);
            }
          }

          let controlAngSpeedAbs = Math.abs(controlAngularSpeed) + (currentAngularAccel / Hz);
          controlAngSpeedAbs = Math.min(controlAngSpeedAbs, turnParams.max_angular_speed);
          controlAngSpeedAbs = Math.max(controlAngSpeedAbs, 0.0);
          controlAngularSpeed = controlAngSpeedAbs * turnParams.sign;

          turnTickCounter++;

          rawData.push({
            x: tMs,
            yLinVel: controlLinearSpeed,
            yAngVel: controlAngularSpeed,
            yLinAcc: 0.0,
            yAngAcc: currentAngularAccel
          });

          maxAngSpeedObserved = Math.max(maxAngSpeedObserved, Math.abs(controlAngularSpeed));

          totalTimeS += dt;

          if (turnTickCounter >= tStopTicks) {
            controlAngularSpeed = 0.0;
            currentAngularAccel = 0.0;
            turnFinished = true;
          }
        }
      }
    }
  }

  // Downsample to max 3000 points to keep Chart.js perfectly smooth while preserving peaks
  const downsampled = downsampleSequenceData(rawData, 3000);

  return {
    data: downsampled,
    summary: {
      totalTimeMs: Math.round(totalTimeS * 10000.0) / 10.0,
      maxLinSpeed: Math.round(maxLinSpeedObserved * 100.0) / 100.0,
      maxAngSpeed: Math.round(maxAngSpeedObserved * 100.0) / 100.0,
      stepCount: steps.length
    }
  };
}

/**
 * Step function for update_linear_target_speed implementing latest safeguards.
 */
function updateLinearTargetSpeedStep(state, maxSpeed, maxAcceleration, deceleration, continuousStartToForward, forwardEndSpeed, targetTravelMm, genParams, brakeMarginMm, accelMarginMm) {
  const Hz = 2000.0;
  const accelJerk = genParams.max_linear_acc_jerk || 625.0;
  const brakeJerk = genParams.max_linear_brake_jerk || 625.0;
  const effAccelMarginMm = (accelMarginMm !== undefined) ? accelMarginMm : (genParams.accel_margin_mm ?? 20.0);
  const effBrakeMarginMm = (brakeMarginMm !== undefined) ? brakeMarginMm : (genParams.brake_margin_mm ?? 20.0);
  const minMoveSpeed = 0.2;

  let dRampM = 0.0;
  let vPeak = state.controlLinearSpeed;
  if (state.currentLinearAccel > 0.0 && brakeJerk > 0.0) {
    const tRamp = state.currentLinearAccel / brakeJerk;
    const deltaV = (state.currentLinearAccel * state.currentLinearAccel) / (2.0 * brakeJerk);
    vPeak = state.controlLinearSpeed + deltaV;
    dRampM = (state.controlLinearSpeed * tRamp) + Math.pow(state.currentLinearAccel, 3) / (3.0 * brakeJerk * brakeJerk);
  }

  let reqBrakeDist = effBrakeMarginMm;
  if (!continuousStartToForward) {
    reqBrakeDist += 1000.0 * (dRampM + getSCurveBrakeDistance(vPeak, forwardEndSpeed, deceleration, brakeJerk));
  } else {
    reqBrakeDist = 0.0;
  }

  const requiresTurnMargin = (state.prevMovement !== 'START') && (state.controlLinearSpeed >= 1.0);
  const shouldAccelerate = !state.isBraking && (continuousStartToForward || (Math.abs(state.traveledDistMm) < (targetTravelMm - reqBrakeDist)));

  if (shouldAccelerate) {
    if (requiresTurnMargin && Math.abs(state.traveledDistMm) <= effAccelMarginMm) {
      return;
    }

    if (state.controlLinearSpeed >= maxSpeed) {
      if (state.currentLinearAccel > 0.0) {
        state.currentLinearAccel -= accelJerk / Hz;
        state.currentLinearAccel = Math.max(state.currentLinearAccel, 0.0);
      } else {
        state.currentLinearAccel = 0.0;
      }
      state.controlLinearSpeed = maxSpeed;
      return;
    }

    if (startAccelRampDown(state.controlLinearSpeed, state.currentLinearAccel, maxSpeed, accelJerk)) {
      state.currentLinearAccel -= accelJerk / Hz;
      state.currentLinearAccel = Math.max(state.currentLinearAccel, 0.0);
      state.controlLinearSpeed += state.currentLinearAccel / Hz;
      state.controlLinearSpeed = Math.min(state.controlLinearSpeed, maxSpeed);
      return;
    }

    const effMaxAccel = getEffectiveMaxAcceleration(state.controlLinearSpeed, maxAcceleration);
    if (state.currentLinearAccel < effMaxAccel) {
      state.currentLinearAccel += accelJerk / Hz;
      state.currentLinearAccel = Math.min(state.currentLinearAccel, effMaxAccel);
    } else if (state.currentLinearAccel > effMaxAccel) {
      state.currentLinearAccel -= accelJerk / Hz;
      state.currentLinearAccel = Math.max(state.currentLinearAccel, effMaxAccel);
    }

    state.controlLinearSpeed += state.currentLinearAccel / Hz;
    state.controlLinearSpeed = Math.min(state.controlLinearSpeed, maxSpeed);
    return;
  }

  if (continuousStartToForward || Math.abs(state.traveledDistMm) <= effAccelMarginMm) {
    return;
  }

  state.isBraking = true;

  const rampingDownPos = (state.controlLinearSpeed <= forwardEndSpeed && state.currentLinearAccel > 0.0);
  if (state.controlLinearSpeed > forwardEndSpeed || state.currentLinearAccel !== 0.0) {
    if (rampingDownPos) {
      state.currentLinearAccel -= brakeJerk / Hz;
      state.currentLinearAccel = Math.max(state.currentLinearAccel, 0.0);
    } else if (startBrakeRampUp(state.controlLinearSpeed, state.currentLinearAccel, forwardEndSpeed, brakeJerk)) {
      state.currentLinearAccel += brakeJerk / Hz;
      state.currentLinearAccel = Math.min(state.currentLinearAccel, 0.0);
    } else {
      state.currentLinearAccel -= brakeJerk / Hz;
      state.currentLinearAccel = Math.max(state.currentLinearAccel, -deceleration);
    }

    state.controlLinearSpeed += state.currentLinearAccel / Hz;
    if (!rampingDownPos) {
      state.controlLinearSpeed = Math.max(state.controlLinearSpeed, forwardEndSpeed);
    }
    if (forwardEndSpeed > 0.0) {
      state.controlLinearSpeed = Math.max(state.controlLinearSpeed, minMoveSpeed);
    }
  } else {
    state.currentLinearAccel = 0.0;
    state.controlLinearSpeed = Math.min(state.controlLinearSpeed, forwardEndSpeed);
    if (forwardEndSpeed > 0.0) {
      state.controlLinearSpeed = Math.max(state.controlLinearSpeed, minMoveSpeed);
    }
  }
}

function downsampleSequenceData(arr, maxPoints) {
  if (arr.length <= maxPoints) return arr;
  const step = Math.ceil(arr.length / maxPoints);
  const out = [];
  for (let i = 0; i < arr.length; i += step) {
    out.push(arr[i]);
  }
  // Ensure last point is included
  if (out[out.length - 1] !== arr[arr.length - 1]) {
    out.push(arr[arr.length - 1]);
  }
  return out;
}
