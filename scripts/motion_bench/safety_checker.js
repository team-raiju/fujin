/**
 * SAFETY CHECKER & VELOCITY VERIFIER FOR MOTION BENCH
 * Validates diagonal sequence correctness and checks physical velocity feasibility
 * across all transitions (curves, diagonals, straightaways, and stops).
 */

// Helper to determine if movement is linear
function isLinearMove(name) {
  return name === 'START' || name === 'FORWARD' || name === 'DIAGONAL' || name === 'STOP';
}

/**
 * Validates if a sequence of movements is achievable in DIAGONAL mode
 * based on firmware/src/services/navigation.cpp (Navigation::get_diagonal_movements).
 * 
 * Rules:
 * - Robot begins in ORTHOGONAL orientation (facing N/E/S/W).
 * - Orthogonal movements: START, FORWARD(n), TURN_RIGHT_90, TURN_LEFT_90, TURN_RIGHT_180, TURN_LEFT_180, STOP.
 * - Entering diagonal: TURN_RIGHT_45, TURN_LEFT_45, TURN_RIGHT_135, TURN_LEFT_135.
 * - On diagonal: DIAGONAL(n), TURN_RIGHT_90_FROM_45, TURN_LEFT_90_FROM_45.
 * - Exiting diagonal: TURN_RIGHT_45_FROM_45, TURN_LEFT_45_FROM_45, TURN_RIGHT_135_FROM_45, TURN_LEFT_135_FROM_45.
 * 
 * @param {Array<{name: string, count: number}>} steps 
 * @returns {{ valid: boolean, issues: Array<{step: number, move: string, error: string, suggestion: string}> }}
 */
function validateDiagonalSequence(steps) {
  const issues = [];
  if (!steps || steps.length === 0) {
    return { valid: false, issues: [{ step: 0, move: '', error: 'Empty sequence.', suggestion: 'Add at least START and STOP movements.' }] };
  }

  let orientation = 'ORTHO'; // 'ORTHO' or 'DIAG'

  for (let i = 0; i < steps.length; i++) {
    const move = steps[i].name;
    const stepNum = i + 1;

    if (move === 'START') {
      if (i !== 0) {
        issues.push({
          step: stepNum,
          move,
          error: 'START can only occur at the very beginning of the maze path.',
          suggestion: 'Move START to step #1.'
        });
      }
      orientation = 'ORTHO';
    } else if (move === 'FORWARD') {
      if (orientation === 'DIAG') {
        issues.push({
          step: stepNum,
          move,
          error: 'Orthogonal FORWARD is impossible while oriented at 45° on a diagonal!',
          suggestion: 'Insert an exit turn (e.g. TURN_LEFT_45_FROM_45 or TURN_RIGHT_45_FROM_45) before moving forward, or use DIAGONAL.'
        });
      }
    } else if (move === 'DIAGONAL') {
      if (orientation === 'ORTHO') {
        issues.push({
          step: stepNum,
          move,
          error: 'DIAGONAL movement is impossible while in orthogonal orientation!',
          suggestion: 'Insert a diagonal entry turn (e.g. TURN_RIGHT_45, TURN_LEFT_45, or TURN_135) before DIAGONAL.'
        });
      }
    } else if (move === 'TURN_RIGHT_45' || move === 'TURN_LEFT_45' || move === 'TURN_RIGHT_135' || move === 'TURN_LEFT_135') {
      if (orientation === 'DIAG') {
        issues.push({
          step: stepNum,
          move,
          error: `${move} enters diagonal from orthogonal, but the robot is already on a diagonal!`,
          suggestion: 'To turn while on diagonal, use TURN_LEFT_90_FROM_45 or TURN_RIGHT_90_FROM_45.'
        });
      }
      orientation = 'DIAG';
    } else if (move === 'TURN_RIGHT_45_FROM_45' || move === 'TURN_LEFT_45_FROM_45' || move === 'TURN_RIGHT_135_FROM_45' || move === 'TURN_LEFT_135_FROM_45') {
      if (orientation === 'ORTHO') {
        issues.push({
          step: stepNum,
          move,
          error: `${move} exits a diagonal, but the robot is currently in orthogonal orientation!`,
          suggestion: 'Use an orthogonal turn (e.g. TURN_RIGHT_90 or TURN_LEFT_90) instead.'
        });
      }
      orientation = 'ORTHO';
    } else if (move === 'TURN_RIGHT_90_FROM_45' || move === 'TURN_LEFT_90_FROM_45') {
      if (orientation === 'ORTHO') {
        issues.push({
          step: stepNum,
          move,
          error: `${move} is a diagonal-to-diagonal turn, but robot is in orthogonal orientation!`,
          suggestion: 'Use TURN_RIGHT_90 or TURN_LEFT_90 when in orthogonal mode.'
        });
      }
      orientation = 'DIAG';
    } else if (move === 'TURN_RIGHT_90' || move === 'TURN_LEFT_90' || move === 'TURN_RIGHT_180' || move === 'TURN_LEFT_180') {
      if (orientation === 'DIAG') {
        issues.push({
          step: stepNum,
          move,
          error: `${move} is an orthogonal turn, but robot is currently oriented at 45° on a diagonal!`,
          suggestion: 'Use a diagonal turn (e.g. TURN_LEFT_90_FROM_45) or exit diagonal first.'
        });
      }
      orientation = 'ORTHO';
    }
  }

  return {
    valid: issues.length === 0,
    issues
  };
}

/**
 * Runs a complete tick-by-tick simulation and verifies all velocity transitions.
 * 
 * @param {Object} testCase 
 * @param {string} presetName 
 * @param {Object} [options]
 * @returns {Object} Test evaluation report
 */
function evaluateSequenceSafety(testCase, presetName, options = {}) {
  const steps = testCase.steps || [];
  const speedTolerance = options.speedTolerance ?? 0.05; // m/s tolerance
  const preset = (options.customPreset && presetName === 'CUSTOM')
    ? options.customPreset
    : getActivePreset(presetName);

  // 1. Structural validity check in DIAGONAL mode
  const valResult = validateDiagonalSequence(steps);
  if (!valResult.valid) {
    return {
      id: testCase.id,
      name: testCase.name,
      category: testCase.category || 'user',
      description: testCase.description || '',
      steps,
      valid: false,
      overallPass: false,
      verdict: 'INVALID',
      validationErrors: valResult.issues,
      stepChecks: [],
      worstDiff: 0,
      totalTimeMs: 0,
      summaryMessage: 'Invalid movement sequence for DIAGONAL mode.',
      data: []
    };
  }

  const genParams = preset.general || {
    max_linear_acc_jerk: 625.0,
    max_linear_brake_jerk: 625.0,
    accel_margin_mm: 10.0,
    brake_margin_mm: 10.0
  };

  const brakeMarginMm = (options.brakeMarginMm !== undefined) ? options.brakeMarginMm : (genParams.linear_brake_margin_mm ?? genParams.brake_margin_mm ?? 10.0);
  const accelMarginMm = (options.accelMarginMm !== undefined) ? options.accelMarginMm : (genParams.linear_accel_margin_mm ?? genParams.accel_margin_mm ?? 10.0);

  const Hz = 2000.0;
  const dt = 1.0 / Hz;
  const rawData = [];

  let controlLinearSpeed = 0.0;
  let currentLinearAccel = 0.0;
  let controlAngularSpeed = 0.0;
  let currentAngularAccel = 0.0;
  let totalTimeS = 0.0;
  let completePrevMoveTravel = 0.0;

  const stepChecks = [];
  let worstDiff = 0.0;

  let waitingForFastParam = false;
  let activePresetMode = presetName;
  const customFallbackToMedium = (options.customFallbackToMedium !== undefined)
    ? options.customFallbackToMedium
    : true;

  for (let i = 0; i < steps.length; i++) {
    const move = steps[i];
    const movement = move.name;
    const count = Math.max(1, move.count || 1);
    const prevMovement = (i > 0) ? steps[i - 1].name : 'NONE';
    const nextMovement = (i + 1 < steps.length) ? steps[i + 1].name : 'STOP';
    const nextMoveCount = (i + 1 < steps.length) ? Math.max(1, steps[i + 1].count || 1) : 1;

    // Fallback matching Navigation::set_movement
    let fallbackTriggeredThisStep = false;
    const shouldFallback = (presetName === 'FAST' || presetName === 'SUPER') ||
      (presetName === 'CUSTOM' && customFallbackToMedium);
    if (movement === 'START') {
      const isNextTurn = !isLinearMove(nextMovement);
      if (isNextTurn && shouldFallback) {
        waitingForFastParam = true;
        activePresetMode = 'MEDIUM';
        fallbackTriggeredThisStep = true;
      }
    } else if (movement === 'FORWARD' || movement === 'DIAGONAL') {
      if (waitingForFastParam && count > 1) {
        waitingForFastParam = false;
        activePresetMode = presetName;
      }
    }

    const currentPreset = (activePresetMode === 'CUSTOM' && options.customPreset)
      ? options.customPreset
      : getActivePreset(activePresetMode);

    const prevTurn = currentPreset.turn[prevMovement] || { end: 0 };
    completePrevMoveTravel = -1.0 * (prevTurn.end || 0.0);

    const isLinear = isLinearMove(movement);
    const fwdDef = currentPreset.forward[movement] || (isLinear
      ? { max_speed: 1.0, acceleration: 10.0, deceleration: 10.0, target_travel_mm: 180.0 }
      : { max_speed: 0.0, acceleration: 0.0, deceleration: 0.0, target_travel_mm: 0.0 });
    const nextTurn = currentPreset.turn[nextMovement] || { start: 0.0 };

    let targetTravelMm = 0.0;
    if (movement === 'FORWARD' || movement === 'DIAGONAL') {
      targetTravelMm = completePrevMoveTravel + (fwdDef.target_travel_mm * count) + (nextTurn.start || 0.0);
    } else if (movement === 'START') {
      targetTravelMm = fwdDef.target_travel_mm + (nextTurn.start || 0.0);
    } else if (movement === 'STOP') {
      // STOP always has its full prescribed travel (95mm) to brake to zero
      targetTravelMm = fwdDef.target_travel_mm;
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
        forwardEndSpeed = currentPreset.forward[nextMovement] ? currentPreset.forward[nextMovement].max_speed : 0.0;
      } else if (currentPreset.turn && currentPreset.turn[nextMovement]) {
        forwardEndSpeed = currentPreset.turn[nextMovement].turn_linear_speed;
      } else if (nextMovement === 'STOP') {
        forwardEndSpeed = 0.0;
      } else {
        forwardEndSpeed = currentPreset.forward['START'] ? currentPreset.forward['START'].max_speed : 0.0;
      }
    } else if (nextMovement === 'FORWARD' || nextMovement === 'DIAGONAL') {
      forwardEndSpeed = currentPreset.forward[nextMovement] ? currentPreset.forward[nextMovement].max_speed : 0.0;
    } else if (nextMovement === 'STOP') {
      forwardEndSpeed = currentPreset.forward['STOP'] ? currentPreset.forward['STOP'].max_speed : 0.0;
    } else {
      const turnDef = currentPreset.turn[nextMovement];
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

    const initialSpeedForStep = controlLinearSpeed;

    if (isLinear) {
      const isStartMove = (movement === 'START');
      const isStopMove = (movement === 'STOP');
      const maxSpeed = (movement === 'START' && continuousStartToForward)
        ? forwardEndSpeed
        : (isStopMove ? Math.min(fwdDef.max_speed, initialSpeedForStep) : ((movement === 'START' && forwardEndSpeed > 0.0) ? Math.min(fwdDef.max_speed, forwardEndSpeed) : fwdDef.max_speed));
      const maxAcceleration = isStopMove ? 0.0 : fwdDef.acceleration;
      const deceleration = fwdDef.deceleration;
      const isForwardAfterStart = (movement === 'FORWARD' || movement === 'DIAGONAL') && (prevMovement === 'START');
      const moveBrakeMarginMm = (isStartMove || isStopMove) ? 0.0 : brakeMarginMm;
      const moveAccelMarginMm = (isStartMove || isForwardAfterStart || isStopMove) ? 0.0 : accelMarginMm;

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
        const tMs = Math.round(totalTimeS * 10000.0) / 10.0;
        rawData.push({
          x: tMs,
          yLinVel: stateObj.controlLinearSpeed,
          yAngVel: 0.0,
          yLinAcc: stateObj.currentLinearAccel,
          yAngAcc: 0.0,
          stepIndex: i,
          moveName: movement
        });

        const stepGenParams = (movement === 'STOP'
          ? { ...genParams, max_linear_acc_jerk: (genParams.max_linear_acc_jerk || 625.0) * 2.0, max_linear_brake_jerk: (genParams.max_linear_brake_jerk || 625.0) * 2.0 }
          : genParams);

        updateLinearTargetSpeedStep(stateObj, maxSpeed, maxAcceleration, deceleration, continuousStartToForward, forwardEndSpeed, targetTravelMm, stepGenParams, moveBrakeMarginMm, moveAccelMarginMm);
        stateObj.traveledDistMm += (stateObj.controlLinearSpeed * 1000.0) / Hz;
        totalTimeS += dt;

        const reachedTarget = Math.abs(stateObj.traveledDistMm) >= targetTravelMm;
        const finishedStop = (movement === 'STOP' && stateObj.isBraking && stateObj.controlLinearSpeed <= 0.0);
        if (reachedTarget || finishedStop) {
          isFinished = true;
        }
      }

      controlLinearSpeed = stateObj.controlLinearSpeed;
      currentLinearAccel = stateObj.currentLinearAccel;

      if (movement === 'STOP') {
        const exitSpeed = controlLinearSpeed;
        const speedDiff = exitSpeed - 0.0;
        const pass = exitSpeed <= speedTolerance;
        const minBrakeM = getSCurveBrakeDistance(initialSpeedForStep, 0.0, deceleration, (genParams.max_linear_brake_jerk || 625.0) * 2.0);
        const minBrakeMm = Math.round(minBrakeM * 1000.0 * 10.0) / 10.0;
        const marginMm = Math.round((targetTravelMm - minBrakeMm) * 10.0) / 10.0;

        if (Math.abs(speedDiff) > Math.abs(worstDiff)) worstDiff = speedDiff;

        let status = 'PASS';
        let message = 'Stopped safely at target point (0.00 m/s).';
        let recommendation = 'Parameters are safe for stop.';

        if (!pass) {
          status = 'COLLISION';
          message = `Collision risk: Robot reached target travel at ${exitSpeed.toFixed(2)} m/s (failed to decelerate to zero). Required ${minBrakeMm}mm, had ${targetTravelMm.toFixed(1)}mm.`;
          recommendation = `Increase STOP deceleration (currently ${deceleration} m/s²) or reduce entry speed before STOP.`;
        }

        stepChecks.push({
          step: i + 1,
          type: 'STOP',
          movement,
          count,
          targetTravelMm: Math.round(targetTravelMm * 10.0) / 10.0,
          initialSpeed: Math.round(initialSpeedForStep * 100.0) / 100.0,
          targetSpeed: 0.0,
          actualSpeed: Math.round(exitSpeed * 100.0) / 100.0,
          speedDiff: Math.round(speedDiff * 100.0) / 100.0,
          reqDistanceMm: minBrakeMm,
          marginMm,
          status,
          message,
          recommendation
        });
      } else {
        // Linear movement before next turn
        const nextIsTurn = !isLinearMove(nextMovement);
        if (nextIsTurn) {
          const expectedNextSpeed = forwardEndSpeed;
          const speedDiff = controlLinearSpeed - expectedNextSpeed;
          const pass = Math.abs(speedDiff) <= speedTolerance;
          if (Math.abs(speedDiff) > Math.abs(worstDiff)) worstDiff = speedDiff;

          let status = 'PASS';
          let message = `Exited linear segment at ${controlLinearSpeed.toFixed(2)} m/s matching next turn requirement (${expectedNextSpeed.toFixed(2)} m/s).`;
          let recommendation = 'Speed matches curve requirements.';

          if (!pass) {
            if (speedDiff > 0) {
              status = 'OVERSPEED';
              message = `Could not brake in time: Speed is ${controlLinearSpeed.toFixed(2)} m/s, exceeds ${nextMovement} speed (${expectedNextSpeed.toFixed(2)} m/s) by +${speedDiff.toFixed(2)} m/s.`;
              recommendation = `Increase ${movement} deceleration (${deceleration} m/s²) or adjust ${nextMovement} start offset (${nextTurn.start} mm) to give more braking distance.`;
            } else {
              status = 'UNDERSPEED';
              message = `Could not accelerate in time: Speed is ${controlLinearSpeed.toFixed(2)} m/s, below ${nextMovement} speed (${expectedNextSpeed.toFixed(2)} m/s) by ${speedDiff.toFixed(2)} m/s.`;
              recommendation = `Increase ${movement} acceleration (${maxAcceleration} m/s²) or adjust turn offsets.`;
            }
          }

          let reqDistMm = 0;
          if (speedDiff > 0) {
            reqDistMm = Math.round(1000.0 * getSCurveBrakeDistance(initialSpeedForStep, expectedNextSpeed, deceleration, genParams.max_linear_brake_jerk || 625.0) * 10.0) / 10.0;
          } else {
            reqDistMm = Math.round(1000.0 * getTorricelliDistance(expectedNextSpeed, initialSpeedForStep, maxAcceleration) * 10.0) / 10.0;
          }
          const marginMm = Math.round((targetTravelMm - reqDistMm) * 10.0) / 10.0;

          stepChecks.push({
            step: i + 1,
            type: 'LINEAR_TRANSITION',
            movement,
            count,
            nextMovement,
            targetTravelMm: Math.round(targetTravelMm * 10.0) / 10.0,
            initialSpeed: Math.round(initialSpeedForStep * 100.0) / 100.0,
            targetSpeed: expectedNextSpeed,
            actualSpeed: Math.round(controlLinearSpeed * 100.0) / 100.0,
            speedDiff: Math.round(speedDiff * 100.0) / 100.0,
            reqDistanceMm: reqDistMm,
            marginMm,
            status,
            message,
            recommendation
          });
        }
      }

    } else {
      // TURN EXECUTION
      const turnParams = currentPreset.turn[movement];
      if (!turnParams) continue;

      if (targetTravelMm <= 0.0) {
        miniFsmState = 'TURN';
        currentAngularAccel = 0.0;
        turnTickCounter = 0;
      } else {
        miniFsmState = 'FORWARD_1';
      }

      let speedAtCurveStart = controlLinearSpeed;
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
            yAngAcc: 0.0,
            stepIndex: i,
            moveName: movement
          });

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
            speedAtCurveStart = controlLinearSpeed;
          }

        } else if (miniFsmState === 'TURN') {
          speedAtCurveStart = controlLinearSpeed;
          const tStartDecelTicks = Math.round(turnParams.t_start_deccel * 2.0);
          const tStopTicks = Math.round(turnParams.t_stop * 2.0);
          const tJerk1Ticks = Math.round((turnParams.time_to_decrease_jerk_1 || 0.0) * 2.0);
          const tJerk2Ticks = Math.round((turnParams.time_to_decrease_jerk_2 || 0.0) * 2.0);

          const maxAngAccel = turnParams.angular_accel;
          const maxAngDecel = -turnParams.angular_accel;

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
            yAngAcc: currentAngularAccel,
            stepIndex: i,
            moveName: movement
          });

          totalTimeS += dt;

          if (turnTickCounter >= tStopTicks) {
            controlAngularSpeed = 0.0;
            currentAngularAccel = 0.0;
            turnFinished = true;
          }
        }
      }

      const expectedSpeed = turnParams.turn_linear_speed;
      const speedDiff = speedAtCurveStart - expectedSpeed;
      const pass = Math.abs(speedDiff) <= speedTolerance;
      if (Math.abs(speedDiff) > Math.abs(worstDiff)) worstDiff = speedDiff;

      let status = 'PASS';
      let message = `Entered turn at ${speedAtCurveStart.toFixed(2)} m/s (prescribed ${expectedSpeed.toFixed(2)} m/s).`;
      let recommendation = 'Turn entry speed is safe and accurate.';

      if (!pass) {
        if (speedDiff > 0) {
          status = 'OVERSPEED';
          message = `Overspeed at curve entry: Entered at ${speedAtCurveStart.toFixed(2)} m/s > ${expectedSpeed.toFixed(2)} m/s (+${speedDiff.toFixed(2)} m/s). Curve radius will distort outward.`;
          recommendation = `Increase pre-turn straight distance or braking deceleration in the preceding segment.`;
        } else {
          status = 'UNDERSPEED';
          message = `Underspeed at curve entry: Entered at ${speedAtCurveStart.toFixed(2)} m/s < ${expectedSpeed.toFixed(2)} m/s (${speedDiff.toFixed(2)} m/s). Curve radius will distort inward.`;
          recommendation = `Increase acceleration leading into this curve or lower turn_linear_speed.`;
        }
      }

      stepChecks.push({
        step: i + 1,
        type: 'TURN_ENTRY',
        movement,
        count,
        targetTravelMm: Math.round(targetTravelMm * 10.0) / 10.0,
        initialSpeed: Math.round(initialSpeedForStep * 100.0) / 100.0,
        targetSpeed: expectedSpeed,
        actualSpeed: Math.round(speedAtCurveStart * 100.0) / 100.0,
        speedDiff: Math.round(speedDiff * 100.0) / 100.0,
        reqDistanceMm: Math.round(1000.0 * getTorricelliDistance(expectedSpeed, initialSpeedForStep, -fwdDef.deceleration) * 10.0) / 10.0,
        marginMm: Math.round((targetTravelMm - 1000.0 * getTorricelliDistance(expectedSpeed, initialSpeedForStep, -fwdDef.deceleration)) * 10.0) / 10.0,
        status,
        message,
        recommendation
      });
    }
  }

  const allPass = stepChecks.every(c => c.status === 'PASS');
  const failureCount = stepChecks.filter(c => c.status !== 'PASS').length;

  let summaryMessage = allPass
    ? 'All velocity transitions are physically consistent and within tolerance.'
    : `${failureCount} transition(s) failed physical safety checks.`;

  return {
    id: testCase.id,
    name: testCase.name,
    category: testCase.category || 'user',
    description: testCase.description || '',
    steps,
    valid: true,
    overallPass: allPass,
    verdict: allPass ? 'PASS' : 'FAIL',
    validationErrors: [],
    stepChecks,
    worstDiff: Math.round(worstDiff * 100.0) / 100.0,
    totalTimeMs: Math.round(totalTimeS * 10000.0) / 10.0,
    summaryMessage,
    data: downsampleSequenceData(rawData, 1500)
  };
}

/**
 * Standard test cases covering user scenarios and worst-case physical transitions in DIAGONAL mode.
 */
const DEFAULT_SAFETY_TESTS = [
  // USER-SPECIFIED SEQUENCES
  {
    id: 'user_1',
    name: 'User 1: 90° Turn Zigzag',
    category: 'user',
    description: 'Alternating 90° turns separated by 1 forward cell (R90 -> FWD(1) -> L90). Tests speed entry into both 90° turns.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_LEFT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_2',
    name: 'User 2: 90° into 45° Diagonal Entry',
    category: 'user',
    description: '90° turn followed by 1 forward cell entering diagonal with 45° turn.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_LEFT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_4',
    name: 'User 4: 2-Cell Diagonal Run with Exit & Re-entry',
    category: 'user',
    description: 'Entering diagonal with 45°, running 2 diagonal cells, exiting, 2 forward cells, then entering diagonal again.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 2 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 2 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_5',
    name: 'User 5: 0-Length Diagonal (Direct 45° to 45° Exit)',
    category: 'user',
    description: 'Single-cell slant shift (R-L-F pattern): TURN_RIGHT_45 immediately followed by TURN_LEFT_45_FROM_45 with no intermediate diagonal cell.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_6',
    name: 'User 6: 90° Turn into 180° Hairpin',
    category: 'user',
    description: 'Single forward cell between a 90° turn and a 180° hairpin turn.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_LEFT_180', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_7',
    name: 'User 7: Right 45° Diag, Left 45° Diag into STOP',
    category: 'user',
    description: 'START -> FWD(1) -> R45 -> DIAG(1) -> L45_FROM_45 -> FWD(1) -> L45 -> DIAG(1) -> STOP(1). Tests diagonal entry, exit, re-entry in opposite direction, and stopping on diagonal.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_LEFT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_8',
    name: 'User 8: Right 45° Diag, Left 45° Diag into STOP',
    category: 'user',
    description: 'START -> -> R45 -> DIAG(1) -> L45_FROM_45 -> FWD(1) -> L45 -> DIAG(1) -> STOP(1). Tests diagonal entry, exit, re-entry in opposite direction, and stopping on diagonal.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_9',
    name: 'User 9: Right 45° Diag, Left 45° Diag into STOP',
    category: 'user',
    description: 'START -> FWD(10) -> R45 -> DIAG(5) -> L45_FROM_45 -> FWD(1) -> R90 -> FWD(1) -> STOP(1). Tests diagonal entry, exit, re-entry in opposite direction, and stopping on diagonal.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 10 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 5 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'user_10',
    name: 'User 10: Right 45° Diag, Left 45° Diag into STOP',
    category: 'user',
    description: 'START -> R90 -> FWD(1) -> R90 -> FWD(1) -> STOP(1). Tests diagonal entry, exit, re-entry in opposite direction, and stopping on diagonal.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },

  // WORST-CASE TIGHT PHYSICAL TRANSITIONS
  {
    id: 'worst_tight_brake',
    name: 'Worst Case: Shortest Travel with Heavy Braking (180° to 45°)',
    category: 'tight_transition',
    description: '180° turn ends far into cell (+55mm) and 45° starts early (-71mm), leaving minimal straight travel (~54mm) while dropping speed.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_180', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_LEFT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_tight_accel',
    name: 'Worst Case: Shortest Travel with Acceleration (45° Exit to 180°)',
    category: 'tight_transition',
    description: '45_FROM_45 ends late (+71mm) and 180° starts early (-50mm). Robot must accelerate from low exit speed to high 180° speed in ~59mm.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_180', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_cold_launch_45',
    name: 'Worst Case: Cold Launch Direct to 45° Turn',
    category: 'tight_transition',
    description: 'Accelerating from dead stop (0 m/s) with only ~45mm straight line to reach 45° curve speed.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_cold_launch_90',
    name: 'Worst Case: Cold Launch Direct to 90° Turn',
    category: 'tight_transition',
    description: 'Accelerating from dead stop (0 m/s) with only ~82mm straight line to reach 90° curve speed.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },

  // DIAGONAL MANEUVERS
  {
    id: 'worst_diag_90',
    name: 'Worst Case: Diagonal 90° D-D Turn with Min Lead-in',
    category: 'diagonal_maneuver',
    description: 'Diagonal-to-diagonal 90° turn with minimum linear lead-in: TURN_LEFT_90_FROM_45 lead-in travel is only 26.2 mm.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'TURN_LEFT_90_FROM_45', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_135_entry_exit',
    name: 'Worst Case: 135° Turn Entry and 135° Exit',
    category: 'diagonal_maneuver',
    description: 'Orthogonal into 135° turn entering diagonal, traversing 1 diagonal cell, exiting with 135° from 45.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_135', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_LEFT_135_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_dd_s_curve',
    name: 'Worst Case: Back-to-Back 90° D-D S-Curve',
    category: 'diagonal_maneuver',
    description: 'Consecutive diagonal 90° turns without intermediate diagonal cells.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'TURN_LEFT_90_FROM_45', count: 1 },
      { name: 'TURN_RIGHT_90_FROM_45', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },

  // HIGH-SPEED SPRINTS
  {
    id: 'worst_sprint_ortho',
    name: 'Worst Case: High-Speed Straight Sprint (8 cells) into 45°',
    category: 'sprint',
    description: 'Robot reaches top sprint speed (5.0 m/s) over 8 cells and must brake down to 45° turn speed without overshooting.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 8 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_RIGHT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_sprint_diag',
    name: 'Worst Case: High-Speed Diagonal Sprint (10 cells) into 45° Exit',
    category: 'sprint',
    description: 'Robot reaches top diagonal speed (4.0 m/s) over 10 diagonal cells and must brake down to exit turn speed before turning.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 10 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },

  // STOP TESTS (Direct turn to stop)
  {
    id: 'worst_stop_after_90',
    name: 'Worst Case: 90° Turn Directly into STOP',
    category: 'stop',
    description: 'Maze terminates immediately after a 90° turn. Exits curve at full speed and has only half a cell minus turn end offset to stop.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_90', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_stop_after_180',
    name: 'Worst Case: 180° Turn Directly into STOP',
    category: 'stop',
    description: 'Path terminates immediately after 180° hairpin. Exits at 2.18–2.5 m/s with ~40mm travel to full stop.',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_180', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  },
  {
    id: 'worst_stop_after_45_exit',
    name: 'Worst Case: 45° Exit Directly into STOP',
    category: 'stop',
    description: 'Path terminates immediately after diagonal exit (45_FROM_45 ends at +71mm, leaving only ~24mm to stop).',
    steps: [
      { name: 'START', count: 1 },
      { name: 'FORWARD', count: 1 },
      { name: 'TURN_RIGHT_45', count: 1 },
      { name: 'DIAGONAL', count: 1 },
      { name: 'TURN_LEFT_45_FROM_45', count: 1 },
      { name: 'STOP', count: 1 }
    ]
  }
];

let userCustomSafetyTests = [];

function getAllSafetyTestCases() {
  return [...DEFAULT_SAFETY_TESTS, ...userCustomSafetyTests];
}

function addCustomSafetyTest(testCase) {
  if (!testCase.id) testCase.id = 'custom_' + Date.now();
  userCustomSafetyTests.push(testCase);
  return testCase;
}

function removeCustomSafetyTest(testId) {
  userCustomSafetyTests = userCustomSafetyTests.filter(t => t.id !== testId);
}

function runAllSafetyChecks(presetName, options = {}) {
  const allTests = getAllSafetyTestCases();
  const reports = allTests.map(tc => evaluateSequenceSafety(tc, presetName, options));

  const total = reports.length;
  const passed = reports.filter(r => r.overallPass).length;
  const failed = reports.filter(r => !r.overallPass && r.valid).length;
  const invalid = reports.filter(r => !r.valid).length;

  let maxError = 0.0;
  let minMargin = Infinity;

  reports.forEach(r => {
    if (Math.abs(r.worstDiff) > Math.abs(maxError)) maxError = r.worstDiff;
    r.stepChecks.forEach(s => {
      if (s.marginMm !== undefined && !isNaN(s.marginMm) && s.marginMm < minMargin) {
        minMargin = s.marginMm;
      }
    });
  });

  return {
    reports,
    summary: {
      total,
      passed,
      failed,
      invalid,
      passRate: total > 0 ? Math.round((passed / total) * 100.0) : 0,
      maxVelocityError: maxError,
      minDistanceMargin: minMargin === Infinity ? 0 : minMargin
    }
  };
}

if (typeof window !== 'undefined') {
  window.validateDiagonalSequence = validateDiagonalSequence;
  window.evaluateSequenceSafety = evaluateSequenceSafety;
  window.DEFAULT_SAFETY_TESTS = DEFAULT_SAFETY_TESTS;
  window.getAllSafetyTestCases = getAllSafetyTestCases;
  window.addCustomSafetyTest = addCustomSafetyTest;
  window.removeCustomSafetyTest = removeCustomSafetyTest;
  window.runAllSafetyChecks = runAllSafetyChecks;
}
if (typeof global !== 'undefined') {
  global.validateDiagonalSequence = validateDiagonalSequence;
  global.evaluateSequenceSafety = evaluateSequenceSafety;
  global.DEFAULT_SAFETY_TESTS = DEFAULT_SAFETY_TESTS;
  global.getAllSafetyTestCases = getAllSafetyTestCases;
  global.addCustomSafetyTest = addCustomSafetyTest;
  global.removeCustomSafetyTest = removeCustomSafetyTest;
  global.runAllSafetyChecks = runAllSafetyChecks;
}

