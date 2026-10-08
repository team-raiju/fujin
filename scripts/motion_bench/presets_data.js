// Auto-generated from firmware/src/utils/movement_params.cpp
const FIRMWARE_PRESETS = {
  "FAST": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 2.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 5.0,
        "acceleration": 25.0,
        "deceleration": 25.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 4.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 127.27922
      },
      "STOP": {
        "max_speed": 2.0,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 95.0
      },
      "TURN_AROUND": {
        "max_speed": 0.75,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.75,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90": {
        "max_speed": 2.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_90": {
        "max_speed": 2.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_180": {
        "max_speed": 2.18,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_180": {
        "max_speed": 2.18,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 67.5
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 73.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 26.2
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 26.2
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 61.0
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 63.0
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -61.89,
        "end": -56.43,
        "turn_linear_speed": 1.7,
        "angular_accel": 1095.0,
        "max_angular_speed": 19.83,
        "t_start_deccel": 37.0,
        "t_stop": 73.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_45": {
        "start": -61.89,
        "end": -61.43,
        "turn_linear_speed": 1.7,
        "angular_accel": 1095.0,
        "max_angular_speed": 19.83,
        "t_start_deccel": 36.5,
        "t_stop": 72.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 59.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90": {
        "start": -27.29,
        "end": 26.65,
        "turn_linear_speed": 1.9,
        "angular_accel": 1000.0,
        "max_angular_speed": 25.0,
        "t_start_deccel": 58.0,
        "t_stop": 102.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 19.0,
        "time_to_decrease_jerk_2": 89.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_90": {
        "start": -21.00,
        "end": 26.65,
        "turn_linear_speed": 1.9,
        "angular_accel": 1000.0,
        "max_angular_speed": 25.0,
        "t_start_deccel": 57.0,
        "t_stop": 101.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 19.0,
        "time_to_decrease_jerk_2": 88.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_RIGHT_135": {
        "start": -12.52,
        "end": -59.0,
        "turn_linear_speed": 1.7,
        "angular_accel": 900.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 85.5,
        "t_stop": 130.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 26.5,
        "time_to_decrease_jerk_2": 118.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135": {
        "start": -8.5,
        "end": -61.28,
        "turn_linear_speed": 1.7,
        "angular_accel": 900.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 85.0,
        "t_stop": 129.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 26.5,
        "time_to_decrease_jerk_2": 118.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_180": {
        "start": -35.0,
        "end": 36.92,
        "turn_linear_speed": 1.85,
        "angular_accel": 960.0,
        "max_angular_speed": 20.74,
        "t_start_deccel": 148.0,
        "t_stop": 185.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 18.0,
        "time_to_decrease_jerk_2": 173.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_180": {
        "start": -35.0,
        "end": 36.92,
        "turn_linear_speed": 1.85,
        "angular_accel": 960.0,
        "max_angular_speed": 20.74,
        "t_start_deccel": 146.5,
        "t_stop": 183.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 18.0,
        "time_to_decrease_jerk_2": 171.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "start": 0.0,
        "end": 65.89,
        "turn_linear_speed": 1.7,
        "angular_accel": 1095.0,
        "max_angular_speed": 19.83,
        "t_start_deccel": 37.0,
        "t_stop": 73.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_45_FROM_45": {
        "start": 0.0,
        "end": 70.89,
        "turn_linear_speed": 1.7,
        "angular_accel": 1095.0,
        "max_angular_speed": 19.83,
        "t_start_deccel": 36.0,
        "t_stop": 72.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 58.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -30.5,
        "turn_linear_speed": 1.7,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 57.0,
        "t_stop": 102.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 85.5,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -27.0,
        "turn_linear_speed": 1.7,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 56.5,
        "t_stop": 101.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 85.0,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": 13.31,
        "turn_linear_speed": 1.7,
        "angular_accel": 900.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 85.5,
        "t_stop": 130.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 26.5,
        "time_to_decrease_jerk_2": 118.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 13.31,
        "turn_linear_speed": 1.7,
        "angular_accel": 900.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 85.0,
        "t_stop": 129.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 26.5,
        "time_to_decrease_jerk_2": 118.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 285.5,
        "t_stop": 380.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 340.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 284.5,
        "t_stop": 379.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 339.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      }
    }
  },
  "SEARCH_MEDIUM": {
    "general": {
      "max_linear_acc_jerk": 40.0,
      "max_linear_brake_jerk": 40.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 180.0
      },
      "STOP": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 90.0
      },
      "TURN_AROUND": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 29.22
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 27.22
      }
    },
    "turn": {
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 285.5,
        "t_stop": 380.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 340.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 284.5,
        "t_stop": 379.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 339.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": -26.27,
        "turn_linear_speed": 0.5,
        "angular_accel": 250.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 143.0,
        "t_stop": 212.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 44.0,
        "time_to_decrease_jerk_2": 187.0,
        "accel_ramp_up_jerk": 10000.0,
        "accel_ramp_down_jerk": 10000.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": -26.27,
        "turn_linear_speed": 0.5,
        "angular_accel": 250.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 143.0,
        "t_stop": 212.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 44.0,
        "time_to_decrease_jerk_2": 187.0,
        "accel_ramp_up_jerk": 10000.0,
        "accel_ramp_down_jerk": 10000.0
      }
    }
  },
  "SEARCH_SLOW": {
    "general": {
      "max_linear_acc_jerk": 40.0,
      "max_linear_brake_jerk": 40.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 0.3,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.3,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 180.0
      },
      "STOP": {
        "max_speed": 0.3,
        "acceleration": 2.0,
        "deceleration": 2.0,
        "target_travel_mm": 90.0
      },
      "TURN_AROUND": {
        "max_speed": 0.3,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.3,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "max_speed": 0.3,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 32.81
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 32.81
      }
    },
    "turn": {
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 288.0,
        "t_stop": 411.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 73.0,
        "time_to_decrease_jerk_2": 361.0,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 288.0,
        "t_stop": 411.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 73.0,
        "time_to_decrease_jerk_2": 361.0,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 80.0,
        "max_angular_speed": 8.0,
        "t_start_deccel": 196.5,
        "t_stop": 323.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 100.0,
        "time_to_decrease_jerk_2": 296.5,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 80.0,
        "max_angular_speed": 8.0,
        "t_start_deccel": 196.5,
        "t_stop": 323.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 100.0,
        "time_to_decrease_jerk_2": 296.5,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      }
    }
  },
  "MEDIUM": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 4.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 3.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 127.27922
      },
      "STOP": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 95.0
      },
      "TURN_AROUND": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 95.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 35.0,
        "target_travel_mm": 95.0
      },
      "TURN_RIGHT_90": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_90": {
        "max_speed": 1.25,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_180": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_180": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 72.65
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 72.65
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 50.59
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 55.59
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 67.2
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 1.25,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 67.2
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -43.45,
        "end": -76.23,
        "turn_linear_speed": 1.25,
        "angular_accel": 1000.0,
        "max_angular_speed": 19.75,
        "t_start_deccel": 36.5,
        "t_stop": 72.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_45": {
        "start": -43.45,
        "end": -79.23,
        "turn_linear_speed": 1.25,
        "angular_accel": 1000.0,
        "max_angular_speed": 19.75,
        "t_start_deccel": 36.0,
        "t_stop": 72.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90": {
        "start": -3.03,
        "end": 5.13,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 82.5,
        "t_stop": 126.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 106.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_90": {
        "start": -3.03,
        "end": 5.13,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 82.5,
        "t_stop": 126.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 106.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_RIGHT_135": {
        "start": -7.29,
        "end": -65.0,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 124.0,
        "t_stop": 168.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 148.0,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_135": {
        "start": -7.29,
        "end": -62.0,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 123.0,
        "t_stop": 167.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 147.0,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_RIGHT_180": {
        "start": -40.0,
        "end": 41.68,
        "turn_linear_speed": 1.25,
        "angular_accel": 600.0,
        "max_angular_speed": 13.8,
        "t_start_deccel": 227.5,
        "t_stop": 265.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 250.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_180": {
        "start": -40.0,
        "end": 41.68,
        "turn_linear_speed": 1.25,
        "angular_accel": 600.0,
        "max_angular_speed": 13.8,
        "t_start_deccel": 227.5,
        "t_stop": 265.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 250.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "start": 0.0,
        "end": 61.5,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 18.4,
        "t_start_deccel": 43.0,
        "t_stop": 86.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 66.0,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_45_FROM_45": {
        "start": 0.0,
        "end": 56.5,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 18.4,
        "t_start_deccel": 43.0,
        "t_stop": 86.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 66.0,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -42.0,
        "turn_linear_speed": 1.25,
        "angular_accel": 900.0,
        "max_angular_speed": 25.1,
        "t_start_deccel": 63.0,
        "t_stop": 109.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 28.0,
        "time_to_decrease_jerk_2": 91.0,
        "accel_ramp_up_jerk": 50000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -43.0,
        "turn_linear_speed": 1.25,
        "angular_accel": 900.0,
        "max_angular_speed": 25.1,
        "t_start_deccel": 63.5,
        "t_stop": 109.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 28.0,
        "time_to_decrease_jerk_2": 91.5,
        "accel_ramp_up_jerk": 50000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": 12.72,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 122.5,
        "t_stop": 166.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 146.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 14.0,
        "turn_linear_speed": 1.25,
        "angular_accel": 800.0,
        "max_angular_speed": 19.0,
        "t_start_deccel": 122.5,
        "t_stop": 166.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 24.0,
        "time_to_decrease_jerk_2": 146.5,
        "accel_ramp_up_jerk": 40000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 1.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 24.75,
        "t_start_deccel": 124.0,
        "t_stop": 165.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 21.0,
        "time_to_decrease_jerk_2": 152.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 1.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 24.75,
        "t_start_deccel": 124.0,
        "t_stop": 165.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 21.0,
        "time_to_decrease_jerk_2": 152.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      }
    }
  },
  "SLOW": {
    "general": {
      "max_linear_acc_jerk": 40.0,
      "max_linear_brake_jerk": 40.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 3.0,
        "acceleration": 8.0,
        "deceleration": 8.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 2.0,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 127.27922
      },
      "STOP": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 16.53
      },
      "TURN_LEFT_90": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 16.53
      },
      "TURN_RIGHT_135": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 4.0
      },
      "TURN_LEFT_135": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 90.0
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 82.2
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 52.0
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 53.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 81.0
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 70.68
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -45.02,
        "end": -81.33,
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 8.45,
        "t_start_deccel": 91.0,
        "t_stop": 174.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 47.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_45": {
        "start": -45.02,
        "end": -81.33,
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 8.45,
        "t_start_deccel": 91.0,
        "t_stop": 174.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 47.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_90": {
        "start": 0.0,
        "end": -15.67,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 10.47,
        "t_start_deccel": 147.0,
        "t_stop": 247.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 70.0,
        "time_to_decrease_jerk_2": 217.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_90": {
        "start": 0.0,
        "end": -15.67,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 10.47,
        "t_start_deccel": 147.0,
        "t_stop": 247.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 70.0,
        "time_to_decrease_jerk_2": 217.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_135": {
        "start": 0.0,
        "end": -74.5,
        "turn_linear_speed": 0.5,
        "angular_accel": 120.0,
        "max_angular_speed": 7.5,
        "t_start_deccel": 311.0,
        "t_stop": 397.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 62.5,
        "time_to_decrease_jerk_2": 373.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_135": {
        "start": -0.1,
        "end": -69.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 120.0,
        "max_angular_speed": 7.5,
        "t_start_deccel": 312.0,
        "t_stop": 398.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 62.5,
        "time_to_decrease_jerk_2": 374.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_180": {
        "start": -10.0,
        "end": 15.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 100.0,
        "max_angular_speed": 5.58,
        "t_start_deccel": 556.0,
        "t_stop": 632.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 56.0,
        "time_to_decrease_jerk_2": 612.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_180": {
        "start": -10.0,
        "end": 15.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 100.0,
        "max_angular_speed": 5.58,
        "t_start_deccel": 560.0,
        "t_stop": 636.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 56.0,
        "time_to_decrease_jerk_2": 616.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "start": 0.0,
        "end": 45.92,
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 8.45,
        "t_start_deccel": 91.0,
        "t_stop": 174.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 47.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_45_FROM_45": {
        "start": 0.0,
        "end": 45.92,
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 8.45,
        "t_start_deccel": 91.0,
        "t_stop": 174.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 47.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -59.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 10.47,
        "t_start_deccel": 147.0,
        "t_stop": 247.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 70.0,
        "time_to_decrease_jerk_2": 217.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -57.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 10.47,
        "t_start_deccel": 147.0,
        "t_stop": 247.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 70.0,
        "time_to_decrease_jerk_2": 217.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": -6.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 120.0,
        "max_angular_speed": 7.5,
        "t_start_deccel": 310.5,
        "t_stop": 397.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 62.5,
        "time_to_decrease_jerk_2": 373.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 12.5,
        "turn_linear_speed": 0.5,
        "angular_accel": 120.0,
        "max_angular_speed": 7.5,
        "t_start_deccel": 310.5,
        "t_stop": 397.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 62.5,
        "time_to_decrease_jerk_2": 373.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 288.0,
        "t_stop": 411.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 73.0,
        "time_to_decrease_jerk_2": 361.0,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 150.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 288.0,
        "t_stop": 411.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 73.0,
        "time_to_decrease_jerk_2": 361.0,
        "accel_ramp_up_jerk": 3000.0,
        "accel_ramp_down_jerk": 3000.0
      }
    }
  },
  "SUPER": {
    "general": {
      "max_linear_acc_jerk": 1650.0,
      "max_linear_brake_jerk": 1650.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 2.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 6.5,
        "acceleration": 35.0,
        "deceleration": 35.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 4.5,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 127.27922
      },
      "STOP": {
        "max_speed": 2.2,
        "acceleration": 20.0,
        "deceleration": 40.0,
        "target_travel_mm": 95.0
      },
      "TURN_AROUND": {
        "max_speed": 0.75,
        "acceleration": 20.0,
        "deceleration": 40.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.75,
        "acceleration": 20.0,
        "deceleration": 40.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "max_speed": 2.2,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 52.0
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 2.2,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 62.29
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 2.2,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 16.33
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 2.2,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 16.33
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 2.3,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 39.29
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 2.3,
        "acceleration": 30.0,
        "deceleration": 30.0,
        "target_travel_mm": 39.29
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -75.47,
        "end": -47.11,
        "turn_linear_speed": 2.1,
        "angular_accel": 1100.0,
        "max_angular_speed": 18.0,
        "t_start_deccel": 39.0,
        "t_stop": 71.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 10.0,
        "time_to_decrease_jerk_2": 62.0,
        "accel_ramp_up_jerk": 120000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_45": {
        "start": -70.47,
        "end": -47.11,
        "turn_linear_speed": 2.1,
        "angular_accel": 1100.0,
        "max_angular_speed": 18.0,
        "t_start_deccel": 37.5,
        "t_stop": 69.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 10.0,
        "time_to_decrease_jerk_2": 60.5,
        "accel_ramp_up_jerk": 120000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90": {
        "start": -37.44,
        "end": 29.91,
        "turn_linear_speed": 2.4,
        "angular_accel": 1300.0,
        "max_angular_speed": 31.85,
        "t_start_deccel": 44.5,
        "t_stop": 88.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 18.0,
        "time_to_decrease_jerk_2": 75.5,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_90": {
        "start": -29.44,
        "end": 32.91,
        "turn_linear_speed": 2.4,
        "angular_accel": 1300.0,
        "max_angular_speed": 31.85,
        "t_start_deccel": 44.0,
        "t_stop": 88.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 18.0,
        "time_to_decrease_jerk_2": 75.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_135": {
        "start": -59.77,
        "end": -16.75,
        "turn_linear_speed": 2.5,
        "angular_accel": 1200.0,
        "max_angular_speed": 36.0,
        "t_start_deccel": 62.0,
        "t_stop": 111.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 25.5,
        "time_to_decrease_jerk_2": 96.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135": {
        "start": -49.77,
        "end": -20.75,
        "turn_linear_speed": 2.5,
        "angular_accel": 1200.0,
        "max_angular_speed": 36.0,
        "t_start_deccel": 60.5,
        "t_stop": 110.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 25.5,
        "time_to_decrease_jerk_2": 95.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_180": {
        "start": -35.0,
        "end": 39.48,
        "turn_linear_speed": 2.3,
        "angular_accel": 1200.0,
        "max_angular_speed": 26.1,
        "t_start_deccel": 116.0,
        "t_stop": 156.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 144.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_180": {
        "start": -35.0,
        "end": 39.48,
        "turn_linear_speed": 2.3,
        "angular_accel": 1200.0,
        "max_angular_speed": 26.1,
        "t_start_deccel": 114.0,
        "t_stop": 154.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 142.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "start": 0.0,
        "end": 76.06,
        "turn_linear_speed": 2.2,
        "angular_accel": 1250.0,
        "max_angular_speed": 22.0,
        "t_start_deccel": 33.0,
        "t_stop": 65.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 54.5,
        "accel_ramp_up_jerk": 120000.0,
        "accel_ramp_down_jerk": 70000.0
      },
      "TURN_LEFT_45_FROM_45": {
        "start": 0.0,
        "end": 76.06,
        "turn_linear_speed": 2.2,
        "angular_accel": 1250.0,
        "max_angular_speed": 22.0,
        "t_start_deccel": 31.5,
        "t_stop": 63.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 53.0,
        "accel_ramp_up_jerk": 120000.0,
        "accel_ramp_down_jerk": 70000.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -12.38,
        "turn_linear_speed": 2.2,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 46.5,
        "t_stop": 87.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 26.0,
        "time_to_decrease_jerk_2": 75.5,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 80000.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -12.38,
        "turn_linear_speed": 2.2,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 46.0,
        "t_stop": 87.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 26.0,
        "time_to_decrease_jerk_2": 75.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 80000.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": 38.92,
        "turn_linear_speed": 2.3,
        "angular_accel": 1200.0,
        "max_angular_speed": 36.0,
        "t_start_deccel": 62.0,
        "t_stop": 111.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 25.5,
        "time_to_decrease_jerk_2": 96.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 38.92,
        "turn_linear_speed": 2.3,
        "angular_accel": 1200.0,
        "max_angular_speed": 36.0,
        "t_start_deccel": 61.0,
        "t_stop": 110.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 25.5,
        "time_to_decrease_jerk_2": 95.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 285.5,
        "t_stop": 380.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 340.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 284.5,
        "t_stop": 379.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 339.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      }
    }
  },
  "CUSTOM": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 3.5,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 3.0,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 127.27922
      },
      "STOP": {
        "max_speed": 1.0,
        "acceleration": 2.0,
        "deceleration": 30.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 79.0
      },
      "TURN_RIGHT_90": {
        "max_speed": 1.3,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 5.0
      },
      "TURN_LEFT_90": {
        "max_speed": 1.3,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 5.0
      },
      "TURN_RIGHT_180": {
        "max_speed": 1.3,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": -7.0
      },
      "TURN_LEFT_180": {
        "max_speed": 1.3,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": -7.0
      },
      "TURN_RIGHT_45": {
        "max_speed": 0.0,
        "acceleration": 0.0,
        "deceleration": 0.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_45": {
        "max_speed": 0.0,
        "acceleration": 0.0,
        "deceleration": 0.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_135": {
        "max_speed": 0.0,
        "acceleration": 0.0,
        "deceleration": 0.0,
        "target_travel_mm": 0.0
      },
      "TURN_LEFT_135": {
        "max_speed": 0.0,
        "acceleration": 0.0,
        "deceleration": 0.0,
        "target_travel_mm": 0.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 67.5
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 63.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 33.0
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 32.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 27.5
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 28.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "max_speed": 0.3,
        "acceleration": 0.85,
        "deceleration": 0.85,
        "target_travel_mm": 11.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.3,
        "acceleration": 0.85,
        "deceleration": 0.85,
        "target_travel_mm": 11.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 6.0,
        "target_travel_mm": 80.0
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -64.0,
        "end": -82.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 20.07,
        "t_start_deccel": 38.0,
        "t_stop": 73.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_45": {
        "start": -64.0,
        "end": -82.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 20.07,
        "t_start_deccel": 38.0,
        "t_stop": 73.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90": {
        "start": 0.0,
        "end": -11.0,
        "turn_linear_speed": 1.3,
        "angular_accel": 785.4,
        "max_angular_speed": 26.18,
        "t_start_deccel": 60.0,
        "t_stop": 108.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90": {
        "start": 0.0,
        "end": -12.5,
        "turn_linear_speed": 1.3,
        "angular_accel": 785.4,
        "max_angular_speed": 26.18,
        "t_start_deccel": 60.0,
        "t_stop": 108.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_135": {
        "start": -46.0,
        "end": -46.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 436.33,
        "max_angular_speed": 20.07,
        "t_start_deccel": 116.0,
        "t_stop": 167.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_135": {
        "start": -46.0,
        "end": -50.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 436.33,
        "max_angular_speed": 20.07,
        "t_start_deccel": 116.0,
        "t_stop": 167.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_180": {
        "start": -10.0,
        "end": -11.0,
        "turn_linear_speed": 1.3,
        "angular_accel": 523.6,
        "max_angular_speed": 14.25,
        "t_start_deccel": 217.0,
        "t_stop": 242.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_180": {
        "start": -10.0,
        "end": -17.0,
        "turn_linear_speed": 1.3,
        "angular_accel": 523.6,
        "max_angular_speed": 14.25,
        "t_start_deccel": 217.0,
        "t_stop": 242.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_45_FROM_45": {
        "start": 0.0,
        "end": 57.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 20.07,
        "t_start_deccel": 38.0,
        "t_stop": 73.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_45_FROM_45": {
        "start": 0.0,
        "end": 54.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 20.07,
        "t_start_deccel": 38.0,
        "t_stop": 73.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -26.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 26.18,
        "t_start_deccel": 60.0,
        "t_stop": 108.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -33.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 785.4,
        "max_angular_speed": 26.18,
        "t_start_deccel": 60.0,
        "t_stop": 108.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": 38.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 436.33,
        "max_angular_speed": 20.07,
        "t_start_deccel": 116.0,
        "t_stop": 167.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 39.0,
        "turn_linear_speed": 1.5,
        "angular_accel": 436.33,
        "max_angular_speed": 20.07,
        "t_start_deccel": 116.0,
        "t_stop": 167.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 52.36,
        "max_angular_speed": 3.49,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 52.36,
        "max_angular_speed": 3.49,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 43.633,
        "max_angular_speed": 4.014,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.3,
        "angular_accel": 43.633,
        "max_angular_speed": 4.014,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      }
    }
  },
  "SEARCH_FAST": {
    "general": {
      "max_linear_acc_jerk": 40.0,
      "max_linear_brake_jerk": 40.0,
      "accel_margin_mm": 10.0,
      "brake_margin_mm": 10.0
    },
    "forward": {
      "START": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 180.0
      },
      "STOP": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 90.0
      },
      "TURN_AROUND": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 13.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.75,
        "acceleration": 5.0,
        "deceleration": 5.0,
        "target_travel_mm": 14.84
      }
    },
    "turn": {
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 285.5,
        "t_stop": 380.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 340.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.75,
        "angular_accel": 200.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 284.5,
        "t_stop": 379.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 55.0,
        "time_to_decrease_jerk_2": 339.5,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": -13.84,
        "turn_linear_speed": 0.75,
        "angular_accel": 400.0,
        "max_angular_speed": 16.0,
        "t_start_deccel": 98.0,
        "t_stop": 171.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 40.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 12000.0,
        "accel_ramp_down_jerk": 12000.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": -11.84,
        "turn_linear_speed": 0.75,
        "angular_accel": 400.0,
        "max_angular_speed": 16.0,
        "t_start_deccel": 98.0,
        "t_stop": 171.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 40.0,
        "time_to_decrease_jerk_2": 138.0,
        "accel_ramp_up_jerk": 12000.0,
        "accel_ramp_down_jerk": 12000.0
      }
    }
  }
};
