// Auto-generated from firmware/src/utils/movement_params.cpp
const FIRMWARE_PRESETS = {
  "FAST": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
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
        "max_speed": 2.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 5.0
      },
      "TURN_LEFT_90": {
        "max_speed": 2.5,
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
        "turn_linear_speed": 2.5,
        "angular_accel": 1300.0,
        "max_angular_speed": 30.0,
        "t_start_deccel": 49.5,
        "t_stop": 89.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 18.5,
        "time_to_decrease_jerk_2": 76.5,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 60000.0
      },
      "TURN_LEFT_90": {
        "start": 0.0,
        "end": -12.5,
        "turn_linear_speed": 2.5,
        "angular_accel": 1300.0,
        "max_angular_speed": 30.0,
        "t_start_deccel": 49.5,
        "t_stop": 89.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 18.5,
        "time_to_decrease_jerk_2": 76.5,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 60000.0
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
        "turn_linear_speed": 1.5,
        "angular_accel": 52.36,
        "max_angular_speed": 3.49,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      }
    }
  },
  "SEARCH_MEDIUM": {
    "general": {
      "max_linear_acc_jerk": 100.0,
      "max_linear_brake_jerk": 100.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
    },
    "forward": {
      "START": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
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
        "deceleration": 3.0,
        "target_travel_mm": 24.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 27.0
      }
    },
    "turn": {
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 104.72,
        "max_angular_speed": 10.47,
        "t_start_deccel": 301.0,
        "t_stop": 401.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 104.72,
        "max_angular_speed": 10.47,
        "t_start_deccel": 301.0,
        "t_stop": 401.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 139.62,
        "max_angular_speed": 10.47,
        "t_start_deccel": 150.0,
        "t_stop": 225.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 139.62,
        "max_angular_speed": 10.47,
        "t_start_deccel": 150.0,
        "t_stop": 225.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      }
    }
  },
  "SEARCH_SLOW": {
    "general": {
      "max_linear_acc_jerk": 40.0,
      "max_linear_brake_jerk": 40.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
    },
    "forward": {
      "START": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 180.0
      },
      "STOP": {
        "max_speed": 0.5,
        "acceleration": 2.0,
        "deceleration": 2.0,
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
        "deceleration": 3.0,
        "target_travel_mm": 19.51
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.5,
        "acceleration": 3.0,
        "deceleration": 3.0,
        "target_travel_mm": 19.51
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
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 143.0,
        "t_stop": 240.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 61.0,
        "time_to_decrease_jerk_2": 204.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.5,
        "angular_accel": 180.0,
        "max_angular_speed": 11.0,
        "t_start_deccel": 143.0,
        "t_stop": 240.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 61.0,
        "time_to_decrease_jerk_2": 204.0,
        "accel_ramp_up_jerk": 5000.0,
        "accel_ramp_down_jerk": 5000.0
      }
    }
  },
  "MEDIUM": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
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
        "max_speed": 1.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND": {
        "max_speed": 1.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 1.0,
        "acceleration": 20.0,
        "deceleration": 25.0,
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
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 64.52
      },
      "TURN_LEFT_45_FROM_45": {
        "max_speed": 1.7,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 64.52
      },
      "TURN_RIGHT_90_FROM_45": {
        "max_speed": 2.0,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 8.36
      },
      "TURN_LEFT_90_FROM_45": {
        "max_speed": 2.0,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 8.36
      },
      "TURN_RIGHT_135_FROM_45": {
        "max_speed": 2.0,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 59.1
      },
      "TURN_LEFT_135_FROM_45": {
        "max_speed": 2.0,
        "acceleration": 15.0,
        "deceleration": 20.0,
        "target_travel_mm": 59.1
      }
    },
    "turn": {
      "TURN_RIGHT_45": {
        "start": -61.89,
        "end": -61.43,
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
        "t_start_deccel": 37.0,
        "t_stop": 73.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90": {
        "start": -29.34,
        "end": 32.06,
        "turn_linear_speed": 2.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 57.5,
        "t_stop": 102.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 86.0,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_90": {
        "start": -29.34,
        "end": 32.06,
        "turn_linear_speed": 2.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 57.5,
        "t_stop": 102.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 86.0,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_135": {
        "start": -15.56,
        "end": 55.8,
        "turn_linear_speed": 2.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 68.0,
        "t_stop": 115.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 100.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135": {
        "start": -15.56,
        "end": 55.8,
        "turn_linear_speed": 2.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 68.0,
        "t_stop": 115.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 100.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_180": {
        "start": -50.0,
        "end": 53.26,
        "turn_linear_speed": 2.18,
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
      "TURN_LEFT_180": {
        "start": -50.0,
        "end": 53.26,
        "turn_linear_speed": 2.18,
        "angular_accel": 1000.0,
        "max_angular_speed": 24.75,
        "t_start_deccel": 124.0,
        "t_stop": 165.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 21.0,
        "time_to_decrease_jerk_2": 152.5,
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
        "end": 65.89,
        "turn_linear_speed": 1.7,
        "angular_accel": 1095.0,
        "max_angular_speed": 19.83,
        "t_start_deccel": 37.0,
        "t_stop": 73.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 14.0,
        "time_to_decrease_jerk_2": 59.5,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_90_FROM_45": {
        "start": 0.0,
        "end": -5.0,
        "turn_linear_speed": 2.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 57.5,
        "t_stop": 102.5,
        "sign": -1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 86.0,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_90_FROM_45": {
        "start": 0.0,
        "end": -5.0,
        "turn_linear_speed": 2.0,
        "angular_accel": 1000.0,
        "max_angular_speed": 26.66,
        "t_start_deccel": 57.5,
        "t_stop": 102.5,
        "sign": 1,
        "time_to_decrease_jerk_1": 25.0,
        "time_to_decrease_jerk_2": 86.0,
        "accel_ramp_up_jerk": 60000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_RIGHT_135_FROM_45": {
        "start": 0.0,
        "end": 18.55,
        "turn_linear_speed": 2.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 68.0,
        "t_stop": 115.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 100.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
      },
      "TURN_LEFT_135_FROM_45": {
        "start": 0.0,
        "end": 18.55,
        "turn_linear_speed": 2.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 33.0,
        "t_start_deccel": 68.0,
        "t_stop": 115.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 23.0,
        "time_to_decrease_jerk_2": 100.0,
        "accel_ramp_up_jerk": 80000.0,
        "accel_ramp_down_jerk": 50000.0
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
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
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
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
    },
    "forward": {
      "START": {
        "max_speed": 1.5,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 4.5,
        "acceleration": 25.0,
        "deceleration": 30.0,
        "target_travel_mm": 180.0
      },
      "DIAGONAL": {
        "max_speed": 3.5,
        "acceleration": 15.0,
        "deceleration": 25.0,
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
        "max_speed": 3.0,
        "acceleration": 12.0,
        "deceleration": 20.0,
        "target_travel_mm": 5.0
      },
      "TURN_LEFT_90": {
        "max_speed": 3.0,
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
        "turn_linear_speed": 3.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 30.0,
        "t_start_deccel": 46.0,
        "t_stop": 92.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 80.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 40000.0
      },
      "TURN_LEFT_90": {
        "start": 0.0,
        "end": -12.5,
        "turn_linear_speed": 3.0,
        "angular_accel": 1200.0,
        "max_angular_speed": 30.0,
        "t_start_deccel": 46.0,
        "t_stop": 92.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 16.0,
        "time_to_decrease_jerk_2": 80.0,
        "accel_ramp_up_jerk": 100000.0,
        "accel_ramp_down_jerk": 40000.0
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
        "turn_linear_speed": 1.5,
        "angular_accel": 52.36,
        "max_angular_speed": 3.49,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      }
    }
  },
  "CUSTOM": {
    "general": {
      "max_linear_acc_jerk": 625.0,
      "max_linear_brake_jerk": 625.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
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
      "max_linear_acc_jerk": 100.0,
      "max_linear_brake_jerk": 100.0,
      "accel_margin_mm": 20.0,
      "brake_margin_mm": 20.0
    },
    "forward": {
      "START": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 4.0,
        "target_travel_mm": 107.5
      },
      "FORWARD": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 4.0,
        "target_travel_mm": 180.0
      },
      "STOP": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 6.0,
        "target_travel_mm": 90.0
      },
      "TURN_AROUND": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 6.0,
        "target_travel_mm": 80.0
      },
      "TURN_AROUND_INPLACE": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 6.0,
        "target_travel_mm": 80.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 4.0,
        "target_travel_mm": 22.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "max_speed": 0.7,
        "acceleration": 4.0,
        "deceleration": 4.0,
        "target_travel_mm": 23.0
      }
    },
    "turn": {
      "TURN_AROUND": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 104.72,
        "max_angular_speed": 10.47,
        "t_start_deccel": 301.0,
        "t_stop": 401.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_AROUND_INPLACE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 104.72,
        "max_angular_speed": 10.47,
        "t_start_deccel": 301.0,
        "t_stop": 401.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 244.346,
        "max_angular_speed": 17.453,
        "t_start_deccel": 96.0,
        "t_stop": 164.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90_SEARCH_MODE": {
        "start": 0.0,
        "end": 0.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 244.346,
        "max_angular_speed": 17.453,
        "t_start_deccel": 96.0,
        "t_stop": 164.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_RIGHT_90": {
        "start": 0.0,
        "end": -22.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 244.346,
        "max_angular_speed": 15.708,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": -1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      },
      "TURN_LEFT_90": {
        "start": 0.0,
        "end": -22.0,
        "turn_linear_speed": 0.7,
        "angular_accel": 244.346,
        "max_angular_speed": 15.708,
        "t_start_deccel": 0.0,
        "t_stop": 0.0,
        "sign": 1,
        "time_to_decrease_jerk_1": 0.0,
        "time_to_decrease_jerk_2": 0.0,
        "accel_ramp_up_jerk": 0.0,
        "accel_ramp_down_jerk": 0.0
      }
    }
  }
};
