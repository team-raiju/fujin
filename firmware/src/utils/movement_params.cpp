#include "utils/movement_params.hpp"
#include "services/config.hpp"

// clang-format off
#define T(ms) services::Config::ms_to_ticks(ms)

/*
Turn Start Position | turn_params.start | foward_params.target_travel 
------------------- | ----------------- | ---------------------------
Before cell start   | E.g: -16.0        | 0.0
After cell start    | 0.0               | E.g: 16.0
Search Curve        | 0.0               | E.g: 16.0
Turn Around         | 0.0               | E.g: 80.0
Turn Around Inplace | 0.0               | E.g: 80.0

Turn End Position   | turn_params.end   | Notes
------------------- | ----------------- | ---------------------------
Before cell start   | E.g: -16.0        | Next FORWARD travels EXTRA (+16 mm)
After cell start    | E.g: 16.0         | Next FORWARD travels LESS (-16 mm)
Search Curve        | 0.0               | Uses FORWARD_2 to dynamically calculate using HALF_CELL_SIZE_MM - std::abs(current_position_mm.y);
Turn Around         | 0.0               | Uses FORWARD_2 to dynamically calculate using target_travel_mm = std::abs(current_position_mm.x);
Turn Around Inplace | 0.0               | Will not do anything after turning 180 degrees
*/

/// @section TURN_PARMS_SEARCH_SLOW
const std::map<Movement, TurnParams> turn_params_search_slow = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 150.0, 11.0, T(288), T(411), -1, T(73), T(361), 3000, 3000}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 150.0, 11.0, T(288), T(411), -1, T(73), T(361), 3000, 3000}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 180, 11.0, T(143), T(240), -1, T(61), T(204), 5000, 5000}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 180, 11.0, T(143), T(240), 1, T(61), T(204), 5000, 5000}},
};

const std::map<Movement, ForwardParams> forward_params_search_slow = {
    {Movement::START, {0.5, 3.0, 3.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM}},
    {Movement::FORWARD, {0.5, 3.0, 3.0, CELL_SIZE_MM}},
    {Movement::STOP, {0.5, 2.0, 2.0, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 19.51}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 19.51}},
};

/// @section TURN_PARMS_SEARCH_MEDIUM
const std::map<Movement, TurnParams> turn_params_search_medium = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 104.72, 10.47, T(301), T(401), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 104.72, 10.47, T(301), T(401), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, T(150), T(225), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, T(150), T(225), 1, T(0), T(0), 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_search_medium = {
    {Movement::START, {0.5, 3.0, 3.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM}},
    {Movement::FORWARD, {0.5, 3.0, 3.0, CELL_SIZE_MM}},
    {Movement::STOP, {0.5, 3.0, 5.0, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 24.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 27.0}},
};

/// @section TURN_PARMS_SEARCH_FAST
const std::map<Movement, TurnParams> turn_params_search_fast = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.7, 104.72, 10.47, T(301), T(401), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.7, 104.72, 10.47, T(301), T(401), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, T(96), T(164), -1, T(0), T(0), 0, 0}}, // -30.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, T(96), T(164), 1, T(0), T(0), 0, 0}},   // -30.0
    {Movement::TURN_RIGHT_90, {0.0, -22.0, 0.7, 244.346, 15.708, T(0), T(0), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -22.0, 0.7, 244.346, 15.708, T(0), T(0), 1, T(0), T(0), 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_search_fast = {
    {Movement::START, {0.7, 4.0, 4.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM}},
    {Movement::FORWARD, {0.7, 4.0, 4.0, CELL_SIZE_MM}},
    {Movement::STOP, {0.7, 4.0, 6.0, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.7, 4.0, 6.0, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 22.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 23.0}},
};

/// @section TURN_PARMS_SLOW
const std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {-45.02, -81.33, 0.5, 180.0, 8.45, T(91.0), T(174.0), -1, T(47.0), T(138.0), 5000, 5000}},        //ok
    {Movement::TURN_LEFT_45, {-45.02, -81.33, 0.5, 180.0, 8.45, T(91.0), T(174.0), 1, T(47.0), T(138.0), 5000, 5000}},          //ok
    {Movement::TURN_RIGHT_90, {0.0, -15.67, 0.5, 150.0, 10.47, T(147), T(247), -1, T(70), T(217), 5000, 5000}},                 //ok
    {Movement::TURN_LEFT_90, {0.0, -15.67, 0.5, 150.0, 10.47, T(147), T(247), 1, T(70), T(217), 5000, 5000}},                   //ok
    {Movement::TURN_RIGHT_135, {0.0, -74.5, 0.5, 120.0, 7.5, T(311.0), T(397.5), -1, T(62.5), T(373.5), 5000, 5000}},           //ok
    {Movement::TURN_LEFT_135, {-0.1, -69.0, 0.5, 120.0, 7.5, T(312.0), T(398.5), 1, T(62.5), T(374.5), 5000, 5000}},            //ok
    {Movement::TURN_RIGHT_180, {-10.0, 15.0, 0.5, 100.0, 5.58, T(556.0), T(632.0), -1, T(56.0), T(612.0), 5000, 5000}},         //ok
    {Movement::TURN_LEFT_180, {-10.0, 15.0, 0.5, 100.0, 5.58, T(560.0), T(636.0), 1, T(56.0), T(616.0), 5000, 5000}},           //ok
    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 45.92, 0.5, 180.0, 8.45, T(91.0), T(174.0), -1, T(47.0), T(138.0), 5000, 5000}},    //ok
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 45.92, 0.5, 180.0, 8.45, T(91.0), T(174.0), 1, T(47.0), T(138.0), 5000, 5000}},      //ok
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -59.00, 0.5, 150.0, 10.47, T(147.0), T(247.0), -1, T(70.0), T(217.0), 5000, 5000}}, //ok
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -57.00, 0.5, 150.0, 10.47, T(147.0), T(247.0), 1, T(70.0), T(217.0), 5000, 5000}},   //ok
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, -6.0, 0.5, 120.0, 7.5, T(310.5), T(397.0), -1, T(62.5), T(373.0), 5000, 5000}},    //ok
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 12.5, 0.5, 120.0, 7.5, T(310.5), T(397.0), 1, T(62.5), T(373.0), 5000, 5000}},      //ok
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 150.0, 11.0, T(288), T(411), -1, T(73), T(361), 3000, 3000}},                       //ok
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 150.0, 11.0, T(288), T(411), -1, T(73), T(361), 3000, 3000}},               //ok
};

const std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::START, {0.5, 3.0, 3.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM}},   //ok
    {Movement::FORWARD, {3.0, 8.0, 8.0, CELL_SIZE_MM}},             //ok
    {Movement::DIAGONAL, {2.0, 5.0, 5.0, CELL_DIAGONAL_SIZE_MM}},   //ok
    {Movement::STOP, {0.5, 3.0, 3.0, (HALF_CELL_SIZE_MM - 10.0)}},  //ok
    {Movement::TURN_AROUND, {0.5, 3.0, 5.0, 80.0}},                 //ok
    {Movement::TURN_AROUND_INPLACE, {0.5, 3.0, 5.0, 80.0}},         //ok


    {Movement::TURN_RIGHT_90, {0.5, 3.0, 3.0, 16.53}},              //ok
    {Movement::TURN_LEFT_90, {0.5, 3.0, 3.0, 16.53}},               //ok
    {Movement::TURN_RIGHT_135, {0.5, 3.0, 3.0, 4.0}},               //ok
    {Movement::TURN_LEFT_135, {0.5, 3.0, 3.0, 0.0}},                //ok
    {Movement::TURN_RIGHT_45_FROM_45, {0.5, 3.0, 3.0, 90.00}},      //ok
    {Movement::TURN_LEFT_45_FROM_45, {0.5, 3.0, 3.0, 82.20}},       //ok
    {Movement::TURN_RIGHT_90_FROM_45, {0.5, 3.0, 3.0, 52.00}},      //ok
    {Movement::TURN_LEFT_90_FROM_45, {0.5, 3.0, 3.0, 53.00}},       //ok
    {Movement::TURN_RIGHT_135_FROM_45, {0.5, 3.0, 3.0, 81.0}},      //ok
    {Movement::TURN_LEFT_135_FROM_45, {0.5, 3.0, 3.0, 70.68}},      //ok
};

/// @section TURN_PARMS_MEDIUM
const std::map<Movement, TurnParams> turn_params_medium = {

    {Movement::TURN_RIGHT_45, {-61.89, -61.43, 1.7, 1095.0, 19.83, T(37.0), T(73.0), -1, T(14.0), T(59.5), 80000, 50000}},
    {Movement::TURN_LEFT_45, {-61.89, -61.43, 1.7, 1095.0, 19.83, T(37.0), T(73.0), 1, T(14.0), T(59.5), 80000, 50000}},
    {Movement::TURN_RIGHT_90, {-29.34, 32.06, 2.0, 1000.0, 26.66, T(57.5), T(102.5), -1, T(25.0), T(86.0), 60000, 50000}},
    {Movement::TURN_LEFT_90, {-29.34, 32.06, 2.0, 1000.0, 26.66, T(57.5), T(102.5), 1, T(25.0), T(86.0), 60000, 50000}},
    {Movement::TURN_RIGHT_135, {-15.56, 55.80, 2.0, 1200.0, 33.0, T(68.0), T(115.0), -1, T(23.0), T(100.0), 80000, 50000}},
    {Movement::TURN_LEFT_135, {-15.56, 55.80, 2.0, 1200.0, 33.0, T(68.0), T(115.0), 1, T(23.0), T(100.0), 80000, 50000}},
    {Movement::TURN_RIGHT_180, {-50.00, 53.26, 2.18, 1000.0, 24.75, T(124.0), T(165.0), -1, T(21.0), T(152.5), 80000, 50000}},
    {Movement::TURN_LEFT_180, {-50.00, 53.26, 2.18, 1000.0, 24.75, T(124.0), T(165.0), 1, T(21.0), T(152.5), 80000, 50000}},
    {Movement::TURN_RIGHT_45_FROM_45, {0.00, 65.89, 1.7, 1095.0, 19.83, T(37.0), T(73.0), -1, T(14.0), T(59.5), 80000, 50000}},
    {Movement::TURN_LEFT_45_FROM_45, {0.00, 65.89, 1.7, 1095.0, 19.83, T(37.0), T(73.0), 1, T(14.0), T(59.5), 80000, 50000}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.00, -5.0, 2.0, 1000.0, 26.66, T(57.5), T(102.5), -1, T(25.0), T(86.0), 60000, 50000}},
    {Movement::TURN_LEFT_90_FROM_45, {0.00, -5.0, 2.0, 1000.0, 26.66, T(57.5), T(102.5), 1, T(25.0), T(86.0), 60000, 50000}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.00, 18.55, 2.0, 1200.0, 33.0, T(68.0), T(115.0), -1, T(23.0), T(100.0), 80000, 50000}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 18.55, 2.0, 1200.0, 33.0, T(68.0), T(115.0), 1, T(23.0), T(100.0), 80000, 50000}},

    {Movement::TURN_AROUND, {0.00, 0.00, 1.0, 1000.0, 24.75, T(124.0), T(165.0), -1, T(21.0), T(152.5), 80000, 50000}},
    {Movement::TURN_AROUND_INPLACE, {0.00, 0.00, 1.0, 1000.0, 24.75, T(124.0), T(165.0), -1, T(21.0), T(152.5), 80000, 50000}},
};

const std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {2.0, 20.0, 25.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {5.0, 25.0, 25.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {4.0, 20.0, 25.0, CELL_DIAGONAL_SIZE_MM}},

    {Movement::STOP, {1.0, 20.0, 25.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.0, 20.0, 25.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND_INPLACE, {1.0, 20.0, 25.0, (HALF_CELL_SIZE_MM - 10.0)}},        

    {Movement::TURN_RIGHT_90, {2.0, 20.0, 25.0, 0.0}},
    {Movement::TURN_LEFT_90, {2.0, 20.0, 25.0, 0.0}},
    {Movement::TURN_RIGHT_180, {2.18, 15.0, 20.0, 0.0}},
    {Movement::TURN_LEFT_180, {2.18, 15.0, 20.0, 0.0}},
    {Movement::TURN_RIGHT_45_FROM_45, {1.7, 15.0, 20.0, 64.52}},
    {Movement::TURN_LEFT_45_FROM_45, {1.7, 15.0, 20.0, 64.52}},
    {Movement::TURN_RIGHT_90_FROM_45, {2.0, 15.0, 20.0, 8.36}},
    {Movement::TURN_LEFT_90_FROM_45, {2.0, 15.0, 20.0, 8.36}},
    {Movement::TURN_RIGHT_135_FROM_45, {2.0, 15.0, 20.0, 59.10}},
    {Movement::TURN_LEFT_135_FROM_45, {2.0, 15.0, 20.0, 59.10}},
};

/// @section TURN_PARMS_FAST
const std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 2.5, 1300, 30, T(49.5), T(89.5), -1, T(18.5), T(76.5), 100000, 60000}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 2.5, 1300, 30, T(49.5), T(89.5), 1, T(18.5), T(76.5), 100000, 60000}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, T(217), T(242), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, T(217), T(242), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, T(60), T(108), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, T(60), T(108), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, T(0), T(0), -1, T(0), T(0), 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {3.0, 15.0, 20.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 79.0}},

    {Movement::TURN_RIGHT_90, {2.5, 12.0, 20.0, 5.0}},
    {Movement::TURN_LEFT_90, {2.5, 12.0, 20.0, 5.0}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -7.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 67.5}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 63.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 33.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 32.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 27.5}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 28.0}},
};

/// @section TURN_PARMS_SUPER
const std::map<Movement, TurnParams> turn_params_super = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 3.0, 1200, 30.0, T(46), T(92), -1, T(16), T(80), 100000, 40000}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 3.0, 1200, 30.0, T(46), T(92), 1, T(16), T(80), 100000, 40000}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, T(217), T(242), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, T(217), T(242), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, T(60), T(108), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, T(60), T(108), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, T(0), T(0), -1, T(0), T(0), 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_super = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {4.5, 25.0, 30.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {3.5, 15.0, 25.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 79.0}},

    {Movement::TURN_RIGHT_90, {3.0, 12.0, 20.0, 5.0}},
    {Movement::TURN_LEFT_90, {3.0, 12.0, 20.0, 5.0}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -7.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 67.5}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 63.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 33.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 32.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 27.5}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 28.0}},
};

/// @section TURN_PARMS_CUSTOM
std::map<Movement, TurnParams> turn_params_custom = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 1.3, 785.40, 26.18, T(60), T(108), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 1.3, 785.40, 26.18, T(60), T(108), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, T(217), T(242), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, T(217), T(242), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, T(38), T(73), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, T(38), T(73), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, T(60), T(108), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, T(60), T(108), 1, T(0), T(0), 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, T(116), T(167), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, T(116), T(167), 1, T(0), T(0), 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 0.3, 52.36, 3.49, T(0), T(0), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.3, 52.36, 3.49, T(0), T(0), -1, T(0), T(0), 0, 0}},

    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, T(0), T(0), -1, T(0), T(0), 0, 0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, T(0), T(0), 1, T(0), T(0), 0, 0}},
};

std::map<Movement, ForwardParams> forward_params_custom = {

    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {3.0, 15.0, 20.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 79.0}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_RIGHT_45, {0.0, 0.0, 0.0, 0.0}},
    {Movement::TURN_LEFT_45, {0.0, 0.0, 0.0, 0.0}},
    {Movement::TURN_RIGHT_135, {0.0, 0.0, 0.0, 0.0}},
    {Movement::TURN_LEFT_135, {0.0, 0.0, 0.0, 0.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 67.5}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 63.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 33.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 32.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 27.5}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 28.0}},

    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 11.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 11.0}},

    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 80.0}},

};
// clang-format on

const GeneralParams general_params_search_slow = {
    0.0,                       // Fan speed
    0.0420,  0.00021, 0.000,   // Angular P,I,D
    0.00035, 0.0003,  0.003,   // Angular acc ff, Angular brake ff, Angular velocity ff
    0.03125, 0.018,   0.105,   // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.0090,  0.0000,  0.0000,  // Wall P,I,D
    2.5000,  0.0100,  25.0000, // Linear velocity P,I,D
    0.0090,  0.0000,  0.0000,  // Diagonal walls P,I,D
    56.0,                      // Start wall break mm left
    64.0,                      // Start wall break mm right
    1.0,                       // Enable wall break correction
    40.0,                      // Max linear acceleration jerk
    40.0,                      // Max linear brake jerk
    13.25,                     // Wheel radius mm
    0.13,                      // Coulomb ff
    0.01,                      // Angular Coulomb ff
    0.01,                      // Angular Static ff
    0.11,                      // Angular Coulomb ff Inplace
    0.13                       // Angular Static ff Inplace
};

const GeneralParams general_params_search_medium = {
    150.0,                    // Fan speed
    0.0550,  0.0045,  0.0000, // Angular P,I,D
    0.00000, 0.00000, 0.0000, // Angular acc ff, Angular brake ff, Angular velocity ff
    0.0,     0.0,     0.0,    // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.0020,  0.0000,  0.0080, // Wall P,I,D
    8.0000,  0.0500,  0.0000, // Linear velocity P,I,D
    0.0000,  0.0000,  0.0000, // Diagonal walls P,I,D
    56.0,                     // Start wall break mm left
    64.0,                     // Start wall break mm right
    1.0,                      // Enable wall break correction
    100.0,                    // Max linear acceleration jerk
    100.0,                    // Max linear brake jerk
    13.1,                     // Wheel radius mm
    0.13,                     // Coulomb ff
    0.0,                      // Angular Coulomb ff
    0.0,                      // Angular Static ff
    0.0,                      // Angular Coulomb ff Inplace
    0.0                       // Angular Static ff Inplace
};

const GeneralParams general_params_search_fast = {
    220.0,                    // Fan speed
    0.0850,  0.0055,  0.0000, // Angular P,I,D
    0.00000, 0.00000, 0.0000, // Angular acc ff, Angular brake ff, Angular velocity ff
    0.0,     0.0,     0.0,    // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.0020,  0.0000,  0.0080, // Wall P,I,D
    8.0000,  0.0500,  0.0000, // Linear velocity P,I,D
    0.0000,  0.0000,  0.0000, // Diagonal walls P,I,D
    56.0,                     // Start wall break mm left
    64.0,                     // Start wall break mm right
    1.0,                      // Enable wall break correction
    100.0,                    // Max linear acceleration jerk
    100.0,                    // Max linear brake jerk
    13.1,                     // Wheel radius mm
    0.13,                     // Coulomb ff
    0.0,                      // Angular Coulomb ff
    0.25,                     // Angular Static ff
    0.0,                      // Angular Coulomb ff Inplace
    0.0                       // Angular Static ff Inplace
};

const GeneralParams general_params_slow = {
    0.0,                       // Fan speed
    0.0420,  0.00021, 0.000,   // Angular P,I,D
    0.00035, 0.0003,  0.003,   // Angular acc ff, Angular brake ff, Angular velocity ff
    0.03125, 0.018,   0.105,   // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.0130,  0.0000,  0.0000,  // Wall P,I,D
    2.5000,  0.0100,  25.0000, // Linear velocity P,I,D
    0.0090,  0.0000,  0.0000,  // Diagonal walls P,I,D
    56.0,                      // Start wall break mm left
    64.0,                      // Start wall break mm right
    1.0,                       // Enable wall break correction
    40.0,                      // Max linear acceleration jerk
    40.0,                      // Max linear brake jerk
    13.25,                     // Wheel radius mm
    0.13,                      // Coulomb ff
    0.01,                      // Angular Coulomb ff
    0.01,                      // Angular Static ff
    0.11,                      // Angular Coulomb ff Inplace
    0.13                       // Angular Static ff Inplace
};

const GeneralParams general_params_medium = {
    600.0,                  // Fan speed
    0.0900, 0.0005, 0.0500, // Angular P,I,D
    0.0006, 0.0006, 0.0056, // Angular acc ff, Angular brake ff, Angular velocity ff
    0.033,  0.025,  0.1410, // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.010, 0.0000, 0.0000,  // Wall P,I,D
    2.500,  0.0100, 0.0000, // Linear velocity P,I,D
    0.0010, 0.0000, 0.0040, // Diagonal walls P,I,D
    56.0,                   // Start wall break mm left
    64.0,                   // Start wall break mm right
    1.0,                    // Enable wall break correction
    625.0,                  // Max linear acceleration jerk
    625.0,                  // Max linear brake jerk
    13.1,                   // Wheel radius mm
    0.18,                   // Coulomb ff
    0.07,                   // Angular Coulomb ff
    0.25,                   // Angular Static ff
    0.0,                    // Angular Coulomb ff Inplace
    0.0                     // Angular Static ff Inplace
};

const GeneralParams general_params_fast = {
    600.0,                  // Fan speed
    0.0900, 0.0005, 0.0500, // Angular P,I,D
    0.0006, 0.0006, 0.0056, // Angular acc ff, Angular brake ff, Angular velocity ff
    0.033,  0.025,  0.1410, // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.010, 0.0000, 0.0000,  // Wall P,I,D
    2.500,  0.0100, 0.0000, // Linear velocity P,I,D
    0.0010, 0.0000, 0.0040, // Diagonal walls P,I,D
    56.0,                   // Start wall break mm left
    64.0,                   // Start wall break mm right
    1.0,                    // Enable wall break correction
    625.0,                  // Max linear acceleration jerk
    625.0,                  // Max linear brake jerk
    13.1,                   // Wheel radius mm
    0.18,                   // Coulomb ff
    0.07,                   // Angular Coulomb ff
    0.25,                   // Angular Static ff
    0.0,                    // Angular Coulomb ff Inplace
    0.0                     // Angular Static ff Inplace
};

const GeneralParams general_params_super = {
    600.0,                  // Fan speed
    0.0900, 0.0005, 0.0500, // Angular P,I,D
    0.0006, 0.0006, 0.0056, // Angular acc ff, Angular brake ff, Angular velocity ff
    0.033,  0.025,  0.1410, // Linear vel acc ff, Linear vel brake ff, Linear velocity ff
    0.010, 0.0000, 0.0000,  // Wall P,I,D
    2.500,  0.0100, 0.0000, // Linear velocity P,I,D
    0.0010, 0.0000, 0.0040, // Diagonal walls P,I,D
    56.0,                   // Start wall break mm left
    64.0,                   // Start wall break mm right
    1.0,                    // Enable wall break correction
    625.0,                  // Max linear acceleration jerk
    625.0,                  // Max linear brake jerk
    13.1,                   // Wheel radius mm
    0.18,                   // Coulomb ff
    0.07,                   // Angular Coulomb ff
    0.25,                   // Angular Static ff
    0.0,                    // Angular Coulomb ff Inplace
    0.0                     // Angular Static ff Inplace
};

const std::map<Movement, TurnParams>& get_turn_params(navigation_mode_t mode) {
    switch (mode) {
    case SEARCH_SLOW:
        return turn_params_search_slow;
    case SEARCH_MEDIUM:
        return turn_params_search_medium;
    case SEARCH_FAST:
        return turn_params_search_fast;
    case SLOW:
        return turn_params_slow;
    case MEDIUM:
        return turn_params_medium;
    case FAST:
        return turn_params_fast;
    case SUPER:
        return turn_params_super;
    case CUSTOM:
    default:
        return turn_params_custom;
    }
}

const std::map<Movement, ForwardParams>& get_forward_params(navigation_mode_t mode) {
    switch (mode) {
    case SEARCH_SLOW:
        return forward_params_search_slow;
    case SEARCH_MEDIUM:
        return forward_params_search_medium;
    case SEARCH_FAST:
        return forward_params_search_fast;
    case SLOW:
        return forward_params_slow;
    case MEDIUM:
        return forward_params_medium;
    case FAST:
        return forward_params_fast;
    case SUPER:
        return forward_params_super;
    case CUSTOM:
    default:
        return forward_params_custom;
    }
}

const GeneralParams& get_general_params(navigation_mode_t mode) {
    switch (mode) {
    case SEARCH_SLOW:
        return general_params_search_slow;
    case SEARCH_MEDIUM:
        return general_params_search_medium;
    case SEARCH_FAST:
        return general_params_search_fast;
    case SLOW:
        return general_params_slow;
    case MEDIUM:
        return general_params_medium;
    case FAST:
        return general_params_fast;
    case SUPER:
        return general_params_super;
    case CUSTOM:
    default:
        return general_params_slow;
    }
}

bool load_movement_preset_to_custom(navigation_mode_t preset) {
    if (preset == CUSTOM || preset > SUPER) {
        return false;
    }

    const auto& forward_src = get_forward_params(preset);
    const auto& turn_src = get_turn_params(preset);

    for (auto& pair : forward_params_custom) {
        pair.second = ForwardParams{};
    }
    for (auto& pair : turn_params_custom) {
        pair.second = TurnParams{};
    }

    for (const auto& pair : forward_src) {
        forward_params_custom[pair.first] = pair.second;
    }
    for (const auto& pair : turn_src) {
        turn_params_custom[pair.first] = pair.second;
    }

    return true;
}
