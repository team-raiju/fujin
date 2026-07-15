#include "utils/movement_params.hpp"

const std::map<Movement, TurnParams> turn_params_search_slow = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.3, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.3, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 55, 5.5, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 55, 5.5, 0, 0, 1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_search_slow = {
    {Movement::START, {0.3, 0.85, 0.85, 109.0}},
    {Movement::FORWARD, {0.3, 0.85, 0.85, CELL_SIZE_MM}},
    {Movement::STOP, {0.3, 0.85, 0.85, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.3, 0.5, 0.5, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.3, 0.5, 0.5, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 24.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 24.0}},
};

const std::map<Movement, TurnParams> turn_params_search_medium = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 104.72, 10.47, 301, 401, -1, 0, 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 104.72, 10.47, 301, 401, -1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, 150, 225, -1, 0, 0, 0}}, // -30.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, 150, 225, 1, 0, 0, 0}},   // -30.0
};

const std::map<Movement, ForwardParams> forward_params_search_medium = {
    {Movement::START, {0.5, 3.0, 3.0, 109.0}},
    {Movement::FORWARD, {0.5, 3.0, 3.0, CELL_SIZE_MM}},
    {Movement::STOP, {0.5, 3.0, 5.0, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.5, 3.0, 5.0, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 24.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 27.0}},
};

const std::map<Movement, TurnParams> turn_params_search_fast = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.7, 104.72, 10.47, 301, 401, -1, 0, 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.7, 104.72, 10.47, 301, 401, -1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 96, 164, -1, 0, 0, 0}}, // -30.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 96, 164, 1, 0, 0, 0}},   // -30.0
    {Movement::TURN_RIGHT_90, {0.0, -22.0, 0.7, 244.346, 15.708, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -22.0, 0.7, 244.346, 15.708, 0, 0, 1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_search_fast = {
    {Movement::START, {0.7, 4.0, 4.0, 109.0}},
    {Movement::FORWARD, {0.7, 4.0, 4.0, CELL_SIZE_MM}},
    {Movement::STOP, {0.7, 4.0, 6.0, (HALF_CELL_SIZE_MM)}},
    {Movement::TURN_AROUND, {0.7, 4.0, 6.0, 80.0}},
    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 80.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 22.0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 23.0}},
};

const std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {-50.0, -86.0, 0.5, 100.00, 7.854, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45, {-50.0, -86.0, 0.5, 100.00, 7.854, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -41.0, 0.5, 104.72, 10.47, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -41.0, 0.5, 104.72, 10.47, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_180, {0.0, 0.0, 0.5, 122.17, 5.65, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_180, {0.0, 0.0, 0.5, 122.17, 5.45, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135, {-15.0, -76.0, 0.5, 100.00, 7.5049, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135, {0.0, -86.0, 0.5, 100.00, 7.5049, 0, 0, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 37.0, 0.5, 100.0, 7.8539, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 37.0, 0.5, 100.0, 7.8539, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -62.0, 0.5, 104.72, 10.47, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -62.0, 0.5, 104.72, 10.47, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 3.0, 0.5, 100.0, 7.5049, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, -12.0, 0.5, 100.0, 7.5049, 0, 0, 1, 0, 0, 0}},
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::START, {0.5, 2.0, 2.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM}},
    {Movement::FORWARD, {0.7, 2.0, 2.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {0.7, 2.0, 2.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {0.5, 2.0, 2.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, 80.0}},

    {Movement::TURN_RIGHT_90, {0.5, 2.0, 2.0, 16.0}},
    {Movement::TURN_LEFT_90, {0.5, 2.0, 2.0, 18.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.5, 2.0, 2.0, 74.0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.5, 2.0, 2.0, 74.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.5, 2.0, 2.0, 43.0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.5, 2.0, 2.0, 48.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.5, 2.0, 2.0, 60.0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.5, 2.0, 2.0, 78.5}},
};

const std::map<Movement, TurnParams> turn_params_medium = {

    {Movement::TURN_RIGHT_45, {-46.0, -91.0, 1.0, 610.86, 17.45, 44, 79, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45, {-46.0, -90.0, 1.0, 610.86, 17.45, 44, 79, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -26.0, 1.0, 610.86, 20.94, 74, 115, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -29.0, 1.0, 610.86, 20.94, 74, 115, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135, {-3.0, -84.0, 1.0, 261.8, 16.58, 142, 206, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135, {-9.0, -84.0, 1.0, 261.8, 16.58, 142, 206, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_180, {0.0, -12.5, 1.0, 523.6, 10.95, 286, 307, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_180, {0.0, -14.5, 1.0, 523.6, 10.80, 290, 309, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 38.0, 1.0, 610.86, 17.45, 44, 79, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 34.0, 1.0, 610.86, 17.45, 44, 79, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -56.0, 1.0, 610.86, 20.94, 74, 115, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -56.0, 1.0, 610.86, 20.94, 74, 115, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 0.0, 1.0, 261.8, 16.58, 142, 206, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, -2.0, 1.0, 261.8, 16.58, 142, 206, 1, 0, 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.0, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {1.0, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {3.0, 12.0, 20.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {2.5, 12.0, 20.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.0, 12.0, 20.0, 80.0}},
    {Movement::TURN_RIGHT_90, {1.0, 12.0, 20.0, 16.0}},
    {Movement::TURN_LEFT_90, {1.0, 12.0, 20.0, 16.0}},
    {Movement::TURN_RIGHT_180, {1.0, 12.0, 20.0, 0.0}},
    {Movement::TURN_LEFT_180, {1.0, 12.0, 20.0, 0.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.0, 12.0, 20.0, 80.0}},
    {Movement::TURN_LEFT_45_FROM_45, {1.0, 12.0, 20.0, 82.0}},

    {Movement::TURN_RIGHT_90_FROM_45, {1.0, 12.0, 20.0, 63.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.0, 12.0, 20.0, 55.0}},

    {Movement::TURN_RIGHT_135_FROM_45, {1.0, 12.0, 20.0, 68.0}},
    {Movement::TURN_LEFT_135_FROM_45, {1.0, 12.0, 20.0, 70.0}},
};

const std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 1.3, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 1.3, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, 217, 242, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, 217, 242, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {3.0, 15.0, 20.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 79.0}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -7.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 67.5}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 63.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 33.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 32.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 27.5}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 28.0}},
};

const std::map<Movement, TurnParams> turn_params_super = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 1.3, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 1.3, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, 217, 242, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, 217, 242, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
};

const std::map<Movement, ForwardParams> forward_params_super = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST}},
    {Movement::FORWARD, {4.5, 25.0, 30.0, CELL_SIZE_MM}},
    {Movement::DIAGONAL, {3.5, 15.0, 25.0, CELL_DIAGONAL_SIZE_MM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_MM - 10.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 79.0}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 5.0}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -7.0}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -7.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 67.5}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 63.0}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 33.0}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 32.0}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 27.5}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 28.0}},
};

std::map<Movement, TurnParams> turn_params_custom = {
    {Movement::TURN_RIGHT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45, {-64.0, -82.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90, {0.0, -11.0, 1.3, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90, {0.0, -12.5, 1.3, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135, {-46.0, -46.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135, {-46.0, -50.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_180, {-10.0, -11.0, 1.3, 523.6, 14.25, 217, 242, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_180, {-10.0, -17.0, 1.3, 523.6, 14.25, 217, 242, 1, 0, 0, 0}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 57.0, 1.5, 785.40, 20.07, 38, 73, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 54.0, 1.5, 785.40, 20.07, 38, 73, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -26.0, 1.5, 785.40, 26.18, 60, 108, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -33.0, 1.5, 785.40, 26.18, 60, 108, 1, 0, 0, 0}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 38.0, 1.5, 436.33, 20.07, 116, 167, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 39.0, 1.5, 436.33, 20.07, 116, 167, 1, 0, 0, 0}},

    {Movement::TURN_AROUND, {0.0, 0.0, 0.3, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.3, 52.36, 3.49, 0, 0, -1, 0, 0, 0}},

    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, 0, 0, -1, 0, 0, 0}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, 0, 0, 1, 0, 0, 0}},
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

const GeneralParams general_params_search_slow = {
    0.0,                      // Fan speed
    0.0480,  0.00042, 0.0000, // Angular P,I,D
    0.00044, 0.0035,          // Angular acc, velocity feed-forward
    0.0,     0.0,     0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.0,     0.0,             // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,             // Angular jerk feed-forward k, Angular jerk limit ms
    0.0006,  0.0000,  0.0010, // Wall P,I,D
    3.0000,  0.0360,  0.0000, // Linear velocity P,I,D
    0.0000,  0.0000,  0.0000, // Diagonal walls P,I,D
    45.0,                     // Start wall break mm left
    57.0,                     // Start wall break mm right
    1.0                       // Enable wall break correction
};

const GeneralParams general_params_search_medium = {
    150.0,                   // Fan speed
    0.0550,  0.0090, 0.0000, // Angular P,I,D
    0.00000, 0.0000,         // Angular acc, velocity feed-forward
    0.0,     0.0,    0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.0,     0.0,            // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,            // Angular jerk feed-forward k, Angular jerk limit ms
    0.0020,  0.0000, 0.0040, // Wall P,I,D
    8.0000,  0.1000, 0.0000, // Linear velocity P,I,D
    0.0000,  0.0000, 0.0000, // Diagonal walls P,I,D
    55.0,                    // Start wall break mm left
    67.0,                    // Start wall break mm right
    1.0                      // Enable wall break correction
};

const GeneralParams general_params_search_fast = {
    220.0,                   // Fan speed
    0.0850,  0.0110, 0.0000, // Angular P,I,D
    0.00000, 0.0000,         // Angular acc, velocity feed-forward
    0.0,     0.0,    0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.0,     0.0,            // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,            // Angular jerk feed-forward k, Angular jerk limit ms
    0.0020,  0.0000, 0.0040, // Wall P,I,D
    8.0000,  0.1000, 0.0000, // Linear velocity P,I,D
    0.0000,  0.0000, 0.0000, // Diagonal walls P,I,D
    55.0,                    // Start wall break mm left
    67.0,                    // Start wall break mm right
    1.0                      // Enable wall break correction
};

const GeneralParams general_params_slow = {
    0.0,                      // Fan speed
    0.0480,  0.00042, 0.0000, // Angular P,I,D
    0.00040, 0.004,          // Angular acc, velocity feed-forward
    0.0,     0.0,     0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.0,     0.0,             // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,             // Angular jerk feed-forward k, Angular jerk limit ms
    0.0015,  0.0000,  0.0025, // Wall P,I,D
    5.0000,  0.0600,  0.0000, // Linear velocity P,I,D
    0.0045,  0.0000,  0.0045, // Diagonal walls P,I,D
    53.0,                     // Start wall break mm left
    75.0,                     // Start wall break mm right
    1.0                       // Enable wall break correction
};

const GeneralParams general_params_medium = {
    600.0,                   // Fan speed
    0.0950,  0.0010, 0.0000, // Angular P,I,D
    0.00055, 0.006,          // Angular acc, velocity feed-forward
    0.0,     0.02,   0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.03,    8.0,            // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,            // Angular jerk feed-forward k, Angular jerk limit ms
    0.0020,  0.0000, 0.0040, // Wall P,I,D
    9.2500,  0.1440, 0.0000, // Linear velocity P,I,D
    0.0010,  0.0000, 0.0020, // Diagonal walls P,I,D
    55.0,                    // Start wall break mm left
    72.0,                    // Start wall break mm right
    1.0                      // Enable wall break correction
};

const GeneralParams general_params_fast = {
    600.0,                   // Fan speed
    0.0950,  0.0010, 0.0000, // Angular P,I,D
    0.00055, 0.006,          // Angular acc, velocity feed-forward
    0.0,     0.02,   0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.03,    8.0,            // Linear jerk feed-forward k, Linear jerk limit ms
    0.0,     0.0,            // Angular jerk feed-forward k, Angular jerk limit ms
    0.0020,  0.0000, 0.0040, // Wall P,I,D
    9.2500,  0.1440, 0.0000, // Linear velocity P,I,D
    0.0010,  0.0000, 0.0020, // Diagonal walls P,I,D
    55.0,                    // Start wall break mm left
    72.0,                    // Start wall break mm right
    1.0                      // Enable wall break correction
};

const GeneralParams general_params_super = {
    675.0,                   // Fan speed
    0.0950,  0.0010, 0.0000, // Angular P,I,D
    0.00059, 0.006,          // Angular acc, velocity feed-forward
    0.0,     0.02,   0.0,    // Linear velocity acc, brake, velocity feed-forward
    0.03,    8.0,            // Linear jerk feed-forward k, Linear jerk limit ms
    0.00042, 5.0,            // Angular jerk feed-forward k, Angular jerk limit ms
    0.0025,  0.0000, 0.0050, // Wall P,I,D
    9.2500,  0.1440, 0.0000, // Linear velocity P,I,D
    0.0010,  0.0000, 0.0020, // Diagonal walls P,I,D
    55.0,                    // Start wall break mm left
    72.0,                    // Start wall break mm right
    1.0                      // Enable wall break correction
};
