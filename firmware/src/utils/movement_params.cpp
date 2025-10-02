#include "utils/movement_params.hpp"

std::map<Movement, TurnParams> turn_params_search_slow = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.3, 52.36, 3.49, 3.1067, 0, 0, -1}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.3, 52.36, 3.49, 3.1067, 0, 0, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.3, 43.633, 4.014, 1.553, 0, 0, 1}},
};

std::map<Movement, ForwardParams> forward_params_search_slow = {
    {Movement::START, {0.3, 0.85, 0.85, 10.9}},
    {Movement::FORWARD, {0.3, 0.85, 0.85, CELL_SIZE_CM}},
    {Movement::STOP, {0.3, 0.85, 0.85, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.3, 0.5, 0.5, 8.0}},
    {Movement::TURN_AROUND_INPLACE, {0.3, 0.5, 0.5, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 1.1}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.3, 0.85, 0.85, 1.1}},
};

std::map<Movement, TurnParams> turn_params_search_medium = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 104.72, 10.47, 3.1241, 301, 401, -1}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 104.72, 10.47, 3.1241, 301, 401, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, 1.5708, 150, 225, -1}}, // -3.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 139.62, 10.47, 1.5708, 150, 225, 1}},   // -3.0
};

std::map<Movement, ForwardParams> forward_params_search_medium = {
    {Movement::START, {0.5, 3.0, 3.0, 10.9}},
    {Movement::FORWARD, {0.5, 3.0, 3.0, CELL_SIZE_CM}},
    {Movement::STOP, {0.5, 3.0, 5.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.5, 3.0, 5.0, 8.0}},
    {Movement::TURN_AROUND_INPLACE, {0.5, 3.0, 5.0, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 2.4}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.5, 3.0, 3.0, 2.7}},
};

std::map<Movement, TurnParams> turn_params_search_fast = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.7, 104.72, 10.47, 3.1241, 301, 401, -1}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.7, 104.72, 10.47, 3.1241, 301, 401, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 1.5708, 96, 164, -1}}, // -3.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 1.5708, 96, 164, 1}},   // -3.0
    {Movement::TURN_RIGHT_90, {0.0, -2.2, 0.7, 244.346, 15.708, 1.5708, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0.0, -2.2, 0.7, 244.346, 15.708, 1.5708, 0, 0, 1}},
};

std::map<Movement, ForwardParams> forward_params_search_fast = {
    {Movement::START, {0.7, 4.0, 4.0, 10.9}},
    {Movement::FORWARD, {0.7, 4.0, 4.0, CELL_SIZE_CM}},
    {Movement::STOP, {0.7, 4.0, 6.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.7, 4.0, 6.0, 8.0}},
    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 2.2}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 2.3}},
};

std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {-5.0, -8.6, 0.5, 100.00, 7.854, 0.7854, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-5.0, -8.6, 0.5, 100.00, 7.854, 0.7854, 0, 0, 1}},
    {Movement::TURN_RIGHT_90, {0.0, -4.1, 0.5, 104.72, 10.47, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0.0, -4.1, 0.5, 104.72, 10.47, 1.553, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {0.0, 0.0, 0.5, 122.17, 5.65, 3.1241, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {0.0, 0.0, 0.5, 122.17, 5.45, 3.1241, 0, 0, 1}},
    {Movement::TURN_RIGHT_135, {-1.5, -7.6, 0.5, 100.00, 7.5049, 2.3387, 0, 0, -1}},
    {Movement::TURN_LEFT_135, {0.0, -8.6, 0.5, 100.00, 7.5049, 2.3387, 0, 0, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 3.7, 0.5, 100.0, 7.8539, 0.7679, 0, 0, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 3.7, 0.5, 100.0, 7.8539, 0.7679, 0, 0, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -6.2, 0.5, 104.72, 10.47, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -6.2, 0.5, 104.72, 10.47, 1.5708, 0, 0, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 0.3, 0.5, 100.0, 7.5049, 2.3387, 0, 0, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, -1.2, 0.5, 100.0, 7.5049, 2.3387, 0, 0, 1}},
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 52.36, 3.49, 3.1067, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_slow = {
    {Movement::START, {0.5, 2.0, 2.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.7, 2.0, 2.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {0.7, 2.0, 2.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {0.5, 2.0, 2.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, 8.0}},

    {Movement::TURN_RIGHT_90, {0.5, 2.0, 2.0, 1.6}},
    {Movement::TURN_LEFT_90, {0.5, 2.0, 2.0, 1.8}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.5, 2.0, 2.0, 7.4}},
    {Movement::TURN_LEFT_45_FROM_45, {0.5, 2.0, 2.0, 7.4}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.5, 2.0, 2.0, 4.3}},
    {Movement::TURN_LEFT_90_FROM_45, {0.5, 2.0, 2.0, 4.8}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.5, 2.0, 2.0, 6.0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.5, 2.0, 2.0, 7.85}},
};

std::map<Movement, TurnParams> turn_params_medium = {

    {Movement::TURN_RIGHT_45, {-4.5, -9.7, 1.0, 610.86, 17.45, 0.7854, 44, 79, -1}},
    {Movement::TURN_LEFT_45, {-4.5, -9.8, 1.0, 610.86, 17.45, 0.7854, 44, 79, 1}},
    {Movement::TURN_RIGHT_90, {0.0, -2.7, 1.0, 610.86, 20.94, 1.5708, 74, 115, -1}},
    {Movement::TURN_LEFT_90, {0.0, -3.0, 1.0, 610.86, 20.94, 1.5708, 74, 115, 1}},
    {Movement::TURN_RIGHT_135, {0.0, -9.1, 1.0, 261.8, 16.58, 2.3562, 142, 206, -1}},
    {Movement::TURN_LEFT_135, {0.0, -9.2, 1.0, 261.8, 16.58, 2.3562, 142, 206, 1}},
    {Movement::TURN_RIGHT_180, {0.0, -1.25, 1.0, 523.6, 10.95, 3.1416, 286, 307, -1}},
    {Movement::TURN_LEFT_180, {0.0, -1.45, 1.0, 523.6, 10.80, 3.1416, 290, 309, 1}},
    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 4.0, 1.0, 610.86, 17.45, 0.7854, 44, 79, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 3.3, 1.0, 610.86, 17.45, 0.7854, 44, 79, 1}},

    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -6.1, 1.0, 610.86, 20.94, 1.5708, 74, 115, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -6.2, 1.0, 610.86, 20.94, 1.5708, 74, 115, 1}},

    {Movement::TURN_RIGHT_135_FROM_45, {0.0, -0.75, 1.0, 261.8, 16.58, 2.3562, 142, 206, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, -1.5, 1.0, 261.8, 16.58, 2.3562, 142, 206, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.0, 52.36, 3.49, 3.1416, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {1.0, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {3.0, 12.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {2.5, 12.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.0, 12.0, 20.0, 8.0}},
    {Movement::TURN_RIGHT_90, {1.0, 12.0, 20.0, 1.7}},
    {Movement::TURN_LEFT_90, {1.0, 12.0, 20.0, 1.7}},
    {Movement::TURN_RIGHT_180, {1.0, 12.0, 20.0, 0.0}},
    {Movement::TURN_LEFT_180, {1.0, 12.0, 20.0, 0.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.0, 12.0, 20.0, 8.2}},
    {Movement::TURN_LEFT_45_FROM_45, {1.0, 12.0, 20.0, 8.2}},

    {Movement::TURN_RIGHT_90_FROM_45, {1.0, 12.0, 20.0, 6.2}},
    {Movement::TURN_LEFT_90_FROM_45, {1.0, 12.0, 20.0, 5.5}},

    {Movement::TURN_RIGHT_135_FROM_45, {1.0, 12.0, 20.0, 6.9}},
    {Movement::TURN_LEFT_135_FROM_45, {1.0, 12.0, 20.0, 8.7}},
};

std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90, {0.0, -1.1, 1.3, 785.40, 26.18, 1.5708, 60, 108, -1}},
    {Movement::TURN_LEFT_90, {0.0, -1.25, 1.3, 785.40, 26.18, 1.5708, 60, 108, 1}},
    {Movement::TURN_RIGHT_135, {-4.6, -4.6, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135, {-4.6, -5.0, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},
    {Movement::TURN_RIGHT_180, {-1.0, -1.1, 1.3, 523.6, 14.25, 3.1416, 217, 242, -1}},
    {Movement::TURN_LEFT_180, {-1.0, -1.7, 1.3, 523.6, 14.25, 3.1416, 217, 242, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 5.7, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 5.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -2.6, 1.5, 785.40, 26.18, 1.5708, 60, 108, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -3.3, 1.5, 785.40, 26.18, 1.5708, 60, 108, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 3.8, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 3.9, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {3.0, 15.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 7.9}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -0.7}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -0.7}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 6.75}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 6.3}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 3.3}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 3.2}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 2.75}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 2.8}},
};

std::map<Movement, TurnParams> turn_params_super = {
    {Movement::TURN_RIGHT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90, {0.0, -1.1, 1.3, 785.40, 26.18, 1.5708, 60, 108, -1}},
    {Movement::TURN_LEFT_90, {0.0, -1.25, 1.3, 785.40, 26.18, 1.5708, 60, 108, 1}},
    {Movement::TURN_RIGHT_135, {-4.6, -4.6, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135, {-4.6, -5.0, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},
    {Movement::TURN_RIGHT_180, {-1.0, -1.1, 1.3, 523.6, 14.25, 3.1416, 217, 242, -1}},
    {Movement::TURN_LEFT_180, {-1.0, -1.7, 1.3, 523.6, 14.25, 3.1416, 217, 242, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 5.7, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 5.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -2.6, 1.5, 785.40, 26.18, 1.5708, 60, 108, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -3.3, 1.5, 785.40, 26.18, 1.5708, 60, 108, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 3.8, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 3.9, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_super = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {4.5, 25.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {3.5, 15.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 7.9}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -0.7}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -0.7}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 6.75}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 6.3}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 3.3}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 3.2}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 2.75}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 2.8}},
};

std::map<Movement, TurnParams> turn_params_custom = {
    {Movement::TURN_RIGHT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45, {-6.4, -8.2, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90, {0.0, -1.0, 1.3, 785.40, 26.18, 1.5708, 60, 108, -1}},
    {Movement::TURN_LEFT_90, {0.0, -1.35, 1.3, 785.40, 26.18, 1.5708, 60, 108, 1}},
    {Movement::TURN_RIGHT_135, {-4.6, -5.3, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135, {-4.6, -5.7, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},
    {Movement::TURN_RIGHT_180, {-0.7, -1.2, 1.5, 523.6, 14.25, 3.1416, 217, 242, -1}},
    {Movement::TURN_LEFT_180, {-0.7, -1.2, 1.5, 523.6, 14.25, 3.1416, 217, 242, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 5.6, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 4.5, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -3.15, 1.5, 785.40, 26.18, 1.5708, 60, 114, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -3.45, 1.5, 785.40, 26.18, 1.5708, 60, 114, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 2.45, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 2.05, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},

    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},

    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 16.5, 1.5708, 96, 164, -1}}, // -2.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 16.5, 1.5708, 96, 164, 1}},   // 2.0

};

std::map<Movement, ForwardParams> forward_params_custom = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {3.0, 15.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 7.9}},

    {Movement::TURN_RIGHT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_LEFT_90, {1.3, 12.0, 20.0, 0.5}},
    {Movement::TURN_RIGHT_180, {1.3, 12.0, 20.0, -0.7}},
    {Movement::TURN_LEFT_180, {1.3, 12.0, 20.0, -0.7}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 6.8}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 6.3}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 3.45}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 3.05}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 2.75}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 3.05}},

    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 2.3}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 2.4}},
    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 8.0}},

};

GeneralParams general_params_search_slow = {
    0.0,                    // Fan speed
    0.0350, 0.0020, 0.0000, // Angular P,I,D
    0.0012, 0.0000, 0.0020, // Wall P,I,D
    3.0000, 0.0360, 0.0000, // Linear velocity P,I,D
    0.8000, 0.0000, 0.0000, // Diagonal walls P,I,D
    5.5,                    // Start wall break cm left
    6.7,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_search_medium = {
    150.0,                  // Fan speed
    0.0550, 0.0090, 0.0000, // Angular P,I,D
    0.0020, 0.0000, 0.0040, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0000, 0.0000, 0.0000, // Diagonal walls P,I,D
    5.5,                    // Start wall break cm left
    6.7,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_search_fast = {
    220.0,                  // Fan speed
    0.0850, 0.0110, 0.0000, // Angular P,I,D
    0.0020, 0.0000, 0.0040, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0000, 0.0000, 0.0000, // Diagonal walls P,I,D
    5.5,                    // Start wall break cm left
    6.7,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_slow = {
    0.0,                    // Fan speed
    0.0500, 0.0050, 0.0000, // Angular P,I,D
    0.0015, 0.0000, 0.0025, // Wall P,I,D
    5.0000, 0.0600, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    5.3,                    // Start wall break cm left
    7.5,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_medium = {
    600.0,                  // Fan speed
    0.1050, 0.0100, 0.0075, // Angular P,I,D
    0.0025, 0.0000, 0.0050, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    7.0,                    // Start wall break cm left
    7.7,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_fast = {
    600.0,                  // Fan speed
    0.1050, 0.0100, 0.0075, // Angular P,I,D
    0.0025, 0.0000, 0.0050, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    7.0,                    // Start wall break cm left
    8.2,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_super = {
    600.0,                  // Fan speed
    0.1050, 0.0100, 0.0075, // Angular P,I,D
    0.0025, 0.0000, 0.0050, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    7.0,                    // Start wall break cm left
    8.2,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};
