#include "utils/movement_params.hpp"


std::map<Movement, TurnParams> turn_params_search_slow = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.25, 52.36, 3.49, 3.1067, 0, 0, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.25, 43.633, 4.014, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.25, 43.633, 4.014, 1.553, 0, 0, 1}},
};

std::map<Movement, ForwardParams> forward_params_search_slow = {
    {Movement::START, {0.25, 0.65, 0.65, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.25, 0.65, 0.65, CELL_SIZE_CM}},
    {Movement::STOP, {0.25, 0.65, 0.65, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.25, 0.5, 0.5, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.25, 0.65, 0.65, 2.2}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.25, 0.65, 0.65, 2.2}},
};

std::map<Movement, TurnParams> turn_params_search_medium = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.5, 52.36, 3.49, 3.1067, 0, 0, -1}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.5, 52.36, 3.49, 3.1067, 0, 0, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 104.72, 10.47, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.5, 104.72, 10.47, 1.553, 0, 0, 1}},
};

std::map<Movement, ForwardParams> forward_params_search_medium = {
    {Movement::START, {0.5, 1.0, 1.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.5, 2.0, 2.0, CELL_SIZE_CM}},
    {Movement::STOP, {0.5, 2.0, 2.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, 8.0}},
    {Movement::TURN_AROUND_INPLACE, {0.5, 2.0, 2.0, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.5, 2.0, 2.0, 1.5}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.5, 2.0, 2.0, 1.5}},
};

std::map<Movement, TurnParams> turn_params_search_fast = {
    {Movement::TURN_AROUND, {0.0, 0.0, 0.7, 104.72, 10.47, 3.1416, 0, 0, -1}},
    {Movement::TURN_AROUND_INPLACE, {0.0, 0.0, 0.7, 104.72, 10.47, 3.1416, 0, 0, -1}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 1.5708, 0, 0, -1}}, // -3.0
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.0, 0.0, 0.7, 244.346, 17.453, 1.5708, 0, 0, 1}},   // -3.0
    {Movement::TURN_RIGHT_90, {0.0, -2.2, 0.7, 244.346, 15.708, 1.5708, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0.0, -2.2, 0.7, 244.346, 15.708, 1.5708, 0, 0, 1}},
};

std::map<Movement, ForwardParams> forward_params_search_fast = {
    {Movement::START, {0.7, 2.0, 2.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM}},
    {Movement::FORWARD, {0.7, 4.0, 4.0, CELL_SIZE_CM}},
    {Movement::STOP, {0.7, 4.0, 6.0, (HALF_CELL_SIZE_CM)}},
    {Movement::TURN_AROUND, {0.7, 4.0, 6.0, 8.0}},
    {Movement::TURN_AROUND_INPLACE, {0.7, 4.0, 6.0, 8.0}},
    {Movement::TURN_RIGHT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 1.9}},
    {Movement::TURN_LEFT_90_SEARCH_MODE, {0.7, 4.0, 4.0, 2.0}},
};

std::map<Movement, TurnParams> turn_params_slow = {
    {Movement::TURN_RIGHT_45, {-5.0, -8.6, 0.5, 100.00, 7.854, 0.7854, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-5.0, -8.6, 0.5, 100.00, 7.854, 0.7854, 0, 0, 1}},
    {Movement::TURN_RIGHT_90, {1.5, -2.2, 0.5, 104.72, 10.47, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {1.5, -2.2, 0.5, 104.72, 10.47, 1.553, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {-1.5, 1.5, 0.5, 100.00, 5.5850, 3.1241, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {-1.5, 1.5, 0.5, 100.00, 5.5850, 3.1241, 0, 0, 1}},
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

    {Movement::TURN_RIGHT_45_FROM_45, {0.5, 2.0, 2.0, 7.4}},
    {Movement::TURN_LEFT_45_FROM_45, {0.5, 2.0, 2.0, 7.4}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.5, 2.0, 2.0, 4.3}},
    {Movement::TURN_LEFT_90_FROM_45, {0.5, 2.0, 2.0, 4.8}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.5, 2.0, 2.0, 6.0}},
    {Movement::TURN_LEFT_135_FROM_45, {0.5, 2.0, 2.0, 7.85}},
};

std::map<Movement, TurnParams> turn_params_medium = {

    {Movement::TURN_RIGHT_45, {-6.3, -8.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45, {-6.3, -8.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90, {-2.4, 0.7, 1.5, 700.0, 17.45, 1.5708, 88, 121, -1}},
    {Movement::TURN_LEFT_90, {-2.1, 0.7, 1.5, 700.0, 17.45, 1.5708, 88, 121, 1}},
    {Movement::TURN_RIGHT_135, {-4.5, -5.4, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135, {-4.5, -5.8, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},
    {Movement::TURN_RIGHT_180, {-6.0, 2.3, 1.5, 261.06, 16.4, 3.1416, 192, 250, -1}},
    {Movement::TURN_LEFT_180, {-6.0, 2.8, 1.5, 261.06, 16.4, 3.1416, 192, 250, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 5.7, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 4.6, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -3.2, 1.5, 785.40, 26.18, 1.5708, 60, 114, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -2.8, 1.5, 785.40, 26.18, 1.5708, 60, 114, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 2.5, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 2.1, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_medium = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {1.7, 12.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 8.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 6.9}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 6.4}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 3.5}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 3.5}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 2.8}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 3.1}},
};

std::map<Movement, TurnParams> turn_params_fast = {
    {Movement::TURN_RIGHT_45, {-2.0, -8.2, 1.5, 785.39, 22.69, 0.7679, 0, 0, -1}},
    {Movement::TURN_LEFT_45, {-2.0, -8.2, 1.5, 785.39, 22.69, 0.7679, 0, 0, 1}},

    {Movement::TURN_RIGHT_90, {0.0, -1.3, 1.5, 785.39, 24.96, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90, {0.0, -1.3, 1.5, 785.39, 24.96, 1.553, 0, 0, 1}},
    {Movement::TURN_RIGHT_135, {-2.9, -9.1, 1.5, 610.865, 20.94, 2.3387, 0, 0, -1}},
    {Movement::TURN_LEFT_135, {-2.9, -9.1, 1.5, 610.865, 20.94, 2.3387, 0, 0, 1}},
    {Movement::TURN_RIGHT_180, {-2.9, 0.2, 1.5, 610.865, 16.05, 3.1241, 0, 0, -1}},
    {Movement::TURN_LEFT_180, {-2.9, 0.2, 1.5, 610.865, 16.05, 3.1241, 0, 0, 1}},

    // Below parameters not tested
    {Movement::TURN_RIGHT_45_FROM_45, {7.7, 5.4, 1.5, 785.39, 22.69, 0.7679, 0, 0, -1}},

    {Movement::TURN_LEFT_45_FROM_45, {7.7, 5.4, 1.5, 785.39, 22.69, 0.7679, 0, 0, 1}},

    {Movement::TURN_RIGHT_90_FROM_45, {4.0, -3.8, 1.5, 785.39, 24.96, 1.553, 0, 0, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {4.0, -3.8, 1.5, 785.39, 24.96, 1.553, 0, 0, 1}},

    {Movement::TURN_RIGHT_135_FROM_45, {7.0, 1.0, 1.5, 610.865, 20.94, 2.3387, 0, 0, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {7.0, 1.0, 1.5, 610.865, 20.94, 2.3387, 0, 0, 1}},

    {Movement::TURN_AROUND, {HALF_CELL_SIZE_CM, HALF_CELL_SIZE_CM, 0.0, 100.00, 6.981, 3.1241, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_fast = {
    {Movement::START,
     {1.0, 5.0, 20.0,
      HALF_CELL_SIZE_CM +
          ROBOT_DIST_FROM_CENTER_START_CM_FAST}}, // Speed reached in the start end is = 1.0m/s (sqrt(2*5*0.11))

    {Movement::FORWARD, {3.5, 15.0, 30.0, CELL_SIZE_CM}},

    {Movement::DIAGONAL, {2.5, 15.0, 35.0, CELL_DIAGONAL_SIZE_CM}},

    {Movement::STOP, {0.75, 2.0, 35.0, (HALF_CELL_SIZE_CM)}},

    {Movement::TURN_AROUND, {0.5, 2.0, 2.0, HALF_CELL_SIZE_CM}},
};

std::map<Movement, TurnParams> turn_params_custom = {
    {Movement::TURN_RIGHT_45, {-6.3, -8.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45, {-6.3, -8.4, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90, {-2.4, 0.7, 1.5, 700.0, 17.45, 1.5708, 88, 121, -1}},
    {Movement::TURN_LEFT_90, {-2.1, 0.7, 1.5, 700.0, 17.45, 1.5708, 88, 121, 1}},
    {Movement::TURN_RIGHT_135, {-4.5, -5.4, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135, {-4.5, -5.8, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},
    {Movement::TURN_RIGHT_180, {-6.0, 2.3, 1.5, 261.06, 16.4, 3.1416, 192, 250, -1}},
    {Movement::TURN_LEFT_180, {-6.0, 2.8, 1.5, 261.06, 16.4, 3.1416, 192, 250, 1}},

    {Movement::TURN_RIGHT_45_FROM_45, {0.0, 5.7, 1.5, 785.40, 20.07, 0.7854, 38, 73, -1}},
    {Movement::TURN_LEFT_45_FROM_45, {0.0, 4.6, 1.5, 785.40, 20.07, 0.7854, 38, 73, 1}},
    {Movement::TURN_RIGHT_90_FROM_45, {0.0, -3.2, 1.5, 785.40, 26.18, 1.5708, 60, 114, -1}},
    {Movement::TURN_LEFT_90_FROM_45, {0.0, -2.8, 1.5, 785.40, 26.18, 1.5708, 60, 114, 1}},
    {Movement::TURN_RIGHT_135_FROM_45, {0.0, 2.5, 1.5, 436.33, 20.07, 2.3562, 116, 167, -1}},
    {Movement::TURN_LEFT_135_FROM_45, {0.0, 2.1, 1.5, 436.33, 20.07, 2.3562, 116, 167, 1}},

    {Movement::TURN_AROUND, {0.0, 0.0, 1.5, 52.36, 3.49, 3.1416, 0, 0, -1}},
};

std::map<Movement, ForwardParams> forward_params_custom = {
    {Movement::START, {1.5, 12.0, 20.0, HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM_FAST}},
    {Movement::FORWARD, {3.5, 15.0, 20.0, CELL_SIZE_CM}},
    {Movement::DIAGONAL, {1.7, 12.0, 20.0, CELL_DIAGONAL_SIZE_CM}},
    {Movement::STOP, {1.0, 2.0, 30.0, (HALF_CELL_SIZE_CM - 1.0)}},
    {Movement::TURN_AROUND, {1.5, 12.0, 20.0, 8.0}},

    {Movement::TURN_RIGHT_45_FROM_45, {1.5, 12.0, 20.0, 6.9}},
    {Movement::TURN_LEFT_45_FROM_45, {1.5, 12.0, 20.0, 6.4}},
    {Movement::TURN_RIGHT_90_FROM_45, {1.5, 12.0, 20.0, 3.5}},
    {Movement::TURN_LEFT_90_FROM_45, {1.5, 12.0, 20.0, 3.5}},
    {Movement::TURN_RIGHT_135_FROM_45, {1.5, 12.0, 20.0, 2.8}},
    {Movement::TURN_LEFT_135_FROM_45, {1.5, 12.0, 20.0, 3.1}},
};

GeneralParams general_params_search_slow = {
    0.0,                    // Fan speed
    0.0350, 0.0020, 0.0000, // Angular P,I,D
    0.0012, 0.0000, 0.0020, // Wall P,I,D
    3.0000, 0.0360, 0.0000, // Linear velocity P,I,D
    0.8000, 0.0000, 0.0000, // Diagonal walls P,I,D
    0.0,                    // Start wall break cm left
    0.0,                    // Start wall break cm right
    0.0                     // Enable wall break correction
};

GeneralParams general_params_search_medium = {
    0.0,                    // Fan speed
    0.0500, 0.0050, 0.0000, // Angular P,I,D
    0.0015, 0.0000, 0.0025, // Wall P,I,D
    5.0000, 0.0600, 0.0000, // Linear velocity P,I,D
    0.0000, 0.0000, 0.0000, // Diagonal walls P,I,D
    0.0,                    // Start wall break cm left
    0.0,                    // Start wall break cm right
    0.0                     // Enable wall break correction
};

GeneralParams general_params_search_fast = {
    200.0,                  // Fan speed
    0.0850, 0.0110, 0.0000, // Angular P,I,D
    0.0020, 0.0000, 0.0040, // Wall P,I,D
    8.0000, 0.1100, 0.0000, // Linear velocity P,I,D
    0.0000, 0.0000, 0.0000, // Diagonal walls P,I,D
    0.0,                    // Start wall break cm left
    0.0,                    // Start wall break cm right
    0.0                     // Enable wall break correction
};

GeneralParams general_params_slow = {
    0.0,                    // Fan speed
    0.0500, 0.0050, 0.0000, // Angular P,I,D
    0.0015, 0.0000, 0.0025, // Wall P,I,D
    5.0000, 0.0600, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    6.1,                    // Start wall break cm left
    7.5,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_medium = {
    600.0,                  // Fan speed
    0.1050, 0.0100, 0.0075, // Angular P,I,D
    0.0025, 0.0000, 0.0050, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    6.5,                    // Start wall break cm left
    8.0,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};

GeneralParams general_params_fast = {
    600.0,                  // Fan speed
    0.1050, 0.0100, 0.0075, // Angular P,I,D
    0.0025, 0.0000, 0.0050, // Wall P,I,D
    8.0000, 0.1000, 0.0000, // Linear velocity P,I,D
    0.0045, 0.0000, 0.0045, // Diagonal walls P,I,D
    6.5,                    // Start wall break cm left
    8.0,                    // Start wall break cm right
    1.0                     // Enable wall break correction
};
