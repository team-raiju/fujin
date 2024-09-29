#include <cstdint>
#include <cstdio>
#include <functional>

#include "services/maze.hpp"

namespace services {

Maze* Maze::instance() {
    static Maze m;
    return &m;
}

bool Maze::cell_is_valid(Point pos) {
    return pos.x > 0 && pos.x < MAX_CELLS_X && pos.y > 0 && pos.y < MAX_CELLS_Y;
}

bool Maze::cell_is_unexplored(Point pos) {
    int x = pos.x;
    int y = pos.y;

    if ((wall_map[x][y].north == UNKNOWN_WALL) || (wall_map[x][y].east == UNKNOWN_WALL) ||
        (wall_map[x][y].south == UNKNOWN_WALL) || (wall_map[x][y].west == UNKNOWN_WALL)) {
        return true;
    } else {
        return false;
    }
}

int Maze::get_cell_priority(Direction robot_dir, Point desired_pos, Direction desired_dir) {
    int priority;
    Movement desired_movement = get_target_movement(robot_dir, desired_dir);

    if (desired_movement == FORWARD) {
        priority = 2;
    } else if (desired_movement == BACKWARD) {
        priority = 0;
    } else {
        priority = 1;
    }

    // If unexplored, add further priority
    if (cell_is_unexplored(desired_pos)) {
        priority += 4;
    }

    return priority;
}

Maze::Maze() {
    for (int x = 0; x < MAX_CELLS_X; x++) {
        for (int y = 0; y < MAX_CELLS_Y; y++) {
            wall_map[x][y].north = UNKNOWN_WALL;
            wall_map[x][y].east = UNKNOWN_WALL;
            wall_map[x][y].south = UNKNOWN_WALL;
            wall_map[x][y].west = UNKNOWN_WALL;
        }
    }

    // Add walls on all external sides
    for (int x = 0; x < MAX_CELLS_X; x++) {
        wall_map[x][0].south = WALL;
        wall_map[x][MAX_CELLS_Y - 1].north = WALL;
    }

    for (int y = 0; y < MAX_CELLS_Y; y++) {
        wall_map[0][y].west = WALL;
        wall_map[MAX_CELLS_X - 1][y].east = WALL;
    }

    // Add walls on the starting point
    wall_map[0][0].east = WALL;
    wall_map[1][0].west = WALL;
}

void Maze::init_step_map(Point goal) {
    for (int x = 0; x < MAX_CELLS_X; x++) {
        for (int y = 0; y < MAX_CELLS_Y; y++) {
            step_map[x][y] = MAX_STEP_VALUE;
        }
    }

    step_map[goal.x][goal.y] = 0;
}

void Maze::calculate_step_map(Point goal, StepMapType type) {
    // Create a static queue for Breadth-First Search (BFS)
    Point queue[MAX_CELLS_X * MAX_CELLS_Y];
    int front = 0;
    int rear = 1;

    // Start from the goal
    queue[0] = goal;

    int8_t dx[4] = {0, 1, 0, -1};
    int8_t dy[4] = {1, 0, -1, 0};

    while (front < rear) {
        Point current_cell = queue[front];
        front++;
        int current_step = step_map[current_cell.x][current_cell.y];

        for (int d = 0; d < 4; d++) {
            Point cell_to_check = {.x = (current_cell.x + dx[d]), .y = (current_cell.y + dy[d])};

            if (cell_is_valid(cell_to_check)) {
                int wall_to_check = 0;
                if (d == NORTH) {
                    wall_to_check = wall_map[current_cell.x][current_cell.y].north;
                } else if (d == EAST) {
                    wall_to_check = wall_map[current_cell.x][current_cell.y].east;
                } else if (d == SOUTH) {
                    wall_to_check = wall_map[current_cell.x][current_cell.y].south;
                } else if (d == WEST) {
                    wall_to_check = wall_map[current_cell.x][current_cell.y].west;
                }

                // If there is no wall and the cell is unvisited, update the step count and add it to the queue
                // In search mode we can visit unknown walls, but in run mode we can't
                bool unvisited_cell = (step_map[cell_to_check.x][cell_to_check.y] == MAX_STEP_VALUE);
                bool update_step = false;
                if (type == SEARCH) {
                    update_step = (wall_to_check == NO_WALL || wall_to_check == UNKNOWN_WALL);
                } else if (type == RUN) {
                    update_step = (wall_to_check == NO_WALL);
                }

                if (update_step && unvisited_cell) {
                    step_map[cell_to_check.x][cell_to_check.y] = current_step + 1;
                    queue[rear] = cell_to_check;
                    rear++;
                }
            }
        }
    }
}

void Maze::update_wall_map(Point robot_pos, Direction robot_dir, bool front_seeing, bool right_seeing,
                           bool left_seeing) {

    int x = robot_pos.x;
    int y = robot_pos.y;

    switch (robot_dir) {
    case NORTH:

        wall_map[x][y].north = static_cast<WallInfo>(front_seeing);
        wall_map[x][y].east = static_cast<WallInfo>(right_seeing);
        wall_map[x][y].west = static_cast<WallInfo>(left_seeing);
        wall_map[x][y].south = NO_WALL; // There's never a wall behind

        break;

    case EAST:

        wall_map[x][y].east = static_cast<WallInfo>(front_seeing);
        wall_map[x][y].south = static_cast<WallInfo>(right_seeing);
        wall_map[x][y].north = static_cast<WallInfo>(left_seeing);
        wall_map[x][y].west = NO_WALL;

        break;

    case SOUTH:

        wall_map[x][y].south = static_cast<WallInfo>(front_seeing);
        wall_map[x][y].west = static_cast<WallInfo>(right_seeing);
        wall_map[x][y].east = static_cast<WallInfo>(left_seeing);
        wall_map[x][y].north = NO_WALL;

        break;

    case WEST:

        wall_map[x][y].west = static_cast<WallInfo>(front_seeing);
        wall_map[x][y].north = static_cast<WallInfo>(right_seeing);
        wall_map[x][y].south = static_cast<WallInfo>(left_seeing);
        wall_map[x][y].east = NO_WALL;

        break;

    default:
        wall_map[x][y].north = NO_WALL;
        wall_map[x][y].east = NO_WALL;
        wall_map[x][y].west = NO_WALL;
        wall_map[x][y].south = NO_WALL;
        break;
    }

    // Write the wall seen from the opposite side
    if (y < MAX_CELLS_Y - 1) {
        wall_map[x][y + 1].south = wall_map[x][y].north;
    }

    if (x < MAX_CELLS_X - 1) {
        wall_map[x + 1][y].west = wall_map[x][y].east;
    }

    if (y > 0) {
        wall_map[x][y - 1].north = wall_map[x][y].south;
    }

    if (x > 0) {
        wall_map[x - 1][y].east = wall_map[x][y].west;
    }
}

Direction Maze::get_target_cell_dir(Point robot_pos, Direction robot_dir, StepMapType type) {

    int priority = 0;
    int tmp_priority;
    int smallest = MAX_STEP_VALUE;
    Direction target_direction = robot_dir;

    // Directions and their corresponding movements
    Direction directions[] = {NORTH, EAST, SOUTH, WEST};
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    for (int d = 0; d < 4; d++) {
        Point neighbour_cell = {.x = (robot_pos.x + dx[d]), .y = (robot_pos.y + dy[d])};
        Direction neighbour_dir = directions[d];

        WallInfo wall_to_check = UNKNOWN_WALL;
        if (d == NORTH) {
            wall_to_check = wall_map[robot_pos.x][robot_pos.y].north;
        } else if (d == EAST) {
            wall_to_check = wall_map[robot_pos.x][robot_pos.y].east;
        } else if (d == SOUTH) {
            wall_to_check = wall_map[robot_pos.x][robot_pos.y].south;
        } else if (d == WEST) {
            wall_to_check = wall_map[robot_pos.x][robot_pos.y].west;
        }

        bool check_cell = false;
        if (type == SEARCH) {
            check_cell = (wall_to_check == NO_WALL || wall_to_check == UNKNOWN_WALL);
        } else if (type == RUN) {
            check_cell = (wall_to_check == NO_WALL);
        }

        if (check_cell && cell_is_valid(neighbour_cell)) {
            tmp_priority = get_cell_priority(robot_dir, neighbour_cell, neighbour_dir);
            if (step_map[neighbour_cell.x][neighbour_cell.y] < smallest) {
                smallest = step_map[neighbour_cell.x][neighbour_cell.y];
                target_direction = neighbour_dir;
                priority = tmp_priority;
            } else if (step_map[neighbour_cell.x][neighbour_cell.y] == smallest && priority < tmp_priority) {
                target_direction = neighbour_dir;
                priority = tmp_priority;
            }
        }
    }

    return target_direction;
}

Movement Maze::get_target_movement(Direction robot_dir, Direction target_dir) {
    // ((4 + target_dir - robot_dir) % 4));
    if (target_dir == robot_dir) {
        return FORWARD;
    } else if ((target_dir == NORTH && robot_dir == WEST) || (target_dir == EAST && robot_dir == NORTH) ||
               (target_dir == SOUTH && robot_dir == EAST) || (target_dir == WEST && robot_dir == SOUTH)) {
        return RIGHT;
    } else if ((target_dir == NORTH && robot_dir == SOUTH) || (target_dir == EAST && robot_dir == WEST) ||
               (target_dir == SOUTH && robot_dir == NORTH) || (target_dir == WEST && robot_dir == EAST)) {
        return BACKWARD;
    } else {
        return LEFT;
    }
}

void Maze::print_maze() {
    // Print top border
    for (int x = 0; x < MAX_CELLS_X; x++) {
        if (wall_map[x][MAX_CELLS_Y - 1].north == WALL) {
            std::printf("+---");
        } else {
            std::printf("+   ");
        }
    }

    std::printf("+\r\n");

    for (int y = (MAX_CELLS_Y - 1); y >= 0; y--) {

        // Print left border
        if (wall_map[0][y].west == WALL) {
            std::printf("|");
        } else {
            std::printf(" ");
        }

        // Print right borders
        for (int x = 0; x < MAX_CELLS_X; x++) {
            std::printf("%03d", step_map[x][y]);
            if (wall_map[x][y].east == WALL) {
                std::printf("|");
            } else {
                std::printf(" ");
            }
        }
        std::printf("\r\n");

        // Print bottom borders
        for (int x = 0; x < MAX_CELLS_X; x++) {
            if (wall_map[x][y].south == WALL) {
                std::printf("+---");
            } else {
                std::printf("+   ");
            }
        }
        std::printf("+\r\n");
    }
}

}
