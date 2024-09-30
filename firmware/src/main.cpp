#include <print>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/leds.hpp"
#include "bsp/timers.hpp"
#include "fsm/fsm.hpp"
#include "fsm/state.hpp"
#include "services/maze.hpp"

void startup() {
    bsp::buzzer::set_frequency(2500);
    bsp::buzzer::set_volume(1);
    bsp::buzzer::start();

    for (int i = 0; i < 10; i++) {
        bsp::leds::indication_toggle();
        bsp::delay_ms(100);
    }

    bsp::leds::indication_off();
    bsp::buzzer::stop();
    bsp::analog_sensors::start();
    bsp::ble::init();
    bsp::ble::start();
}

using Walls::E;
using Walls::N;
using Walls::S;
using Walls::W;

// clang-format off
// PC TESTING
uint8_t mock_maze[16][16] = {
    {0|E|S|W, 0|E|0|W, N|0|0|W, 0|0|S|W, 0|0|0|W, 0|0|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|0|0|W, 0|0|0|W, N|0|0|W, 0|0|S|W, 0|E|0|W, N|E|0|W},
    {0|E|S|W, 0|E|0|W, 0|0|0|0, N|E|0|0, N|0|S|0, N|0|S|0, 0|0|S|W, 0|E|0|W, 0|0|0|W, N|0|0|W, N|0|S|0, N|0|S|0, 0|E|S|0, 0|0|0|0, 0|E|0|W, N|E|0|W},
    {0|E|S|W, 0|0|0|W, 0|E|0|0, 0|E|0|W, N|E|0|0, N|0|S|0, 0|0|S|0, N|0|0|W, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, 0|E|0|W, 0|E|0|0, 0|0|0|W, N|E|0|W},
    {0|0|S|W, 0|E|0|0, 0|E|0|W, 0|E|0|W, 0|E|0|W, N|E|0|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|0, N|0|0|W},
    {N|0|S|0, 0|0|S|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, N|E|0|0, N|0|S|0, N|0|S|0, 0|E|S|0, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, N|0|0|W, N|0|S|0},
    {N|0|S|0, N|0|S|0, 0|0|S|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, N|E|0|0, 0|E|S|0, 0|0|0|W, 0|E|0|W, 0|0|0|W, N|0|0|W, N|0|S|W, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, 0|0|S|W, N|0|0|W, 0|0|S|W, 0|0|0|W, N|E|0|W, 0|E|S|W, 0|0|0|0, N|0|0|W, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|0|S|W, N|0|0|W, N|E|S|0, 0|0|S|0, N|0|0|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, 0|E|0|0, N|0|0|W, N|E|S|0, 0|0|S|0, 0|0|0|0, N|E|0|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, 0|0|0|W, 0|0|0|W, N|E|0|0, 0|E|S|W, N|E|0|0, 0|E|S|0, N|0|0|W, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|0|S|W, N|0|0|0, N|0|S|0, 0|0|S|W, N|0|0|W, 0|0|S|W, N|0|0|W, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, N|0|0|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|0|S|W, 0|0|0|0, N|0|0|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0, 0|E|S|0, N|E|0|0, 0|E|S|0, N|E|0|0, N|0|S|0, N|0|S|0, N|0|S|0, N|0|S|0},
    {N|0|S|0, N|0|S|0, 0|E|S|0, N|E|0|0, 0|0|S|0, 0|E|0|0, 0|E|0|0, N|0|0|0, 0|E|S|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|0, N|E|0|0, N|0|S|0, N|0|S|0},
    {0|E|S|0, N|E|0|0, 0|E|S|W, 0|E|0|W, 0|E|0|0, 0|E|0|W, 0|E|0|W, N|E|0|0, 0|E|S|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|W, 0|E|0|0, N|E|0|0},
};
// clang-format on

int main() {
    auto maze = services::Maze::instance();

    Point pos = {0, 0};

    // Print mock maze
    // for (int x = 0; x < 16; x++)
    //     for (int y = 0; y < 16; y++)
    //         maze->next_step({x, y}, mock_maze[x][y]);

    // algorithm::flood_fill(maze->map, services::Maze::GOAL_POS);
    // std::println("");
    // maze->print(pos);
    for (int i = 0; i < 2; i++) {
        bool returning = false;
        for (;;) {
            auto dir = maze->next_step(pos, mock_maze[pos.x][pos.y], returning);

            std::printf("\033c");
            maze->print(pos);
            std::println("");

            switch (dir) {
            case NORTH:
                pos.y += 1;
                break;
            case EAST:
                pos.x += 1;
                break;
            case SOUTH:
                pos.y -= 1;
                break;
            case WEST:
                pos.x -= 1;
                break;
            }

            // return
            if (pos == services::Maze::GOAL_POS) {
                returning = true;
            }

            if (returning && pos == services::Maze::ORIGIN) {
                break;
            }

            getchar();
        }
    }

    // Run version would be this
    algorithm::flood_fill(maze->map, services::Maze::GOAL_POS);
    std::println("");
    maze->print(pos);

    return 0;

    bsp::init();

    startup();

    fsm::FSM fsm;

    fsm.start();

    for (;;) {
        fsm.spin();
    }
}
