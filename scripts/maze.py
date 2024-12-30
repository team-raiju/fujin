import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Maze:
    def __init__(self, rows, cols):
        self.rows_size = rows
        self.cols_size = cols
        self.walls = []
        
    
    def draw_walls(self):
        fig, ax = plt.subplots(figsize=(self.rows_size, self.cols_size))
        

        plt.xlim(0, self.cols_size)
        plt.ylim(0, self.rows_size)
        ax.set_aspect('equal')

        # Draw external walls
        plt.plot([0, 0], [0, self.rows_size], 'k-', linewidth=5)
        plt.plot([0, self.cols_size], [0, 0], 'k-', linewidth=5)
        plt.plot([self.cols_size, self.cols_size], [0, self.rows_size], 'k-', linewidth=5)
        plt.plot([0, self.cols_size], [self.rows_size, self.rows_size], 'k-', linewidth=5)


        # Draw user define walls
        for row in range(self.rows_size):
            for col in range(self.cols_size):

                # North
                if (self.walls[row * self.cols_size + col][0] == 1):
                    plt.plot([col, col + 1], [row + 1, row + 1], 'k-', linewidth=3)
                
                # East
                if (self.walls[row * self.cols_size + col][1] == 1):
                    plt.plot([col + 1, col + 1], [row, row + 1], 'k-', linewidth=3)
                
                # South
                if (self.walls[row * self.cols_size + col][2] == 1):
                    plt.plot([col, col + 1], [row, row], 'k-', linewidth=3)
                
                # West
                if (self.walls[row * self.cols_size + col][3] == 1):
                    plt.plot([col, col], [row, row + 1], 'k-', linewidth=3)


    def draw_line(self, start: Point, end: Point):
        plt.plot([start.x, end.x], [start.y, end.y], 'b-', linewidth=2)
    
    def draw_arc(self, start_point: Point, radius, angle, current_direction):

        DIRECTION_MAP = {
            'NORTH': {'base_angle_1': 0, 'base_angle_2': 180, 'x_multiplier': -1, 'y_multiplier': 0},
            'SOUTH': {'base_angle_1': 180, 'base_angle_2': 0, 'x_multiplier': 1, 'y_multiplier': 0},
            'EAST': {'base_angle_1': 270, 'base_angle_2': 90, 'x_multiplier': 0, 'y_multiplier': 1},
            'WEST': {'base_angle_1': 90, 'base_angle_2': 270, 'x_multiplier': 0, 'y_multiplier': -1}
        }
        
        direction_info = DIRECTION_MAP[current_direction]
        x_offset = radius * direction_info['x_multiplier'] * (1 if angle > 0 else -1)
        y_offset = radius * direction_info['y_multiplier'] * (1 if angle > 0 else -1)
        
        center_point = Point(start_point.x + x_offset, start_point.y + y_offset)
        
        start_angle = 0
        end_angle = 0
        base_angle = 0
        if (angle > 0):
            base_angle = direction_info['base_angle_1']
            start_angle = base_angle 
            end_angle = start_angle + abs(angle)
        else:
            base_angle = direction_info['base_angle_2']
            end_angle = base_angle
            start_angle = end_angle + angle


        print("angle: ", angle)
        print(f"Start angle: {start_angle}, End angle: {end_angle}")
        
        arc = patches.Arc((center_point.x, center_point.y),
                        width=radius * 2,
                        height=radius * 2,
                        theta1=start_angle,
                        theta2=end_angle,
                        color='blue',
                        linewidth=2)
    

        

        plt.gca().add_patch(arc)

        return arc
            

    def draw_path(self):
        current_direction = 'NORTH'
        current_position = Point(0.5, 0.0)
        self.draw_line(current_position, Point(0.5, 1.0))
        current_position = Point(0.5, 1.0)
        plt.plot(current_position.x, current_position.y, 'ro', markersize=3)

        # movements = [
        #     ['FORWARD', 1], 
        #     ['RIGHT',  1],
        #     ['RIGHT', 1] ,
        #     ['FORWARD', 1], 
        #     ['LEFT',  1],
        #     ['FORWARD', 1],
        #     ['LEFT',  1],
        #     ['FORWARD', 5],
        #     ['LEFT',  1],
        #     ['LEFT', 1],
        #     ['FORWARD', 1],
        #     ['RIGHT',  1],
        #     ['RIGHT',  1],
        #     ['FORWARD', 1],
        #     ['LEFT',  1],
        # ]

        movements = [
            ['FORWARD', 1], 
            ['TURN_180_RIGHT', 1],
            ['FORWARD', 1], 
            ['TURN_90_LEFT',  1],
            ['FORWARD', 1],
            ['TURN_90_LEFT',  1],
            ['FORWARD', 5],
            ['TURN_180_LEFT', 1],
            ['FORWARD', 1],
            ['TURN_90_RIGHT',  1],
            ['TURN_90_RIGHT',  1],
            ['FORWARD', 1],
            ['TURN_90_LEFT',  1],
        ]

        MOVEMENT_MAP = {
            'NORTH': {
                'FORWARD': {'x': 0, 'y': 1, 'next_direction': 'NORTH'},
                'RIGHT': {'x1': 0.0, 'y1': 0.5, 'x2': 0.5, 'y2': 0.0, 'next_direction': 'EAST'},
                'LEFT': {'x1': 0.0, 'y1': 0.5, 'x2': -0.5, 'y2': 0.0, 'next_direction': 'WEST'},
                'TURN_180_RIGHT': {'x': 1, 'y': 0, 'next_direction': 'SOUTH', 'arc_angle': -180},
                'TURN_180_LEFT': {'x': -1, 'y': 0, 'next_direction': 'SOUTH', 'arc_angle': 180},
                'TURN_90_RIGHT': {'x': 0.5, 'y': 0.5, 'next_direction': 'EAST', 'arc_angle': -90},
                'TURN_90_LEFT': {'x': -0.5, 'y': 0.5, 'next_direction': 'WEST', 'arc_angle': 90}
            },
            'SOUTH': {
                'FORWARD': {'x': 0, 'y': -1, 'next_direction': 'SOUTH'},
                'RIGHT': {'x1': 0.0, 'y1': -0.5, 'x2': -0.5, 'y2': 0.0, 'next_direction': 'WEST'},
                'LEFT': {'x1': 0.0, 'y1': -0.5, 'x2': 0.5, 'y2': 0.0, 'next_direction': 'EAST'},
                'TURN_180_RIGHT': {'x': -1, 'y': 0, 'next_direction': 'NORTH', 'arc_angle': -180},
                'TURN_180_LEFT': {'x': 1, 'y': 0, 'next_direction': 'NORTH', 'arc_angle': 180},
                'TURN_90_RIGHT': {'x': -0.5, 'y': -0.5, 'next_direction': 'WEST', 'arc_angle': -90},
                'TURN_90_LEFT': {'x': 0.5, 'y': -0.5, 'next_direction': 'EAST', 'arc_angle': 90}
            },
            'EAST': {
                'FORWARD': {'x': 1, 'y': 0, 'next_direction': 'EAST'},
                'RIGHT': {'x1': 0.5, 'y1': 0.0, 'x2': 0.0, 'y2': -0.5, 'next_direction': 'SOUTH'},
                'LEFT': {'x1': 0.5, 'y1': 0.0, 'x2': 0.0, 'y2': 0.5, 'next_direction': 'NORTH'},
                'TURN_180_RIGHT': {'x': 0, 'y': -1, 'next_direction': 'WEST', 'arc_angle': -180},
                'TURN_180_LEFT': {'x': 0, 'y': 1, 'next_direction': 'WEST', 'arc_angle': 180},
                'TURN_90_RIGHT': {'x': 0.5, 'y': -0.5, 'next_direction': 'SOUTH', 'arc_angle': -90},
                'TURN_90_LEFT': {'x': 0.5, 'y': 0.5, 'next_direction': 'NORTH', 'arc_angle': 90}
            },
            'WEST': {
                'FORWARD': {'x': -1, 'y': 0, 'next_direction': 'WEST'},
                'RIGHT': {'x1': -0.5, 'y1': 0.0, 'x2': 0.0, 'y2': 0.5, 'next_direction': 'NORTH'},
                'LEFT': {'x1': -0.5, 'y1': 0.0, 'x2': 0.0, 'y2': -0.5, 'next_direction': 'SOUTH'},
                'TURN_180_RIGHT': {'x': 0, 'y': 1, 'next_direction': 'EAST', 'arc_angle': -180},
                'TURN_180_LEFT': {'x': 0, 'y': -1, 'next_direction': 'EAST', 'arc_angle': 180},
                'TURN_90_RIGHT': {'x': -0.5, 'y': 0.5, 'next_direction': 'NORTH', 'arc_angle': -90},
                'TURN_90_LEFT': {'x': -0.5, 'y': -0.5, 'next_direction': 'SOUTH', 'arc_angle': 90}
            }
        }

        for movement in movements:
            move_type = movement[0]
            steps = movement[1]

            movement = MOVEMENT_MAP[current_direction][move_type]
    
            if move_type == 'FORWARD':
                end_point = Point(current_position.x + movement['x'] * steps, 
                                current_position.y + movement['y'] * steps)
                self.draw_line(current_position, end_point)
                current_position.x = end_point.x
                current_position.y = end_point.y
            
            elif move_type in ['RIGHT', 'LEFT']:
                mid_point = Point(current_position.x + movement['x1'], current_position.y + movement['y1'])

                self.draw_line(current_position, mid_point)

                end_point = Point(mid_point.x + movement['x2'], mid_point.y + movement['y2'])
                self.draw_line(mid_point, end_point)
                current_position.x = end_point.x
                current_position.y = end_point.y
            
            elif move_type in ['TURN_180_RIGHT', 'TURN_180_LEFT']:
                self.draw_arc(current_position, 0.5, movement['arc_angle'], current_direction)
                current_position.x += movement['x']
                current_position.y += movement['y']
            
            elif move_type in ['TURN_90_RIGHT', 'TURN_90_LEFT']:
                self.draw_arc(current_position, 0.5, movement['arc_angle'], current_direction)
                current_position.x += movement['x']
                current_position.y += movement['y']

            current_direction = movement['next_direction']

            # Draw a point to current position
            plt.plot(current_position.x, current_position.y, 'ro', markersize=3)
                

            


def main():
    maze = Maze(rows=7, cols=4)
    
    # Wall coordinates
    # N, E, S, W
    maze.walls = [
        (0, 1, 1, 1), (0, 0, 1, 1), (1, 0, 1, 0), (0, 1, 1, 0),
        (0, 1, 0, 1), (0, 0, 0, 1), (1, 1, 1, 0), (0, 1, 0, 1),
        (1, 0, 0, 1), (0, 1, 0, 0), (0, 1, 1, 1), (0, 1, 0, 1),
        (0, 0, 1, 1), (1, 1, 0, 0), (0, 1, 0, 1), (0, 1, 0, 1),
        (0, 1, 0, 1), (0, 0, 1, 1), (0, 1, 0, 0), (0, 1, 0, 1),
        (1, 1, 0, 1), (0, 1, 0, 1), (0, 1, 0, 1), (0, 1, 0, 1), 
        (1, 0, 1, 1), (1, 1, 0, 0), (1, 0, 0, 1), (1, 1, 0, 0)

    ]
 
    maze.draw_walls()

    maze.draw_path()

    plt.title('Maze Layout')
    plt.xlabel('Column')
    plt.ylabel('Row')

    plt.show()

if __name__ == "__main__":
    main()