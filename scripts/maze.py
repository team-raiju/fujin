import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Maze:
    def __init__(self, rows, cols):
        self.rows_size = rows
        self.cols_size = cols
        self.walls = []
        
    
    def display(self):
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

        
        plt.title('Maze Layout')
        plt.xlabel('Column')
        plt.ylabel('Row')
        
        
        # Show the plot
        plt.show()

def add_blue_line(self, start, end):
    plt.plot([start[0], end[0]], [start[1], end[1]], 'b-', linewidth=2)


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
    
    
    # Display the maze
    maze.display()

if __name__ == "__main__":
    main()