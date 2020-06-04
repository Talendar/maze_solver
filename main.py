""" Simple maze solver program.

This program implements a maze solver agent that uses different search algorithms to deal with the pathfinding problem.

@Author: Gabriel Nogueira (Talendar)
"""


from maze import *
from pathlib import Path
from datetime import datetime
import os


OUT_IMG_HEIGHT = 800


def save_results(maze, dir_name):
    """ Saves the results. """
    if not os.path.isdir("./out/" + dir_name):
        Path("./out/" + dir_name).mkdir()

    with open("./out/" + dir_name + "/results.txt", "w") as file:
        file.write(str(datetime.now()))
        maze_img(maze.ascii_matrix, OUT_IMG_HEIGHT).save("./out/%s/maze.png" % dir_name)
        for p in maze.pathfinders:
            file.write(
                "\n\n\n"
                "> %s:\n" % p +
                "   . Execution time: %fms\n" % (1000*maze.solution_time(p)) +
                "   . Exit found: %s\n" % str(maze.exit_found(p)) +
                "   . Path length: %d" % maze.path_len(p)
            )
            maze_img(maze.solved_ascii(p), OUT_IMG_HEIGHT).save("./out/%s/%s.png" % (dir_name, p))


def opt0():
    """ Executes the menu option number 0 (load maze from file). """
    print("File pathname (e.g: ./data/maze1.txt):  ", end="")
    pathname = input()
    maze = maze_from_file(pathname)
    out_dir = "custom%dx%d_" % (len(maze.ascii_matrix), len(maze.ascii_matrix[0])) + f"{datetime.now():%y-%m-%d-%H-%M-%S}"
    save_results(maze, out_dir)
    print("Done. Results saved to: ./out/%s" % out_dir)


def opt1():
    """ Executes the menu option number 1 (random maze) """
    print("Dimensions of the maze (e.g: 36 36): ", end="")
    rows, cols = [int(x) for x in input().split(" ")]

    print("Number of executions per algorithm (each execution with a different maze): ", end="")
    num_exe = int(input())
    print()
    performance = {}

    for i in range(num_exe):
        print("Creating and solving maze %d/%d..." % (i+1, num_exe), end="")
        #TO_DO
        print(" done!")


if __name__ == "__main__":
    opt = None
    while opt != "2":
        print(
            "\n< MAZE SOLVER BY TALENDAR >\n"
            "   [0] Load maze from file\n"
            "   [1] Random maze\n"
            "   [2] Exit\n"
            "Option: ", end=""
        )
        opt = input()
        print()
        if opt == "0":
            opt0()
        elif opt == "1":
            opt1()


    """#maze = maze_from_file("./data/maze1.txt")
    maze = random_maze(16, 16, noise=0.1)

    maze_img(maze.ascii_matrix, 800).save("./out/imgs/maze.png")
    print("SOLUTION TIME & PATH LEN:")

    for p in maze.pathfinders:
        maze_img(maze.solved_ascii(p), 800).save("./out/imgs/%s.png" % p)
        print("   %s: %fms  |  %d" % (p, 1000*maze.solution_time(p), maze.path_len(p)))"""

