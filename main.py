""" Simple maze solver program.

This program implements a maze solver agent that uses different search algorithms to deal with the pathfinding problem.

@Author: Gabriel Nogueira (Talendar)
"""


from maze import *
from pathlib import Path
from datetime import datetime
import os


######### CONFIG ############
OUT_IMG_HEIGHT = 800        # height of the mazes' output images; the width will be adjusted accordingly
RANDOM_MAZE_NOISE = 0.1     # percentage of "wall" cells that are going to be converted to "walkable" cells in the randomly generated maze
#############################


def opt0():
    """ Executes the menu option number 0 (load maze from file). """

    # GETTING MAZE FILE PATHNAME FROM THE USER AND LOADING THE MAZE
    print("File pathname (e.g: ./data/maze1.txt):  ", end="")
    pathname = input()
    maze = maze_from_file(pathname)

    # CREATING OUTPUT DIRECTORY
    out_dir = "./out/custom%dx%d_" % (len(maze.ascii_matrix), len(maze.ascii_matrix[0])) + f"{datetime.now():%y-%m-%d-%H-%M-%S}"
    if not os.path.isdir(out_dir):
        Path(out_dir).mkdir()

    # WRITING RESULTS AND GENERATING AND SAVING IMGS
    with open(out_dir + "/results.txt", "w") as file:
        file.write(str(datetime.now()))
        maze_img(maze.ascii_matrix, OUT_IMG_HEIGHT).save(out_dir + "/maze.png")
        for p in maze.pathfinders:
            file.write(
                "\n\n\n"
                "> %s:\n" % p +
                "   . Execution time: %fms\n" % (1000*maze.solution_time(p)) +
                "   . Exit found: %s\n" % str(maze.exit_found(p)) +
                "   . Path length: %d" % maze.path_len(p)
            )
            maze_img(maze.solved_ascii(p), OUT_IMG_HEIGHT).save(out_dir + "/%s.png" % p)
    print("Done. Results saved at: " + out_dir)


def add_results(results, key_field, value):
    """ Adds the given value to the results dictionary.

    If the key is already present in the dictionary, the value is added to the existing one. If the key is not present,
    it's created and assigned the given value.
    """
    if key_field in results:
        results[key_field] += value
    else:
        results[key_field] = value


def opt1():
    """ Executes the menu option number 1 (random maze) """

    # GETTING USER INPUT
    print("Dimensions of the maze (e.g: 36 36): ", end="")
    rows, cols = [int(x) for x in input().split(" ")]   # number of rows and columns of the maze

    print("Number of executions (each execution with a different maze): ", end="")
    num_exe = int(input())
    print("Save the generated maze images (y or n): ", end="")
    save_imgs = True if input() == "y" else False

    # CREATING OUTPUT DIRECTORY
    out_dir = "./out/random%dx%d_" % (rows, cols) + f"{datetime.now():%y-%m-%d-%H-%M-%S}"
    if not os.path.isdir(out_dir):
        Path(out_dir).mkdir()

    # GENERATING AND PROCESSING MAZES AND SAVING THEIR GENERATED IMGS
    results = {}    # stores the sum of the results achieved by the algorithms in different mazes
    maze = None
    for i in range(num_exe):
        # generating a random maze and solving it
        print("\n[%d/%d] Creating and solving maze..." % (i+1, num_exe), end="")
        maze = random_maze(rows, cols, RANDOM_MAZE_NOISE)
        print(" done!")

        # saving results and imgs
        print("[%d/%d] Generating images and saving results..." % (i+1, num_exe), end="")
        for p in maze.pathfinders:
            add_results(results, p + "_exit_found", int(maze.exit_found(p)))   # number of times that the exit was found
            add_results(results, p + "_path_len", maze.path_len(p))            # sum of the lengths of the found paths
            add_results(results, p + "_exe_time", maze.solution_time(p)*1000)  # sum of all the execution times
            if save_imgs:
                maze_img(maze.solved_ascii(p), OUT_IMG_HEIGHT).save(out_dir + "/%d_%s.png" % (i + 1, p))
        print(" done!")

    # WRITING RESULTS TO .txt FILE
    print("\nWriting results to file...", end="")
    if maze is not None:
        with open(out_dir + "/results.txt", "w") as file:
            file.write(str(datetime.now()))
            for p in maze.pathfinders:
                file.write(
                    "\n\n\n"
                    "> %s:\n" % p +
                    "   . Average execution time: %fms\n" % (results[p + "_exe_time"]/num_exe) +
                    "   . Exit found: %d/%d\n" % (results[p + "_exit_found"], num_exe) +
                    "   . Average path length: %d" % (results[p + "_path_len"]/num_exe)
                )
    print(" done!")
    print("Results and images saved at: %s" % out_dir)


""" Main body of the program. Handles the main menu. """
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
        elif opt != "2":
            print("Invalid option!")

    print("Leaving...")

