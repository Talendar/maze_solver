""" Defines the structure and operations of a maze.

This module contains the class that will represent a maze. The goal is to compare the efficiency of different
pathfinding algorithms in solving mazes.

@Author: Gabriel Nogueira (Talendar)
"""

from graphs.undirected_graph import Graph
from graphs.search import DFS, BFS
import numpy as np
import PIL
from PIL import Image


class Maze:
    """ Defines a maze.

    This class represents a maze. The maze can be initialized from a file with special characters (check below). The
    graph that represents the maze is encapsulated here. The maze will be solved by different pathfinding algorithms.

    Attributes:
        _graph: Graph that represents the maze's walkable tiles.
        _maze_ascii: Matrix containing the ASCII representation of the maze.
        _start: Coordinates of the starting position.
        _exit: Coordinates of the exit.
    """

    def __init__(self, input_file):
        """ Inits the maze from an input file.

        :param input_file: Txt file that contains the ASCII representation of the maze. The first line should contain
        the dimensions of the maze. The remaining lines should contain the structure of the maze. The character "*"
        represents a walkable space, "#" the starting point, "$" the exit and "-" an obstacle. Check the example below:

            29 26
            **#*********--************
            *----*-----*--*-----*----*
            *----*-----*--*-----*----*
            *----*-----*--*-----*----*
            **************************
            -----*--*--------*--*----*
            -----*--*--------*--*----*
            ******--****--****--******
            -----*-----*--*-----*-----
            -----*-----*--*-----*-----
            -----*--**********--*-----
            -----*--*--------*--*-----
            -----*--*--------*--*-----
            *********--------*********
            -----*--*--------*--*-----
            -----*--*--------*--*-----
            -----*--**********--*-----
            -----*--*--------*--*-----
            -----*--*--------*--*-----
            ************--************
            *----*-----*--*-----*----*
            *----*-----*--*-----*----*
            ***--****************--***
            --*--*--*--------*--*--*--
            --*--*--*--------*--*--*--
            ******--****--****--******
            *----------*--*----------*
            *----------*--*----------*
            **********************$***
        """
        self._maze_ascii = []
        self._start = self._exit = None

        # reading the file
        with open(input_file, "r") as file:
            r, c = [int(i) for i in file.readline().split(" ")]  # first line: number of rows and columns

            for i in range(r):
                line = file.readline().replace("\n", "")
                self._maze_ascii.append([])

                for j in range(c):
                    self._maze_ascii[i].append(line[j])

                    # checking for invalid symbol
                    if line[j] not in ("*", "#", "$", "-"):
                        raise RuntimeError("Invalid symbol at (%d, %d)." % (i, j))

                    # checking for starting point
                    if line[j] == '#':
                        if self._start is not None:
                            raise RuntimeError("More than one starting point found: (%d, %d) and (%d, %d)."
                                               % (self._start[0], self._start[1], i, j))
                        self._start = (i, j)

                    # checking for exit point
                    elif line[j] == '$':
                        if self._exit is not None:
                            raise RuntimeError("More than one exit point found: (%d, %d) and (%d, %d)."
                                               % (self._exit[0], self._exit[1], i, j))
                        self._exit = (i, j)

        if self._start is None:
            raise RuntimeError("Starting point not found!")
        if self._exit is None:
            raise RuntimeError("Exit point not found!")

        # creating the graph
        self._graph = ascii2graph(self._maze_ascii)

    def find_path(self):
        """ Test. """
        bfs = BFS(self._graph, self._graph.indexof(str(self._start)))
        # converting path from list of strings to list of int tuples
        return [tuple( map( int, self._graph.nameof(v).replace("(", "").replace(")", "").split(", ") ) )
                for v in bfs.path_to(self._graph.indexof(str(self._exit)))]


def ascii2graph(maze_ascii):
    """ Creates a graph from a maze's ascii representation.

    This function reads a maze's ascii matrix and uses it to create an undirected and unweighted graph. Each walkable
    position in the maze becomes a vertex in the graph. The vertices IDs will be the coordinates of the corresponding
    tile in the maze's matrix.

    :param maze_ascii: Matrix containing the characters that represents the maze.
    :return: An undirected and unweighted graph in which each vertex represents a walkable tile in the maze.
    """
    graph = Graph()
    walkable = ("*", "#", "$")

    for r in range(len(maze_ascii)):
        for c in range(len(maze_ascii[r])):
            if maze_ascii[r][c] in walkable:
                v = str((r, c))

                # right neighbour
                if c < (len(maze_ascii[r]) - 1) and maze_ascii[r][c+1] in walkable:
                    w = str((r, c + 1))
                    graph.add_edge(v, w)
                # bottom neighbour
                if r < (len(maze_ascii) - 1) and maze_ascii[r+1][c] in walkable:
                    w = str((r + 1, c))
                    graph.add_edge(v, w)

    return graph


def solved_maze(maze_ascii, path):
    """
    to_do

    :param maze_ascii:
    :param path:
    :return:
    """
    solved = [row.copy() for row in maze_ascii]  # copying the maze character matrix
    for v in path[0:-1]:
        solved[v[0]][v[1]] = "@"

    return solved


def maze_img(maze_ascii):
    """
    to_do

    :param maze_ascii:
    :return:
    """
    imgs_names = ["walkable.png", "wall.png", "start.png", "exit.png", "path.png"]
    walkable, wall, start, exit, path = [np.asarray(PIL.Image.open("./data/imgs/" + i)) for i in imgs_names]
    char2img = {"*": walkable, "-": wall, "#": start, "$": exit, "@": path}

    comb = []
    for row in maze_ascii:
        comb.append([])
        for c in row:
            comb[-1].append(char2img[c])
        comb[-1] = np.hstack(comb[-1])
    return PIL.Image.fromarray(np.vstack(comb))