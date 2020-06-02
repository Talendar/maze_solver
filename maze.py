""" Defines the structure and operations of a maze.

This module contains the class that will represent a maze. The goal is to compare the efficiency of different
pathfinding algorithms in solving mazes.

@Author: Gabriel Nogueira (Talendar)
"""

from graphs.undirected_graph import Graph
from graphs.pathfinding import *
import numpy as np
import PIL
from PIL import Image
from random import randint


class Maze:
    """ Defines a maze.

    This class represents a maze. The maze can be initialized from a file with special characters (check below). The
    graph that represents the maze is encapsulated here. The maze will be solved by different pathfinding algorithms.

    Attributes:
        _graph: Graph that represents the maze's walkable tiles.
        _ascii_matrix: Matrix containing the ASCII representation of the maze.
        _start: Coordinates of the starting position.
        _exit: Coordinates of the exit.
    """

    def __init__(self, ascii_matrix, start, exit):
        """ Inits the maze from a matrix containing an ASCII representation of the maze.

        :param ascii_matrix: Matrix containing an ASCII representation of the maze. Each element of the matrix represents
        a tile on the maze. "#" is the starting point, "$" the exit, "*" a walkable tile and "-" a wall.
        :param start: Tuple containing the position of the starting point of the maze in the matrix.
        :param exit: Tuple containing the position of the exiting point of the maze in the matrix.
        """
        self._ascii_matrix = ascii_matrix
        self._start, self._exit = start, exit
        self._graph = ascii2graph(self._ascii_matrix)

    @property
    def ascii_matrix(self):
        """ Returns the ASCII matrix that represents the maze. """
        return self._ascii_matrix

    def find_path(self):
        """ Test. """
        dfs = DFSPath(self._graph, self._graph.indexof(str(self._start)), self._graph.indexof(str(self._exit)))
        # converting path from list of strings to list of int tuples
        return [tuple( map( int, self._graph.nameof(v).replace("(", "").replace(")", "").split(", ") ) )
                for v in dfs.path()]


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


def maze_insert_path(maze_ascii, path):
    """ Fills the walkable tiles that composes the given path on the maze.

    :param maze_ascii: The matrix with the ASCII representation of the maze.
    :param path: The path that should be inserted.
    :return: A matrix with the new ASCII representation of the maze.
    """
    solved = [row.copy() for row in maze_ascii]  # copying the maze character matrix
    for v in path[0:-1]:
        solved[v[0]][v[1]] = "@"

    return solved


def maze_img(maze_ascii):
    """ Generates an image representing the maze.

    :param maze_ascii: A matrix with the ASCII representation of the maze.
    :return: An instance of PIL.Image containing the generated image.
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


def _get_neighbours(maze, c):
    """ Returns the coordinates of the given cell's neighbours.

    A tuple with two lists is returned. The first list contains all the given cell's already explored neighbours. The
    second list contains all the given cell's unexplored neighbours.

    :param maze: Matrix with the ascii representation of the maze.
    :param c: Tuple containing the coordinates of the cell.
    :return: Tuple with two lists containing, respectively, the explored neighbours and the unexplored neighbours.
    """
    neighbours = []

    # north neighbour
    if c[0] > 0:
        neighbours.append( (c[0] - 1, c[1]) )
    # south neighbour
    if c[0] < (len(maze) - 1):
        neighbours.append((c[0] + 1, c[1]))
    # west neighbour
    if c[1] > 0:
        neighbours.append((c[0], c[1] - 1))
    # east neighbour
    if c[1] < (len(maze[0]) - 1):
        neighbours.append((c[0], c[1] + 1))

    explored, unexplored = [], []
    for n in neighbours:
        t = unexplored if maze[n[0]][n[1]] == "-" else explored
        t.append(n)

    return explored, unexplored


def random_maze(rows, cols):
    """ Generate a random maze with the given dimensions.

    This function uses an adapted version of the Prim's Algorithm. It executes the following steps:
        > Start with a grid of filled cells.
        > Pick a cell, mark it as part of the maze. Add the surrounding filled cells of the cell to the cell list.
        > While there are cells in the list:
            > Pick a random cell from the list. If the cell doesn't have 2 explored neighbours:
                > Clear the cell.
                > Add the neighbouring filled cells to the cell list.
            > Remove the cell from the list.

    :param rows: Number of rows of the maze (height).
    :param cols: Number of columns of the maze (width).
    :return: An instance of Maze representing the generated maze.
    """
    maze = [["-" for _ in range(cols)] for _ in range(rows)]  # starting the maze with walls only
    cells = []   # list of cells to explore

    # choosing a random starting point on the maze's border
    i, j = -1, -1
    if rows < cols:
        i = randint(0, rows - 1)
    elif rows > cols:
        j = randint(0, cols - 1)
    else:
        i = randint(0, rows - 1)
        j = randint(0, cols - 1) if i == 0 or i == (rows - 1) else (0, cols - 1)[randint(0, 1)]
        if randint(0, 1) == 0:
            i, j = j, i

    i = (0, rows - 1)[randint(0, 1)] if i == -1 else i
    j = (0, cols - 1)[randint(0, 1)] if j == -1 else j
    s = (i, j)

    maze[s[0]][s[1]] = "#"  # marking the cell as the starting point
    cells += _get_neighbours(maze, s)[1]

    # generating the maze's paths
    while cells:
        c = cells.pop(randint(0, len(cells) - 1))   # picking a random cell from the list
        explored_neighbours, unexplored_neighbours = _get_neighbours(maze, c)
        if len(explored_neighbours) < 2:
            maze[c[0]][c[1]] = "*"
            cells += [w for w in unexplored_neighbours if w not in cells]

    # choosing a random exit point on the maze's border and on the opposite side of the starting point
    i = 0 if s[0] == (rows - 1) else (rows - 1) if s[0] == 0 else -1
    j = 0 if s[1] == (cols - 1) else (cols - 1) if s[1] == 0 else -1

    if i == -1:
        i = randint(0, rows - 1)
        while not maze[i][j] == "*":
            i = randint(0, rows - 1)
    elif j == -1:
        j = randint(0, cols - 1)
        while not maze[i][j] == "*":
            j = randint(0, cols - 1)

    maze[i][j] = "$"  # marking the cell as the exit point
    return Maze(maze, s, (i, j))


def maze_from_file(input_file):
    """ Inits a maze from an input file.

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
    :return: An instance of Maze containing the representation of the read maze.
    """
    maze_ascii = []
    start = exit = None

    with open(input_file, "r") as file:
        r, c = [int(i) for i in file.readline().split(" ")]  # first line: number of rows and columns

        for i in range(r):
            line = file.readline().replace("\n", "")
            maze_ascii.append([])

            for j in range(c):
                maze_ascii[i].append(line[j])

                # checking for invalid symbol
                if line[j] not in ("*", "#", "$", "-"):
                    raise RuntimeError("Invalid symbol at (%d, %d)." % (i, j))

                # checking for starting point
                if line[j] == '#':
                    if start is not None:
                        raise RuntimeError("More than one starting point found: (%d, %d) and (%d, %d)."
                                           % (start[0], start[1], i, j))
                    start = (i, j)

                # checking for exit point
                elif line[j] == '$':
                    if exit is not None:
                        raise RuntimeError("More than one exit point found: (%d, %d) and (%d, %d)."
                                           % (exit[0], exit[1], i, j))
                    exit = (i, j)

    if start is None:
        raise RuntimeError("No starting point was found!")
    if exit is None:
        raise RuntimeError("No exit point was found!")

    return Maze(maze_ascii, start, exit)