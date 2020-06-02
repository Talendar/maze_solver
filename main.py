""" Simple maze solver program.

This program implements a maze solver agent that uses different search algorithms to deal with the pathfinding problem.

@Author: Gabriel Nogueira (Talendar)
"""

from graphs.undirected_graph import Graph, read_graph
from graphs.search import DFS, BFS
from maze import *


if __name__ == "__main__":
    #maze = maze_from_file("./data/maze1.txt")
    maze = random_maze(128, 128)
    path = maze.find_path()

    solved = maze_insert_path(maze.ascii_matrix, path)
    maze_img(solved).show()


    """graph = read_graph()
    print(graph)
    print("\n")

    source, dest = "JFK", "LAX"
    bfs = BFS(graph, graph.indexof(source))

    print("[%s] is connected to: " % source, end="")
    for v in range(graph.num_vertices()):
        if bfs.connected_to(v):
            print(" %s " % graph.nameof(v), end="")

    print("\nPath from [%s] to [%s]: " % (source, dest)
        + str([graph.nameof(v) for v in bfs.path_to(graph.indexof(dest))]))"""

