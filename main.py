""" Simple maze solver program.

This program implements a maze solver agent that uses different search algorithms to deal with the pathfinding problem.

@Author: Gabriel Nogueira (Talendar)
"""

from graphs.undirected_graph import Graph, read_graph
from graphs.search import DFS, BFS
from maze import Maze, solved_maze, maze_img


if __name__ == "__main__":
    maze = Maze("./data/maze1.txt")
    path = maze.find_path()

    solved = solved_maze(maze._maze_ascii, path)
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

