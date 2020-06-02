""" API for different pathfinding algorithms on graphs.

This module implements the following pathfinding algorithms:
    Depth-First Search (DFS)
    Breadth-First Search (BFS)
    Best-First Search
    A*
    Hill Climbing
    Dijkstra's

@Author: Gabriel Nogueira (Talendar).
"""

from abc import ABC, abstractmethod
from time import time


class Pathfind(ABC):
    """ Defines the pathfinding algorithms API.

    This abstract class defines the general structure and operations of a graph pathfinding algorithm. It should be
    inherited by the classes representing those algorithms. Pathfinding algorithms build on top of graph search
    algorithms and explore routes between vertices, starting at one vertex and traversing through relationships until
    the destination has been reached. These algorithms are used to identify optimal routes through a graph.

    Attributes:
        _graph: The graph on which the search will be performed.
        _source_vertex: The index of the starting vertex of the search.
        _dest_vertex: The index of the destination vertex (the vertex that should be reached).
        _marked: Boolean array that will be used to mark visited vertices during the search.
         _edge_to: Array used to keep track of a path from each of the graph's vertex to the source vertex. This is done
            by remembering the edge v-w that takes us to each vertex w for the first time, by setting edge_to[w] to v.
            This means that v-w is the last edge on the known path from the source vertex to w.
        _pathfind_time: The time spent with the pathfinding algorithm, in seconds.
    """

    def __init__(self, graph, source_vertex: int, dest_vertex: int):
        """ Inits a Pathfind instance.

        The pathfinding algorithm is executed in this step.

        :param graph: Graph on which the search will be performed.
        :param source_vertex: Index of the starting vertex.
        :param dest_vertex: Index of the destination vertex (the vertex that to be reached).
        """
        self._graph = graph
        self._source_vertex = source_vertex
        self._dest_vertex = dest_vertex
        self._marked = [False] * graph.num_vertices()
        self._edge_to = [-1] * graph.num_vertices()

        start_time = time()
        self._pathfind(self._source_vertex)
        self._pathfind_time = time() - start_time

    def path(self):
        """ Returns, if found, the path from the source vertex to the destination vertex.

        :return: A tuple containing a sequence of indices of the vertices in the found path. If no path was found,
        returns None.
        """
        if not self._marked[self._dest_vertex]:
            return None

        path = []
        v = self._dest_vertex
        while v != self._source_vertex:
            path.insert(0, v)
            v = self._edge_to[v]

        return tuple(path)

    @property
    def pathfind_time(self):
        """ Returns the time, in seconds, spent by the pathfinding algorithm. """
        return self._pathfind_time

    @abstractmethod
    def _pathfind(self, s: int):
        """ Implementation of the pathfinding algorithm.

        By the end of execution of this method, a path to the destination vertex will be stored in the member variable
        "_path".

        :param s: Index of the starting vertex.
        :return: -
        """
        pass


class DFSPath(Pathfind):
    """ Implementation of the Depth-First Search as a pathfinding algorithm. Subclass of Pathfinding. """

    def _pathfind(self, s: int):
        """ Depth-First Search algorithm applied to pathfinding.

        Since it's focused on pathfinding, the algorithm will stop executing when the destination vertex is reached.
        Thus, it's NOT guaranteed that all the vertices connected to the source are going to be visited. The path found
        is NOT guaranteed to be the shortest one.

        :param s: Index of the starting vertex.
        :return: -
        """
        self._marked[s] = True
        if not self._marked[self._dest_vertex]:
            for v in self._graph.adj(s):
                if not self._marked[v]:
                    self._edge_to[v] = s
                    self._pathfind(v)

                    if self._marked[self._dest_vertex]:
                        break


class BFSPath(Pathfind):
    """ Implementation of the Breadth-First Search as a pathfinding algorithm. Subclass of Pathfinding. """

    def _pathfind(self, s: int):
        """ Breadth-First Search algorithm applied to pathfinding.

        Since it's focused on pathfinding, the algorithm will stop executing when the destination vertex is reached.
        Thus, it's NOT guaranteed that all the vertices connected to the source are going to be visited. The path found
        is guaranteed to be the shortest one.

        :param s: Index of the starting vertex.
        :return: -
        """
        queue = [s]
        self._marked[s] = True

        while queue:
            v = queue.pop(0)
            for w in self._graph.adj(v):
                if not self._marked[w]:
                    self._marked[w] = True
                    self._edge_to[w] = v
                    queue.append(w)

                    if w == self._dest_vertex:
                        break
            if self._marked[self._dest_vertex]:
                break
