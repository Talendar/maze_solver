""" API for different searching algorithms on graphs.

This module implements the following graphs searching algorithms:
    Depth-First Search (DFS)
    Breadth-First Search (BFS)

@Author: Gabriel Nogueira (Talendar).
"""

from abc import ABC, abstractmethod
from time import time


class Search(ABC):
    """ Defines the searching algorithms API.

    This abstract class defines the general structure and operations of a graph searching algorithm. It should be
    inherited by the classes representing those algorithms. Graph search algorithms explore a graph either for general
    discovery or explicit search. These algorithms carve paths through the graph, but there is no expectation that those
    paths are computationally optimal.

    Attributes:
        _graph: The graph on which the search will be performed.
        _source_vertex: The index of the starting vertex of the search.
        _marked: Boolean array that will be used to mark visited vertices during the search.
        _edge_to: Array used to keep track of a path from each of the graph's vertex to the source vertex. This is done
            by remembering the edge v-w that takes us to each vertex w for the first time, by setting edge_to[w] to v.
            This means that v-w is the last edge on the known path from the source vertex to w.
        _search_time: The time spent with the search algorithm, in seconds.
    """

    def __init__(self, graph, source_vertex: int):
        """ Inits a Search instance.

        The searching algorithm is executed in this step.

        :param graph: Graph on which the search will be performed.
        :param source_vertex: Index of the starting vertex of the search.
        """
        self._graph = graph
        self._source_vertex = source_vertex
        self._marked = [False for _ in range(graph.num_vertices())]
        self._edge_to = [-1 for _ in range(graph.num_vertices())]

        start_time = time()
        self._search(self._source_vertex)
        self._search_time = time() - start_time

    @property
    def search_time(self):
        """ Returns the time, in seconds, spent with the search operation. """
        return self._search_time

    def connected_to(self, v: int):
        """ Checks whether the vertex v is connected to the source vertex.

        :param v: The index of the vertex that will be checked for connectivity with the source vertex.
        :return: True if v is connected to the source vertex and False otherwise.
        """
        return self._marked[v]

    @abstractmethod
    def _search(self, s: int):
        """ Implementation of the search algorithm.

        At the end of the execution of this method, the member variable "_marked" will be marked with all the vertices
        visited during the search.

        :param s: The index of the starting vertex.
        :return: -
        """
        pass

    def path_to(self, v: int):
        """ Tries to find a path connecting v to the source vertex.

        :param v: The index of the vertex to which the path will be sought.
        :return: A tuple containing the indices of the vertices that forms the path from the source vertex to v. If no
        such path exists, returns None.
        """
        if not self.connected_to(v):
            return None

        path = []
        w = v
        while w != self._source_vertex:
            path.insert(0, w)
            w = self._edge_to[w]

        return tuple(path)


class DFS(Search):
    """ Implementation of the Depth-First Search algorithm.

    Subclass of the Search class. It's meant to explore all the vertices connected to the source, so it's not optimal
    for pathfinding purposes.
    """

    def _search(self, s: int):
        """ Depth-First Search algorithm.

        Explores all the vertices connected to the source vertex. For each visited vertex, the edge that lead there is
        remembered, so that it's possible to determine the path used by the algorithm from the source to that vertex.
        That path is NOT guaranteed to be the shortest one.
        """
        self._marked[s] = True
        for v in self._graph.adj(s):
            if not self._marked[v]:
                self._edge_to[v] = s
                self._search(v)


class BFS(Search):
    """ Implementation of the Breadth-First Search algorithm.

    Subclass of the Search class. It's meant to explore all the vertices connected to the source, so it's not optimal
    for pathfinding purposes.
    """

    def _search(self, s: int):
        """ BFS algorithm

        Explores all the vertices connected to the source vertex. For each visited vertex, the edge that lead there is
        remembered, so that it's possible to determine the path used by the algorithm from the source to that vertex. In
        the case of BFS, that path is guaranteed to be the shortest one.
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
