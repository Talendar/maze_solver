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
from timeit import default_timer as timer


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
        :raise ValueError: When the source vertex and the destination vertex have the same value.
        """
        if source_vertex == dest_vertex:
            raise ValueError("The source vertex's index can't be equal to the destination vertex's index!")

        self._graph = graph
        self._source_vertex = source_vertex
        self._dest_vertex = dest_vertex
        self._marked = [False] * graph.num_vertices()
        self._edge_to = [-1] * graph.num_vertices()

        start_time = timer()
        self._pathfind(self._source_vertex)
        self._pathfind_time = timer() - start_time

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

    def marked_list(self):
        """ Returns a list with the indices of the vertices visited during the search. """
        return [i for i in range(len(self._marked)) if self._marked[i] and i != self._source_vertex and i != self._dest_vertex]

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
        stack = [s]
        self._marked[s] = True

        while stack:
            v = stack.pop()
            for w in self._graph.adj(v):
                if not self._marked[w]:
                    self._marked[w] = True
                    self._edge_to[w] = v

                    if w == self._dest_vertex:
                        return
                    stack.append(w)


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

                    if w == self._dest_vertex:
                        return
                    queue.append(w)


class BestFirstSearch(Pathfind):
    """ Implementation of the Greedy Best-First Search algorithm.

        Attributes:
            ...
            _cost_to: Cost to get to the given vertex from the source vertex.
    """

    def __init__(self, graph, source_vertex: int, dest_vertex: int, heuristic):
        """ Inits an instance of BestFirstSearch.

        Wrapper for the super class constructor. Used to solve a pathfinding problem with the greedy version of the
        Best-First Search algorithm. The pathfinding algorithm is executed in this step.

        :param graph: Graph on which the search will be performed.
        :param source_vertex: Index of the starting vertex.
        :param dest_vertex: Index of the destination vertex (the vertex that to be reached).
        :param heuristic: Heuristic function to be used by the algorithm. Gives the estimated cost of the cheapest path
        from the state at the given node to a goal state. It's expected to receive a graph and the index of two of its
        vertices as argument and to return some comparable information about them (how far apart they are, for instance).
        """
        self._h = heuristic
        self._cost_to = [-1 for _ in range(graph.num_vertices())]
        super().__init__(graph, source_vertex, dest_vertex)

    def _f(self, v):
        """ Evaluation function  used by the Greedy Best-First Search algorithm.

        During the execution of the algorithm, a node is selected for expansion based on this evaluation function. It's
        constructed as a cost estimate, so the node with the lowest evaluation is expanded first. In the context of the
        Greedy Best-First Search, the evaluation function is computed by using just the heuristic function.

        :param v: Index of the node to be evaluated.
        :return: A cost estimate.
        """
        return self._h(self._graph, v, self._dest_vertex)

    def _insert_sorted(self, frontier, v):
        """ Inserts a new vertex into the frontier list while keeping it sorted.

        :param frontier: List with vertices.
        :param v: Vertex to be added.
        :return: -
        """
        f_v = self._f(v)
        for i, w in enumerate(frontier):
            if f_v <= w[1]:
                frontier.insert(i, (v, f_v))
                return
        frontier.append((v, f_v))

    def _pathfind(self, s: int):
        """ Greedy Best-First Search algorithm applied to pathfinding on an unweighted graph.

        :param s: Index of the starting vertex.
        :return: -
        """
        frontier = [(s, self._f(s))]
        self._marked[s] = True
        self._cost_to[s] = 0

        while frontier:
            v = frontier.pop(0)[0]  # getting the index of the vertex with the lowest value of h in the frontier list
            if v == self._dest_vertex:
                return

            for w in self._graph.adj(v):
                if not self._marked[w]:
                    self._marked[w] = True
                    self._edge_to[w] = v
                    self._cost_to[w] = self._cost_to[v] + 1
                    self._insert_sorted(frontier, w)


class AStar(BestFirstSearch):
    """ Implementation of the A* pathfinding algorithm. Subclass of BestFirstSearch."""

    def __init__(self, graph, source_vertex: int, dest_vertex: int, heuristic, g):
        """ Inits an instance of AStar, used to solve a pathfinding problem with the A* algorithm.

        Wrapper for the super class constructor. The pathfinding algorithm is executed in this step. The main difference
        between A* and Greedy Best-First Search is that A*'s evaluation function uses, besides the heuristic function,
        an auxiliary function g, that gives the path cost from the start node to the node currently being evaluated.

        :param graph: Graph on which the search will be performed.
        :param source_vertex: Index of the starting vertex.
        :param dest_vertex: Index of the destination vertex (the vertex that to be reached).
        :param heuristic: Heuristic function to be used by the algorithm. Gives the estimated cost of the cheapest path
        from the state at the given node to a goal state. It's expected to receive a graph and the index of two of its
        vertices as argument and to return the path cost to reach the second node (the search destination node) from the
        first (the node that's current being evaluated).
        :param g: The function g(n) gives the path cost from the start node to node n. It's expected to receive a graph,
        the index of two of its vertices and the length of the path found linking the two vertices. It's expected to
        return the path cost to reach the second node (the node that's currently being evaluated) from the first (the
        search's starting node).
        """
        self._g = g
        super().__init__(graph, source_vertex, dest_vertex, heuristic)

    def _f(self, v):
        """ Evaluation function used by the A* algorithm.

        During the execution of the algorithm, a node is selected for expansion based on this evaluation function. It's
        constructed as a cost estimate, so the node with the lowest evaluation is expanded first. In the context of the
        A* algorithm, we have f(v) = g(v) + h(v).

        :param v: Index of the node to be evaluated.
        :return: A cost estimate.
        """
        return self._h(self._graph, v, self._dest_vertex) + self._g(self._graph, self._source_vertex, v, self._cost_to[v])


class HillClimbing(Pathfind):
    """ Implementation of the Hill Climbing algorithm as a pathfinder. """

    def __init__(self, graph, source_vertex: int, dest_vertex: int, heuristic):
        """ Inits an instance of HillClimbing, used to solve a pathfinding problem with the Hill Climbing algorithm.

        Wrapper for the super class constructor. The pathfinding algorithm is executed in this step.

        :param graph: Graph on which the search will be performed.
        :param source_vertex: Index of the starting vertex.
        :param dest_vertex: Index of the destination vertex (the vertex that to be reached).
        :param heuristic: Heuristic function to be used by the algorithm. It's expected to receive a graph and the index
        of two of its vertices as argument and to return some comparable information about them (how far apart they are,
        for instance).
        """
        self._h = heuristic
        self._recorded_path = []
        super().__init__(graph, source_vertex, dest_vertex)

    def path(self):
        """ Returns a list with the indices of the vertices in the path walked by the algorithm. """
        return self._recorded_path[:]

    def _pathfind(self, s: int):
        """ Executes the Hill Climbing algorithm.

        This algorithm might get stuck into a local maxima/minima, in which case it won't be able to find a path to the
        goal. In this case, only the path the algorithm used to reach the ending vertex is saved.

        :param s: Index of the starting vertex.
        :return: -
        """
        v, v_h = s, self._h(self._graph, s, self._dest_vertex)

        while v != self._dest_vertex:
            n, n_h = v, v_h
            self._recorded_path.append(v)

            for w in self._graph.adj(v):
                w_h = self._h(self._graph, w, self._dest_vertex)
                if w_h <= n_h:
                    if w == self._dest_vertex:
                        return
                    n = w
                    n_h = w_h

            if n != v:
                self._edge_to[n] = v
                v = n
                v_h = n_h
            else:
                break
