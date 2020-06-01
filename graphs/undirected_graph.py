""" Contains a class that implements an unweighted and undirected graph.

This module implements an API that define the fundamental graph operations. It contains only one class, called Graph,
that implements an unweighted and undirected graph.

@Author: Gabriel Nogueira (Talendar)
"""


class Graph:
    """ Implementation of an unweighted and undirected graph.

    Attributes:
        _adj_lists: Stores the adjacency lists for the graph's vertices. Each index is associated with a vertex.
        _symbol2index: Dictionary that maps the name of a vertex (string) to its index.
        _index2symbol: Maps the index of a vertex to its name (string).
    """

    def __init__(self):
        """ Inits an empty unweighted and undirected graph. """
        self._adj_lists = []
        self._symbol2index = {}
        self._index2symbol = []

    def num_vertices(self):
        """ Returns the number of vertices in the graph. """
        return len(self._adj_lists)

    def nameof(self, v):
        """ Given the index of a vertex, returns its name.

        :param v: The index of the vertex (index).
        :return: The name of the vertex (string).
        """
        return self._index2symbol[v]

    def indexof(self, v):
        """ Given a vertex's name, returns its index.

        :param v: Name of the vertex.
        :return: The index of the vertex.
        """
        return self._symbol2index[v]

    def contains(self, v):
        """ Checks whether the graph contains the given vertex.

        :param v: The vertex's index (integer) or name (string).
        :return: True if the vertex is in the graph and False otherwise.
        """
        if type(v) is int:
            return 0 <= v < len(self._adj_lists)
        else:
            return v in self._symbol2index

    def add_vertex(self, v: str):
        """ Adds a vertex to the graph.

        :param v: String containing an unique identifier for the vertex.
        :return: -
        :raises ValueError: The vertex is already present in the graph.
        """
        if self.contains(v):
            raise ValueError("The vertex is already present in the graph!")

        self._adj_lists.append([])
        self._symbol2index[v] = len(self._adj_lists) - 1
        self._index2symbol.append(v)

    def add_edge(self, v, w):
        """ Adds an edge connecting the vertices v and w.

        Use this method to add an undirected and unweighted edge connecting the vertices v and w. If either v or w is
        not yet present in the graph, this method will add it. The params v and w expects strings containing unique
        identifiers for v and w, respectively. This method doesn't check whether the two vertices are already
        neighbours, so it allows the adding of multiple edges between a same pair of vertices!

        :param v: String containing the ID (name) of a vertex (present in the graph or yet to be added).
        :param w: String containing the ID (name) of a vertex (present in the graph or yet to be added).
        :return: -
        """
        if v not in self._symbol2index:
            self.add_vertex(v)
        if w not in self._symbol2index:
            self.add_vertex(w)

        self._adj_lists[ self.indexof(v) ].append( self.indexof(w) )
        self._adj_lists[ self.indexof(w) ].append( self.indexof(v) )

    def adj(self, v):
        """ Returns the adjacency list of vertex v.

        Use this function in order to retrieve a list containing all the neighbours of the given vertex.

        :param v: The index (integer) or name (string) of the vertex.
        :return: The adjacency list of the vertex (list containing the indexes of v's neighbours). Returns an empty list
        if v has no neighbours.
        """
        if type(v) is int:
            return self._adj_lists[v]
        else:
            return self._adj_lists[ self.indexof(v) ]

    def __str__(self):
        """  Represents the graph in a string.

        Each line of the string represents a vertex followed by its adjacency list.

        :return: A string representing the graph.
        """
        lines_list = []
        for i, v in enumerate(self._adj_lists):
            lines_list.append( "> %s[%d]:  " % (self.nameof(i), i) +
                               "".join(["%s[%d]  " % (self.nameof(w), w) for w in v]) + "\n")
        return "".join(lines_list)


def read_graph():
    """ Creates a graph from the information retrieved from STDIN.

    :return: The generated graph.
    """
    graph = Graph()
    edges_count = int(input())
    for i in range(edges_count):
        v, w = input().split(" ")
        graph.add_edge(v, w)

    return graph
