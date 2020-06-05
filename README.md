# Maze Solver

This program, written in Python as part of the course [SCC0530 - Artificial Intelligence](https://uspdigital.usp.br/jupiterweb/obterDisciplina?nomdis=&sgldis=SCC0530), implements different graph search algorithms and analyzes them in the context of maze solving. They are the following:

  - Depth-First Search
  - Breadth-First Search
  - Greedy Best-First Search
  - A* (A-star)
  - Hill Climbing

The mazes are modeled with undirected unweighted graphs. The ADT was implemented for general use, so its use is not restricted to this program. The mazes can either be provided in a file as an input from the user (check the documentations for more information) or be randomly generated, in which case a variation of the [Prim's algorithm](https://en.wikipedia.org/wiki/Prim%27s_algorithm) is used. 
