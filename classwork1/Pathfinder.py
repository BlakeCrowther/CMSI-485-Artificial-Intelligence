'''
The Pathfinder class is responsible for finding a solution (i.e., a
sequence of actions) that takes the agent from the initial state to the
optimal goal state.

This task is done in the Pathfinder.solve method, as parameterized
by a maze pathfinding problem, and is aided by the SearchTreeNode DS.
'''

from MazeProblem import MazeProblem
from SearchTreeNode import SearchTreeNode
from queue import *
import unittest

class Pathfinder:

    # solve is parameterized by a maze pathfinding problem
    # (see MazeProblem.py and unit tests below), and will
    # return a list of actions that solves that problem. An
    # example returned list might look like:
    # ["U", "R", "R", "U"]
    def solve(problem):    # finds a path from initial to goal in maze
        queue = Queue()
        possibleMoves = []
        queue.put(SearchTreeNode(problem.initial, None, None))  # pylint: disable=no-member
        parent = None
        while not queue.empty():
            parent = queue.get()
            if problem.goalTest(parent.state):            # pylint: disable=no-member
                return Pathfinder.findPath(parent)
            possibleMoves = problem.transitions(parent.state)   # pylint: disable=no-member
            for x in possibleMoves:
                queue.put(SearchTreeNode(x[1], x[0], parent))


    def findPath(node):        # creates action path from goal back up to initial node
        solnPath = []
        while node.parent is not None:      # pylint: disable=no-member
            solnPath.append(node.action)    # pylint: disable=no-member
            node = node.parent
        return list(reversed(solnPath))     # reverses order so path is from initial to goal


class PathfinderTests(unittest.TestCase):       #Tests that pathfinder finds correct solution to maze
    def test_maze1(self):
        maze = ["XXXXX", "X..GX", "X...X", "X*..X", "XXXXX"]
        problem = MazeProblem(maze)
        soln = Pathfinder.solve(problem)
        solnTest = problem.solnTest(soln)
        self.assertTrue(solnTest[1])
        self.assertEqual(solnTest[0], 4)

    def test_maze2(self):
        maze = ["XXXXX", "XG..X", "XX..X", "X*..X", "XXXXX"]
        problem = MazeProblem(maze)
        soln = Pathfinder.solve(problem)
        solnTest = problem.solnTest(soln)
        self.assertTrue(solnTest[1])
        self.assertEqual(solnTest[0], 4)

    def test_maze3(self):
        maze = ["XXXXX", "X...X", "X...X", "X*G.X", "XXXXX"]
        problem = MazeProblem(maze)
        soln = Pathfinder.solve(problem)
        solnTest = problem.solnTest(soln)
        self.assertTrue(solnTest[1])
        self.assertEqual(solnTest[0], 1)

    def test_maze4(self):
        maze = ["XXXXX", "X.*.X", "X.G.X", "X...X", "XXXXX"]
        problem = MazeProblem(maze)
        soln = Pathfinder.solve(problem)
        solnTest = problem.solnTest(soln)
        self.assertTrue(solnTest[1])
        self.assertEqual(solnTest[0], 1)

    def test_maze5(self):
        maze = ["XXXXX", "X..GX", "X.X.X", "X.*.X", "XXXXX"]
        problem = MazeProblem(maze)
        soln = Pathfinder.solve(problem)
        solnTest = problem.solnTest(soln)
        self.assertTrue(solnTest[1])
        self.assertEqual(solnTest[0], 3)

if __name__ == '__main__':
    unittest.main()
