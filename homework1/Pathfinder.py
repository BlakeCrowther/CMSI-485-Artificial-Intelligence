#Blake Crowther
#if printing final path works in test one and the rest of tests will be either reveresed or correct but still dont pass for some reason
#Have a formal tonight and dont know if more credit now or as 80% tomorrow
'''
The Pathfinder class is responsible for finding a solution (i.e., a
sequence of actions) that takes the agent from the initial state to all
of the goals with optimal cost.

This task is done in the solve method, as parameterized
by a maze pathfinding problem, and is aided by the SearchTreeNode DS.
'''

from MazeProblem import MazeProblem
from SearchTreeNode import SearchTreeNode
from queue import *
import itertools
import unittest

#Solves maze and returns actions to goals
def solve (problem, initial, goals):
    path = []
    searchStates = goals
    searchStates.insert(0, initial)
    costPathDict = {}
    for x in range(len(searchStates)):
        for y in range(x+1, len(searchStates)):
            costPathDict[x, y] = aStarSearch(problem, searchStates[x], searchStates[y])
            if costPathDict[x, y] == (None, None):
                return None
            costPathDict[y, x] = (flipPath(costPathDict[x, y][0]), costPathDict[x, y][1])
    possiblePaths = [state for state in itertools.permutations(range(0, len(searchStates))) if state[0] == 0]
    minCost = float("inf")
    minPermutation = []
    finalPath = []
    for permutations in possiblePaths:
        currentCost = 0
        for pairs in pairwise(permutations):
            currentCost += costPathDict[pairs[0], pairs[1]][1]
        if currentCost < minCost:
            minCost = currentCost
            minPermutation = permutations
    for pairs in pairwise(minPermutation):
        finalPath.extend(costPathDict[pairs[0], pairs[1]][0])
    print(finalPath)
    return finalPath

def pairwise(iterable):                         #itertools recipe from python.org
    a, b = itertools.tee(iterable)
    next(b, None)
    return list(zip(a, b))

#flips path so that going opposite directed
def flipPath(path):
    path.reverse()
    for action in path:
        if action is 'U':
            action = 'D'
        elif action is 'D':
            action = 'U'
        elif action is 'R':
            action = 'L'
        elif action is 'L':
            action = 'R'
    return path

#finds path between two states
def aStarSearch(problem, startState, endState):
    openQueue = PriorityQueue()
    graveyardList = set()
    posssibleTransitions = []
    openQueue.put(SearchTreeNode(startState, None, None, 0, findHeuristic(startState, endState)))
    while not openQueue.empty():
        current = openQueue.get()
        graveyardList.add(current.state)
        if current.state == endState:
            return (findPath(current), current.totalCost)
        possibleTransitions = problem.transitions(current.state)
        for (action, cost, state) in possibleTransitions:
            if state not in graveyardList:
                openQueue.put(SearchTreeNode(state, action, current, current.totalCost + cost, findHeuristic(state, endState)))
    return (None, None)

#finds heuristic cost between two states
def findHeuristic(currentState, endState):
    heuristic = abs(currentState[0] - endState[0] + currentState[1] - endState[1])
    return heuristic

#finds path from end goal to head
def findPath(endNode):
    path = [endNode.action]
    while endNode.parent.action is not None:
        endNode = endNode.parent
        path.append(endNode.action)
    path.reverse()
    return path



class PathfinderTests(unittest.TestCase):

    def test_maze1(self):
        maze = ["XXXXXXX",
                "X.....X",
                "X.M.M.X",
                "X.X.X.X",
                "XXXXXXX"]
        problem = MazeProblem(maze)
        initial = (1, 3)
        goals   = [(5, 3)]
        soln = solve(problem, initial, goals)
        (soln_cost, is_soln) = problem.soln_test(soln, initial, goals)
        self.assertTrue(is_soln)
        self.assertEqual(soln_cost, 8)

    def test_maze2(self):
        maze = ["XXXXXXX",
                "X.....X",
                "X.M.M.X",
                "X.X.X.X",
                "XXXXXXX"]
        problem = MazeProblem(maze)
        initial = (1, 3)
        goals   = [(3, 3),(5, 3)]
        soln = solve(problem, initial, goals)
        (soln_cost, is_soln) = problem.soln_test(soln, initial, goals)
        self.assertTrue(is_soln)
        self.assertEqual(soln_cost, 12)

    def test_maze3(self):
        maze = ["XXXXXXX",
                "X.....X",
                "X.M.MMX",
                "X...M.X",
                "XXXXXXX"]
        problem = MazeProblem(maze)
        initial = (5, 1)
        goals   = [(5, 3), (1, 3), (1, 1)]
        soln = solve(problem, initial, goals)
        (soln_cost, is_soln) = problem.soln_test(soln, initial, goals)
        self.assertTrue(is_soln)
        self.assertEqual(soln_cost, 12)

    def test_maze4(self):
        maze = ["XXXXXXX",
                "X.....X",
                "X.M.XXX",
                "X.X.X.X",
                "XXXXXXX"]
        problem = MazeProblem(maze)
        initial = (5, 1)
        goals   = [(1, 3), (1, 1)]
        soln = solve(problem, initial, goals)
        self.assertTrue(soln == None)


if __name__ == '__main__':
    unittest.main()
