# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
import copy

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def goalTest(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
        Given a state, returns available actions.
        Returns a list of actions
        """        
        util.raiseNotDefined()

    def getResult(self, state, action):
        """
        Given a state and an action, returns resulting state.
        """
        util.raiseNotDefined()

    def getCost(self, state, action):
        """
        Given a state and an action, returns step cost, which is the incremental cost 
        of moving to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

class Node:
    """
    Search node object for your convenience.

    This object uses the state of the node to compare equality and for its hash function,
    so you can use it in things like sets and priority queues if you want those structures
    to use the state for comparison.

    Example usage:
    >>> S = Node("Start", None, None, 0)
    >>> A1 = Node("A", S, "Up", 4)
    >>> B1 = Node("B", S, "Down", 3)
    >>> B2 = Node("B", A1, "Left", 6)
    >>> B1 == B2
    True
    >>> A1 == B2
    False
    >>> node_list1 = [B1, B2]
    >>> B1 in node_list1
    True
    >>> A1 in node_list1
    False
    """
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __hash__(self):
        return hash(self.state)

    def __eq__(self, other):
        return self.state == other.state

    def __ne__(self, other):
        return self.state != other.state


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    You are not required to implement this, but you may find it useful for Q5.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

"""
Function that calls the recursive DLS function with default inputs
"""
def depthLimitedSearch(problem, limit):
    return recursiveDepthLimitedSearch(Node(problem.getStartState(), None, None, 0), problem, limit, [])

"""
Recursive function that builds the graph from choices made using the Node data structure from our selected path.
"""
def recursiveDepthLimitedSearch(currentNode, problem, limit, nodeList):
    if problem.goalTest(currentNode.state):
        # if we reached the goal then return the list of actions needed to get to this state
        position, path, totalCost = currentNode, [], 0
        while position.parent:
            path.insert(0, position.action)
            totalCost += position.path_cost
            position = position.parent
        return path
    elif limit == 0:
        # we didn't find a path within this limit so return the cutoff val
        return []
    else:
        # otherwise iterate through all possible actions from currentNode
        cutoffOcurred = False
        for action in problem.getActions(currentNode.state):
            child = Node(problem.getResult(currentNode.state, action), currentNode, action, problem.getCost(currentNode.state, action))
            if child not in nodeList:
                nodeList.append(child)
                result = recursiveDepthLimitedSearch(child, problem, limit - 1, nodeList)
                if result == []:
                    cutoffOcurred = True
                elif result != -1:
                    return result
        if cutoffOcurred:
            return []
        else:
            return -1
            
"""
Iterative Deepending Search function that increases the depth limit until a result is found
"""          
def iterativeDeepeningSearch(problem):
    for limit in range(1000000):
        result = depthLimitedSearch(problem, limit)
        if result != []:
            return result
    

"""
Function that finds pacman solutions using the A* search method
"""
def aStarSearch(problem, heuristic=nullHeuristic):
    # the nodes we've expanded
    visited = []

    # all nodes we could possibly expand
    frontier = util.PriorityQueue()
    
    # every node is added here as soon as it is created so we know when we don't need to expand it
    seenNodes = []

    # creating the starting node
    curr = Node(problem.getStartState(), None, None, 0)
    visited.append(curr)
    
    while problem.goalTest(curr.state) == False:
        # create a node for each possible action we could expand from our current position
        actions = problem.getActions(curr.state)
        for action in actions:
            n = Node(problem.getResult(curr.state, action), curr, action, problem.getCost(curr.state, action))      
            # if we haven't seen this node, calculate its f value and add/update it in the priority queue
            if n not in seenNodes:
                f = n.path_cost + heuristic(n.state, problem)
                frontier.update(n, f)
                seenNodes.append(n)
        # pop the val with the lowest f from the frontier set
        curr = frontier.pop()
        visited.append(curr)
        
    # trace our path back from the end using the final current node
    path = []
    while curr.parent:
        path.insert(0, curr.action)
        curr = curr.parent
    
    return path

# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
