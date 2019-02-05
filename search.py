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
import searchAgents

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

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm."""
    
    visited = []    #Set up the initial state for dfs
    path = {}
    stack = util.Stack()
    hold = util.Stack()
    state = problem.getStartState()
    stack.push(state)
    
    while not stack.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
        curState = stack.pop()
        if problem.isGoalState(curState):   #If the current state is the goal state, we can exit and move on to generating the path
            break;
        if curState in visited: #Because this is graph search, if we have already visited this state, we just continue
            continue
        visited.append(curState)    #If we haven't visited it, mark it as visited
        
        successors = problem.getSuccessors(curState)
        for node in successors:
            if node[0] not in visited:
                path[node[0]] = [curState, node[1]] #Mark down which state we were in before reaching this one and which action was taken to reach it
                hold.push(node[0])  #Push the next state into a secondary stack to make sure it goes into the main stack in the right order
                
        while not hold.isEmpty():   #For dfs, we need to visit nodes in the order we got them from the getSuccessors function. To do that, we need to reverse the order they are pushed into the main stack. That is what hold is for
            stack.push(hold.pop())
            
    ans = []
    while curState != problem.getStartState():  #Until we reach the starting state, push actions into the beginning of the answer
        ans.insert(0, path[curState][1])
        curState = path[curState][0]
    return ans
    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    
    visited = []    #Set up the initial state for dfs
    path = {}
    queue = util.Queue()
    state = problem.getStartState()
    queue.push(state)
    visited.append(state)
    
    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
        curState = queue.pop()
        if problem.isGoalState(curState):   #If the current state is the goal state, we can exit and move on to generating the path
            break;
        
        successors = problem.getSuccessors(curState)
        for node in successors:
            if node[0] not in visited:
                path[node[0]] = [curState, node[1]] #Mark down which state we were in before reaching this one and which action was taken to reach it
                queue.push(node[0])
                visited.append(node[0])    #If we haven't visited it, mark it as visited
            
    ans = []
    while curState != problem.getStartState():  #Until we reach the starting state, push actions into the beginning of the answer
        ans.insert(0, path[curState][1])
        curState = path[curState][0]
    return ans

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    visited = []    #Set up the initial state for dfs
    path = {}
    queue = util.PriorityQueue()
    state = problem.getStartState()
    queue.push([state, 0], 0)
    
    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
        curState = queue.pop()
        if problem.isGoalState(curState[0]):   #If the current state is the goal state, we can exit and move on to generating the path
            break;
        #if curState in visited: #Because this is graph search, if we have already visited this state, we just continue
         #   continue
        if curState[0] not in visited:
            visited.append(curState[0])    #If we haven't visited it, mark it as visited
            #print curState
            
            successors = problem.getSuccessors(curState[0])
            for node in successors:
                if node[0] not in visited:
                    if (node[0] not in path) or (path[node[0]][2] > curState[1] + node[2]):
                        path[node[0]] = [curState[0], node[1], curState[1] + node[2]] #Mark down which state we were in before reaching this one and which action was taken to reach it
                    queue.push([node[0], curState[1] + node[2]], curState[1] + node[2])
            
    ans = []
    curState = curState[0]
    while curState != problem.getStartState():  #Until we reach the starting state, push actions into the beginning of the answer
        ans.insert(0, path[curState][1])
        curState = path[curState][0]
    return ans

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    visited = []    #Set up the initial state for dfs
    path = {}
    queue = util.PriorityQueue()
    state = problem.getStartState()
    queue.push([state, 0], 0)
    
    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
        curState = queue.pop()
        if problem.isGoalState(curState[0]):   #If the current state is the goal state, we can exit and move on to generating the path
            break;
        #if curState in visited: #Because this is graph search, if we have already visited this state, we just continue
         #   continue
        if curState[0] not in visited:
            visited.append(curState[0])    #If we haven't visited it, mark it as visited
            #print curState
            
            successors = problem.getSuccessors(curState[0])
            for node in successors:
                if node[0] not in visited:
                    if (node[0] not in path) or (path[node[0]][2] > (curState[1] + node[2])):
                        path[node[0]] = [curState[0], node[1], curState[1] + node[2]] #Mark down which state we were in before reaching this one and which action was taken to reach it
                    queue.push([node[0], curState[1] + node[2]], curState[1] + node[2] + heuristic(node[0], problem))
            
    ans = []
    curState = curState[0]
    while curState != problem.getStartState():  #Until we reach the starting state, push actions into the beginning of the answer
        ans.insert(0, path[curState][1])
        curState = path[curState][0]
    return ans

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
