
# Attribution Information: The AI Pacman projects by Group 27 
# ( Naveen D, Rithesh Reddy G and Alisha K) were implemented at UCR (Riverside)

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

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
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #states to be explored (LIFO). holds nodes in form (state, action)
    startNode = problem.getStartState()
    if problem.isGoalState(startNode):
        return []
    queue = util.Stack()
    #previously explored states (for path checking), holds states
    exploredNodes = []
    #define start node
    queue.push((startNode,[]))
    while not queue.isEmpty():
        #begin exploring last (most-recently-pushed) node on queue
        currentNode, actions = queue.pop()
        if currentNode not in exploredNodes:
            #mark current node as explored
            exploredNodes.append(currentNode)
            if problem.isGoalState(currentNode):
                return actions
            #get list of possible successor nodes in form (successor, action, stepCost)
            #push each successor to queue
            for nextNode, action, cost in problem.getSuccessors(currentNode):
                if nextNode in exploredNodes:
                    continue
                queue.push((nextNode, actions + [action]))

    util.raiseNotDefined()

def breadthFirstSearch(prob):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    startNode = prob.getStartState()
    if prob.isGoalState(startNode):
        return []
    queue = util.Queue()
    #previously explored states (for path checking), holds states
    exploredNodes = []
    queue.push((startNode, []))

    while not queue.isEmpty():
        #begin exploring last (most-recently-pushed) node on queue
        currentNode, actions = queue.pop()
        if currentNode not in exploredNodes:
            #mark current node as explored
            exploredNodes.append(currentNode)
            if prob.isGoalState(currentNode):
                return actions
            #get list of possible successor nodes in form (successor, action, stepCost)
            #push each successor to queue
            for succState, succAction, succCost in prob.getSuccessors(currentNode):
                if succState in exploredNodes:
                    continue
                queue.push((succState, actions + [succAction]))
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #to be explored (FIFO): holds (item, cost)
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    exploredNodes = []
    frontier = util.PriorityQueue()
    #previously expanded states (for cycle checking), holds state:cost
    frontier.push((startState, [], 0), 0)
    
    while not frontier.isEmpty():
        #begin exploring first (lowest-cost) node on frontier
        currentState, actions, currentCost = frontier.pop()
        if (currentState not in exploredNodes) :
            #put popped node's state into explored list
            exploredNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
                #list of (successor, action, stepCost)
            
            for succState, succAction, succCost in problem.getSuccessors(currentState):
                newAction = actions + [succAction]
                newCost = currentCost + succCost
                frontier.update((succState, newAction, newCost), newCost)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***" 
    #to be explored (FIFO): takes in item, cost+heuristic
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []
    exploredNodes = [] #holds (state, cost)
    frontier = util.PriorityQueue()
    frontier.push((startState, [], 0), 0)
    while not frontier.isEmpty():
        #begin exploring first (lowest-combined (cost+heuristic) ) node on frontier
        currentState, actions, currentCost = frontier.pop()
        if currentState not in exploredNodes:
            #put popped node into explored list
            exploredNodes.append(currentState)
            if problem.isGoalState(currentState):
                return actions
            #examine each successor
            for succState, succAction, succCost in problem.getSuccessors(currentState):
                newAction = actions + [succAction]
                newCostToNode = currentCost + succCost
                heuristicCost = newCostToNode + heuristic(succState, problem)
                frontier.push((succState, newAction, newCostToNode), heuristicCost)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
