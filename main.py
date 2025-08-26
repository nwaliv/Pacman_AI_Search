from pyamaze import maze, agent, COLOR
from queue import PriorityQueue
# Breadth First Search - uninformed search 
def bfs(start,goal):
    # Initialise the list of visited nodes
    visited = [start]
    # Initiliase the queue
    queue = [start]
    dSearched=[]
    # Initiliase the dict; which stores the parent cell for each cell
    revPath = {}

    # Loop while the queue is not empty
    while queue is not []:
        # FIFO, enable first in cell as the current cell
        currentCell = queue.pop(0)
        dSearched.append(currentCell)
        # Stop when the current cell is the goal
        if currentCell == goal:
            break
        # determine possible next cells by the walls of the maze
        for direction in 'ESNW':
            # if the wall is not present for a direction, the next cell in that direction is valid
            if m.maze_map[currentCell][direction] == True:
                # Choose the next cell
                if direction=='E':
                    nextCell = (currentCell[0],currentCell[1]+1)
                elif direction=='W':
                    nextCell = (currentCell[0],currentCell[1]-1)
                elif direction=='N':
                    nextCell = (currentCell[0]-1,currentCell[1])
                elif direction=='S':
                    nextCell = (currentCell[0]+1,currentCell[1])

                # add neighbouring cell to queue and visited list if not already visited    
                if nextCell not in visited:
                    visited.append(nextCell)
                    queue.append(nextCell)
                    # add the parent cell to the path with the nieghbouring cell as its key
                    revPath[nextCell]=currentCell
    # reversing the path so that currentCell is the key to the nextCell on path                
    fwdPath={}
    # start swap from end goal
    cell = goal

    while cell != start:
        # swaps so that current cell points to next cell on path
        fwdPath[revPath[cell]] = cell
        # assigns new next cell for reversed path
        cell = revPath[cell]
    # return the next cell to move to for all cell states using BFS
    print("BFS Complete")
    return fwdPath, dSearched

# Depth First Search - uninformed search
def dfs(start,goal):
    # Initialise the list of visited nodes
    visited = [start]
    # Initiliase the stack
    stack = [start]

    # Initiliase the dict; which stores the parent cell for each cell
    revPath = {}

    # List of cells searched
    dSearched=[]

    # Loop while the stack is not empty
    while stack is not []:
        # LIFO, enable last in cell as the current cell
        currentCell = stack.pop()
        # Add to the list of cells searched by DFS
        dSearched.append(currentCell)
        # Stop when the current cell is the goal
        if currentCell == goal:
            break
        # determine possible next cells by the walls of the maze
        for direction in 'ESNW':
            # if the wall is not present for a direction, the next cell in that direction is valid
            if m.maze_map[currentCell][direction] == True:
                # Choose the next cell
                if direction=='E':
                    nextCell = (currentCell[0],currentCell[1]+1)
                elif direction=='W':
                    nextCell = (currentCell[0],currentCell[1]-1)
                elif direction=='N':
                    nextCell = (currentCell[0]-1,currentCell[1])
                elif direction=='S':
                    nextCell = (currentCell[0]+1,currentCell[1])

                # add neighbouring cell to stack and visited list if not already visited    
                if nextCell not in visited:
                    visited.append(nextCell)
                    stack.append(nextCell)
                    # add the parent cell to the path with the neighbouring cell as its key
                    revPath[nextCell]=currentCell
    # reversing the path so that currentCell is the key to the nextCell on path                
    fwdPath={}
    # start swap from end goal
    cell = goal

    while cell != start:
        # swaps so that current cell points to next cell on path
        fwdPath[revPath[cell]] = cell
        # assigns new next cell for reversed path
        cell = revPath[cell]
    # return the next cell to move to for all cell states using DFS
    print("DFS Complete")
    return fwdPath, dSearched

# Heuristic estimate
def manhattanDistance(cell,goal):
    x1,y1 = cell
    x2,y2 = goal
    return abs(y2-y1)+abs(x2-x1)

# Another heuristic estimate
def euclideanDistance(cell,goal):
    x1,y1 = cell
    x2,y2 = goal
    return ((y2-y1)**2+(x2-x1)**2)**0.5

# A-Star - informed search
def aStar(m,start, goal):
    # Initiliase the grid with infinite g(n)
    g_score = {cell:float('inf') for cell in m.grid}
    g_score[start] = 0
    f_score = {cell:float('inf') for cell in m.grid}
    f_score[start] = manhattanDistance(start,goal)
    # Initiliase the priority queue
    queue = PriorityQueue()
    queue.put((f_score[start],start))
    # Initiliase the dict; which stores the parent cell for each cell
    revPath = {}
    # List of cells searched
    dSearched=[]
    # Loop while the queue is not empty
    while not queue.empty():
        # FIFO, enable first in cell as the current cell
        currentCell = queue.get()[1]
        # Add to the list of cells searched by A*
        dSearched.append(currentCell)
        # Stop when the current cell is the goal
        if currentCell == goal:
            break
        # determine possible next cells by the walls of the maze
        for direction in 'ESNW':
            if m.maze_map[currentCell][direction] == True:
                # Choose the next cell
                if direction=='E':
                    nextCell = (currentCell[0],currentCell[1]+1)
                elif direction=='W':
                    nextCell = (currentCell[0],currentCell[1]-1)
                elif direction=='N':
                    nextCell = (currentCell[0]-1,currentCell[1])
                elif direction=='S':
                    nextCell = (currentCell[0]+1,currentCell[1])

                # Calculate the g(n) for the next cell
                g_score_next = g_score[currentCell] + 1
                f_score_next = g_score_next + manhattanDistance(nextCell,goal)
                # If the next cell has a lower f(n) than the previous f(n) for that cell, update the f(n) and add to the queue
                if f_score_next < f_score[nextCell]:
                    revPath[nextCell]=currentCell
                    g_score[nextCell] = g_score_next
                    f_score[nextCell] = f_score_next
                    queue.put((f_score_next,nextCell))
    # reversing the path so that currentCell is the key to the nextCell on path
    fwdPath={}
    # start swap from end goal
    cell = goal
    while cell != start:
        # swaps so that current cell points to next cell on path
        fwdPath[revPath[cell]] = cell
        # assigns new next cell for reversed path
        cell = revPath[cell]
    # return the next cell to move to for all cell states using A*
    print("A* Complete")
    return fwdPath, dSearched

# A function to solve the maze using value iteration
def valueIteration(m,start,goal,gamma=0.9,theta=0.0001,reward=100,penalty=-100,stepCost=-1,maxIterations=100,noise=0.2):
    # Initialise the grid with 0
    grid = {cell:0 for cell in m.grid}
    # Initialise the list of cells searched by value iteration
    dSearched=[]
    # Loop for the maximum number of iterations
    for _iter in range(maxIterations):
        # Initialise the delta
        delta = 0
        # Loop through all the cells in the grid
        for cell in grid:
            # Initialise the value of the cell
            value = grid[cell]
            # Initialise the list of possible next cells of order East, West, North, South
            nextCells = [(cell[0],cell[1]+1),(cell[0],cell[1]-1),(cell[0]-1,cell[1]),(cell[0]+1,cell[1])]
            # Initialise the dict of possible next cells of order East, West, North, South
            nextCellsDict = {
                nextCells[0]:'E',
                nextCells[1]:'W',
                nextCells[2]:'N',
                nextCells[3]:'S'
            }     
            # Initialise the list of current values of the next cells
            nextValues = []
            # Loop through all the possible next cells
            for nextCell in nextCells:
                # If the next cell is the goal, add the reward to the list of next values
                if nextCell == goal:
                    nextValues.append(reward)
                # If the next cell is a wall, add the penalty to the list of next values
                elif m.maze_map[cell][nextCellsDict[nextCell]] == False:
                    nextValues.append(penalty)
                # If the next cell is neither the goal nor a wall, add the value of the next cell to the list of next values
                else:
                    nextValues.append(grid[nextCell])

            # Calculate the possible new values of the cell from taking each action before the max
            newValueEast = (1-noise) * (stepCost + gamma*nextValues[0]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[1] + nextValues[2] + nextValues[3])))
            newValueWest = (1-noise) * (stepCost + gamma*nextValues[1]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[2] + nextValues[3])))
            newValueNorth = (1-noise) * (stepCost + gamma*nextValues[2]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[3])))
            newValueSouth = (1-noise) * (stepCost + gamma*nextValues[3]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[2])))
            # Calculate the new value of the cell
            newValue = max(newValueEast, newValueWest, newValueNorth, newValueSouth)
            # Update the value of the cell
            grid[cell] = reward if cell == goal else newValue
            # Update the delta
            delta = max(delta,abs(value-newValue))
            # Add the current cell to the list of cells searched by value iteration
            dSearched.append(cell)
        # Stop if the delta is less than the convergence threshold
        if delta < theta:
            print("Value Iteration converged after",_iter+1,"iterations")
            break
    print("Value Iteration Complete")
    # Initialise the list of cells on the optimal path
    dPath = []
    if noise > 0:
        print("Noise is greater than 0, so the optimal path will not be found")
        return dSearched, dPath, grid
    # Initialise the current cell
    currentCell = start
    # Loop until the current cell is the goal
    while currentCell != goal:
        # Add the current cell to the list of cells on the optimal path
        dPath.append(currentCell)
        # Initialise the list of possible next cells of order East, West, North, South
        nextCells = [(currentCell[0],currentCell[1]+1),(currentCell[0],currentCell[1]-1),(currentCell[0]-1,currentCell[1]),(currentCell[0]+1,currentCell[1])]

        # Initialise the dict of possible next cells of order East, West, North, South
        nextCellsDict = {
            nextCells[0]:'E',
            nextCells[1]:'W',
            nextCells[2]:'N',
            nextCells[3]:'S'
        }

        # Initialise the list of current values of the next cells
        nextValues = []
        # Loop through all the possible next cells
        for nextCell in nextCells:
            # If the next cell is the goal, add the reward to the list of next values
            if nextCell == goal:
                nextValues.append(reward)
            # If the next cell is a wall, add the penalty to the list of next values
            elif m.maze_map[currentCell][nextCellsDict[nextCell]] == False:
                nextValues.append(penalty)
            # If the next cell is neither the goal nor a wall, add the value of the next cell to the list of next values
            else:
                nextValues.append(grid[nextCell])

        # Find the index of the maximum value
        maxIndex = nextValues.index(max(nextValues))

        if m.maze_map[currentCell][nextCellsDict[nextCells[maxIndex]]] == False:
            print("An action taken by the agent was invalid")
           
        # Update the current cell
        currentCell = nextCells[maxIndex]
    # Add the goal to the list of cells on the optimal path
    dPath.append(goal)
    # Return the list of cells searched by value iteration, the list of cells on the optimal path and the grid
    return dSearched, dPath, grid

# This is the policy iteration function
def policyIteration(m,start,goal,policy,gamma=0.9,theta=0.0001,reward=100,penalty=-100,stepCost=-1,maxIterations=100, noise=0.2):
    # Initialise the grid with 0
    grid = {cell:0 for cell in m.grid}
    # Initialise the list of cells searched by policy iteration
    pSearched=[]
    # Loop for the maximum number of iterations
    for i in range(maxIterations):
        # This is the policy evaluation step
        # Initialise the delta
        delta = 0
        # Loop through all the cells in the grid
        for cell in grid:
            # Initialise the value of the cell
            value = grid[cell]
            # Initialise the action for the cell from the policy
            action = policy[cell]
            # Initialise the list of possible next cells of order East, West, North, South
            nextCells = [(cell[0],cell[1]+1),(cell[0],cell[1]-1),(cell[0]-1,cell[1]),(cell[0]+1,cell[1])]
            # Initialise the dict of possible next cells of order East, West, North, South
            nextCellsDict = {
                nextCells[0]:'E',
                nextCells[1]:'W',
                nextCells[2]:'N',
                nextCells[3]:'S'
            }
            # Initialise the list of current values of the next cells
            nextValues = []
            # Loop through all the possible next cells
            for nextCell in nextCells:
                # If the next cell is the goal, add the reward to the list of next values
                if nextCell == goal:
                    nextValues.append(reward)
                # If the next cell is a wall, add the penalty to the list of next values
                elif m.maze_map[cell][nextCellsDict[nextCell]] == False:
                    nextValues.append(penalty)
                # If the next cell is neither the goal nor a wall, add the value of the next cell to the list of next values
                else:
                    nextValues.append(grid[nextCell])    
            # Choose the next cell
            if action=='E':
                newValue = (1-noise) * (stepCost + gamma*nextValues[0]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[1] + nextValues[2] + nextValues[3])))               
            elif action=='W':
                newValue = (1-noise) * (stepCost + gamma*nextValues[1]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[2] + nextValues[3])))                
            elif action=='N':
                newValue = (1-noise) * (stepCost + gamma*nextValues[2]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[3])))                
            elif action=='S':
                newValue = (1-noise) * (stepCost + gamma*nextValues[3]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[2])))             
            else:
                print("Error: Invalid action")
                return
            # Calculate the value of the cell
            grid[cell] = reward if cell == goal else penalty if m.maze_map[cell][action] == False else newValue
            # Calculate the delta which is the maximum change in value for all cells
            delta = max(delta,abs(value-grid[cell]))
            # Add the current cell to the list of cells searched by policy iteration
            pSearched.append(cell)
        # If the delta is less than the convergence threshold, stop
        if delta < theta:
            print("Policy Iteration converged after",i+1,"iterations")
            break
        # This is the policy improvement step
        # Initialise the policy stable flag
        policyStable = True
        # Loop through all the cells in the grid
        for cell in grid:
            # Initialise the old action
            oldAction = policy[cell]
            # Initialise the list of possible next cells of order East, West, North, South
            nextCells = [(cell[0],cell[1]+1),(cell[0],cell[1]-1),(cell[0]-1,cell[1]),(cell[0]+1,cell[1])]
            # Initialise the dict of possible next cells of order East, West, North, South
            nextCellsDict = {
                nextCells[0]:'E',
                nextCells[1]:'W',
                nextCells[2]:'N',
                nextCells[3]:'S'
            }
            # Initialise the list of current values of the next cells
            nextValues = []
            # Loop through all the possible next cells
            for nextCell in nextCells:
                # If the next cell is the goal, add the reward to the list of next values
                if nextCell == goal:
                    nextValues.append(reward)
                # If the next cell is a wall, add the penalty to the list of next values
                elif m.maze_map[cell][nextCellsDict[nextCell]] == False:
                    nextValues.append(penalty)
                # If the next cell is neither the goal nor a wall, add the value of the next cell to the list of next values
                else:
                    nextValues.append(grid[nextCell])

            newValueEast = (1-noise) * (stepCost + gamma*nextValues[0]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[1] + nextValues[2] + nextValues[3])))
            newValueWest = (1-noise) * (stepCost + gamma*nextValues[1]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[2] + nextValues[3])))
            newValueNorth = (1-noise) * (stepCost + gamma*nextValues[2]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[3])))
            newValueSouth = (1-noise) * (stepCost + gamma*nextValues[3]) + (noise/3.0) * (3 * stepCost + (gamma * (nextValues[0] + nextValues[1] + nextValues[2])))

            # Calculate the new value of the cell
            newValue = max(newValueEast, newValueWest, newValueNorth, newValueSouth)
            # Choose the next action with the highest value
            actions = ['E','W','N','S']
            if newValue == newValueEast:
                policy[cell] = 'E'
            elif newValue == newValueWest:
                policy[cell] = 'W'
            elif newValue == newValueNorth:
                policy[cell] = 'N'
            elif newValue == newValueSouth:
                policy[cell] = 'S'
            else:
                print("Error: Invalid action")
                return
            # If the new action is different from the old action, set the policy stable flag to False
            if policy[cell] != oldAction:
                policyStable = False
        # If the policy is stable, stop
        if policyStable == True:
            print("Policy Iteration converged after",i+1,"iterations")
            break
    print("Policy Iteration complete")
    # Initialise the list of cells on the optimal path
    path = []
    if noise > 0:
        print("Since noise is greater the optimal path will not be found")
        return policy, pSearched, path
    # Initialise the current cell
    cell = start
    # Loop until the goal cell is reached
    while cell != goal:
        # Add the current cell to the list of cells on the optimal path
        path.append(cell)
        # Initialise the action for the cell from the policy
        action = policy[cell]
        # Initialise the next cell
        nextCell = ()
        # Choose the next cell
        if action=='E':
            nextCell = (cell[0],cell[1]+1)
        elif action=='W':
            nextCell = (cell[0],cell[1]-1)
        elif action=='N':
            nextCell = (cell[0]-1,cell[1])
        elif action=='S':
            nextCell = (cell[0]+1,cell[1])
        # Set the current cell to the next cell
        cell = nextCell
    # Add the goal cell to the list of cells on the optimal path
    path.append(goal)
    # Return the next cell to move to for all cell states, the list of cells searched by policy iteration and the list of cells on the optimal path
    return policy,pSearched,path

# ----------------------- Main Program -----------------------
# Create the maze object with the given dimensions
mazeSize = input("Enter 1 for small maze, 2 for medium maze, 3 for large maze: ")
goal = (1,1)
while True:
    if mazeSize == "1":
        mrows = 5; mcols = 5
        m=maze(mrows,mcols)
        m.CreateMaze(goal[0],goal[1],loadMaze="mazeSmall.csv")
        break
    elif mazeSize == "2":
        mrows = 8; mcols = 8
        m=maze(mrows,mcols)
        m.CreateMaze(goal[0],goal[1],loadMaze="mazeMedium.csv")
        break
    elif mazeSize == "3":
        mrows = 11; mcols = 11
        m=maze(mrows,mcols)
        m.CreateMaze(goal[0],goal[1],loadMaze="mazeLarge.csv")
        break
    else:
        mazeSize = input("Invalid input. Enter 1 for small maze, 2 for medium maze, 3 for large maze: ")

# Create the agents
agentStart = (mrows,mcols)
a = agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.blue)
b=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.red)
c=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.yellow)
d=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.green)
e=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.cyan)

# Solve the maze using DFS, BFS, A* and Value Iteration
dfsPath, dfsSearched = dfs(agentStart,goal)
bfsPath, bfsSearched= bfs(agentStart,goal)
aStarPath, aStarSearched = aStar(m,agentStart,goal)
gamma = 1
theta = 0.001
reward = 100
penalty = -100
stepCost = -10
maxIterations = 100
noise = 0
#randomPolicy = {cell:random.choice(['E','W','N','S']) for cell in m.grid}
alwaysEast = {cell:'E' for cell in m.grid}
valueIterationSearched, valueIterationPaths, valueIterationGrid = valueIteration(m,agentStart,goal,gamma,theta,reward,penalty,stepCost,maxIterations,noise)
policyIterationGrid, policyIterationSearched, policyIterationPath = policyIteration(m,agentStart,goal,alwaysEast,gamma,theta,reward,penalty,stepCost,maxIterations, noise)

# Trace the path for each agent
m.tracePath({a:dfsPath})
m.tracePath({b:bfsPath})
m.tracePath({c:aStarPath})
m.tracePath({d:valueIterationPaths})
m.tracePath({e:policyIterationPath})

# Print the results
print("Comparing DFS, BFS, A* and Value Iteration")
print("----------------------------------------------")
print("# Nodes searched by DFS:", len(dfsSearched)+1)
print("DFS Path length:", len(dfsPath)+1)
print("----------------------------------------------")
print("# Nodes searched by BFS:", len(bfsSearched)+1)
print("BFS Path length:", len(bfsPath)+1)
print("----------------------------------------------")
print("# Nodes searched by A*:", len(aStarSearched)+1)
print("A* Path length:", len(aStarPath)+1)
print("----------------------------------------------")
print("# Nodes searched by Value Iteration:", len(valueIterationSearched))
print("Value Iteration Path length:", len(valueIterationPaths))
print("----------------------------------------------")
print("# Nodes searched by Policy Iteration:", len(policyIterationSearched))
print("Policy Iteration Path length:", len(policyIterationPath))
print("----------------------------------------------")

print("Value Iteration Grid: \ngamma = ",gamma," theta = ",theta," reward = ",reward," penalty = ",penalty," stepCost = ",stepCost," maxIterations = ",maxIterations, " noise = ",noise)
print("----------------------------------------------")
for i in range(1,mrows+1):
    for j in range(1,mcols+1):
        print(round(valueIterationGrid[(i,j)],2),end="\t")
    print()

print("Policy Iteration Grid: \ngamma = ",gamma," theta = ",theta," reward = ",reward," penalty = ",penalty," stepCost = ",stepCost," maxIterations = ",maxIterations, " noise = ",noise)
print("----------------------------------------------")
for i in range(1,mrows+1):
    for j in range(1,mcols+1):
        print(policyIterationGrid[(i,j)],end="\t")
    print()

# Run the simulation
m.run()