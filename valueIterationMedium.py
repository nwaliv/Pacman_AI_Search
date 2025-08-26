from pyamaze import maze, agent, COLOR

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

# ----------------------- Main Program -----------------------

# Create the maze object with the given dimensions
mrows = 8; mcols = 8
m=maze(mrows,mcols)
# Create the maze with the given loop percentage
goal = (1,1)
# load the maze from the csv file
m.CreateMaze(goal[0],goal[1],loadMaze="mazeMedium.csv")
#m.CreateMaze(goal[0],goal[1],loopPercent=50,saveMaze=True)
# Create the agents
agentStart = (mrows,mcols)

d=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.green)

# Solve the maze using DFS, BFS, A* and Value Iteration
gamma = 1
theta = 0.001
reward = 100
penalty = -100
stepCost = -10
maxIterations = 100
noise = 0

valueIterationSearched, valueIterationPaths, valueIterationGrid = valueIteration(m,agentStart,goal,gamma,theta,reward,penalty,stepCost,maxIterations,noise)

# Trace the path for each agent
m.tracePath({d:valueIterationPaths})

# Print the results
print("# Nodes searched by Value Iteration:", len(valueIterationSearched))
print("Value Iteration Path length:", len(valueIterationPaths))

print("Value Iteration Grid: \ngamma = ",gamma," theta = ",theta," reward = ",reward," penalty = ",penalty," stepCost = ",stepCost," maxIterations = ",maxIterations, " noise = ",noise)
print("----------------------------------------------")
for i in range(1,mrows+1):
    for j in range(1,mcols+1):
        print(round(valueIterationGrid[(i,j)],2),end="\t")
    print()

# Run the simulation
m.run()