from pyamaze import maze, agent, COLOR

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
    if noise != 0:
        return policy,pSearched,path
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
mrows = 5; mcols = 5
m=maze(mrows,mcols)
# Create the maze with the given loop percentage
goal = (1,1)
# load the maze from the csv file
m.CreateMaze(goal[0],goal[1],loadMaze="mazeSmall.csv")
#m.CreateMaze(goal[0],goal[1],loopPercent=50,saveMaze=True)
# Create the agents
agentStart = (mrows,mcols)
a = agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.blue)
b=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.red)
c=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.yellow)
d=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.green)
e=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.cyan)

# Solve the maze using DFS, BFS, A* and Value Iteration
gamma = 1
theta = 0.001
reward = 100
penalty = -100
stepCost = -10
maxIterations = 100
noise = 0
#randomPolicy = {cell:random.choice(['E','W','N','S']) for cell in m.grid}
alwaysEast = {cell:'E' for cell in m.grid}
policyIterationGrid, policyIterationSearched, policyIterationPath = policyIteration(m,agentStart,goal,alwaysEast,gamma,theta,reward,penalty,stepCost,maxIterations, noise)

# Trace the path for each agent
m.tracePath({e:policyIterationPath})

# Print the results

print("Policy Iteration Grid: \ngamma = ",gamma," theta = ",theta," reward = ",reward," penalty = ",penalty," stepCost = ",stepCost," maxIterations = ",maxIterations, " noise = ",noise)
print("----------------------------------------------")
for i in range(1,mrows+1):
    for j in range(1,mcols+1):
        print(policyIterationGrid[(i,j)],end="\t")
    print()

# Run the simulation
m.run()