from pyamaze import maze, agent, COLOR

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

# ----------------------- Main Program -----------------------

# Create the maze object with the given dimensions
mrows = 8; mcols = 8
m=maze(mrows,mcols)
goal = (1,1)
# load the maze from the csv file
m.CreateMaze(goal[0],goal[1],loadMaze="mazeMedium.csv")
#m.CreateMaze(goal[0],goal[1],loopPercent=50,saveMaze=False)
# Create the agents
agentStart = (mrows,mcols)
a = agent(m,agentStart[0],agentStart[1], filled=True,footprints=True, color=COLOR.blue)
b = agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.red)

# Solve the maze using DFS, BFS, A* and Value Iteration
dfsPath, dfsSearched = dfs(agentStart,goal)

# Trace the path for each agent
m.tracePath({a:dfsSearched})
m.tracePath({b:dfsPath})

# Print the results
print("# Nodes searched by DFS:", len(dfsSearched)+1)
print("DFS Path length:", len(dfsPath)+1)

# Run the simulation
m.run()