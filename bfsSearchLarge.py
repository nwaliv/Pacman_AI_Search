from pyamaze import maze, agent, COLOR

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

# ----------------------- Main Program -----------------------

# Create the maze object with the given dimensions
mrows = 11; mcols = 11
m=maze(mrows,mcols)
# Create the maze with the given loop percentage
goal = (1,1)
# load the maze from the csv file
m.CreateMaze(goal[0],goal[1],loadMaze="mazeLarge.csv")
#m.CreateMaze(goal[0],goal[1],loopPercent=50,saveMaze=True)
# Create the agents
agentStart = (mrows,mcols)
b=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.red)

# Solve the maze using DFS, BFS, A* and Value Iteration
bfsPath, bfsSearched= bfs(agentStart,goal)
# Trace the path for each agent
m.tracePath({b:bfsPath})

# Print the results
print("# Nodes searched by BFS:", len(bfsSearched)+1)
print("BFS Path length:", len(bfsPath)+1)

# Run the simulation
m.run()