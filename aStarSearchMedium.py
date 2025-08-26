from pyamaze import maze, agent, COLOR
from queue import PriorityQueue

# Heuristic estimate
def manhattanDistance(cell,goal):
    x1,y1 = cell
    x2,y2 = goal
    return abs(y2-y1)+abs(x2-x1)

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
a=agent(m,agentStart[0],agentStart[1], filled=True,footprints=True, color=COLOR.blue)
b=agent(m,agentStart[0],agentStart[1], filled=False,footprints=True, color=COLOR.yellow)

# Solve the maze using DFS, BFS, A* and Value Iteration

aStarPath, aStarSearched = aStar(m,agentStart,goal)

# Trace the path for each agent
m.tracePath({a:aStarSearched})
m.tracePath({b:aStarPath})

# Print the results
print("# Nodes searched by A*:", len(aStarSearched)+1)
print("A* Path length:", len(aStarPath)+1)


# Run the simulation
m.run()