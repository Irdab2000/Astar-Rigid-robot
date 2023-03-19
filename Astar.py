#!/usr/bin/python3
import math
import heapq
from obstacle_gen import obs_coord
import pygame
import numpy as np
# Define the robot radius, clearance, tolerance, and goal tolerance
ROBOT_RADIUS = 5
OBSTACLE_CLEARANCE = 5
VISITED_TOLERANCE = 0.5
ORIENTATION_TOLERANCE = 30
GOAL_TOLERANCE = 1.5
MAP_WIDTH = 599
MAP_HEIGHT = 249
heap = []   #Open list
visited = np.zeros((2*MAP_HEIGHT,2*MAP_WIDTH,12)) #Closed list

class Node:
    def __init__(self):
        self.state = None
        self.parent = None
        self.cost_to_come = float('inf')
        self.cost_to_goal = float('inf')
       
    def __lt__(self, other):
        return (self.cost_to_goal + self.cost_to_come) < (other.cost_to_goal + other.cost_to_come)

#Checking if given coordinates is in any obstacle
def is_obstacle(x,y):
    ind=obs_coord(x,y)
    if(ind == 1):
        print("On obstacle")
        return True
    else:
        ind1 = obs_coord(x+ROBOT_RADIUS,y)
        ind2 = obs_coord(x,y+ROBOT_RADIUS)
        ind3 = obs_coord(x-ROBOT_RADIUS,y)
        ind4 = obs_coord(x,y-ROBOT_RADIUS)
        if(ind1 or ind2 or ind3 or ind4 == True):
            return True
        else:
            return False

# Define a function to check if a point has been visited
def is_visited(node, visited):
    
    if visited[int(node.state[0]*2)][int(node.state[1]*2)][int(node.state[2]/30)]==1:
        return True
    return False

# Define a function to check if the goal has been reached
def reached_goal(node, goal):
    dx = node.state[0] - goal[0]
    dy = node.state[1] - goal[1]
    dist_to_goal = math.sqrt(dx**2 + dy**2)
    if(node.state[2] == goal[2]):
        return (dist_to_goal <= GOAL_TOLERANCE)
    else:
        return False
# Define get_cost function
def get_cost(current_node, neighbor_node):
    return math.sqrt((current_node.x - neighbor_node.x)**2 + (current_node.y - neighbor_node.y)**2)


# Define get_heuristic function
def get_heuristic(node, goal_node):
    tolerance = 1.5
    
    dx = node.x - goal_node.x
    dy = node.y - goal_node.y
    dtheta = abs(node.theta - goal_node.theta)
    
    if dtheta > math.pi:
        dtheta = 2 * math.pi - dtheta
    
    return math.sqrt(dx**2 + dy**2 + dtheta**2) + tolerance

# Define is_valid_node function
def is_valid_node(node):
    if(node.state[0]> MAP_WIDTH or node.state[1]>MAP_HEIGHT):
        return False
    else:
        if(is_obstacle(node.state[0],node.state[1])):
            return False
        return True

def normalize(val):
    return (val*2)/2

def move_forward(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])))
    new_y = normalize(node.state[1] + step*math.sin(math.radians(node.state[2])))
    new_theta  = node.state[2]
    if new_theta<0:
        new_theta +=360
    new_theta %= 360 
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move forward")

def turn_left_30(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/6))
    new_y = normalize(node.state[1] + step*math.sin(math.radians(node.state[2])+math.pi/6))
    new_theta  = node.state[2] + 30
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 30 degrees to the left")

def turn_right_30(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/6))
    new_y = normalize(node.state[1] - step*math.sin(math.radians(node.state[2])+math.pi/6))
    new_theta  = node.state[2] - 30
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 30 degrees to the right")

def turn_left_60(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/3))
    new_y = normalize(node.state[1] + step*math.sin(math.radians(node.state[2])+math.pi/3))
    new_theta  = node.state[2] + 60
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 60 degrees to the left")

def turn_right_60(node,goal,step):
    new_x = normalize(node.state[0] + step*math.cos(math.radians(node.state[2])+math.pi/3))
    new_y = normalize(node.state[1] - step*math.sin(math.radians(node.state[2])+math.pi/3))
    new_theta  = node.state[2] - 60
    if new_theta<0:
        new_theta +=360
    new_theta %= 360
    new_cost_to_come = node.cost_to_come + step
    new_cost_to_goal = math.sqrt((new_x - goal[0])**2 +(new_y - goal[1])**2)
    new_node = Node()
    new_node.state = [new_x,new_y,new_theta]
    new_node.parent = node
    new_node.cost_to_come = new_cost_to_come
    new_node.cost_to_goal = new_cost_to_goal
    if is_valid_node(new_node):
        heapq.heappush(heap, (new_node))
    else:
        print("Cannot move 60 degrees to the right")

def backtrack(node):
    path = []
    path.append(node.state)
    print(node.state)
    while node.parent is not None:
        node = node.parent
        path.append(node.state)
        print(node.state)
    print("Done backtracking")
    visualize(path)

def astar(start, goal,step):
    heapq.heappush(heap, (start))
    while heap:
        curr_node = heapq.heappop(heap)
        print(curr_node.state)
        if is_valid_node(curr_node):
        
            if reached_goal(curr_node, goal):
                print("Goal reached, Backtracking")
                backtrack(curr_node)
                break                           #Backtracking
        
            if is_visited(curr_node, visited):
                continue                            #Checking if node is already visited
        
            visited[int(curr_node.state[0]*2)][int(curr_node.state[1]*2)][int(curr_node.state[2]/30)] = 1
        
            move_forward(curr_node,goal,step)
            turn_left_30(curr_node,goal,step)
            turn_right_30(curr_node,goal,step)
            turn_left_60(curr_node,goal,step)
            turn_right_60(curr_node,goal,step)
    print("Couldn't find a path")       
    return None

def visualize(path_gen): #Function to visualize the graph
    pygame.init()

    # Set the window dimensions
    WINDOW_WIDTH = 600
    WINDOW_HEIGHT = 250

    # Create the Pygame window
    window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Obstacle Course")

    # Define the colors
    BACKGROUND_COLOR = pygame.Color("red")
    OBSTACLE_COLOR = pygame.Color("black")
    CLEARANCE_COLOR = pygame.Color("white")
    VISITED_COLOR = pygame.Color("green")
    PATH_COLOR = pygame.Color("blue")

    # Create the surface for the obstacle course
    surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
    # Fill the surface with the background color
    surface.fill(BACKGROUND_COLOR)
    #obstacles = []
    pygame.draw.rect(surface, CLEARANCE_COLOR, (100-5, 145-5,50+10 ,100+10))
    pygame.draw.polygon(surface, CLEARANCE_COLOR, ((300,200+5),(365+4,162),(365+4,87),(300,50-5),(235-4,87),(235-4,162))) 
    pygame.draw.rect(surface, CLEARANCE_COLOR, (100-5,5-5,50+10,100+10))                                                                #Printing directly using the coordinates for visualization.
    pygame.draw.polygon(surface, CLEARANCE_COLOR, ((460-3,225+12),(460-3,25-12),(510+5,125)))
    # Draw the obstacles on the surface
    pygame.draw.rect(surface, OBSTACLE_COLOR, (100, 145,50 ,100))  
    pygame.draw.polygon(surface, OBSTACLE_COLOR, ((300,200),(365,162),(365,87),(300,50),(235,87),(235,162)))
    pygame.draw.rect(surface, OBSTACLE_COLOR, (100,5,50,100))
    pygame.draw.polygon(surface, OBSTACLE_COLOR, ((460,225),(460,25),(510,125)))   

    for idx,any in enumerate(heap):
        pygame.draw.rect(surface,VISITED_COLOR,(any.state[0],any.state[1],1,1))
        window.blit(surface,(0,0))
        pygame.display.update()
    
    for idx, every in enumerate(path_gen):
        pygame.draw.rect(surface,PATH_COLOR,(every[0],every[1],1,1))
    # Blit the updated surface onto the Pygame window
        window.blit(surface, (0, 0))
    # Update the Pygame window display
        pygame.display.update()
    # Wait for a short time to show the current coordinate
        pygame.time.wait(2)

    
    # Blit the surface onto the Pygame window
    window.blit(surface, (0, 0))

    # Run the game loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pygame.display.flip()

    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    node = Node()
    x = int(input("start x - coordinate"))
    y = int(input("start y coordinate:"))
    theta = int(input("start orientation"))
    start = list([x,y,theta])
    node.state = start
    node.parent = None
    node.cost_to_come = 0
    node.cost_to_goal = 100000
    goal = [90,5,0]
    step = int(input("Enter step between 1 and 10"))
    astar(node,goal,step)
