import numpy as np
import heapq
import cv2

def astar(map_array, start, goal):
    height, width = map_array.shape

    def heuristic(node):
        return np.linalg.norm(np.array(node) - np.array(goal))

    # Initializing
    queue = []
    cost = {start: 0}
    heapq.heappush(queue, (0, start))
    parent = {start: None}


    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            break

        neighbors = []
        x, y = current

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx = x + dx
                ny = y + dy

                if dx == 0 and dy == 0:
                    continue

                #check if within bounds
                if 0 <= ny < height and 0 <= nx < width:
                    #Check if obstacle
                    if map_array[ny][nx] < 200:
                        continue
                    else:
                        neighbors.append((nx, ny))

        for neighbor in neighbors:
            # if map_array[neighbor[0]][neighbor[1]] == 0:
            #     continue
            # Calculate the cost of reaching the neighbor
            neighbor_cost = cost[current] + np.linalg.norm(np.array(neighbor) - np.array(current)) + (255 - map_array[neighbor[1]][neighbor[0]])*5/255

            if neighbor not in cost or neighbor_cost < cost[neighbor]:
                #g
                cost[neighbor] = neighbor_cost
                #f = g+h
                priority = neighbor_cost + heuristic(neighbor)
                # adding new nodes to the q
                heapq.heappush(queue, (priority, neighbor))
                #connect node to its parent
                parent[neighbor] = current

    # Retrieve the path from the goal to the start
    path = []
    current = goal

    while current:
        path.append(current)
        current = parent[current]

    path.reverse()
    return path


image = cv2.imread('map_new.jpeg')
# Use the cvtColor() function to grayscale the image
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#cv2_imshow( gray_image)
# Apply binary thresholding to create a binary image
threshold_value = 210
max_value = 255
ret, binary_image = cv2.threshold(gray_image, threshold_value, max_value, cv2.THRESH_BINARY)
im = np.asarray(binary_image)
im_gray = np.asarray(gray_image)
#box_blur_ker = (1/25)*np.array([[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1]])
box_blur_ker = (1/225)*np.ones((15,15))

Box_blur = cv2.filter2D(src=im, ddepth=-1, kernel=box_blur_ker)
# cv2_imshow(Box_blur)
imblur = np.asarray(Box_blur)

map_array = imblur

# Callback function for mouse events
def mouse_callback(event, x, y, flags, param):
    global start_point, goal_point

    if event == cv2.EVENT_LBUTTONDOWN:
        if start_point is None:
            start_point = (x, y)
            print("Start point selected: ({}, {})".format(x, y))
        else:
            goal_point = (x, y)
            print("Goal point selected: ({}, {})".format(x, y))

# Load the map image
map_image = cv2.imread('map_new.jpeg')

# Create a window to display the image
cv2.namedWindow('Map')
cv2.imshow('Map', map_image)

# Initialize the start and goal points as None
start_point = None
goal_point = None

# Set the mouse callback function
cv2.setMouseCallback('Map', mouse_callback)

# Wait for the user to select the start and goal points
while start_point is None or goal_point is None:
    cv2.waitKey(1)

# Close the image window
cv2.destroyAllWindows()

# Print the selected start and goal points

path = astar(map_array, start_point, goal_point)
for i,j in path:
    imblur[j][i] = 100
cv2.imshow('image',imblur)
cv2.waitKey(0)