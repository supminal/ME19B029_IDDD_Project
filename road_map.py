import numpy as np
import heapq
import time

# Represents the pixel value of each point of graph
# coordinate Conversion to simulation map
def coordinate_convert(x,y):
     X = -8.49 + (x - 195)*0.05
     Y = -12.8 + (577 - y)*0.05
     return [round(X,3),round(Y,3)]


def sim_img_convert(x, y):
   X= (x + 8.49)*20 + 195
   Y = 577 - (y +12.8)*20
   return [round(X), round(Y)]


def astar(graph_str,road_map, start, goal):
    start_time = time.time_ns()
    def heuristic(node):

        return np.linalg.norm(np.array(node) - np.array(road_map[int(goal)]))

    # Initializing
    queue = []
    cost = {start: 0}
    heapq.heappush(queue, (0, start))
    parent = {start: None}


    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            break

        neighbors = graph_str[current]



        for neighbor in neighbors:
            # if map_array[neighbor[0]][neighbor[1]] == 0:
            #     continue
            # Calculate the cost of reaching the neighbor
            neighbor_cost = cost[current] + np.linalg.norm(np.array(road_map[int(neighbor)]) - np.array(road_map[int(current)]))

            if neighbor not in cost or neighbor_cost < cost[neighbor]:
                #g
                cost[neighbor] = neighbor_cost
                #f = g+h
                priority = neighbor_cost + heuristic(road_map[int(neighbor)])
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
    print(time.time_ns() - start_time)
    return path

road_map = {
        1: [195, 577],  2: [111, 568],  3: [120, 218],  4: [431, 218],  5: [476, 218],
        6: [654, 218],  7: [654, 68],  8: [195, 562],  9: [130, 562],  10: [129, 238],
        11: [458, 231],  12: [458, 426],  13: [458, 533],  14: [476, 533],  15: [561, 533],
        17: [476, 426],  18: [590, 422],  19: [590, 613],  20: [654, 621],
        21: [750, 621],  22: [654, 428],  25: [645, 406],
        26: [482, 411],  27: [475, 238],  28: [642, 231]
    }
road_map_sim = {1: [-8.49, -12.8], 2: [-12.69, -12.35], 3: [-12.24, 5.15],
                4: [3.31, 5.15], 5: [5.56, 5.15],
                6: [14.46, 5.15], 7: [14.46, 12.65],
                8: [-8.49, -12.05], 9: [-11.74, -12.05], 10: [-11.79, 4.15],
                11: [4.66, 4.5], 12: [4.66, -5.25], 13: [4.66, -10.60],
                14: [5.56, -10.60], 15: [9.81, -10.60], 17: [5.56, -5.25],
                18: [11.26, -5.05], 19: [11.26, -14.60], 20: [14.46, -15.0],
                21: [19.26, -15.0], 22: [14.46, -5.35],  25: [14.01, -4.25],
                26: [5.86, -4.5], 27: [5.51, 4.15], 28: [13.86, 4.5]
                }


# Neighbours of each node
# Direction dependent, robot stays to the left
graph = {
            1: [2],  2: [3],  3: [4],  4: [5],  5: [6, 27],
            6: [7, 28],  7: [6],  8: [],  9: [8],  10: [9],
            11: [5, 10],  12: [11, 26],  13: [12],  14: [13, 15],  15: [14],
            17: [12, 14],  18: [17, 25],  19: [18],  20: [19, 21],
            21: [20],  22: [18, 20],  25: [22, 28], 26: [17, 25],
            27: [11, 26],  28: [6, 27]
        }
graph_str = {
            '1': ['2'],  '2': ['3'],  '3': ['4'],  '4': ['5'],  '5': ['6', '27'],
            '6': ['7', '28'],  '7': ['6'],  '8': [],  '9': ['8'],  '10': ['9'],
            '11': ['5', '10'],  '12': ['11', '26'],  '13': ['12'],  '14': ['13', '15'],  '15': ['14'],
            '17': ['12', '14'],  '18': ['17', '25'],  '19': ['18'],  '20': ['19', '21'],
            '21': ['20'],  '22': ['18', '20'],  '25': ['22', '28'],
            '26': ['17', '25'],  '27': ['11', '26'],  '28': ['6', '27']
        }


if __name__ == '__main__':
    road_map_path_pixel_value = [road_map[int(i)] for i in astar(graph_str,road_map,'1','21')]
    road_map_path_sim_value = []
            #Converting to CoppeliaSim coordinates
    for i,j in road_map_path_pixel_value:
        road_map_path_sim_value.append(coordinate_convert(i,j))

    print(road_map_path_pixel_value)
    print(road_map_path_sim_value)