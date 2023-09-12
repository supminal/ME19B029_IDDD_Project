#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""
import time

#Import files
import sim_interface
import control
import road_map
import local_planner_three
import cv2
import numpy as np




def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):
            # global planner called here
            # States of robot, right now points only from the road map created
            robot_state = '1'
            final_goal_state = '21'

            theta1 = (-2, 0)
            theta2 = (0, -2) # theta of second waypoint in final path, yet to be calculated

            # path through the road map
            road_map_path = road_map.astar(road_map.graph_str, road_map.road_map, robot_state, final_goal_state)

            road_map_path_pixel_value = [road_map.road_map[int(i)] for i in road_map_path]

            road_map_path_sim_value = []
            # Converting to CoppeliaSim coordinates
            for i, j in road_map_path_pixel_value:
                road_map_path_sim_value.append(road_map.coordinate_convert(i, j))

            print("final path in simulation coordinates is:", road_map_path_sim_value)

            # Local Planner
            for i in range(len(road_map_path_sim_value)-1):
                start_state = road_map_path_pixel_value[i]
                if i == 0:
                    start_state.append(theta1)
                print(start_state)
                #theta should be calculated but you have hardcoded now
                goal_state = road_map_path_pixel_value[i+1]
                goal_state.append(theta2)
                print(goal_state)
                local_path_px = local_planner_three.astar(local_planner_three.map_array,tuple(start_state),tuple(goal_state))
                local_path_sim = []
                for x,y,z in local_path_px:
                    cord_conv = road_map.coordinate_convert(x,y)
                    cord_conv.append(z)
                    local_path_sim.append(cord_conv)
                theta1 = theta2
                theta2 = z
                #Stop robot
                sim_interface.setvel_pioneers(0.0, 0.0)

                #Set goal state

                for i in local_path_sim:
                    goal_state = i
                #Obtain robots position
                    robot_state = sim_interface.localize_robot()

                    while not control.at_goal(robot_state, goal_state):
                        [V,W] = control.gtg(robot_state, goal_state)
                        sim_interface.setvel_pioneers(V, W)
                        time.sleep(0.5)
                        robot_state = sim_interface.localize_robot()
                        sim_interface.localize_bills()

                    #Stop robot
                    sim_interface.setvel_pioneers(0.0, 0.0)

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 