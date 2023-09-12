#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian
"""
import time

# Import files
import sim_interface
import control
import road_map
import lp_with_kp_and_pruning
import sim
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def main():
    if (sim_interface.sim_init()):
        plt.ion()

        # Obtain handles to sim elements
        sim_interface.get_handles()

        # Start simulation
        if (sim_interface.start_simulation()):
            # global planner called here
            # States of robot, right now points only from the road map created
            while(1):
                start_t = time.time_ns()
                bill_pos = sim_interface.localize_bills()
                bill_pos_img_frame = road_map.sim_img_convert(bill_pos[0], bill_pos[1])
                lp_with_kp_and_pruning.obstacle = bill_pos_img_frame
                # print("from main", bill_pos_img_frame)
                theta1 = 180
                theta2 = 0
                start_state = 194, 575, theta1
                goal_state = 167, 218, theta2
                # Local Planner
                local_path_px = lp_with_kp_and_pruning.astar(lp_with_kp_and_pruning.map_array, start_state,goal_state)
                local_path_sim = []
                for x, y, z in local_path_px:
                    cord_conv = road_map.coordinate_convert(x, y)
                    #cord_conv.append(z)
                    local_path_sim.append(cord_conv)
                # pathHandle = sim.simxcreatePath(local_path_sim,options = 2)
                img = plt.imread("map_new.jpeg")
                # fig,ax = plt.subplots()
                fig = plt.gcf()
                plt.cla()

                ax = plt.gca()
                x = range(300)
                ax.imshow(img, extent=[0, 795, -661, 0])
                X_final_path = []
                Y_final_path = []
                for i, j, k in local_path_px:
                    X_final_path.append(i)
                    Y_final_path.append(-j)
                Drawing_colored_circle = plt.Circle((lp_with_kp_and_pruning.obstacle[0], - lp_with_kp_and_pruning.obstacle[1]), 6.5, color='g')
                ax.add_patch(Drawing_colored_circle)
                plt.scatter(X_final_path, Y_final_path,marker='.')
                plt.pause(0.1)
                plt.show()

                print(time.time_ns() - start_t)
                        # while not control.at_goal(robot_state, goal_state):
                        #     [V, W] = control.gtg(robot_state, goal_state)
                        #     sim_interface.setvel_pioneers(V, W)
                        #     time.sleep(0.5)
                        #     robot_state = sim_interface.localize_robot()
                        #     sim_interface.localize_bills()
                        #
                        # # Stop robot
                        # sim_interface.setvel_pioneers(0.0, 0.0)

        else:
            print('Failed to start simulation')
    else:
        print('Failed connecting to remote API server')

    # stop robots
    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return


# run
if __name__ == '__main__':
    main()
    print('Program ended')


