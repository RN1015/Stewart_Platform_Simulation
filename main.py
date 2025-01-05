import math
from ik_class import InvKin
import pandas as pd
import numpy as np
import math as m
base_length = 0.142
top_length = 0.067
d_b = 0.23
d_t = 0.25
l1 = 0.075
l2 = 0.23

def print_hi(name):

    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
    # robot = InvKin()
    zamanov = InvKin(6.7, 23, 14.2, 25, 7.5, 18)
    # zamanov = InvKin()
    # zamanov.inv_k_raw_input(10, 0, 0, 12, 0, -10)
    zamanov.inv_k_raw_input(9, 0, 0, 12, 0, -10)
    # zamanov.inv_k([10, 0, 0, 12, 0, 10])
    # zamanov.inv_k([10, 0, 0, 12, 0, -10])
    # joint_set = ik_of_trajectory(zamanov)
    # zamanov.ik_of_trajectory()
    # print(zamanov.task_set)
    # print(zamanov.joint_set)
    # print(zamanov.task_to_joint_set())
    zamanov.treg_motion_time = 20
    zamanov.tregectory_ploting()
    zamanov.tregectory_ploting_task_3()
    # print(zamanov.top_vertices_local())
    # print(zamanov.top_vertices_global(9, 0, 0, 12, 0, -10))
    print(zamanov.bottom_vertices())

    # print(zamanov.Rx_clock_dev[1])
    # print(zamanov.t_with_b[:, 0:3])
    # print(zamanov.transform(4, 5, 0, 0, 0, 0, [0, 0, 0, 1]))
    # print("000000000000000000000000000000000")
    # zamanov.inv_k_raw_input(0, 0, -9.3646, 0, 0, 0)
    # zamanov.inv_k_raw_input(0, 0, 12.1335, 0, 0, 0)
    # zamanov.inv_k_raw_input(0, 0, -5.6646, 0, 0, 0)
    # zamanov.inv_k_raw_input(0, 0, 5.522, 0, 0, 0)
    # height = 10.52
    # zamanov.inv_k_raw_input(0, 0, 8.754, 0, 0, 0)


    # zamanov.inv_k_raw_input(0, 0, -11.1866, 0, 0, 0)
    # print(zamanov.T_global)
    # zamanov.revers_tf()
    # print(zamanov.t_with_b[:, 0:3])


    # loc = [2, 2, 6, 3, 5, 4]
    # x = robot.inv_k(loc)
    # print(robot.T_global)
    # print(robot.B_origin)
    # print(robot.T_local)
    # print(x)
    # print(y)
    # print(robot.home_pos)
    # print(robot.position)
    # print(robot.orientation)
    # print(robot.crank_angle)
    # print(robot.Rx_clock_dev)
    # top, base = robot.dev()
    # print(base)
    # print(top)
    # # robot.joint_variables(top, base)
    # print(robot.crank_angle)
    #
    # robot.print_motor_frames()
    #
    # print(robot.top_vertices_local())
    # print(robot.top_centre_global())
    # print(robot.transformation_matrix())
    print("-----------------------------")
    # print(robot.bottom_vertices()[0])
    # print(robot.top_vertices_local().T)
    # print("-----------------------------")
    # print((robot.bottom_vertices()[1]).transpose())
    #
    # print("-----------------------------")
    # print("-----------------------------")
    # print(robot.top_vertices_global())
    # print("-----------------------------")
    # print("-----------------------------")
    # print(robot.inv_k_raw_input(0, 0, 0, 0, 0, 0))
    # print(robot.T_centre_global)
    # robot.plot_bottom_vertices_3D()
    # robot.plot_top_vertices_local_3D()
    # robot.plot_top_vertices_global_3D()
    # zamanov.plot_bottom_vertices()
    # zamanov.plot_top_vertices_local()
    # zamanov.plot_top_vertices_global()


    # zamanov.plot_top_vertices_local()

    # zamanov.slop()
    # zamanov.plot_ref_all_2d()

    # print(zamanov.joint_angle_dev(6, 3, 4))
    # print(zamanov.t_with_b)
    # a, b, c = zamanov.abc_return()
    # print(zamanov.joint_angle_dev(a, b, c))
    # print(zamanov.crank_angle_dev())
    print(zamanov.crank_angle)
    # print(zamanov.t_with_b)
    # zamanov.show_plot()




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')


