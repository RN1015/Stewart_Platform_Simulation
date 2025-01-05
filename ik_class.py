import math as m
import numpy as np
import pandas as pd
from sp_visuals_class import SpVisual
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

zero_6d = [0, 0, 0, 0, 0, 0]


class InvKin(SpVisual):
    def __init__(self, t_up=5, a_up=12, t_down=8, b_down=15, crank=2, rod=18, height=16.052):
        super().__init__()
        self.semihex_up_short = t_up
        self.semihex_up_long = a_up
        self.semihex_down_short = t_down
        self.semihex_down_long = b_down

        self.crank_length = crank
        self.rod_length = rod
        self.sp_height = height

        self.home_pos = np.array([0, 0, 0, 0, 0, 0])
        self.position = self.home_pos[0:3]
        self.orientation = self.home_pos[3:6]

        # self.rangle_vectclock = np.array([0, m.pi *2/ 3, -m.pi / 3, -4 * m.pi / 6, 2 * m.pi / 6, m.pi ])
        self.rangle_vectclock = np.array([m.pi, m.pi * 2 / 3, -m.pi / 3, -4 * m.pi / 6, 2 * m.pi / 6, 0])
        # self.rangle_vectclock = np.array([m.pi / 6, m.pi / 2, -m.pi / 6, -5 * m.pi / 6, m.pi / 2, 5 * m.pi / 6])
        # self.rangle_vectclock = np.array([5 * m.pi / 6, -5 * m.pi / 6, m.pi / 6, m.pi / 2, -m.pi / 2, -m.pi / 6 ])
        # self.rangle_vectclock = np.array([-5 * m.pi / 6, +5 * m.pi / 6, -m.pi / 6, -m.pi / 2, +m.pi / 2, m.pi / 6])
        self.Rx_clock_dev = [self.axis_rotation(val, "z") for val in self.rangle_vectclock]

        # self.T_local = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.B_origin = self.bottom_vertices()
        self.T_local = self.top_vertices_local()
        self.T_global = self.top_vertices_global()

        self.lim_extra = 5
        self.limits_of_sp = self.sp_limits()

        self.T_centre_global = self.top_centre_global()
        # self.T_centre_global = np.array([0.0, 0.0, 0.0, 1])

        self.xlims = self.limits_of_sp[0]
        self.ylims = self.limits_of_sp[1]
        self.zlims = self.limits_of_sp[2]
        self.crank_angle = []
        self.ref_axis_len = self.lim_extra * 0.5
        self.tf_ret = [0, 0, 0, 0]
        self.t_with_b = self.revers_tf()

        self.motion_time = 20
        self.task_set = self.tregectory_gen()
        self.joint_set = self.ik_of_trajectory()
        self.task_to_joint_sets = self.task_to_joint_set()
        self.legend_joint_pos = ["Joint-1", "Joint-2", "Joint-3", "Joint-4", "Joint-5", "Joint-6"]
        self.legend_task_pos = ["X-axis", "Y-axis", "Z-axis", "Roll", "Pitch", "Yaw"]

    def sp_limits(self):
        x_min = np.min(
            [np.min(self.B_origin[:, 0]), np.min(self.T_local[:, 0]), np.min(self.T_global[:, 0])]) - self.lim_extra
        y_min = np.min(
            [np.min(self.B_origin[:, 1]), np.min(self.T_local[:, 1]), np.min(self.T_global[:, 1])]) - self.lim_extra
        z_min = np.min([np.min(self.B_origin[:, 2]), np.min(self.T_local[:, 2]), np.min(self.T_global[:, 2])])
        x_max = np.max(
            [np.max(self.B_origin[:, 0]), np.max(self.T_local[:, 0]), np.max(self.T_global[:, 0])]) + self.lim_extra
        y_max = np.max(
            [np.max(self.B_origin[:, 1]), np.max(self.T_local[:, 1]), np.max(self.T_global[:, 1])]) + self.lim_extra
        z_max = np.max(
            [np.max(self.B_origin[:, 2]), np.max(self.T_local[:, 2]), np.max(self.T_global[:, 2])]) + self.lim_extra
        x_ = np.max([np.abs(x_min), x_max])
        y_ = np.max([np.abs(y_min), y_max])
        z_ = np.max([np.abs(z_min), z_max])
        f_lim = np.max([x_, y_, z_])
        # self.limits_of_sp = [[-x_, x_], [-y_, y_], [-z_, z_]]
        self.limits_of_sp = [[-f_lim, f_lim], [-f_lim, f_lim], [-f_lim, f_lim]]
        return self.limits_of_sp

    def inv_k_raw_input(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0):
        loc = np.array([px, py, pz, roll, pitch, yaw])
        self.position = loc[0:3]
        self.orientation = loc[3:6]
        diff = self.bottom_vertices() - self.top_vertices_global(loc[0], loc[1], loc[2], loc[3], loc[4], loc[5])
        diff_square = np.square(diff)
        link_len = np.apply_along_axis(self.row_sum, axis=1, arr=diff_square)
        link_len = np.sqrt(link_len)
        self.T_centre_global = self.top_centre_global(loc[0], loc[1], loc[2], loc[3], loc[4], loc[5])
        self.t_with_b = self.revers_tf()
        # self.joint_variables()
        self.crank_angle = self.crank_angle_dev_true()
        return link_len

    def inv_k(self, location=zero_6d):
        loc = np.array(location)
        self.position = loc[0:3]
        self.orientation = loc[3:6]
        diff = self.bottom_vertices() - self.top_vertices_global(loc[0], loc[1], loc[2], loc[3], loc[4], loc[5])
        diff_square = np.square(diff)
        link_len = np.apply_along_axis(self.row_sum, axis=1, arr=diff_square)
        link_len = np.sqrt(link_len)
        self.T_centre_global = self.top_centre_global(loc[0], loc[1], loc[2], loc[3], loc[4], loc[5])
        self.t_with_b = self.revers_tf()
        # self.joint_variables()
        self.crank_angle = self.crank_angle_dev_true()
        return link_len

    def row_sum(self, row):
        return np.sum(row)

    def rotation_matrix(self, theta1, theta2, theta3, order='zyx'):  # yxz is has chosen there(r- x, p- y, yaw- z)
        """
        input
            theta1, theta2, theta3 = rotation angles in rotation order (degrees)
            oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'  yxz
        output
            3x3 rotation matrix (numpy array)
        """
        c1 = np.cos(theta1 * np.pi / 180)
        s1 = np.sin(theta1 * np.pi / 180)
        c2 = np.cos(theta2 * np.pi / 180)
        s2 = np.sin(theta2 * np.pi / 180)
        c3 = np.cos(theta3 * np.pi / 180)
        s3 = np.sin(theta3 * np.pi / 180)

        if order == 'xzx':
            matrix = np.array([[c2, -c3 * s2, s2 * s3],
                               [c1 * s2, c1 * c2 * c3 - s1 * s3, -c3 * s1 - c1 * c2 * s3],
                               [s1 * s2, c1 * s3 + c2 * c3 * s1, c1 * c3 - c2 * s1 * s3]])
        elif order == 'xyx':
            matrix = np.array([[c2, s2 * s3, c3 * s2],
                               [s1 * s2, c1 * c3 - c2 * s1 * s3, -c1 * s3 - c2 * c3 * s1],
                               [-c1 * s2, c3 * s1 + c1 * c2 * s3, c1 * c2 * c3 - s1 * s3]])
        elif order == 'yxy':
            matrix = np.array([[c1 * c3 - c2 * s1 * s3, s1 * s2, c1 * s3 + c2 * c3 * s1],
                               [s2 * s3, c2, -c3 * s2],
                               [-c3 * s1 - c1 * c2 * s3, c1 * s2, c1 * c2 * c3 - s1 * s3]])
        elif order == 'yzy':
            matrix = np.array([[c1 * c2 * c3 - s1 * s3, -c1 * s2, c3 * s1 + c1 * c2 * s3],
                               [c3 * s2, c2, s2 * s3],
                               [-c1 * s3 - c2 * c3 * s1, s1 * s2, c1 * c3 - c2 * s1 * s3]])
        elif order == 'zyz':
            matrix = np.array([[c1 * c2 * c3 - s1 * s3, -c3 * s1 - c1 * c2 * s3, c1 * s2],
                               [c1 * s3 + c2 * c3 * s1, c1 * c3 - c2 * s1 * s3, s1 * s2],
                               [-c3 * s2, s2 * s3, c2]])
        elif order == 'zxz':
            matrix = np.array([[c1 * c3 - c2 * s1 * s3, -c1 * s3 - c2 * c3 * s1, s1 * s2],
                               [c3 * s1 + c1 * c2 * s3, c1 * c2 * c3 - s1 * s3, -c1 * s2],
                               [s2 * s3, c3 * s2, c2]])
        elif order == 'xyz':
            matrix = np.array([[c2 * c3, -c2 * s3, s2],
                               [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
                               [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]])
        elif order == 'xzy':
            matrix = np.array([[c2 * c3, -s2, c2 * s3],
                               [s1 * s3 + c1 * c3 * s2, c1 * c2, c1 * s2 * s3 - c3 * s1],
                               [c3 * s1 * s2 - c1 * s3, c2 * s1, c1 * c3 + s1 * s2 * s3]])
        elif order == 'yxz':   #.........
            matrix = np.array([[c1 * c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3, c2 * s1],
                               [c2 * s3, c2 * c3, -s2],
                               [c1 * s2 * s3 - c3 * s1, c1 * c3 * s2 + s1 * s3, c1 * c2]])
        elif order == 'yzx':
            matrix = np.array([[c1 * c2, s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3],
                               [s2, c2 * c3, -c2 * s3],
                               [-c2 * s1, c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3]])
        elif order == 'zyx':
            matrix = np.array([[c1 * c2, c1 * s2 * s3 - c3 * s1, s1 * s3 + c1 * c3 * s2],
                               [c2 * s1, c1 * c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3],
                               [-s2, c2 * s3, c2 * c3]])
        elif order == 'zxy':
            matrix = np.array([[c1 * c3 - s1 * s2 * s3, -c2 * s1, c1 * s3 + c3 * s1 * s2],
                               [c3 * s1 + c1 * s2 * s3, c1 * c2, s1 * s3 - c1 * c3 * s2],
                               [-c2 * s3, s2, c2 * c3]])

        return matrix

    def rotation_angles(self, matrix, order):
        """
        input
            matrix = 3x3 rotation matrix (numpy array)
            oreder(str) = rotation order of x, y, z : e.g, rotation XZY -- 'xzy'
        output
            theta1, theta2, theta3 = rotation angles in rotation order
        """
        r11, r12, r13 = matrix[0]
        r21, r22, r23 = matrix[1]
        r31, r32, r33 = matrix[2]

        if order == 'xzx':
            theta1 = np.arctan(r31 / r21)
            theta2 = np.arctan(r21 / (r11 * np.cos(theta1)))
            theta3 = np.arctan(-r13 / r12)

        elif order == 'xyx':
            theta1 = np.arctan(-r21 / r31)
            theta2 = np.arctan(-r31 / (r11 * np.cos(theta1)))
            theta3 = np.arctan(r12 / r13)

        elif order == 'yxy':
            theta1 = np.arctan(r12 / r32)
            theta2 = np.arctan(r32 / (r22 * np.cos(theta1)))
            theta3 = np.arctan(-r21 / r23)

        elif order == 'yzy':
            theta1 = np.arctan(-r32 / r12)
            theta2 = np.arctan(-r12 / (r22 * np.cos(theta1)))
            theta3 = np.arctan(r23 / r21)

        elif order == 'zyz':
            theta1 = np.arctan(r23 / r13)
            theta2 = np.arctan(r13 / (r33 * np.cos(theta1)))
            theta3 = np.arctan(-r32 / r31)

        elif order == 'zxz':
            theta1 = np.arctan(-r13 / r23)
            theta2 = np.arctan(-r23 / (r33 * np.cos(theta1)))
            theta3 = np.arctan(r31 / r32)

        elif order == 'xzy':
            theta1 = np.arctan(r32 / r22)
            theta2 = np.arctan(-r12 * np.cos(theta1) / r22)
            theta3 = np.arctan(r13 / r11)

        elif order == 'xyz':
            theta1 = np.arctan(-r23 / r33)
            theta2 = np.arctan(r13 * np.cos(theta1) / r33)
            theta3 = np.arctan(-r12 / r11)

        elif order == 'yxz':
            theta1 = np.arctan(r13 / r33)
            theta2 = np.arctan(-r23 * np.cos(theta1) / r33)
            theta3 = np.arctan(r21 / r22)

        elif order == 'yzx':
            theta1 = np.arctan(-r31 / r11)
            theta2 = np.arctan(r21 * np.cos(theta1) / r11)
            theta3 = np.arctan(-r23 / r22)

        elif order == 'zyx':
            theta1 = np.arctan(r21 / r11)
            theta2 = np.arctan(-r31 * np.cos(theta1) / r11)
            theta3 = np.arctan(r32 / r33)

        elif order == 'zxy':
            theta1 = np.arctan(-r12 / r22)
            theta2 = np.arctan(r32 * np.cos(theta1) / r22)
            theta3 = np.arctan(-r31 / r33)

        theta1 = theta1 * 180 / np.pi
        theta2 = theta2 * 180 / np.pi
        theta3 = theta3 * 180 / np.pi

        return (theta1, theta2, theta3)

    def axis_rotation(self, theta, axis_name):
        # calculate single rotation of \(\theta\) matrix around x,y or z
        """
        input
            theta = rotation angle(degrees)
            axis_name = 'x', 'y' or 'z'
        output
            3x3 rotation matrix
        """

        c = np.cos(theta * np.pi / 180)
        s = np.sin(theta * np.pi / 180)
        if axis_name == 'x':
            rotation_matrix = np.array([[1, 0, 0],
                                        [0, c, -s],
                                        [0, s, c]])
        if axis_name == 'y':
            rotation_matrix = np.array([[c, 0, s],
                                        [0, 1, 0],
                                        [-s, 0, c]])
        elif axis_name == 'z':
            rotation_matrix = np.array([[c, -s, 0],
                                        [s, c, 0],
                                        [0, 0, 1]])
        return rotation_matrix

    def confirm_matrix(self, theta1, theta2, theta3, order='xyz'):
        # Calculate Rotation matrix of Euler angles from three single rotation
        """
        input
            theta1, theta2, theta3 = rotation angles in rotation order
            order = rotation order　e.g. XZY rotation -- 'xzy'
        output
            3x3 rotation matrix
        """
        axis_name1 = order[0]
        axis_name2 = order[1]
        axis_name3 = order[2]

        rotation1 = self.axis_rotation(-theta1, axis_name1)
        rotation2 = self.axis_rotation(-theta2, axis_name2)
        rotation3 = self.axis_rotation(-theta3, axis_name3)

        # rotation matrix of global world rotation around the local object
        matrix_local = np.dot(rotation3, np.dot(rotation2, rotation1))

        # inverse matrix for rotation of the local object
        matrix_global = matrix_local.T

        return matrix_global

    def bottom_vertices(self):
        r3 = m.sqrt(3)
        # Bottom plate vertices wrt base frame
        XB1 = (r3 / 6) * (2 * self.semihex_down_short + self.semihex_down_long)
        YB1 = (0.5 * self.semihex_down_long)
        ZB1 = 0.0

        XB2 = (-r3 / 6) * (self.semihex_down_short - self.semihex_down_long)
        YB2 = 0.5 * (self.semihex_down_short + self.semihex_down_long)
        ZB2 = 0.0

        XB3 = (-r3 / 6) * (self.semihex_down_short + 2 * self.semihex_down_long)
        YB3 = (0.5 * self.semihex_down_short)
        ZB3 = 0.0

        XB4 = (-r3 / 6) * (self.semihex_down_short + 2 * self.semihex_down_long)
        YB4 = -(0.5 * self.semihex_down_short)
        ZB4 = 0.0

        XB5 = (-r3 / 6) * (self.semihex_down_short - self.semihex_down_long)
        YB5 = -0.5 * (self.semihex_down_short + self.semihex_down_long)
        ZB5 = 0.0

        XB6 = (r3 / 6) * (2 * self.semihex_down_short + self.semihex_down_long)
        YB6 = -(0.5 * self.semihex_down_long)
        ZB6 = 0.0

        B1 = [XB1, YB1, ZB1, 1]
        B2 = [XB2, YB2, ZB2, 1]
        B3 = [XB3, YB3, ZB3, 1]
        B4 = [XB4, YB4, ZB4, 1]
        B5 = [XB5, YB5, ZB5, 1]
        B6 = [XB6, YB6, ZB6, 1]
        self.B_origin = np.array([B1, B2, B3, B4, B5, B6])
        return self.B_origin

    def top_vertices_local(self):
        r3 = m.sqrt(3)
        # Top plate vertices wrt base frame
        xT1 = (r3 / 6) * (2 * self.semihex_up_short + self.semihex_up_long)
        yT1 = (0.5 * self.semihex_up_long)
        zT1 = 0.0

        xT2 = (-r3 / 6) * (self.semihex_up_short - self.semihex_up_long)
        yT2 = 0.5 * (self.semihex_up_short + self.semihex_up_long)
        zT2 = 0.0

        xT3 = (-r3 / 6) * (self.semihex_up_short + 2 * self.semihex_up_long)
        yT3 = (0.5 * self.semihex_up_short)
        zT3 = 0.0

        xT4 = (-r3 / 6) * (self.semihex_up_short + 2 * self.semihex_up_long)
        yT4 = -(0.5 * self.semihex_up_short)
        zT4 = 0.0

        xT5 = (-r3 / 6) * (self.semihex_up_short - self.semihex_up_long)
        yT5 = -0.5 * (self.semihex_up_short + self.semihex_up_long)
        zT5 = 0.0

        xT6 = (r3 / 6) * (2 * self.semihex_up_short + self.semihex_up_long)
        yT6 = -(0.5 * self.semihex_up_long)
        zT6 = 0.0
        # T6 = [xT1, yT1, zT1, 1]
        # T1 = [xT2, yT2, zT2, 1]
        # T2 = [xT3, yT3, zT3, 1]
        # T3 = [xT4, yT4, zT4, 1]
        # T4 = [xT5, yT5, zT5, 1]
        # T5 = [xT6, yT6, zT6, 1]

        T1 = [xT1, yT1, zT1, 1]
        T2 = [xT2, yT2, zT2, 1]
        T3 = [xT3, yT3, zT3, 1]
        T4 = [xT4, yT4, zT4, 1]
        T5 = [xT5, yT5, zT5, 1]
        T6 = [xT6, yT6, zT6, 1]
        self.T_local = np.array([T1, T2, T3, T4, T5, T6])
        return self.T_local

    def top_vertices_global(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0):
        T_global = (self.transformation_matrix(px, py, pz, roll, pitch, yaw)).dot(self.top_vertices_local().T)
        self.T_global = T_global.T
        return self.T_global

    def top_centre_global(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0):
        # print(self.transformation_matrix(px, py, pz, roll, pitch, yaw))
        T_centre_global = (self.transformation_matrix(px, py, pz, roll, pitch, yaw)).dot(np.array([0, 0, 0, 1]).T)
        self.T_centre_global = T_centre_global.T
        return self.T_centre_global

    def transformation_matrix(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0):
        rotation_mat = self.rotation_matrix(roll, pitch, yaw, order='yxz')
        row_stacking = np.row_stack((rotation_mat, np.array([0.0, 0.0, 0.0])))
        transform = np.column_stack((row_stacking, np.array([px, py, pz + self.sp_height, 1])))
        return transform

    def plot_bottom_vertices(self):
        bot_vert_stack = np.row_stack((self.bottom_vertices(), self.bottom_vertices()[0]))
        self.plot(bot_vert_stack[:, 1], bot_vert_stack[:, 0])
        self.put_lim()
        self.put_centre(0, 0)

    def plot_top_vertices_global(self):
        top_vert_stack = np.row_stack((self.T_global, self.T_global[0]))
        self.plot(top_vert_stack[:, 1], top_vert_stack[:, 0])
        self.put_lim()
        self.put_centre(self.T_centre_global[0], self.T_centre_global[1])

    def plot_top_vertices_local(self):
        top_vert_local_stack = np.row_stack((self.top_vertices_local(), self.top_vertices_local()[0]))
        self.plot(top_vert_local_stack[:, 1], top_vert_local_stack[:, 0])
        self.put_lim()
        self.put_centre(0, 0)

    def plot_bottom_vertices_3D(self):
        # fig = plt.figure()
        fig = plt.figure(1)
        # ax = fig.add_subplot(projection='3d')
        bot_vert_stack = np.row_stack((self.bottom_vertices(), self.bottom_vertices()[0]))
        self.plot3(bot_vert_stack[:, 0], bot_vert_stack[:, 1], bot_vert_stack[:, 2], zlim=self.zlims, fig=fig)
        # ax.hold = True
        self.plot3(0, 0, 0, zlim=self.zlims)

        self.put_lim()
        # ax.set_zlim(self.zlims)

    def plot_top_vertices_global_3D(self):
        top_vert_stack = np.row_stack((self.T_global, self.T_global[0]))
        self.plot3(top_vert_stack[:, 0], top_vert_stack[:, 1], top_vert_stack[:, 2])
        self.put_lim()
        self.put_centre3D(self.T_centre_global[0], self.T_centre_global[1], self.T_centre_global[2])

    def plot_top_vertices_local_3D(self):
        top_vert_local_stack = np.row_stack((self.top_vertices_local(), self.top_vertices_local()[0]))
        self.plot3(top_vert_local_stack[:, 0], top_vert_local_stack[:, 1], top_vert_local_stack[:, 2])
        self.put_lim()
        self.put_centre3D(0, 0, 0)

    def put_centre(self, x_cen, y_cen):
        self.plot(y_cen, x_cen, xlim=self.xlims, ylim=self.ylims, marker=".")

    def put_centre3D(self, x_cen, y_cen, z_cen):
        self.plot3(x_cen, y_cen, z_cen, xlim=self.xlims, ylim=self.ylims, marker=".")

    def show_plot(self):
        plt.grid(axis="both")
        plt.show()

    def put_lim(self):
        plt.xlim(self.xlims)
        plt.ylim(self.ylims)
        try:
            plt.zlim(self.zlims)
        except:
            pass

    # zmnove..................................................................
    def joint_variables(self):
        for i in range(0, 6):
            # T = self.Rx_clock_dev[i].dot(self.T_global[i, 0:3] - self.B_origin[i, 0:3])
            # T = self.Rx_clock_dev[i].dot(self.T_global[i, 0:3] - self.B_origin[i, 0:3])
            # print(T)
            T = self.t_with_b[i, 0:3]
            # print(T)
            th3 = m.asin(T[0] / self.rod_length)
            th3 = (180 / m.pi) * th3
            pl2 = self.rod_length * m.cos(th3)
            dev_val = ((T[1] * T[1]) + (T[2] * T[2]) - ((self.crank_length * self.crank_length) + (pl2 * pl2))) / (
                        2 * self.crank_length * pl2)
            # print(f"dev_val  {dev_val}")
            th2 = m.acos(dev_val)
            th2 = (180 / m.pi) * th2
            r = m.sqrt(
                (self.crank_length * self.crank_length) + (pl2 * pl2) + (2 * self.crank_length * pl2 * m.cos(th2)))
            phi = m.atan((pl2 * m.sin(th2) / (self.crank_length + pl2 * m.cos(th2))))
            # phi = m.atan2((self.crank_length + pl2 * m.cos(th2)), (pl2 * m.sin(th2)))
            # print(phi)
            # print(T[0] / r)
            print((T[0] / r) - phi)
            th1 = m.acos((T[1] / r) - phi)
            th1_deg = th1 * (180 / m.pi)
            print(th1_deg)
            self.crank_angle.append(th1_deg)
        return self.crank_angle

    def transform(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0, vector=[0, 0, 0, 1]):
        t_mat = self.transformation_matrix(px, py, (pz - self.sp_height), roll, pitch, yaw)
        # print(t_mat)
        vector_out = (t_mat.dot(np.array(vector)))
        # print(vector_out)
        return vector_out

    # def revers_tf(self):
    #     for i in range(0, 6):
    #         bottom_points = self.B_origin[i, :]
    #         val = self.transform(-bottom_points[0], -bottom_points[1], -bottom_points[2], 0, 0, (-self.rangle_vectclock[i] * (180/m.pi)), self.T_global[i, :])
    #         # print(val)
    #         self.tf_ret = np.row_stack((self.tf_ret, val))
    #     self.tf_ret = self.tf_ret[1:7, :]
    #     return self.tf_ret

    def revers_tf(self):
        val = [[], [], [], [], [], []]
        for i in range(0, 6):
            # bottom_points = -self.B_origin[i, :]
            # bottom_points = self.axis_rotation(self.rangle_vectclock[i], "z").dot(-self.B_origin[i, :])
            bottom_points = self.transform(0, 0, 0, 0, 0, -self.rangle_vectclock[i] * (180 / m.pi),
                                           -self.B_origin[i, :])
            # print(bottom_points)
            # print(f"dev {i}")
            # print(self.T_global[i, :])
            val[i] = self.transform(bottom_points[0], bottom_points[1], bottom_points[2], 0, 0,
                                    (-self.rangle_vectclock[i] * (180 / m.pi)), self.T_global[i, :])
            # print(val[i])
        self.t_with_b = np.array(val)
        return self.t_with_b

    def joint_variables_new(self, top_vertises_matrix, base_vertises_matrix):
        # T = [item.dot(top_vertises_matrix[:, self.Rx_clock_dev.index(item)] - base_vertises_matrix[:, self.Rx_clock_dev.index(item)]) for item in self.Rx_clock_dev]
        T = self.t_with_b[:, 0:3]
        th3 = [-m.asin(item[2] / self.rod_length) for item in T]
        pl2 = [self.rod_length * m.cos(item) for item in th3]
        th2 = [m.acos((T[pl2.index(item)][0] * T[pl2.index(item)][0] + T[pl2.index(item)][1] * T[pl2.index(item)][
            1] - self.crank_length * self.crank_length - item * item) / (2 * self.crank_length * item)) for item in pl2]
        r = [m.sqrt(self.crank_length * self.crank_length + pl2[th2.index(item)] * pl2[
            th2.index(item)] + 2 * self.crank_length * pl2[th2.index(item)] * m.cos(item)) for item in th2]
        phi = [m.atan((self.crank_length + pl2[th2.index(item)] * m.cos(item)) / (pl2[th2.index(item)] * m.sin(item)))
               for item in th2]
        th1 = [m.asin(T[phi.index(item)][1] / r[phi.index(item)]) + item for item in phi]
        self.crank_angle = [(item * 180 / m.pi) for item in th1]

    def print_motor_frames(self):
        motor_frame = self.Rx_clock_dev
        motor_frames = [pd.DataFrame(item) for item in motor_frame]
        motor_frames_disp = [print(i) for i in motor_frames]

    def dev(self):
        # time.sleep(2)
        zero_corr = [-28.35 + 24.75, 16.22 - 24.75, -39.6 + 24.75, 26.77 - 24.75, -33.1 + 24.75, 14.64 - 24.75]
        firstflag = 0

        # Define 6RUS Zamanov design parameter
        base_length = 0.142
        top_length = 0.067
        d_b = 0.23
        d_t = 0.25
        l1 = 0.075
        l2 = 0.23

        inter_value = ((d_b / 2 + base_length / 4) * 2) / m.sqrt(3)
        r_b = m.sqrt(inter_value * inter_value + (base_length / 2) * (base_length / 2))
        rem_angle = 2 * m.asin(base_length / (2 * r_b))
        half_angle = (2 * m.pi / 3) - rem_angle
        inter_value2 = ((d_t / 2 + top_length / 4) * 2) / m.sqrt(3)
        r_p = m.sqrt(inter_value2 * inter_value2 + (top_length / 2) * (top_length / 2))
        rem_angle_top = 2 * m.asin(top_length / (2 * r_p))
        theta_p = (2 * m.pi / 3) - rem_angle_top

        # Define Base Platform
        # Vertex 1 of base platform
        b1 = np.array([[float(0)], [-float(base_length) / (2 * m.tan(rem_angle / 2))], [-float(base_length) / 2]])
        # Vertex 2 of base platform
        rotangle = rem_angle
        Rx_clock = np.array(
            [[1, 0, 0], [0, m.cos(rotangle), m.sin(rotangle)], [0, -m.sin(rotangle), m.cos(rotangle)]])
        b2 = Rx_clock.dot(b1)
        # Defining the rest vertices of base platform
        rotangle = rem_angle + half_angle
        Rx_clock = np.array(
            [[1, 0, 0], [0, m.cos(rotangle), m.sin(rotangle)], [0, -m.sin(rotangle), m.cos(rotangle)]])
        b3 = Rx_clock.dot(b1)
        b4 = Rx_clock.dot(b2)
        b5 = Rx_clock.dot(b3)
        b6 = Rx_clock.dot(b4)

        # b1 to b6 are column vectors
        Base_matrix = np.concatenate((b1, b2, b3, b4, b5, b6), axis=1)
        ex = 0.000025 + .10
        ey = 0.000025
        ez = 0.000025
        roll = 0
        pitch = 0
        yaw = 0
        translate = np.array([[ex], [ey], [ez]])
        R_roll = np.array([[m.cos(roll), -m.sin(roll), 0], [m.sin(roll), m.cos(roll), 0], [0, 0, 1]])
        R_yaw = np.array([[1, 0, 0], [0, m.cos(yaw), -m.sin(yaw)], [0, m.sin(yaw), m.cos(yaw)]])
        R_pitch = np.array([[m.cos(pitch), 0, m.sin(pitch)], [0, 1, 0], [-m.sin(pitch), 0, m.cos(pitch)]])

        # Defining the top platform
        t1 = np.array([[0], [-r_p * m.cos(((2 * m.pi / 3) - theta_p) / 2)],
                       [-r_p * m.sin(((2 * m.pi / 3) - theta_p) / 2)]])
        # Vertex 2 of top platform
        rotangle = (2 * m.pi / 3) - theta_p
        Rx_clock = np.array(
            [[1, 0, 0], [0, m.cos(rotangle), m.sin(rotangle)], [0, -m.sin(rotangle), m.cos(rotangle)]])
        t2 = Rx_clock.dot(t1)
        # Defining the rest vertices of top platform
        rotangle = 2 * m.pi / 3
        Rx_clock = np.array(
            [[1, 0, 0], [0, m.cos(rotangle), m.sin(rotangle)], [0, -m.sin(rotangle), m.cos(rotangle)]])
        t3 = Rx_clock.dot(t1)
        t4 = Rx_clock.dot(t2)
        t5 = Rx_clock.dot(t3)
        t6 = Rx_clock.dot(t4)
        t1f = R_roll.dot(R_pitch).dot(R_yaw).dot(t1) + translate
        t2f = R_roll.dot(R_pitch).dot(R_yaw).dot(t2) + translate
        t3f = R_roll.dot(R_pitch).dot(R_yaw).dot(t3) + translate
        t4f = R_roll.dot(R_pitch).dot(R_yaw).dot(t4) + translate
        t5f = R_roll.dot(R_pitch).dot(R_yaw).dot(t5) + translate
        t6f = R_roll.dot(R_pitch).dot(R_yaw).dot(t6) + translate
        Top_matrix = np.concatenate((t1f, t2f, t3f, t4f, t5f, t6f), axis=1)
        return Top_matrix, Base_matrix

    def slop(self):
        slop_rad = m.atan((self.B_origin[3, 1] - self.B_origin[3, 0]) / (self.B_origin[2, 1] - self.B_origin[2, 0]))
        slop_rad_3 = m.atan((self.B_origin[3, 1] - self.B_origin[3, 0]) / (self.B_origin[2, 1] - self.B_origin[2, 0]))
        slop_deg = slop_rad * (180 / np.pi)
        print(slop_deg)
        print(slop_rad)
        print(slop_rad_3)

    # Plotting of reference frames...........................................
    def plot_ref_2d(self, data):
        x = data[0]
        y = data[1]
        self.plot(x[1], x[0], c="r")
        self.plot(y[1], y[0], c="g")

    def plot_ref(self, px=0, py=0, pz=0, roll=0, pitch=0, yaw=0):
        t_mat = self.transformation_matrix(px, py, (pz - self.sp_height), roll, pitch, yaw)
        ref_point_x = (t_mat.dot(np.array([self.ref_axis_len, 0, 0, 1])))
        ref_point_y = (t_mat.dot(np.array([0, self.ref_axis_len, 0, 1])))
        ref_point_z = (t_mat.dot(np.array([0, 0, self.ref_axis_len, 1])))
        data_x = [[px, ref_point_x[0]], [py, ref_point_x[1]]]
        data_y = [[px, ref_point_y[0]], [py, ref_point_y[1]]]
        data_z = [[px, ref_point_z[0]], [py, ref_point_z[1]]]
        data = np.array([data_x, data_y, data_z])
        self.plot_ref_2d(data)

    def plot_ref_global(self):
        self.plot_ref(0, 0, 0, 0, 0, 0)

    def plot_ref_top_origin(self):
        top_points = self.T_centre_global
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_bottom_1(self):
        bottom_points = self.B_origin[0, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[0] * (180 / m.pi))

    def plot_ref_bottom_2(self):
        bottom_points = self.B_origin[1, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[1] * (180 / m.pi))

    def plot_ref_bottom_3(self):
        bottom_points = self.B_origin[2, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[2] * (180 / m.pi))

    def plot_ref_bottom_4(self):
        bottom_points = self.B_origin[3, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[3] * (180 / m.pi))

    def plot_ref_bottom_5(self):
        bottom_points = self.B_origin[4, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[4] * (180 / m.pi))

    def plot_ref_bottom_6(self):
        bottom_points = self.B_origin[5, :]
        self.plot_ref(bottom_points[0], bottom_points[1], 0, 0, 0, self.rangle_vectclock[5] * (180 / m.pi))

    def plot_ref_top_1(self):
        top_points = self.T_global[0, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_top_2(self):
        top_points = self.T_global[1, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_top_3(self):
        top_points = self.T_global[2, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_top_4(self):
        top_points = self.T_global[3, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_top_5(self):
        top_points = self.T_global[4, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_top_6(self):
        top_points = self.T_global[5, :]
        orient = self.orientation
        self.plot_ref(top_points[0], top_points[1], top_points[2], orient[0], orient[1], orient[2])

    def plot_ref_bottom_all_2d(self):
        self.plot_ref_bottom_1()
        self.plot_ref_bottom_2()
        self.plot_ref_bottom_3()
        self.plot_ref_bottom_4()
        self.plot_ref_bottom_5()
        self.plot_ref_bottom_6()
        self.plot_ref_global()

    def plot_ref_top_all_2d(self):
        # self.plot_ref_top_1()
        # self.plot_ref_top_2()
        # self.plot_ref_top_3()
        # self.plot_ref_top_4()
        # self.plot_ref_top_5()
        # self.plot_ref_top_6()
        self.plot_ref_top_origin()

    def plot_ref_all_2d(self):
        self.plot_ref_bottom_all_2d()
        self.plot_ref_top_all_2d()

    # function for finding roots
    def sridharacharya_roots(self, a, b, c):

        # calculating discriminant using formula
        dis = b * b - 4 * a * c
        sqrt_val = m.sqrt(abs(dis))

        # checking condition for discriminant
        if dis > 0:
            print("real and different roots")
            print((-b + sqrt_val) / (2 * a))
            print((-b - sqrt_val) / (2 * a))
            root_1 = (-b + sqrt_val) / (2 * a)
            root_2 = (-b - sqrt_val) / (2 * a)

        elif dis == 0:
            print("real and same roots")
            print(-b / (2 * a))
            root_1 = -b / (2 * a)

            # when discriminant is less than 0
        else:
            print("Complex Roots")
            root_1 = 0
            root_2 = 0
        return root_1, root_2

    def abc_return(self):
        a = self.t_with_b[:, 1]
        # print(a)
        b = self.t_with_b[:, 2]
        # print(b)
        vx_squire = self.t_with_b[:, 0] * self.t_with_b[:, 0]
        # print(vx_squire)
        vy_squire = self.t_with_b[:, 1] * self.t_with_b[:, 1]
        # print(vy_squire)
        vz_squire = self.t_with_b[:, 2] * self.t_with_b[:, 2]
        # print(vz_squire)
        l1_squire = self.crank_length * self.crank_length
        l2_squire = self.rod_length * self.rod_length
        # print(2 * self.crank_length)
        c = ((vx_squire + vy_squire + vz_squire + l1_squire) - l2_squire) / (2 * self.crank_length)
        # print(c)
        return a, b, c

    def joint_angle_dev(self, a, b, c):
        c = (c - a)
        a = a + b
        b = (-2 * b)
        # print(a)
        # print(b)
        # print(c)
        # calculating discriminant using formula
        dis = b * b - 4 * a * c
        # print(dis)

        # checking condition for discriminant
        if dis > 0:
            # print("real and different roots")
            theta_11 = 2 * m.atan2((-b + m.sqrt(b ** 2 - 4 * a * c)), (2 * a))
            theta_12 = 2 * m.atan2((-b - m.sqrt(b ** 2 - 4 * a * c)), (2 * a))

        elif dis == 0:
            # print("real and same roots")
            theta_11 = 2 * m.atan2((-b + m.sqrt(b ** 2 - 4 * a * c)), (2 * a))
            theta_12 = 2 * m.atan2((-b - m.sqrt(b ** 2 - 4 * a * c)), (2 * a))

            # when discriminant is less than 0
        else:
            print("Complex Roots")
            theta_11 = "complex"
            theta_12 = "complex"
        theta_11 = theta_11 * 180 / m.pi
        theta_12 = theta_12 * 180 / m.pi
        return theta_11, theta_12

    def crank_angle_dev(self):
        a, b, c = self.abc_return()
        theta_1, theta_2 = [], []
        for i in range(0, 6):
            aa = self.joint_angle_dev(a[i], b[i], c[i])
            theta_1 = np.append(theta_1, aa[0])
            theta_2 = np.append(theta_2, aa[1])
        return theta_1, theta_2

    def maping_dev(self, mapent, y2, y1, x2, x1):
        slop = ((y2 - y1) / (x2 - x1))
        value = slop * (mapent - x1) + y1
        return value

    def crank_angle_dev_true(self):
        zita_1, zita_2 = self.crank_angle_dev()
        for k in [0, 2, 4]:
            zita_2[k] = self.maping_dev(zita_2[k], 360, 180, -90, 90)
        for k in [1, 3, 5]:
            zita_2[k] = self.maping_dev(zita_2[k], 0, 180, -90, 90)
        return zita_2

    def crank_angle_dev_mirror(self):
        zita_1, zita_2 = self.crank_angle_dev()
        for k in [1, 3, 5]:
            zita_1[k] = self.maping_dev(zita_1[k], 360, 180, -90, 90)
        for k in [0, 2, 4]:
            zita_1[k] = self.maping_dev(zita_1[k], 0, 180, -90, 90)
        return zita_1

# tregectory............................................................
    def ik_of_trajectory_dev(self, robot, ):
        z_motion = range(-5, 5, 1)
        joint_set = np.zeros(6)
        for i in z_motion:
            robot.inv_k([0, 0, i, 0, 0, 0])
            joint_set = np.row_stack((joint_set, robot.crank_angle))
        return joint_set

    def tregectory_time(self):
        treg_time = np.arange(0, self.motion_time, 0.1, dtype=float)
        return treg_time

    def tregectory_gen(self):
        t = self.tregectory_time()
        x_motion = [4.5 * m.sin(3 * t) for t in t]
        y_motion = [4.5 * m.cos(3 * t) for t in t]
        z_motion = [2.5 * m.cos(3 * t) for t in t]
        roll = [9.5 * m.sin(1.5 * t) for t in t]
        pitch = [9.5 * m.cos(1.5 * t) for t in t]
        yaw = [4.5 * m.cos(1.5 * t) for t in t]
        size = np.size(z_motion)
        task_set = np.array([x_motion, y_motion, z_motion, roll, pitch, yaw]).T
        return task_set

    def tregectory_gen_2(self):
        z_motion = range(-5, 5, 1)
        size = np.size(z_motion)
        task_set = np.array([np.zeros(size), np.zeros(size), np.array(z_motion), np.zeros(size), np.zeros(size), np.zeros(size)]).T
        return task_set

    def ik_of_trajectory(self):
        task_tregectory = self.tregectory_gen()
        joint_set = np.zeros(6)
        for instant in task_tregectory:
            self.inv_k(instant)
            joint_set = np.row_stack((joint_set, self.crank_angle))
        return joint_set[1:]
# tregectory ploting............................................................

    def task_to_joint_set(self):
        task_to_joints = np.column_stack((self.task_set, self.joint_set))
        return task_to_joints

    def tregectory_ploting(self):
        t = self.tregectory_time()
        plt.figure(2)
        plt.subplot(1, 1, 1)
        self.plot(self.joint_set[:, 0], t, legends=self.legend_joint_pos[0])
        self.plot(self.joint_set[:, 1], t, legends=self.legend_joint_pos[1])
        self.plot(self.joint_set[:, 2], t, legends=self.legend_joint_pos[2])
        self.plot(self.joint_set[:, 3], t, legends=self.legend_joint_pos[3])
        self.plot(self.joint_set[:, 4], t, legends=self.legend_joint_pos[4])
        self.plot(self.joint_set[:, 5], t, legends=self.legend_joint_pos[5])
        plt.xlabel("Time (sec)")
        plt.ylabel("Joint position (°)")
        plt.xlim([0, t[-1]])
        plt.ylim([0, 360])
        plt.grid()
        # plt.legend(loc="upper right", ncol=6)
        plt.legend(bbox_to_anchor=(.98, 1.1), ncol=6)
        
    def tregectory_ploting_task(self):
        t = self.tregectory_time()
        plt.figure(3)
        plt.subplot(2, 3, 1)
        self.plot(self.task_set[:, 0], t, legends=self.legend_task_pos[0], xlabel="Time (sec)", ylabel="X-axis (cm)", c="b")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.subplot(2, 3, 2)
        self.plot(self.task_set[:, 1], t, legends=self.legend_task_pos[1], xlabel="Time (sec)", ylabel="Y-axis (cm)", c="g")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.subplot(2, 3, 3)
        self.plot(self.task_set[:, 2], t, legends=self.legend_task_pos[2], xlabel="Time (sec)", ylabel="Z-axis (cm)", c="r")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.subplot(2, 3, 4)
        self.plot(self.task_set[:, 3], t, legends=self.legend_task_pos[3], xlabel="Time (sec)", ylabel="Roll about X-axis (°)", c="m")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.subplot(2, 3, 5)
        self.plot(self.task_set[:, 4], t, legends=self.legend_task_pos[4], xlabel="Time (sec)", ylabel="Pitch about Y-axis (°)", c="c")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.subplot(2, 3, 6)
        self.plot(self.task_set[:, 5], t, legends=self.legend_task_pos[5], xlabel="Time (sec)", ylabel="Yaw about Z-axis (°)", c="m")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.98, 1.1))
        plt.grid()
        plt.grid()

    def tregectory_ploting_task_2(self):
        t = self.tregectory_time()
        plt.figure(3)
        plt.subplot(2, 1, 1)
        self.plot(self.task_set[:, 0], t, legends=self.legend_task_pos[0], xlabel="Time (sec)", ylabel="X-axis (cm)", c="b")
        self.plot(self.task_set[:, 1], t, legends=self.legend_task_pos[1], xlabel="Time (sec)", ylabel="Y-axis (cm)", c="g")
        self.plot(self.task_set[:, 2], t, legends=self.legend_task_pos[2], xlabel="Time (sec)", ylabel="Z-axis (cm)", c="r")
        plt.xlabel("Time (sec)")
        plt.ylabel("Linear position (cm)")
        plt.xlim([0, t[-1]])
        # plt.legend(bbox_to_anchor=(.6, 1.1), ncol=3)
        plt.legend(loc="upper right", ncol=3)
        plt.grid()
        plt.subplot(2, 1, 2)
        self.plot(self.task_set[:, 3], t, legends=self.legend_task_pos[3], xlabel="Time (sec)", ylabel="Roll about X-axis (°)", c="m")
        self.plot(self.task_set[:, 4], t, legends=self.legend_task_pos[4], xlabel="Time (sec)", ylabel="Pitch about Y-axis (°)", c="c")
        self.plot(self.task_set[:, 5], t, legends=self.legend_task_pos[5], xlabel="Time (sec)", ylabel="Yaw about Z-axis (°)", c="orange")
        plt.xlabel("Time (sec)")
        plt.ylabel("Angular orientation (°)")
        plt.xlim([0, t[-1]])
        # plt.legend(bbox_to_anchor=(.6, 1), ncol=3)
        plt.legend(loc="upper right", ncol=3)
        plt.grid()
        plt.grid()

    def tregectory_ploting_task_3(self):
        t = self.tregectory_time()
        plt.figure(5)
        plt.subplot(1, 1, 1)
        self.plot(self.task_set[:, 0], t, legends=self.legend_task_pos[0], xlabel="Time (sec)", ylabel="X-axis (cm)", c="b")
        self.plot(self.task_set[:, 1], t, legends=self.legend_task_pos[1], xlabel="Time (sec)", ylabel="Y-axis (cm)", c="g")
        self.plot(self.task_set[:, 2], t, legends=self.legend_task_pos[2], xlabel="Time (sec)", ylabel="Z-axis (cm)", c="r")
        plt.xlabel("Time (sec)")
        plt.ylabel("Linear position (cm)")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.6, 1.1), ncol=3)
        # plt.legend(loc="upper right", ncol=3)
        plt.grid()
        plt.figure(6)
        plt.subplot(1, 1, 1)
        self.plot(self.task_set[:, 3], t, legends=self.legend_task_pos[3], xlabel="Time (sec)", ylabel="Roll about X-axis (°)", c="m")
        self.plot(self.task_set[:, 4], t, legends=self.legend_task_pos[4], xlabel="Time (sec)", ylabel="Pitch about Y-axis (°)", c="c")
        self.plot(self.task_set[:, 5], t, legends=self.legend_task_pos[5], xlabel="Time (sec)", ylabel="Yaw about Z-axis (°)", c="orange")
        plt.xlabel("Time (sec)")
        plt.ylabel("Angular orientation (°)")
        plt.xlim([0, t[-1]])
        plt.legend(bbox_to_anchor=(.6, 1.1), ncol=3)
        # plt.legend(loc="upper right", ncol=3)
        plt.grid()
        plt.grid()