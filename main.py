x_product = [-0.19, 1.9]  # Position of the product on the plane
import matplotlib

import numpy as np
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
from mpl_toolkits import mplot3d
from matplotlib.animation import FuncAnimation
import math as m


class robot_arm():
    l1 = 2.3  # 1st arm length
    l2 = 2  # 2st arm length
    l3 = 2.2  # 3st arm length

    center = [0, 1.5, 0.1]  # Coordinated of the center of the third circle
    ro = 0.2  # Radius of the circles
    height = 0.2
    lim_min = np.array([-np.pi, -np.pi, -np.pi])  # Joints min angles
    lim_max = np.array([np.pi, np.pi, np.pi])  # Joints max angles

    def init(self):
        pass

    def gen_rand_conf(self):
        """ Creates three random angles within the limits of the robot """
        rnd = np.random.random((3,))  # Generate 3 random numbers from 0 to 1
        rnd_thetas = rnd * (self.lim_max - self.lim_min).reshape((3,)) + self.lim_min.reshape((3,))

        return rnd_thetas  # Return a random vector with 3 angles in the range of each joint

    def direct_kinematics(self, q):
        x = np.cos(q[0]) * (self.l2 * np.cos(q[1]) + self.l3 * np.cos(q[1] + q[2]))
        y = np.sin(q[0]) * (self.l2 * np.cos(q[1]) + self.l3 * np.cos(q[1] + q[2]))
        z = self.l1 + (self.l2 * np.sin(q[1]) + self.l3 * np.sin(q[1] + q[2]))
        return np.array([x, y, z])  # Position of end-effector

    def inverse_kinematics(self, position):
        theta1 = m.atan2(position[1], position[0])
        ex = position[0] / m.cos(theta1)
        ez = position[2] - self.l1
        theta3 = m.acos(((ex * ex) + (ez * ez) - (self.l1 * self.l1) - (self.l2 * self.l2)) / (2 * self.l1 * self.l2))
        theta2 = m.atan2(ez, ex) - m.atan2(self.l3 * m.sin(theta3), self.l1 + self.l2 * m.cos(theta3))
        angles = np.array([theta1, theta2, theta3])
        return angles

    def get_indix_of_min_distance_point(self, Q_free, point):
        mini = np.inf  # Initialize mini to a very large number
        min_index = 0  # Initialize the index of the minimum distance element
        for i, q in enumerate(Q_free):  # Loop through all elements in Q_free
            distance = np.linalg.norm(q - point)  # Calculate the distance between q and point
            if distance < mini:  # If the distance is smaller than the current minimum
                mini = distance  # Update the minimum distance
                min_index = i  # Update the index of the minimum distance element
        return min_index  # Retur

    def find_path(self, X_free, start,
                  end):  # this func finds the point of the path by searching the points is x-free array which we find randomly
        test = X_free.copy()  # Make a copy of x_free to modify within the loop
        path = np.zeros((0, 3))  # Initialize the path array
        current_point = start.copy()  # Initialize the current point
        path = np.insert(path, 0, start, axis=0)  # Add the start point to the path
        threshold = 0.005
        while np.linalg.norm(end - current_point) > 0 and len(test) > 0:  # Loop until test is empty
            min_index = self.get_indix_of_min_distance_point(test,
                                                             current_point)  # Find the index of the closest point in test
            min_point = test[min_index]  # Find the closest point
            min_distance = np.linalg.norm(
                min_point - current_point)  # Calculate the distance between the closest point and the current point
            if np.linalg.norm(min_point - end) > np.linalg.norm(
                    current_point - end):  # If the closest point is farther from the end than the current point
                test = np.delete(test, min_index, axis=0)  # Remove the closest point from test
                continue  # Skip the rest of the iteration and move to the next one
            path = np.insert(path, 0, min_point, axis=0)  # Add the closest point to the path
            current_point = min_point.copy()  # Update the current point
            test = np.delete(test, min_index, axis=0)
        path = np.insert(path, 0, end, axis=0)  # Add the end point to the path
        return np.array(path)  # Return the path as a numpy array

    def collision_checker(self, x, circle_nb):
        """ If x is outside of the upper or lower bound or if it is inside of a circle,
        return that there is a collision. """
        position = np.array([x[0], x[1], 0])
        CDist = np.linalg.norm(position - self.center) - self.ro  # Distance to the center
        bondDist = m.sqrt((x[0] * x[0]) + (x[1] * x[1]))
        if CDist < 0:
            if x[2] < 0.2:
                Dist = 0
                return Dist
        if bondDist < 1:
            Dist = 0
            return Dist
        if bondDist > 2:
            Dist = 0
            return Dist
        if x[1] < 0:
            Dist = 0
            return Dist
        if x[2] < 0.005:
            Dist = 0
            return Dist

        Dist = CDist  # No collision, save the shortest distance to an object
        return Dist

    def insert_points(self, path, n_new_points):  # this function connect between 2 points
        new_path = []
        time = []
        t = 0
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            step = (p2 - p1) / (n_new_points + 1)
            new_path.append(p1)
            time.append(t)
            t += 1
            for j in range(1, n_new_points + 1):
                new_path.append(p1 + j * step)
                time.append(t)
                t += 1
        new_path.append(path[-1])
        time.append(t)
        return np.array(new_path), np.array(time)

    def get_path_angles(self, path):
        angles = np.zeros((0, 3))
        for point in path:
            angle = self.inverse_kinematics(point)
            angles = np.insert(angles, 0, angle, axis=0)

        return angles

    def calculate_angles(self, path):
        angles = []
        for point in path:
            x, y, z = point
            angle = m.atan2(np.sqrt(x * 2 + y * 2), z)
            angles.append(angle)
        return np.array(angles)

        ############################################################################################################

    ############################################## Question A ##############################################
    ############################################################################################################

    def Q_X_free(self, circle_nb):
        """ Builds a 30 000 point cloud in the Q space (angle of each joint) and in the X space (x-y plane)
        to show all the possible positions of the end effector"""
        Q_free = np.zeros((0, 3))  # Array of valid angles and angles
        free_angles = np.zeros((0, 3))
        X_free = np.zeros((0, 3))  # Array of valid positions and angles
        obsticle = np.zeros((0, 3))
        Dist = np.zeros((0, 1))  # Distance from valid points to the nearest obstacle
        # Fills 30 000 points that are not in collision into Q_free, X_free and Dist
        while len(Q_free) < 30000:
            q = self.gen_rand_conf()  # Get 3 random joint angles
            x = self.direct_kinematics(q)  # Get the end effector position
            dist = self.collision_checker(x, circle_nb)  # Get the distance to the nearest collision
            if dist > 0:
                Q_free = np.insert(Q_free, 0, q, axis=0)  # Add the vector to the angle array
                X_free = np.insert(X_free, 0, x, axis=0)  # Add the vector to the position array
                Dist = np.insert(Dist, 0, dist, axis=0)  # Add the distance to the distance array
            else:
                position = np.array([x[0], x[1], 0])
                CDist = np.linalg.norm(position - self.center) - self.ro  # Distance to the center
                if CDist < 0:
                    if x[2] < 0.2:
                        obsticle = np.insert(obsticle, 0, x, axis=0)  # Add the vector to the position array
        start = np.array([1.5, 0, 0.1])
        global x_product
        end = np.zeros(3)
        end[0] = x_product[0]
        end[1] = x_product[1]
        end[2] = 0.1
        X_free = np.insert(X_free, 0, end, axis=0)  # Add the vector to the position array
        dist = self.collision_checker(end, circle_nb)  # Get the distance to the nearest collision
        Dist = np.insert(Dist, 0, dist, axis=0)  # Add the distance to the distance array

        path = self.find_path(X_free, start, end)
        new_path, time = self.insert_points(path, 35)
        # define center, radius, and height
        center = np.array([0, 1.5, 0])
        radius = 0.2
        height = 0.2
        theta = np.linspace(0, 2 * np.pi, 100)
        z = np.linspace(0, height, 100)
        theta_grid, z_grid = np.meshgrid(theta, z)
        x_cylinder = center[0] + radius * np.cos(theta_grid)
        y_cylinder = center[1] + radius * np.sin(theta_grid)
        z_cylinder = z_grid + center[2]
        # plot cylinder
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        s_point = new_path[0]
        e_point = new_path[-1]
        start_point = np.zeros((0, 3))
        start_point = np.insert(start_point, 0, s_point, axis=0)
        end_point = np.zeros((0, 3))
        end_point = np.insert(end_point, 0, e_point, axis=0)
        ax.scatter(new_path[:, 0], new_path[:, 1], new_path[:, 2], c='black', s=5, marker='.')
        ax.scatter(start_point[:, 0], start_point[:, 1], start_point[:, 2], c='blue', s=15, marker='o')
        ax.scatter(end_point[:, 0], end_point[:, 1], end_point[:, 2], c='red', s=15, marker='o')
        ax.plot_surface(x_cylinder, y_cylinder, z_cylinder, color='b')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_facecolor("#1DD4AF")
        ax.set_title('Robot_path')
        plt.show()

        ############################################################################################################
        ############################################## Question B ##############################################
        ############################################################################################################
        angles_relate_to_z = self.calculate_angles(new_path)
        fig, ax = plt.subplots()
        ax.plot(time, angles_relate_to_z)
        ax.set_xlabel('Time')
        ax.set_ylabel('Angle (radians)')
        ax.set_title('Angle_Relative_To_Z_AXIS vs Time')
        plt.show()
        ############################################################################################################
        ############################################## Question c ##############################################
        ############################################################################################################
        angles_of_robot = self.get_path_angles(new_path)
        fig, ax = plt.subplots()
        ax = plt.axes(projection='3d')
        ax.scatter(angles_of_robot[:, 0], angles_of_robot[:, 1], angles_of_robot[:, 2], c=time, cmap='viridis', s=3)
        ax.set_xlabel('theta1')
        ax.set_ylabel('theta2')
        ax.set_zlabel('theta3')
        ax.set_title('Robot_joins_angles vs Time')
        plt.show()
        ############################################################################################################
        ############################################## Question D ##############################################
        ############################################################################################################

        fig = plt.figure()
        ay = fig.add_subplot(111, projection='3d')
        ay.plot_surface(x_cylinder, y_cylinder, z_cylinder, color='b', alpha=0.5)
        ani = FuncAnimation(fig, lambda i: ay.scatter(new_path[:i, 0], new_path[:i, 1], new_path[:i, 2]),
                            frames=len(new_path), repeat=False, interval=2)
        ay.plot_surface(x_cylinder, y_cylinder, z_cylinder, color='b', alpha=0.5)
        plt.show()
        return ani


abbas = robot_arm()
abbas.Q_X_free(1)
