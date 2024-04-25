#!/usr/bin/env python3
"""
@file: bezier_curve.py
@breif: Bezier curve generation
@author: Winter
@update: 2023.7.25
"""
import numpy as np
import rospy
import tf2_ros
import tf
import math
import numpy as np
import math
from scipy.special import comb
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, QuaternionStamped, Point, Quaternion, PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion

class Bezier():
	
    def __init__(self, step: float, offset: float):
        self.offset = offset
        self.step = step
		
        # create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)
        
        self.trigger = True
        # self.robot_radius = (0.14802 + (0.1025 / 2))
        self.robot_radius = (0.52072 + (0.1025 / 2))
        self.dock_displacement = 0.2

        # Publisher:
        self.pub_computed_path = rospy.Publisher("compute_path", Path, queue_size=10)
        
    
    def get_rotation_angle(self, p1, p2):
        v = (p2[0]-p1[0], p2[1]-p1[1])

        try:
            # Tính độ dài của vector
            norm_v = math.sqrt(v[0]**2 + v[1]**2)
        except ZeroDivisionError:
            print("Error: Division by zero occurred.")
        
        # Chuyển đổi thành vector đơn vị
        unit_v = (v[0]/norm_v, v[1]/norm_v)

        # Vector đơn vị theo trục x
        x_unit = (1, 0)

        # Tính tích vô hướng của hai vector
        dot_product = x_unit[0]*unit_v[0] + x_unit[1]*unit_v[1]

        # Tính góc
        if dot_product == 0:
            angle = 90
        else:
            angle = math.degrees(math.acos(dot_product))

        return angle
    
    def get_2D_pose(self, target_link=None, base_link=None):

        if target_link is None:
            target_link = "charger_frame"
        if base_link is None:
            base_link = "odom"

        try:
            trans = self.__tfBuffer.lookup_transform(
                base_link,
                target_link,
                rospy.Time(0), rospy.Duration(1.0))

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr(f"Failed lookup: {target_link}, from {base_link}")
            return None

        return trans

    def calc_4points_bezier_path(self, sx, sy, syaw, ex, ey, eyaw, offset):
        """
        Compute control points and path given start and end position.

        :param sx: (float) x-coordinate of the starting point
        :param sy: (float) y-coordinate of the starting point
        :param syaw: (float) yaw angle at start
        :param ex: (float) x-coordinate of the ending point
        :param ey: (float) y-coordinate of the ending point
        :param eyaw: (float) yaw angle at the end
        :param offset: (float)
        :return: (numpy array, numpy array)
        """
        dist = np.hypot(sx - ex, sy - ey) / offset
        control_points = np.array(
            [[sx, sy],
            [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
            [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
            [ex, ey]])

        path = self.calc_bezier_path(control_points, n_points=50)

        return path, control_points

    def calc_bezier_path(self, control_points, n_points=50):
        """
        Compute bezier path (trajectory) given control points.

        :param control_points: (numpy array)
        :param n_points: (int) number of points in the trajectory
        :return: (numpy array)
        """
        traj = []
        for t in np.linspace(0, 1, n_points):
            traj.append(self.bezier(t, control_points))

        return np.array(traj)
    
    def bernstein_poly(self, n, i, t):
        """
        Bernstein polynom.

        :param n: (int) polynom degree
        :param i: (int)
        :param t: (float)
        :return: (float)
        """
        return comb(n, i) * t ** i * (1 - t) ** (n - i)

    def bezier(self, t, control_points):
        """
        Return one point on the bezier curve.

        :param t: (float) number in [0, 1]
        :param control_points: (numpy array)
        :return: (numpy array) Coordinates of the point
        """
        n = len(control_points) - 1
        return np.sum([self.bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)
    
    def bezier_derivatives_control_points(self, control_points, n_derivatives):
        """
        Compute control points of the successive derivatives of a given bezier curve.

        A derivative of a bezier curve is a bezier curve.
        See https://pomax.github.io/bezierinfo/#derivatives
        for detailed explanations

        :param control_points: (numpy array)
        :param n_derivatives: (int)
        e.g., n_derivatives=2 -> compute control points for first and second derivatives
        :return: ([numpy array])
        """
        w = {0: control_points}
        for i in range(n_derivatives):
            n = len(w[i])
            w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                                for j in range(n - 1)])
        return w
    
    def curvature(self, dx, dy, ddx, ddy):
        """
        Compute curvature at one point given first and second derivatives.

        :param dx: (float) First derivative along x axis
        :param dy: (float)
        :param ddx: (float) Second derivative along x axis
        :param ddy: (float)
        :return: (float)
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)
    
    def flip_yaw(self, yaw: float) -> float:
        """
        Use this function when the target behind the robot, othewise not use!
        """
        if yaw >= 0:
            return yaw - math.pi
        else:
            return yaw + math.pi
    
    def compute_path(self):
        dock_pose = self.get_2D_pose("charger_frame")
        robot_pose = self.get_2D_pose("base_footprint")

        if (dock_pose is not None 
            and robot_pose is not None):

            point_origin = Point(x = 0.0, y = 0.0, z = 0.0)
            point        = Point(x = (self.robot_radius + self.dock_displacement), y = 0.0, z = 0.0)

            x = dock_pose.transform.translation.x
            y = dock_pose.transform.translation.y
            z = dock_pose.transform.translation.z

            dock_translation = (x, y, z)
            dock_quaternion  = dock_pose.transform.rotation

            dock_matrix = np.dot(tf.transformations.translation_matrix(dock_translation),
                                 tf.transformations.quaternion_matrix([dock_quaternion.x, dock_quaternion.y, dock_quaternion.z, dock_quaternion.w])) 
            
            origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1])
            origin_new   = np.dot(dock_matrix, origin_point)

            v         = np.array([point.x, point.y, point.z, 1])
            v_new     = np.dot(dock_matrix, v)
            point_new = Point(x = v_new[0], y = v_new[1], z = v_new[2])
            
            x_ = robot_pose.transform.translation.x
            y_ = robot_pose.transform.translation.y
            rotation = euler_from_quaternion([robot_pose.transform.rotation.x,
                                              robot_pose.transform.rotation.y,
                                              robot_pose.transform.rotation.z,
                                              robot_pose.transform.rotation.w])
            
            # If the target is behind the robot, we'll use flip_yaw function, otherwise not use
            yaw_ = self.flip_yaw(rotation[2])

            path, control_points = self.calc_4points_bezier_path(x_, y_, yaw_,
                                                                 point_new.x, point_new.y, np.deg2rad(self.get_rotation_angle(v_new,origin_new)), 3.0)

            yaw_list = []
            for i in range(len(path) - 1):
                dx = path[i + 1, 0] - path[i, 0]
                dy = path[i + 1, 1] - path[i, 1]
                yaw = np.arctan2(dy, dx)
                yaw_list.append(yaw)
            
            index = len(path) - 1
            
            yaw_list.append(np.arctan2(path[index, 1] - path[index -1, 1], path[index, 0] - path[index -1, 0]))

            path_ = Path()
            path_.header.stamp = rospy.Time.now()
            path_.header.frame_id = "odom"
            
            n = 1
            for pt in path:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "odom"
                pose_stamped.pose.position.x = pt[0]
                pose_stamped.pose.position.y = pt[1]
                pose_stamped.pose.position.z = 0.0
                rotation_matrix = tf.transformations.rotation_matrix((yaw_list[n-1]), (0, 0, 1))
                result_q = tf.transformations.quaternion_from_matrix(rotation_matrix)
                quaternion = Quaternion()
                quaternion.x = result_q[0]
                quaternion.y = result_q[1]
                quaternion.z = result_q[2]
                quaternion.w = result_q[3]
                pose_stamped.pose.orientation = quaternion
                path_.poses.append(pose_stamped)
                n += 1

            return path_

        else:
            print("Can't calculate path!")

    def main(self):

        while not rospy.is_shutdown():
            dock_pose = self.get_2D_pose("charger_frame")
            robot_pose = self.get_2D_pose("base_footprint")

            if (dock_pose is not None 
            and robot_pose is not None):
                self.trigger = True
            else:
                self.trigger = False

            if not self.trigger:
                rospy.sleep(0.05)
                continue

            path = self.compute_path()
            
            if (len(path.poses) != 0):
                self.pub_computed_path.publish(path)
            
            rospy.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("compute_path")
    try:
        compute_path = Bezier(1.0, 3.0)
        rospy.loginfo("%s is running!", rospy.get_name())
        compute_path.main()
    except rospy.ROSInterruptException:
        pass

