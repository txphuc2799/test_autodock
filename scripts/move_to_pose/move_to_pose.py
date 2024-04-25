#!/usr/bin/env python3
"""
Reference document: - Robotics Vison and Control Fundamental - Chaper 2
                    - https://github.com/felixchenfy/ros_turtlebot_control
                    - https://github.com/OkDoky/docking_module/blob/master/docking_planner/src/transform.py
                    - https://github.com/AtsushiSakai/PythonRobotics/blob/master/Control/move_to_pose/move_to_pose.py
"""

import rospy
import math
import numpy as np
import tf2_ros
import tf

from typing import Tuple
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


Pose2D = Tuple[float, float, float]

class PIDController(object):
    """
    PID Controller
    """
    def __init__(self, T, P=0.0, I=0.0, D=0.0):
        ''' Arguments
        T {float}: Control period. Unit: second.
            This is the inverse of control frequency.
        P {float or np.array}: Proportional control coefficient.
        I {float or np.array}: Integral control coefficient.
        D {float or np.array}: Differential control coefficient.
        '''

        # -- Check input data
        b1 = all(isinstance(d, float) for d in [P, I, D])
        b2 = all(isinstance(d, np.ndarray) for d in [P, I, D])
        if not b1 and not b2:
            pid_coef_types = [type(d) for d in [P, I, D]]
            err_msg = "PidController: Data type of P,I,D coefficients "\
                "are wrong: " + str(pid_coef_types)
            raise RuntimeError(err_msg)
        dim = 1 if b1 else len(P)  # Dimension of the control variable

        # -- Initialize arguments.
        self._T = T
        self._P = np.zeros(dim)+P
        self._I = np.zeros(dim)+I
        self._D = np.zeros(dim)+D
        self._err_inte = np.zeros(dim)  # Integration error.
        self._err_prev = np.zeros(dim)  # Previous error.

    def compute(self, err):
        ''' Given the error, compute the desired control value . '''

        ctrl_val = 0
        err = np.array(err)

        # P
        ctrl_val += np.dot(err, self._P)

        # I
        self._err_inte += err
        ctrl_val += self._T * np.dot(self._err_inte, self._I)

        # D
        ctrl_val += np.dot(err-self._err_prev, self._D) / self._T

        self._err_prev = err
        return ctrl_val


class Utility():

    def __init__(self):
        
        # Create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Params
        self.sleep_period = rospy.Duration(1/2.0)

        # Publisher:
        self.pub_cmd_vel_ = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        # Subscribers:
        self.sub_cancel = rospy.Subscriber("CANCEL_AMR", Bool, self.sub_cancel_cb)
        self.sub_pause  = rospy.Subscriber("PAUSE_AMR", Bool, self.sub_pause_cb)

        # Variables:
        self.is_cancel = False
        self.is_pause  = False

    
    def reset(self):
        self.is_cancel = False
        self.is_pause  = False


    def sub_pause_cb(self, msg):
        self.is_pause = True if msg.data else False
        
    
    def sub_cancel_cb(self, msg):
        self.is_cancel = True if msg.data else False
    

    def pub_cmd_vel(self, v=0.0, w=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w

        if (msg.linear.x > 0.2):
            msg.linear.x = 0.2
        elif(msg.linear.x < -0.2):
            msg.linear.x = -0.2

        if (msg.angular.x > 0.15):
            msg.angular.x = 0.15
        elif(msg.angular.x < -0.15):
            msg.angular.x = -0.15

        self.pub_cmd_vel_.publish(msg)


    def pi2pi(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    

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


class MoveToControlPose(Utility):

    def __init__(self):

        super().__init__()

        # Init params
        self.robot_radius         = rospy.get_param("~" + "robot_radius", 0.1993)
        self.dock_displacement    = rospy.get_param("~" + "dock_displacement", 0.1)
        self.max_linear_vel       = rospy.get_param("~" + "max_linear_vel", 0.05)
        self.max_angular_vel      = rospy.get_param("~" + "max_angular_vel", 0.05)
        self.xy_tolerance         = rospy.get_param("~" + "xy_tolerance", 0.015)
        self.yaw_tolerance        = rospy.get_param("~" + "yaw_tolerance", 0.03)
        self.control_period       = rospy.get_param("~" + "control_period", 0.01)
        self.controller_frequency = rospy.get_param("~" + "controller_frequency", 20)
        self.p_rho                = rospy.get_param("~" + "p_rho", 0.5)
        self.p_alpha              = rospy.get_param("~" + "p_alpha", 1.0)
        self.p_beta               = rospy.get_param("~" + "p_beta", -0.2)
        self.kp                   = rospy.get_param("~" + "kp", 2.5)
        self.kd                   = rospy.get_param("~" + "kd", 0.005)
        self.last_error           = 0.0

        self.is_behind_robot = False

        # Server:
        # rospy.Service("move_to_pose", MoveToPose, self.move_to_pose_cb)


    def PIDcontroller(self, dis_y):
        e_D = dis_y - self.last_error
        angle = self.kp*dis_y + self.kd*e_D
        self.last_error = dis_y
        
        return angle
    
    def getRotationAngle(self, p1, p2):
        v = (p2[0]-p1[0], p2[1]-p1[1])

        try:
            # Calculate the length of the vector
            norm_v = math.sqrt(v[0]**2 + v[1]**2)
        except ZeroDivisionError:
            print("Error: Division by zero occurred.")
        
        # Convert to unit vector
        unit_v = (v[0]/norm_v, v[1]/norm_v)

        # Unit vector along the x-axis
        x_unit = (1, 0)

        # Calculate the dot product of two vectors
        dot_product = x_unit[0]*unit_v[0] + x_unit[1]*unit_v[1]

        # Calculate the angle
        if dot_product == 0:
            angle = 90
        else:
            angle = math.degrees(math.acos(dot_product))

        return angle
    
    def calStartAndGoal(self, start_frame_name:str, goal_frame_name:str):
        start_pose = self.get_2D_pose(start_frame_name)
        goal_pose  = self.get_2D_pose(goal_frame_name)

        if (start_pose is None or goal_pose is None):
            rospy.logerr(f"Can not transfrom {start_frame_name} or {goal_frame_name}!")
            return rospy.signal_shutdown()

        point_origin = Point(x = 0.0, y = 0.0, z = 0.0)
        point        = Point(x = (self.robot_radius + self.dock_displacement), y = 0.0, z = 0.0)

        x = goal_pose.transform.translation.x
        y = goal_pose.transform.translation.y
        z = goal_pose.transform.translation.z

        dock_translation = (x, y, z)
        dock_quaternion  = goal_pose.transform.rotation

        dock_matrix = np.dot(tf.transformations.translation_matrix(dock_translation),
                             tf.transformations.quaternion_matrix([dock_quaternion.x, dock_quaternion.y, dock_quaternion.z, dock_quaternion.w])) 
        
        origin_point = np.array([point_origin.x, point_origin.y, point_origin.z, 1])
        origin_new   = np.dot(dock_matrix, origin_point)

        v         = np.array([point.x, point.y, point.z, 1])
        v_new     = np.dot(dock_matrix, v)
        point_new = Point(x = v_new[0], y = v_new[1], z = v_new[2])

        s_x = start_pose.transform.translation.x
        s_y = start_pose.transform.translation.y
        rotation = euler_from_quaternion([start_pose.transform.rotation.x,
                                          start_pose.transform.rotation.y,
                                          start_pose.transform.rotation.z,
                                          start_pose.transform.rotation.w])
        s_yaw = rotation[2]

        g_x   = point_new.x
        g_y   = point_new.y

        goal_direction = self.pi2pi(math.atan2(g_y - s_y, g_x - s_x) - s_yaw)

        if (abs(goal_direction) > math.pi/2):
            self.is_behind_robot = True
        else:
            self.is_behind_robot = False
        
        if (self.is_behind_robot):
            goal_rotation = euler_from_quaternion([goal_pose.transform.rotation.x,
                                                   goal_pose.transform.rotation.y,
                                                   goal_pose.transform.rotation.z,
                                                   goal_pose.transform.rotation.w])
            g_yaw = goal_rotation[2]

        else:
            g_yaw = np.deg2rad(self.getRotationAngle(v_new,origin_new))

        return (s_x, s_y, s_yaw, g_x, g_y, g_yaw)
    

    def isCloseToGoal(self, xy=None, s_yaw=None, g_yaw=None):

        b1 = True if (xy is None) else (xy <= self.xy_tolerance)
        b2 = True if (s_yaw is None and g_yaw is None) else \
             (abs(self.pi2pi(s_yaw - g_yaw)) <= self.yaw_tolerance)

        return b1 and b2


    def controlToPose(self):
        """
        Control robot to the target pose
        """
        # Init PID Controller
        loop_controller = rospy.Rate(self.controller_frequency)
        pid_rho = PIDController(self.control_period, P=self.p_rho)
        pid_alpha = PIDController(self.control_period, P=self.p_alpha)
        pid_beta = PIDController(self.control_period, P=self.p_beta)

        reached_xy_tolerance = False

        self.reset()

        while (not rospy.is_shutdown()):
            s_x, s_y, s_yaw, g_x, g_y, g_yaw = self.calStartAndGoal("base_footprint", "charger_frame")

            rho = math.hypot(g_x - s_x, g_y - s_y)
            alpha = self.pi2pi(math.atan2(g_y - s_y, g_x - s_x) - s_yaw)
            beta = self.pi2pi(g_yaw - s_yaw) - alpha

            # Check moving direction
            sign = 1
            # Check whether the goal is behind robot
            if (abs(alpha) > math.pi/2):    # The goal is behind robot
                alpha = self.pi2pi(math.pi - alpha)
                beta = self.pi2pi(math.pi - beta)
                sign = -1
            
            # PID Control
            val_rho = pid_rho.compute(err=rho)[0]
            val_alpha = pid_alpha.compute(err=alpha)[0]
            val_beta = pid_beta.compute(err=beta)[0]

            if (not reached_xy_tolerance):
                if (self.isCloseToGoal(xy=rho)):
                    reached_xy_tolerance = True
            
            if (reached_xy_tolerance):
                sign = 1  # Default to be forward.
                val_rho = 0  # No linear motion.
                val_alpha = 0  # No rotating towards target point.
                # Rotate towards target orientation.
                val_beta = self.pi2pi(g_yaw - s_yaw)
            
            # Get desired speed
            v = sign * val_rho
            w = sign * (val_alpha + val_beta)

            # Threshold on velocity
            v = min(abs(v), self.max_linear_vel) * \
                (1 if v > 0 else -1)  # limit v
            w = min(abs(w), self.max_angular_vel) * \
                (1 if w > 0 else -1)  # limit w
            
            # Publish speed
            self.pub_cmd_vel(v, w)

            print(f"Remaning distance: {rho:.2f}m.")

            if (reached_xy_tolerance):
                if (self.isCloseToGoal(s_yaw=s_yaw, g_yaw=g_yaw)):
                    print("Reached goal!")
                    break

            loop_controller.sleep()
        
        self.pub_cmd_vel()
    

if __name__ == "__main__":
    rospy.init_node("move_to_pose")
    try:
        move_to_pose = MoveToControlPose()
        rospy.loginfo("Initialized move_to_pose controller.")
        move_to_pose.controlToPose()

    except rospy.ROSInterruptException:
        pass