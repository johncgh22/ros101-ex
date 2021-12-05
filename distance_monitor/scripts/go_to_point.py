#! /usr/bin/env python3

# Programa Go to Point - Python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from distance_monitor.msg import RobotState
from tf import transformations
import math
import sys

class RobotNav():

    def __init__(self, goal_x = 5, goal_y = -5, yaw_err = math.pi/90, dist_tol = 0.1):
        self._pose2d = Pose2D()
        # Definición del Estado del Robot
        self._state_code = 0
        self._states = ['STOP', 'TWIST', 'GO', 'GOAL']
        self._goal = Pose2D()
        self._goal.x = goal_x
        self._goal.y = goal_y
        # Parametros para Controlar al Robot
        self._yaw_err = 0.0    # radianes [rad]
        self._yaw_tol = yaw_err 
        self._dist_tol = dist_tol   # metros [m]
        self._dist_to_goal = 0.0
        # Publicadores y Subscriptores
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_update)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._robot_state_pub = rospy.Publisher('/robot_state_nav', RobotState, queue_size=10)

    def _on_odom_update(self, odom_msg):
        self._pose2d.x = odom_msg.pose.pose.position.x
        self._pose2d.y = odom_msg.pose.pose.position.y
        quaternion = (
            odom_msg.pose.pose.orientation.x,  # i
            odom_msg.pose.pose.orientation.y,  # j
            odom_msg.pose.pose.orientation.z,  # k
            odom_msg.pose.pose.orientation.w   # w
        )
        euler = transformations.euler_from_quaternion(quaternion)
        self._pose2d.theta = euler[2] # Rescatammos la rotacion sobre "z"

    def _head_to_goal(self):
        self._update_goal_vector()

        # Si el Error de Orientacion en mayor al Error de la Tolerancia
        if math.fabs(self._yaw_err) > self._yaw_tol:
            twist_msg = Twist()
            twist_msg.angular.z = 0.2 if self._yaw_err > 0 else -0.2
            self._cmd_vel_pub.publish(twist_msg)
        else:
            self._state_code = 2

    def _go_to_goal(self):
        self._update_goal_vector()

        # Si la Distancia hacia la Meta es mayor a la Distancia de Tolerancia
        if self._dist_to_goal > self._dist_tol:
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            self._cmd_vel_pub.publish(twist_msg)

         # Si la Distancia hacia la Meta es menor que la Tolerancia   
         # GOAL REACHED
        else:
            self._state_code = 3

        # Validamos el Error en la Orientacion
        if math.fabs(self._yaw_err) > self._yaw_tol:
            self._state_code = 1

    def _goal_reached(self):
        rospy.loginfo('Deteniendo al Robot....')
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(1)    

    def _update_goal_vector(self):
        dx = self._goal.x - self._pose2d.x
        dy = self._goal.y - self._pose2d.y
        self._dist_to_goal = math.hypot(dy, dx)
        goal_yaw = math.atan2(dy,dx)
        self._yaw_err = goal_yaw - self._pose2d.theta

    def _publish_state(self):
        robot_state_msg = RobotState()
        robot_state_msg.state_code = self._state_code
        robot_state_msg.state_name = self._states[self._state_code]
        robot_state_msg.yaw_err = self._yaw_err
        robot_state_msg.dist_err = self._dist_to_goal
        self._robot_state_pub.publish(robot_state_msg)
    

    def execute(self):
        while self._state_code != 3:
            # Estado 0: El Robot esta detenido
            if self._state_code == 0:
                # Si el Robot esta detenido, cambia a [GO]
                self._state_code = 2
            # Estado 1: El Robot esta girando    
            if self._state_code == 1:
                # Ejecuta head_to_goal
                self._head_to_goal()  # Gira hacia la meta [GOAL]
            if self._state_code == 2:
            # Estado 2: El Robot esta avanzando hacia la meta
                # Ejecuta go_to_goal
                self._go_to_goal()    # Ve a la meta [GOAL]
            
            self._publish_state()
        
        # El Robot llegó a la meta
        if self._state_code == 3:
            rospy.loginfo('Llegue a la Meta...')
            self._goal_reached()
            self._publish_state()
        # Assert Error
        else:
            rospy.logerr('Algo raro paso. Estado Final:%s'%self._state_code)

def main():
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    rospy.init_node('robot_nav_go2point')
    robo_nav = RobotNav()
    robo_nav.execute()

if __name__ == '__main__':
    main()