#! /usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class RobotDistance():

    def __init__(self):
        # Inicializamos los valores de la clase
        # Declaramos la variable para almacenar y publicar la distancia recorrida
        self._moved_distance = Float64()
        self._moved_distance.data = 0.0
        # Variable almacena la posicion actual del robot
        self._current_position = Point()
        # declaramos el publicador de la distancia calculada
        self.robot_distance_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)
        # llamada a la funcion para inicializar la posicion actual
        # y garantizar que el topico /odom exista
        self._init()
        # declaramos el subscriptor al topico de odometria
        self.robot_distance_sub = rospy.Subscriber('/odom', Odometry, self.odom_clbck)


    def _init(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message('/odom', Odometry, timeout=1)
            except Exception as e:
                rospy.logerr(str(e))
        # Asignamos los valores de la posicion de la primera lectura a
        # la variable de clase que almacena la posicion actual
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_clbck(self, msg):
        # Obtiene la posicion del mensaje de odometria
        new_position = msg.pose.pose.position
        #calculamos la distancia recorrida de la nueva posicion a la posicion actual
        self._moved_distance.data += self.calculate_distance(new_position, self._current_position)
        #actualizamos la posicion actual
        self._current_position = new_position

        # Publicamos la distancia recorrida
        if self._moved_distance.data < 0.00001: # 0.00001 de metro
            aux =Float64()
            aux.data = 0.0
            self.robot_distance_pub.publish(aux)
        else:
            self.robot_distance_pub.publish(self._moved_distance)


    def calculate_distance(self, new_position, old_position):
        x1 = new_position.x
        x0 = old_position.x
        y1 = new_position.y
        y0 = old_position.y
        # Calculamos la distancia con los valores de las posiciones
        dist = math.hypot(x1-x0, y1-y0)
        #distancia = math.sqrt(math.pow((y1-y0), 2) +  math.pow((x1-x0), 2))  
        return dist

    def keep_alive(self):
        # spin() simplemente evita que Python salga (o termine) hasta que el nodo se detenga
        rospy.spin()


def main():
    rospy.init_node('robot_control')
    robot_dist = RobotDistance()
    robot_dist.keep_alive()

if __name__ == '__main__':
    main()