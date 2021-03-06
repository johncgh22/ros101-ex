#! /usr/bin/env python

import rospy
import actionlib
import simple_action_msg.msg

def fibonacci_client():
    # Declaramos el cliente para conectarnos al servidor
    client = actionlib.SimpleActionClient('fibonacci', simple_action_msg.msg.FibonacciAction)
    # Esperamos por la respuesta del conexion del servidor
    client.wait_for_server()
    # Declaramos una meta
    goal = simple_action_msg.msg.FibonacciGoal(order = 10)
    # Enviamos la meta al servidor
    client.send_goal(goal)
    # Esperamos el resultado del servidor
    client.wait_for_result()
    # Devolvemos el resultado
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('fibonacci_client')
    result = fibonacci_client()
    print result