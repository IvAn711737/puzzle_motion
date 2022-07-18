#!/usr/bin/env python
#coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot

if __name__ == "__main__":
    rospy.init_node("square_flight_node", anonymous=True)
   
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    ap_type = rospy.get_param("~ap_type", AutoPilot.ArduPilot)
    # Размер стороны квадрата в метрах
    square_side = rospy.get_param("~square_side", 0.25)
    # Точность прилета в точку
    tolerance = rospy.get_param("~tolerance", 0.1)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", True)

    # Создаем контроллер дрона и летим по квадратной траектории
    drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)
    try:
        drone.square_flight(altitude, size=square_side, tolerance=tolerance)
    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()

    
