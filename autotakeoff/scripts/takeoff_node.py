#!/usr/bin/env python
# coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot

if __name__ == "__main__":
    rospy.init_node("takeoff_node", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.ArduPilot)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", True)
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        drone.takeoff(height=altitude)  
    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()
    
