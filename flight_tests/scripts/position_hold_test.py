#!/usr/bin/env python3
# coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot

if __name__ == "__main__":
    rospy.init_node("position_hold_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
    hover_time = int(rospy.get_param("~hover_time", 5))
    if hover_time <= 0: hover_time = 5
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        drone.takeoff(height=altitude)
        rate = rospy.Rate(1)
        i = 0
        for i in range(hover_time):
            rate.sleep()
            rospy.loginfo("Hold position for {0} sec".format(i + 1))
        drone.land()
    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()
    
