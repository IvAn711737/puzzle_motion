#!/usr/bin/env python
# coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot

if __name__ == "__main__":
    rospy.init_node("rotate_angle_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
    ang_vel = rospy.get_param("~ang_vel", -1.0)
    # Угол должен быть больше нуля
    angle = rospy.get_param("~angle", 3.14)
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        # Взлетаем
        drone.takeoff(height=altitude)
        
        # Поворачиваемся на угол
        stop_cond = lambda a,l: a >= angle
        rospy.loginfo("Start rotation to angle: {0} with angular velocity: {1}".format(angle, ang_vel))
        drone.spin(ang_vel, altitude, stop_cond, on_finish=drone.land)
        
        # Ожидаем завершения поворота
        rate = rospy.Rate(5)
        while drone._spin_mode:
            rate.sleep()
        rospy.loginfo("Rotated")

    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()
    
