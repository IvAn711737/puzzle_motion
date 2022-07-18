#!/usr/bin/env python
# coding=utf-8

from collections import namedtuple

from numpy.lib.npyio import load
import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot


def load_points(num_points):
    """Загружаем точки траектории. Если их нет, то генерируем
    квадратную траекторию по умолчанию

    Args:
        num_points (uint): количество точек траектории

    Returns:
        [ndarray]: массив 2d-точек траектории: [[x0,y0], [x1,y1], ..., [xN, yN]]
    """
    points = []
    if num_points > 0:
        for i in range(num_points):
            # Считываем точку
            x = int(rospy.get_param("~point{0}_x".format(i), 0))
            y = int(rospy.get_param("~point{0}_y".format(i), 0))
            points.append(Point2d(x=x, y=y))
    else:
        # По умолчанию задается траектория полета по квадрату
        points = [Point2d(0, 1), Point2d(-1, 1), Point2d(-1, 0), Point2d(0, 0)]
    return points


if __name__ == "__main__":
    Point2d = namedtuple("Point2d", ["x", "y"])

    rospy.init_node("flight_through_points_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
    tolerance = rospy.get_param("~tolerance", 0.1)
    num_points = int(rospy.get_param("~num_points", 0))
    rospy.logwarn("Num points: {0}".format(num_points))
    # Считываем точки траектории
    points = load_points(num_points)
    
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        # Взлетаем
        drone.takeoff(height=altitude)
        # Летим по траектории
        for p in points:
            rospy.loginfo("Drone goes to point: ({0}; {1}; {2})".format(p.x, p.y, altitude))
            drone.send_pos(p.x, p.y, altitude)
            drone.wait_position(p.x, p.y, altitude, tolerance)
            rospy.loginfo("Reached")
            rospy.sleep(1.0)
        # Приземляемся
        drone.land()
    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()
    
