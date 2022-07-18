#!/usr/bin/env python
# coding=utf-8

import rospy
from autotakeoff.srv import Takeoff, TakeoffResponse
from takeoff_common.takeoffpy import MavController, AutoPilot

class TakeoffHandler:
    """Класс для обработки запросов к сервису takeoff"""

    def __init__(self):
        ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)

    def handle_takeoff(self, req):
        """Обрабатывает запрос к сервису takeoff_landing

        Args:
            req (Takeoff): запрос к сервису, содержащий высоту и требование посадки

        Returns:
            TakeoffResponse: True - удачно взлет/приземлился, False - нет
        """
        try:
            if req.land:
                self.drone.land()
            else:
                self.drone.takeoff(height=req.height)
            return TakeoffResponse(True)
        except:
            return TakeoffResponse(False)

def takeoff_landing_server():
    """Запускает сервер сервиса takeoff_landing """
    rospy.init_node('takeoff_server')
    th = TakeoffHandler()
    takeoff_srv = rospy.Service('takeoff_landing', Takeoff, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()


if __name__ == "__main__":
    takeoff_landing_server()
