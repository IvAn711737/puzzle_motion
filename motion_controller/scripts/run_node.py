#!/usr/bin/env python3
# coding=utf-8

import rospy
from motion_controller.srv import Spin, SpinResponse, SpinRequest
from takeoff_common.takeoffpy import MavController, AutoPilot
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest


class MotionController:
    def __init__(self):
        rospy.init_node("motion_controller")
        self._ap_type = rospy.get_param("~ap_type", AutoPilot.ArduPilot)
        self._use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.is_stopped = False
        self.spin_service = rospy.Service(
            "motion_controller/spin", Spin, self.on_spin
        )
        self.stop_service = rospy.Service(
            "motion_controller/stop", Empty, self.on_stop
        )
        self.drone = MavController.create_controller(
            self._ap_type, use_vision_odometry=self._use_vision_odometry
        )
        rospy.spin()


    def on_spin(self, req: SpinRequest) -> SpinResponse:
        self.is_stopped = False
        self.is_finished = False
        def on_finished(self):
            self.is_finished = True
        self.drone.spin(ang_vel=req.angular_velocity, altitude=req.altitude, 
            stop_cond=lambda angle, laps: laps >= req.laps_count or self.is_stopped, 
            on_finish=lambda: on_finished(self))
        rate = rospy.Rate(10)
        while not self.is_finished:
            rate.sleep()
        return SpinResponse()
    
    def on_stop(self, _: EmptyRequest) -> EmptyResponse:
        self.is_stopped = True
        return EmptyResponse()


if __name__ == "__main__":
    MotionController()
