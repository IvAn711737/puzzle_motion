#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from takeoff_common.utils import RosAuxTools
from takeoff_common.takeoffpy import MavController, AutoPilot




class CmdVelRemapper:
    """Класс, перенаправляющий сообщения из топика cmd_vel в топик управления дроном"""

    def load_params(self):
        """Загружает и устанавливает параметры"""

        self.ap_type = rospy.get_param("~ap_type", AutoPilot.ArduPilot)

        self.use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        # Взлет при получении сообщений в топик
        self.takeoff_on_msg = rospy.get_param("~takeoff_on_msg", False)
        #  Высота, на которой летает дрон в метрах
        self.altitude = rospy.get_param("~altitude", 1.5)
        # Время получения последнего сообщения в топик cmd_vel в секундах
        self.last_twist_time = None
        self.max_cmd_vel_timeout = 400

        # Ограничения по высоте
        self.min_altitude = 0.3
        self.max_altitude = 2.5
        # Замедляющий коэффициент скорости по оси z
        self.z_vel_coeff = 0.1


    def __init__(self):
        self.load_params()

        # Таймер для удержания высоты (нужен только для PX4)
        if self.ap_type == AutoPilot.PX4:
            self.alt_hold_timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self.drone = MavController.create_controller(
            self.ap_type, use_vision_odometry=self.use_vision_odometry
        )

        self.pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=4
        )
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_cb)

    def get_current_pos_target(self):
        """Создает точку дрона с текущим положением

        Returns:
            mavros_msgs.msg.PositionTarget: Сообщение точки для дрона
        """
        x_p = round(self.drone.local_position.pose.position.x, 1)
        y_p = round(self.drone.local_position.pose.position.y, 1)
        z_p = round(self.drone.local_position.pose.position.z, 1)
        z_p = self.altitude
        return RosAuxTools.pos2PosTarget(x_p, y_p, z_p)

    def twist_cb(self, msg):
        """Принимает сообщения из /cmd_vel и публикует их в /mavros/setpoint_raw/local

        Args:
            msg (geometry_msgs.Twist): сообщение из топика /cmd_vel
        """
        # Взлетаем, если дрон на земле
        if hasattr(self, "drone"):
            if self.drone.is_landed and self.takeoff_on_msg:
                self.drone.takeoff(height=self.altitude)
        # Запоминаем время получения последнего сообщения скорости
        self.last_twist_time = rospy.get_time()
        # Регулирование высоты полета
        self.altitude += msg.linear.z * self.z_vel_coeff
        self.altitude = np.clip(self.altitude, self.min_altitude, self.max_altitude)
        vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        # Создаем сообщение скорости дрону
        tgt = RosAuxTools.cmdVel2PosTarget(vel, self.altitude)
        # Если приходит нулевая скорость, то публикуем нулевую точку
        if vel == [0, 0, 0] and msg.linear.z == 0:
            tgt = self.get_current_pos_target()
        self.pub.publish(tgt)

    def timer_cb(self, event):
        """Callback таймера - удерживает высоту при отсутствии команд скорости

        Args:
            event (rospy.TimerEvent): информация о времени вызова таймера
        """
        if not self.last_twist_time is None:
            # Время, с момента получения последней команды взлета в миллисекундах
            diff = (event.current_real.to_sec() - self.last_twist_time) * 1e3
            # Если прошло больше 400 мс с момента получения последней команды взлета и дрон в воздухе
            if diff > self.max_cmd_vel_timeout and not self.drone.is_landed:
                tgt = self.get_current_pos_target()
                self.pub.publish(tgt)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_remapper")
    remapper = CmdVelRemapper()
    rospy.spin()
