#!/usr/bin/env python
#coding=utf-8

import fysom
import rospy
from std_msgs.msg import Empty, UInt32
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from takeoff_common.takeoffpy import MavController, AutoPilot

class PosCmdRemapper:
    """Класс, перенаправляющий сообщения из топика cmd_vel в топик управления дроном"""

    def load_params(self):
        """Загружает и устанавливает параметры"""
        # Тип автопилота
        self.ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        # Использовать визуальную одометрию или нет
        self.use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        # Взлет при получении сообщений в топик
        self.takeoff_on_msg = rospy.get_param("~takeoff_on_msg", True)
        # Удерживать высоту при получении нулевой скорости
        self.enable_althold = rospy.get_param("~enable_althold", False)
        #  Высота, на которой летает дрон в метрах
        self.altitude = rospy.get_param("~altitude", 1.5)

        # TODO: сделать параметры ros-параметрами
        # TODO: придумать понятные названия параметрам

        # Предыдущее расстояние до цели
        self.old_goal_dist = float("inf")
        # Считает, как долго дрон не трогается с места
        self.paralysis_counter = 0
        # Максимальное количество итераций простоя для дрона
        self.paralysis_threshold = 400
        # Минимальное расстояние до цели, чтобы считать ее достигнутой
        self.goal_min_dist = 0.2
        # Минимальный прирост расстояния до цели, чтобы не увеличивать счетчик простоя
        self.error_dist_threshold = 0.01
    
    ###### Управление дроном ######

    def send_traj_point(self):
        """Отправляет точку траектории дрону"""
        self.drone.send_pos(self.traj_msg.position.x, self.traj_msg.position.y, self.traj_msg.position.z, self.traj_msg.yaw)

    ###############################
    
    ####### FSM ###################

    def trigger_if_can(self, event):
        """Запускает переход конечного автомата,
        если это возможно в текущем состоянии

        Args:
            event (string): имя перехода/события
        """
        if self.fsm.can(event):
            self.fsm.trigger(event)

    def on_takeoff(self, e):
        # Взлетаем, если дрон на земле
        if hasattr(self, "drone"):
            if self.drone.is_landed:
                self.drone.takeoff(height=self.altitude)

        # Предыдущее положение дрона
        self.old_pos = self.drone.local_position.pose.position    
        # Переходим в start
        self.trigger_if_can("start")
    
    def on_send_goal(self, e):
        # Запоминаем расстояние до цели
        self.goal_distance = self.drone.distance_2d(self.goal.pose.position)
        # Отправляем цель в fast-planner
        self.goal_pub.publish(self.goal)

    def on_traj_point(self, e):
        # Вычисляем на сколько процентов достигнута цель
        percentage = 0
        if self.old_goal_dist != float("inf"):
            percentage = round(self.old_goal_dist * 100 / self.goal_distance)

            percentage = 100 - percentage if percentage < 100 else 0
        # Публикуем процент достижения цели
        self.feedback_pub.publish(UInt32(percentage))
        # Отправляем последнюю полученную точку траектории
        self.send_traj_point()

    def on_stop(self, e):
        # Отправляем текущее положение дрона
        self.drone.send_altitude(self.altitude)
        # Инициируем событие остановлен
        self.trigger_if_can("stopped")
        # "Обнуляем" предыдущее расстояние до цели
        self.old_goal_dist = float("inf")

    def on_success(self, e):
        self.success_pub.publish(Empty())

    def on_error(self, e):
        self.error_pub.publish(Empty())


    ###############################


    ###### Callbacks ##############

    def cancel_cb(self, cancel_msg):
        """Callback-метод топика отмены цели

        Args:
            cancel_msg (std_msgs.msg.Empty): пустое сообщение
        """
        # Инициируем отмену движения к цели
        self.trigger_if_can("cancel")

    def goal_cb(self, goal_msg):
        """Callback-метод топика цели

        Args:
            goal_msg (geometry_msgs.msg.PoseStamped): цель - точка, в кот. должен прилететь дрон
        """
        # Запоминаем цель
        self.goal = goal_msg
        # Инициируем событие goal
        self.trigger_if_can("goal")

    def trajectory_cb(self, traj_msg):
        # TODO: Если в callback не приходят сообщения от fast-planner'а
        # то дрон не будет держать высоту
        """Callback-метод топика с точками траектории дрона

        Args:
            traj_msg (quadrotor_msgs.msg.PositionCommand): точка траектории и скорость в ней
        """
        # Сохраняем полученное сообщение
        self.traj_msg = traj_msg

        # Достиг ли дрон цели
        goal_dist = self.drone.distance_2d(self.goal.pose.position)
        reach_goal = goal_dist <= self.goal_min_dist

        # Произошла ли ошибка
        if abs(self.old_goal_dist - goal_dist) < self.error_dist_threshold:
            self.paralysis_counter += 1
        else:
            self.paralysis_counter = 0

        self.old_goal_dist = goal_dist
        is_error = self.paralysis_counter > self.paralysis_threshold

        if reach_goal:
            self.trigger_if_can("success")
        elif is_error:
            self.trigger_if_can("error")
        else:
            self.trigger_if_can("traj_point")

        # Обновляем значение предыдущего положения
        self.old_pos = traj_msg.position
    
    ###############################

    def __init__(self):
        # Загрузка параметров
        self.load_params()

        # Создаем дрона
        self.drone = MavController.create_controller(self.ap_type, use_vision_odometry=self.use_vision_odometry)
        # Конечный автомат управления движением дрона к цели
        self.fsm = fysom.Fysom(
            {
                "initial": "wait",
                "events": [
                    {"name": "goal", "src": "wait", "dst": "takeoff"},
                    {"name": "start", "src": "takeoff", "dst": "send_goal"},
                    {"name": "goal", "src": ["takeoff", "send_goal", "exec_traj"], "dst": "send_goal"},
                    {"name": "traj_point", "src": ["send_goal", "exec_traj"], "dst": "exec_traj"},
                    {"name": "cancel", "src": "exec_traj", "dst": "stop"},
                    {"name": "cancel", "src": ["takeoff", "send_goal"], "dst": "wait"},
                    {"name": "stopped", "src": "stop", "dst": "wait"},

                    # Доп переходы
                    {"name": "success", "src": "exec_traj", "dst": "stop"},
                    {"name": "error", "src": "exec_traj", "dst": "stop"},

                ],
                "callbacks":{
                    "ontakeoff": self.on_takeoff,
                    "onsend_goal": self.on_send_goal,
                    "ontraj_point": self.on_traj_point,
                    "onstop": self.on_stop,
                    "onsuccess": self.on_success,
                    "onerror": self.on_error
                }
            }
        )

        # Выводим в консоль изменения состояния КА
        # self.fsm.onchangestate = lambda e: rospy.logwarn("[REPUB FSM] {0}: {1} -> {2}".format(e.event, e.src, e.dst))

        # Публикует точку для дрона
        self.point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=4)

        # Публикует цель для fast-planner
        self.goal_pub = rospy.Publisher("/fast_planner/goal", PoseStamped, queue_size=4)

        # Публикует сообщение, когда цель достигнута
        self.success_pub = rospy.Publisher("/fast_planner/success", Empty, queue_size=4)
        
        # Публикует сообщение о степеи достижения цели
        self.feedback_pub = rospy.Publisher("/fast_planner/feedback", UInt32, queue_size=4)

        # Публикует сообщение, когда цель недостижима
        self.error_pub = rospy.Publisher("/fast_planner/error", Empty, queue_size=4)

        # Точка траектории от fast-planner
        self.pos_cmd_sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.trajectory_cb)
        
        # TODO: добавить префикс fast_planner
        # Сигнал отмены цели
        self.cancel_sub = rospy.Subscriber("/cancel_goal", Empty, self.cancel_cb)
        # Цель
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)


if __name__ == "__main__":
    rospy.init_node("pos_cmd_remapper")
    r = PosCmdRemapper()
    rospy.spin()
