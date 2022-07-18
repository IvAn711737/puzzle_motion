#!/usr/bin/env python
# coding=utf-8

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from fast_planner_server.msg import FastPlannerAction


class FastPlannerActionCient:
    """Класс action-клиента для fast-planner'а. Нужен для подачи 2D Nav Goal в action-сервер"""

    # Имя action-клиента
    _client_name = "fast_planner_client"

    def __init__(self):
        # Высота точки цели
        self.altitude = rospy.get_param("~altitude", 1.0)
        # Ожидать ли достижения цели
        self.wait_result = rospy.get_param("~wait_result", False)
        # Создаем action-клиента
        self.client = SimpleActionClient("fast_planner_server", FastPlannerAction)
        # Ожидаем сервер
        self.client.wait_for_server()
        # Подписываемся на топик цели от move_base
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_cb
        )

    def goal_cb(self, msg):
        """Callback цели

        Args:
            msg (geometry_msgs.msg.PoseStamped): цель
        """
        # Задаем высоту всем целям в 1.5 метра
        msg.pose.position.z = self.altitude
        rospy.loginfo("[{0}] Resend goal: {1}".format(self._client_name, msg.pose))
        # Отправляем цель в action-сервер
        self.client.send_goal(msg)
        if self.wait_result:
            # Ждем результата
            rospy.loginfo("[{0}] Move to goal".format(self._client_name))
            self.client.wait_for_result()
            # Запрашиваем результат
            res = self.client.get_result()
            rospy.loginfo("[{0}] Success: {1}; Error {2}".format(self._client_name, res.success, res.error))




if __name__ == "__main__":
    node_name = "fast_planner_client_node"
    rospy.init_node(node_name)
    rospy.loginfo("{0} was launched!".format(node_name))
    fp_client = FastPlannerActionCient()
    rospy.spin()
