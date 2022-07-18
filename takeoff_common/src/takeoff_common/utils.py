#!/usr/bin/env python
# coding=utf-8

import six
import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped, Pose, Vector3


class RosAuxTools:
    """Класс, содержащий вспомогательный методы для работы с ROS"""

    @staticmethod
    def extract_point(msg):
        msg_type = type(msg)
        if msg_type is Point:
            return msg
        elif msg_type is PointStamped:
            return msg.point
        elif msg_type is Pose:
            return msg.position
        elif msg_type is PoseStamped:
            return msg.pose.position
        elif msg_type is Vector3:
            return Point(x=msg.x, y=msg.y,z=msg.z)
        else:
            return None

    @staticmethod
    def point_to_stamped(point, header):
        return PointStamped(header = header, point = point)

    @staticmethod
    def position_pos_target(
        x, y, z, yaw, coordinate_frame=PositionTarget.FRAME_LOCAL_NED
    ):
        """Формирует сообщение точки для дрона в формате PositionTarget

        Args:
            x (float): x-координата
            y (float): y-координата
            z (float): z-координата
            yaw (float): угол ориентации вокруг оси z
            coordinate_frame (uint, optional): система координат дрона. Defaults to PositionTarget.FRAME_LOCAL_NED.

        Returns:
            mavros_msgs.msg.PositionTarget: сообщение точки для дрона
        """

        tgt = PositionTarget()

        # Точки и скорости от FastPlanner приходят в СК world
        tgt.coordinate_frame = coordinate_frame
        tgt.header.stamp = rospy.Time.now()

        tgt.type_mask = tgt.IGNORE_VX | tgt.IGNORE_VY | tgt.IGNORE_VZ
        tgt.type_mask |= tgt.IGNORE_AFX | tgt.IGNORE_AFY | tgt.IGNORE_AFZ
        # Для fast-planner
        tgt.type_mask |= tgt.IGNORE_YAW_RATE

        tgt.position.x = x
        tgt.position.y = y
        tgt.position.z = z
        tgt.yaw = yaw

        return tgt

    @staticmethod
    def velocity_pos_target(
        vx, vy, vz, ang_z, coordinate_frame=PositionTarget.FRAME_LOCAL_NED
    ):
        """Создает ROS-сообщение со скоростью для дрона

        Args:
            vx (float): линейная скорость по оси x
            vy (float): линейная скорость по оси y
            vz (float): линейная скорость по оси z
            ang_z (float): угловая скорость по оси z
            coordinate_frame (uint, optional): система координат дрона. Defaults to PositionTarget.FRAME_LOCAL_NED.

        Returns:
            mavros_msgs.msg.PositionTarget: сообщение скорости для дрона
        """
        tgt = PositionTarget()
        # Точки и скорости от FastPlanner приходят в СК world
        tgt.coordinate_frame = coordinate_frame
        tgt.header.stamp = rospy.Time.now()

        tgt.type_mask = tgt.IGNORE_PX | tgt.IGNORE_PY | tgt.IGNORE_PZ
        tgt.type_mask |= tgt.IGNORE_AFX | tgt.IGNORE_AFY | tgt.IGNORE_AFZ
        # Для fast-planner
        tgt.type_mask |= tgt.IGNORE_YAW

        tgt.velocity.x = vx
        tgt.velocity.y = vy
        tgt.velocity.z = vz
        tgt.yaw_rate = ang_z

        return tgt

    @staticmethod
    def rotation_pos_target(
        altitude, ang_z, coordinate_frame=PositionTarget.FRAME_LOCAL_NED
    ):
        """Создает ROS-сообщение вращения для дрона

        Args:
            altitude (float): высота в метрах
            ang_z (float): угловая скорость в рад/с
            coordinate_frame (uint, optional): система координат дрона. Defaults to PositionTarget.FRAME_LOCAL_NED.

        Returns:
            mavros_msgs.msg.PositionTarget: ROS-сообщение вращения для дрона
        """
        tgt = PositionTarget()
        # Точки и скорости от FastPlanner приходят в СК world
        tgt.coordinate_frame = coordinate_frame
        tgt.header.stamp = rospy.Time.now()

        tgt.type_mask = tgt.IGNORE_VX | tgt.IGNORE_VY | tgt.IGNORE_VZ
        tgt.type_mask |= tgt.IGNORE_AFX | tgt.IGNORE_AFY | tgt.IGNORE_AFZ
        # Для fast-planner
        tgt.type_mask |= tgt.IGNORE_YAW
        # ЗАдаем высоту и угловую скорость
        tgt.position.z = altitude
        tgt.yaw_rate = ang_z
        return tgt

    @staticmethod
    def pose_stamped(x, y, z, yaw=None, frame_id="world"):
        """Создает ROS-сообщение с положением и ориентацией дрона

        Args:
            x (float): x-координата
            y (float): y-координата
            z (float): z-координата
            yaw (float, optional): ориентация вокруг оси z. Defaults to None.
            frame_id (str, optional): система координат. Defaults to "world".

        Returns:
            geometry_msgs.msg.PoseStamped: ROS-сообщение с позицией
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        if yaw is None:
            pose_msg.pose.orientation.w = 1
        else:
            q = Algebra.euler2quat(0, 0, yaw)
            pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose_msg

    # TODO: изменить верблюжий регистр на змеиный :)
    # DEPRECATED
    @staticmethod
    def cmdVel2PosTarget(vel, altitude):
        """Создает сообщение скорости для дрона из сообщения скорости для мобильного неголономного робота

        Args:
            vel (tuple): линейная скорость дрона по осям x,y и угловая вокруг z: [lin_x, lin_y, ang_z]
            altitude (float): высота, на которой летает дрон в метрах

        Returns:
            mavros_msgs.msg.PositionTarget: Сообщение скорости для дрона
        """
        tgt = PositionTarget()

        tgt.type_mask = (
            tgt.IGNORE_PX
            | tgt.IGNORE_PY
            | tgt.IGNORE_AFX
            | tgt.IGNORE_AFY
            | tgt.IGNORE_AFZ
            | tgt.IGNORE_YAW
        )

        tgt.velocity.x = vel[0]
        tgt.velocity.y = vel[1]
        tgt.yaw_rate = vel[2]

        tgt.position.z = altitude

        tgt.coordinate_frame = tgt.FRAME_BODY_NED
        tgt.header.stamp = rospy.Time.now()

        return tgt

    # DEPRECATED
    @staticmethod
    def pos2PosTarget(x, y, altitude):
        """Создает точку для дрона из заданных координат

        Args:
            x (float): x-координата
            y (float): y-координата
            altitude (float): высота

        Returns:
            mavros_msgs.msg.PositionTarget: Сообщение точки для дрона
        """
        tgt = PositionTarget()
        tgt.type_mask = (
            tgt.IGNORE_VX
            | tgt.IGNORE_VY
            | tgt.IGNORE_VZ
            | tgt.IGNORE_AFX
            | tgt.IGNORE_AFY
            | tgt.IGNORE_AFZ
            | tgt.IGNORE_YAW
            | tgt.IGNORE_YAW_RATE
        )
        tgt.position.x = x
        tgt.position.y = y
        tgt.position.z = altitude

        tgt.coordinate_frame = tgt.FRAME_LOCAL_NED
        tgt.header.stamp = rospy.Time.now()

        return tgt

    @staticmethod
    def safe_decorator(func):
        """Декоратор для безопасного вызова функций

        Args:
            func (Callable): функция
        """
        def wrapper(*args, **kwargs):
            try:
                func(*args, **kwargs)
            except Exception as e:
                rospy.logerr(e)
        return wrapper

    @staticmethod
    def safe_subscriber(topic_name, data_class, callback, verbose=True):
        """Создает подписчика на топик только после получения сообщения в этот топик

        Args:
            topic_name (str): имя топика
            data_class ([type]): тип сообщений топика
            callback (Callable): функция-обработчик сообщений в топик
            verbose (bool, optional): вывод информации о процессе подключения в консоль. Defaults to True.

        Returns:
            Subscriber: объект подписчика на топик
        """
        if verbose: rospy.loginfo("Wait for topic {0}".format(topic_name))
        rospy.wait_for_message(topic_name, data_class)
        sub = rospy.Subscriber(topic_name, data_class, callback)
        if verbose: rospy.loginfo("OK")
        return sub

    @staticmethod
    def safe_srv_proxy(srv_name, data_class, timeout=10, verbose=True):
        """Создает клиент только "живого" сервиса 

        Args:
            srv_name (str): имя сервиса
            data_class ([type]): тип сообщений сервиса
            timeout (int, optional): максимальное время ожидания подключения сервиса. Defaults to 10.
            verbose (bool, optional): вывод информации о процессе подключения в консоль. Defaults to True.

        Returns:
            [type]: [description]
        """
        try:
            if verbose: rospy.loginfo("Wait for service {0}".format(srv_name))
            rospy.wait_for_service(srv_name, timeout)
            srv_proxy = rospy.ServiceProxy(srv_name, data_class)
            if verbose: rospy.loginfo("OK")
            return srv_proxy
        except rospy.ROSException:
            rospy.logerr("Failed connect to service {0}".format(srv_name))


class Algebra:

    @staticmethod
    def angle2positive(angle):
        """Переводит угол из области определения [-3.14; 3.14] в [0; 6.28]

        Args:
            angle (float): угол из области определения [-3.14; 3.14]

        Returns:
            float: угол из области определения [0; 6.28]
        """

        return angle if angle >= 0 else 2 * np.pi + angle

    @staticmethod
    def length(v):
        return np.linalg.norm(v)

    @staticmethod
    def angle_between_vectors(v1, v2):
        """Вычисляет угол между векторами в пространстве

        Args:
            v1 (ndarray): вектор 1
            v2 (ndarray): вектор 2

        Returns:
            float: Угол между векторами в диапазоне [-pi; pi]
        """
        cos = np.dot(v1, v2) / (Algebra.length(v1) * Algebra.length(v2))
        angle = np.arccos(cos)
        return angle

    @staticmethod
    def signed_angle(v1, v2):
        return np.sign(v2[1]) * Algebra.angle_between_vectors(v1, v2)

    @staticmethod
    def unit_vector(v):
        return v / Algebra.length(v)

    @staticmethod
    def euler2quat(r,p,y):
        """Преобразует углы Эйлера в кватернион. Совместима с python2 и python3.

        Args:
            r (float): roll
            p (float): pitch
            y (float): yaw

        Returns:
            ndarray: [x,y,z,w]
        """
        if six.PY2:
            import tf
            return tf.transformations.quaternion_from_euler(r,p,y)
        else:
            import transforms3d
            q = transforms3d.euler.euler2quat(r,p,y)
            return np.roll(q, -1)

    @staticmethod
    def quat2euler(q):
        """Преобразует кватернион в углы Эйлера. Совместима с python2 и python3.

        Args:
            q (geometry_msgs.msg.Quaternion): кватернион

        Returns:
            list: [roll, pitch, yaw]
        """
        if six.PY2:
            import tf
            quat = [q.x, q.y, q.z, q.w]
            return tf.transformations.euler_from_quaternion(quat)
        else:
            import transforms3d
            quat = [q.w, q.x, q.y, q.z]
            return transforms3d.euler.quat2euler(quat)

    @staticmethod
    def get_orientation(x_direction):
        v1 = np.array([1, 0, 0])
        # Находим угол между векторами
        angle = Algebra.signed_angle(v1, x_direction)
        # Создаем поворот, применив который ось x системы map будет направлена по пути
        q = Algebra.euler2quat(0, 0, angle)
        return q

