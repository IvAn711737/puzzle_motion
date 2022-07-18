#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
from enum import IntEnum
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from takeoff_common.utils import Algebra, RosAuxTools
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL



class AutoPilot(IntEnum):
    """Перечисление поддерживаемых типов автопилотов"""

    PX4 = 0
    ArduPilot = 1

class MavController:
    """Класс для управления дроном через mavros"""

    # Делаем класс MavController абстрактным
    __metaclass__ = ABCMeta

    @staticmethod
    def create_controller(ap_type, use_vision_odometry=False, vicinity=0.1):
        """Создаем класс контроллера для заданного автопилота

        Args:
            ap_type (AutoPilot): тип автопилота
            vicinity (float, optional): точность прилета в точку. Defaults to 0.1.

        Raises:
            ValueError: исключение, при некорректных значениях типа автопилота

        Returns:
            MavController: объект автопилота, наследующий от MavController
        """
        if ap_type == AutoPilot.PX4:
            return PX4Controller(vicinity, use_vision_odometry=False)
        elif ap_type == AutoPilot.ArduPilot:
            return ArduController(vicinity, use_vision_odometry)
        else:
            raise ValueError("Inappropriate value of ap_type!")

    def __init__(self, use_vision_odometry=False):
        self.rate = rospy.Rate(5)
        # Режим вращения выключен
        self._spin_mode = False
        # Флаг нахождения на земле
        self.is_landed = True
        self._on_finish = None

        # Создаем publisher'а точки, в которую будет лететь дрон
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        # Publisher точки взлета дрона для PX4
        self.pos_target_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=4
        )

        # Создаем  service callers для разблокировки дрона и выставления его в нужный режим
        self.arm = RosAuxTools.safe_srv_proxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_srv = RosAuxTools.safe_srv_proxy("/mavros/set_mode", SetMode)

        # Сервисы взлета и приземления
        self.takeoff_srv = RosAuxTools.safe_srv_proxy("/mavros/cmd/takeoff", CommandTOL)
        self.land_srv = RosAuxTools.safe_srv_proxy("/mavros/cmd/land", CommandTOL)

        rospy.loginfo("ROS services are available")

        # Используем один из двух топиков для определения положения дрона
        self.pose_topic_gazebo = "/mavros/local_position/pose"
        self.pose_topic_vision = "/mavros/vision_pose/pose"
        pose_topic = (
            self.pose_topic_vision if use_vision_odometry else self.pose_topic_gazebo
        )
            
        # "Безопасно" подписываемся на топики
        self.local_pos_sub = RosAuxTools.safe_subscriber(
            pose_topic, PoseStamped, self.local_pose_cb
        )
        self.state_sub = RosAuxTools.safe_subscriber(
            "mavros/state", State, self.state_cb
        )
        rospy.loginfo("ROS topics are available")

        self.setpoint_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, self.setpoint_cb)
        self.pos_target_sub = rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self.pos_target_cb)

        # Таймер вращения
        self.spin_timer = rospy.Timer(rospy.Duration(0.1), self.spin_timer_cb)

        # Удержание высоты при отсутствии команд
        self._last_pos = None
        self._last_pos_time = None
        self._hover_flag = False
        self.hover_timer = rospy.Timer(rospy.Duration(0.1), self.hover_timer_cb)

    def setpoint_cb(self, msg):
        """Запоминает последнюю отправленную точку

        Args:
            msg (geometry_msgs.msg.PoseStamped): сообщение точки дрону
        """
        point = msg.pose.position
        self._last_pos = [point.x,point.y,point.z]
        self._last_pos_time = rospy.get_time()

    def pos_target_cb(self, msg):
        """Запоминает последнюю отправленную точку

        Args:
            msg (masvros_msgs.msg.PositionTarget): сообщение точки дрону
        """
        pos_mask = msg.IGNORE_PX | msg.IGNORE_PY | msg.IGNORE_PZ
        is_ignored = msg.type_mask & pos_mask
        point = self.local_position.pose.position if is_ignored else msg.position
        self._last_pos = [point.x, point.y, point.z]
        self._last_pos = np.round(self._last_pos, 1)
        self._last_pos_time = rospy.get_time()

    def hover_timer_cb(self, e):
        """Callback-функция таймера, используемая для стабилизации положения дрона в воздухе

        Args:
            e ([type]): [description]
        """
        # Если удержание включено (включается при takeoff, выключается при land), дрон в воздухе и не вращается
        if self._hover_flag and not self.is_landed and not self._spin_mode:
            # Если с момента получения последней точки прошло более 0,4 секунд
            if rospy.get_time() - self._last_pos_time > 0.4:
                # Удерживаем дрона в последней точке
                self.send_pos(*self._last_pos)

    # TODO: Перенести в Algebra
    def positive_yaw_from_start(self, start_yaw, direction):
        """Положительный угол дрона относительно стартового угла

        Args:
            start_yaw (float): стартовый угол вращения
            direction (float): направление вращения: +1 против часовой, -1 по часовой

        Returns:
            float: положительный угол дрона относительно стартового угла
        """
        yaw = Algebra.quat2euler(self.local_position.pose.orientation)[2]
        # Переводим угол из области определения [-3.14; 3.14] в [0; 6.28]
        yaw = round(Algebra.angle2positive(yaw), 3)
        # Вычисляем угловое положение дрона относительно угла старта
        angle = yaw - start_yaw
        # Переводим угол в положительную область определения
        angle = Algebra.angle2positive(direction * angle)
        # Округляем угол
        angle = round(angle, 3)
        return angle

    def spin_timer_cb(self, e):
        if self._spin_mode:
            # Вычисляем угол дрона относительно угла начала вращения
            self._angle = self.positive_yaw_from_start(
                self._start_yaw, np.sign(self._spin_vel)
            )
            # Костыльная эвристика: в начале вращения угол не может быть близок к 2*pi
            # Считаем, что ему нужно минимум 5 итераций, чтобы достичь такого значения
            if self._spin_iterations < 5 and self._angle > 6: self._angle = 0

            # Вычисляем количество сделанных кругов
            # Предыдущий угол больше текущего и целая часть предыдущего угла равна 6
            if self._angle < self._spin_last_angle and round(self._spin_last_angle) == 6:
                self._spinned_laps += 1

            # Запоминаем предыдущий угол
            self._spin_last_angle = self._angle

            # Если сработал критерий останова, то перестаем вращаться
            if self._stop_cond(self._angle, self._spinned_laps):
                self._spin_mode = False
                # Вызываем функцию после остановки вращения
                if not self._on_finish is None: self._on_finish()
            else:
                # Отправляем угловую скорость дрону
                tgt = RosAuxTools.rotation_pos_target(self._spin_point.z, self._spin_vel)
                tgt.position = self._spin_point
                self.pos_target_pub.publish(tgt)

    def local_pose_cb(self, data):
        """Сохраняет в поле класса информацию о положении дрона

        Args:
            data (geometry_msgs/PoseStamped): положение (x,y,z, q1, q2, q3, q4) дрона в пространстве
        """
        self.local_position = data

    def state_cb(self, data):
        """Сохраняет в поле класса информацию о состоянии дрона

        Args:
            data (mavros_msgs/State): Состояние дрона
        """
        self.state = data
        self.is_landed = True if self.state.system_status == 3 else False

    def is_at_position(self, x, y, z, offset):
        """Определяет, находится ли дрон в указанной точке (x,y,z)

        Args:
            x (float): x-координата точки
            y (float): y-координата точки
            z (float): z-координата точки
            offset (float): точность попадания в точку в метрах (чем меньше, тем точнее дрон должен быть в точке)

        Returns:
            [Bool]: True - находится в точке,  False - нет
        """

        # Точка, в которой должен находиться дрон
        desired = np.array((x, y, z))
        # Текущее положения дрона
        pos = np.array(
            (
                self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z,
            )
        )
        return np.linalg.norm(desired - pos) < offset

    def is_at_height(self, height, offset):
        """Определяет, достиг ли дрон заданной высоты

        Args:
            height (float): высота в метрах, на которой должен находиться дрон
            offset (float): допуск на достижение высоты в метрах

        Returns:
            [bool]: True - достигнута высота, False - нет
        """
        return abs(height - self.local_position.pose.position.z) < offset

    def init_autopilot(self, x=0, y=0, z=0.5):
        """Инициализирует автопилот отправкой точки на взлет

        Args:
            x (int, optional): x-координата отправляемой точки. Defaults to 0.
            y (int, optional): y-координата отправляемой точки. Defaults to 0.
            z (float, optional): z-координата отправляемой точки. Defaults to 0.5.
        """
        for i in range(10):
            self.send_pos(x,y,z, yaw=0)
            self.rate.sleep()

    def wait_conection(self, verbose=True):
        """Ожидает подключения к автопилоту

        Args:
            verbose (bool, optional): вывод информации о процессе подключения в консоль. Defaults to True.
        """
        if verbose:
            rospy.loginfo("Wait for connect to drone...")
        while not self.state.connected:
            self.rate.sleep()
        if verbose:
            rospy.loginfo("Connected!")

    def wait_position(self, x, y, z, tolerance, func=None):
        """Ожидает достижения точки дроном

        Args:
            x (float): -координата точки
            y (float): -координата точки
            z (float): -координата точки
            tolerance (float): точность достижения точки
            func (Callable, optional): функция, которая вызывается каждый цикл ожидания: func(x,y,z). Defaults to None.
        """
        while not rospy.is_shutdown():
            if self.is_at_position(x, y, z, tolerance):
                break
            if func: func(x,y,z)
            self.rate.sleep()

    def wait_height(self, height):
        """Ожидает достижения дроном заданной высоты

        Args:
            height (float): высота в метрах

        Raises:
            rospy.ServiceException: исключение при блокировке дрона
        """
        # Изначально дрон заблокирован
        last_armed_state = False
        while rospy.is_shutdown() == False:
            if self.is_at_height(height, self.vicinity):
                break
            # Считываем текущее состояние
            cur_armed_state = self.state.armed
            # Если diff = 1, то дрон разблокировался, если diff = -1, то дрон заблокировался
            diff = int(cur_armed_state) - int(last_armed_state)
            # Если дрон заблокировался, то выбрасываем исключение
            if diff == -1:
                raise rospy.ServiceException(
                    "Drone disarmed by unknown reason. Check correctness of set mode"
                )
            self.rate.sleep()
            # Запоминаем предыдущее состояние
            last_armed_state = cur_armed_state

    def set_mode(self, mode, timeout=5):
        """Устанавливает режим автопилота

        Args:
            mode (str): название режима
            timeout (int, optional): время, в течение которого скрипт пытается выставить режим. Defaults to 5.
        """
        loop_freq = 4
        rate = rospy.Rate(loop_freq)
        rospy.loginfo("Setting FCU mode: {0}".format(mode.upper()))
        for i in range(timeout * loop_freq):
            if self.state.mode == mode:
                rospy.loginfo("Mode {0} is set in {1} secs".format(mode.upper(), float(i) / loop_freq))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)
                    if not res.mode_sent:
                        rospy.logerr("Failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()

    def arm_drone(self):
        """Разблокирует дрона (активирует двигатели)
        """
        if not self.state.armed:
            if self.arm(True):
                rospy.loginfo("Successfully armed drone")
            else:
                rospy.logerr("Error arming drone")
        else:
            rospy.loginfo("Drone is already armed")

    def distance(self, pt):
        """Расстояние до точки в пространстве

        Args:
            pt (geometry_msgs.msg.Point): точка в пространстве

        Returns:
            float: расстояние от дрона до точки в метрах
        """
        pt1 = self.local_position.pose.position
        d = (pt1.x - pt.x)**2
        d += (pt1.y - pt.y)**2
        d += (pt1.z - pt.z)**2
        d = np.sqrt(d)
        return d
    
    def distance_2d(self, pt):
        """Расстояние до точки в плоскости xOy

        Args:
            pt (geometry_msgs.msg.Point): точка в пространстве

        Returns:
            float: расстояние от дрона до точки в метрах
        """
        pt1 = self.local_position.pose.position
        d = (pt1.x - pt.x)**2
        d += (pt1.y - pt.y)**2
        d = np.sqrt(d)
        return d

    def send_pos(self, x, y, z, yaw=None, frame_id="world", pos_target=False):
        """Отправляет дрона в заданную позицию. Если угол не задан, то
        дрон прилетит в точку с текущей ориентацией.

        Args:
            x (float): x-координата
            y (float): y-координата
            z (float): z-координата
            yaw (float, optional): ориентация вокруг оси z. Defaults to None.
            frame_id (str, optional): система координат. Defaults to "world".
            pos_target (bool, optional): отправлять точку в формате PositionTarget
        """
        if pos_target:
            tgt = RosAuxTools.pos2PosTarget(x, y, z)
            self.pos_target_pub.publish(tgt)
        else:
            tgt = RosAuxTools.pose_stamped(x, y, z, yaw, frame_id)
            # TODO: изменить отправку текущего угла - здесь может закрасться ошибка
            if yaw is None:
                tgt.pose.orientation = self.local_position.pose.orientation
                yaw = Algebra.quat2euler(tgt.pose.orientation)[-1]
            self.setpoint_pub.publish(tgt)

    def send_vel(self, vx, vy, vz, ang_z):
        """Отправляет команду скорости дрону

        Args:
            vx (float): линейная скорость по оси x
            vy (float): линейная скорость по оси y
            vz (float): линейная скорость по оси z
            ang_z (float): угловая скорость по оси z
        """
        vel_tgt = RosAuxTools.velocity_pos_target(vx, vy, vz, ang_z)
        self.pos_target_pub.publish(vel_tgt)

    def send_altitude(self, alt, frame_id="world"):
        """Отправляет дрона на заданную высоту

        Args:
            alt (float): высота в метрах
            frame_id (str, optional): система координат. Defaults to "world".
        """
        x = round(self.local_position.pose.position.x, 1)
        y = round(self.local_position.pose.position.y, 1)
        self.send_pos(x, y, alt, frame_id=frame_id)

    # DEPRECATED
    def send_yaw(self, yaw, altitude):
        x = round(self.local_position.pose.position.x, 1)
        y = round(self.local_position.pose.position.y, 1)
        self.send_pos(x, y, altitude, yaw)

    @abstractmethod
    def send_takeoff(self, height=0.5):
        """Отправляет команду взлета полетному контроллеру

        Args:
            height (float, optional): высота взлета. Defaults to 0.5.
        """
        pass

    @abstractmethod
    def takeoff(self, height=0.5):
        """Осуществляет взлет дрона (полный цикл взлета)"""
        pass

    @abstractmethod
    def go2point(self, x, y, z, tolerance):
        """Летит в указанную точку с заданной точностью

        Args:
            x (float): x-координата точки
            y (float): y-координата точки
            z (float): z-координата точки
            tolerance (float): заданная точность в метрах

        """
        pass

    @abstractmethod
    def land(self):
        """Приземляет квадракоптер"""
        pass

    def follow_trajectory(self, poses, altitude, tolerance=0.1):
        """Летит по заданной траектории

        Args:
            poses (np.array): точки плоской траектории в формате: [[x0,y0], [x1,y1], ..., [xN, yN]]
            altitude (double): высота траектории над землей
            tolerance (float, optional): точность прилета в точки. Defaults to 0.1.
        """
        for [x, y] in poses:
            rospy.loginfo("Drone goes to point: ({0}; {1}; {2})".format(x, y, altitude))
            self.go2point(x, y, altitude, tolerance)

    def create_square_traj(self, size=2):
        """Генерирует траекторию полета по квадрату

        Args:
            size (int, optional): размер стороны квадрата в метрах. Defaults to 2.

        Returns:
            np.array: двумерный массив x,y координат точек
        """
        # Создаем квадратную траекторию
        poses = np.array(((0, 0), (0, 1), (-1, 1), (-1, 0), (0, 0)), dtype="float64")
        # Масштабируем ее
        poses *= size
        return poses

    def square_flight(self, altitude, size=2, tolerance=0.1):
        """Осуществляет полет по квадрату

        Args:
            altitude (double): высота траектории над землей
            size (int, optional): размер стороны квадрата в метрах. Defaults to 2.
            tolerance (float, optional): точность прилета в точки. Defaults to 0.1.
        """
        # Получаем траекторию квадрата
        poses = self.create_square_traj(size)
        # Взлетаем
        self.takeoff(height=altitude)
        try:
            # Летим по траектории
            self.follow_trajectory(poses, altitude, tolerance)
            # Приземляемся
            self.land()
        except Exception as e:
            rospy.logerr(e)

    def spin(
        self, ang_vel, altitude, stop_cond=lambda a, l: False, on_finish=lambda: None
    ):
        """Вращает дрона с заданной скоростью на заданной высоте. Дрон при этом удерживает свою текущую позицию.

        Args:
            ang_vel (float): угловая скорость в рад/с
            altitude (float): высота в метрах
            stop_cond (Callable, optional): предикат остановки вращения - stop_cond(angle, laps). Defaults to False.
            on_finish (Callable, optional): функция, которая вызывается после остановки вращения: on_finish(). Defaults to False.
        """

        # Сохраняем точку вращения
        self._spin_point = RosAuxTools.extract_point(self.local_position)
        self._spin_point.z = altitude
        # Скорость вращения
        self._spin_vel = ang_vel
        # Критерий остановки вращения
        self._stop_cond = stop_cond
        # Запоминаем угол начала вращения
        self._start_yaw = Algebra.quat2euler(self.local_position.pose.orientation)[2]
        self._start_yaw = round(Algebra.angle2positive(self._start_yaw), 3)
        # Предыдущий угол вращения
        self._spin_last_angle = 0
        # Количество прокрученных кругов
        self._spinned_laps = 0
        # Функция, вызываемая по окончании вращения
        self._on_finish = on_finish
        # Счетчик итераций таймера при вращении
        self._spin_iterations = 0
        # Угол дрона при вращении
        self._angle = 0
        # Включаем режим вращения
        self._spin_mode = True

    def cancel_spin(self):
        """Отменяет вращение"""
        # Выключаем режим вращения
        self._spin_mode = False
        # # Вызываем функцию после остановки вращения
        # if not self.on_finish is None:
        #     self.on_finish()

    
    # DEPRECATED
    def rotate(self, ang_vel, altitude, stop_cond=lambda a,l: False):

        spin_point = RosAuxTools.extract_point(self.local_position)

        start_yaw = Algebra.quat2euler(self.local_position.pose.orientation)[2]
        start_yaw = round(Algebra.angle2positive(start_yaw), 3)      
        
        loop_freq = 10
        yaw_rate = ang_vel / loop_freq
        rate = rospy.Rate(loop_freq)

        # Предыдущий угол вращения
        spin_last_angle = 0
        # Количество прокрученных кругов
        spinned_laps = 0
        # Счетчик итераций таймера при вращении
        spin_iterations = 0
        # Угол дрона при вращении
        angle = 0

        while not stop_cond(angle, spinned_laps):
            # Вычисляем угол дрона относительно угла начала вращения
            angle = self.positive_yaw_from_start(
                start_yaw, np.sign(yaw_rate)
            )
            # Костыльная эвристика:
            # В начале вращения угол не может быть близок к 2*pi
            # Считаем, что ему нужно минимум 5 итераций, чтобы достичь такого значения
            if spin_iterations < 5 and angle > 6:
                angle = 0

            # Вычисляем количество сделанных кругов
            # Предыдущий угол больше текущего и целая часть предыдущего угла равна 6
            if angle < spin_last_angle and round(spin_last_angle) == 6:
                spinned_laps += 1

            # Запоминаем предыдущий угол
            spin_last_angle = angle

            # Вычисялем новый угол дрона
            new_angle = start_yaw + np.sign(yaw_rate) * angle + yaw_rate
            # Отправляем новый угол
            self.send_pos(spin_point.x, spin_point.y, altitude, new_angle)           
            rate.sleep()

class PX4Controller(MavController):
    """Класс для управления дроном с автопилотом PX4"""

    def __init__(self, vicinity=0.1, use_vision_odometry=False):
        # Окрестность точки взлета, в которую надо попасть
        self.vicinity = vicinity
        MavController.__init__(self, use_vision_odometry)

    def send_takeoff(self, height=0.5):
        try:
            self._hover_flag = True
            rospy.loginfo("Taking off")
            # Запоминаем координаты взлета
            x = round(self.local_position.pose.position.x, 1)
            y = round(self.local_position.pose.position.y, 1)
            while rospy.is_shutdown() == False:
                self.send_pos(x, y, height, frame_id="world")
                if self.is_at_height(height, self.vicinity):
                    break
                self.rate.sleep()
            rospy.loginfo("Reach the height: {0}".format(height))
        except Exception as e:
            self._hover_flag = False
            rospy.logerr(e)


    def takeoff(self, height=0.5):
        # Публикуем сперва точку для инициализации
        self.init_autopilot()

        # Ждем подключения к дрону
        self.wait_conection()

        # Устанавливаем режим
        self.set_mode("OFFBOARD")

        # Разблокируем дрона
        self.arm_drone()

        # Взлетаем
        self.send_takeoff(height)

    def go2point(self, x, y, z, tolerance):
        self.send_pos(x, y, z)
        self.wait_position(x, y, z, tolerance, self.send_pos)

    @RosAuxTools.safe_decorator
    def land(self):
        self._hover_flag = False
        rospy.loginfo("Sending land signal")
        # self.set_mode_srv(0, "AUTO.LAND")
        self.set_mode("AUTO.LAND")
        self.arm(False)

class ArduController(MavController):
    """Класс для управления дроном с автопилотом ArduPilot"""

    def __init__(self, vicinity=0.1, use_vision_odometry=False):
        # Окрестность точки взлета, в которую надо попасть
        self.vicinity = vicinity
        MavController.__init__(self, use_vision_odometry)

    def send_takeoff(self, height=0.5):
        try:
            takeoff_resp = self.takeoff_srv(0.0, 0.0, 0.0, 0.0, height)
            #  Ожидаем достижения заданной точки дроном
            self.wait_height(height)
            rospy.loginfo(takeoff_resp.success)
            rospy.loginfo(takeoff_resp.result)
            return takeoff_resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
            return False

    def takeoff(self, height=0.5):
        # Публикуем сперва точку для инициализации
        self.init_autopilot()

        # Ждем подключения к дрону
        self.wait_conection()

        # Устанавливаем режим
        self.set_mode("GUIDED")

        # Разблокируем дрона
        self.arm_drone()

        # Взлетаем
        self.send_takeoff(height)

    def go2point(self, x, y, z, tolerance):
        self.send_pos(x, y, z)
        self.wait_position(x, y, z, tolerance)

    @RosAuxTools.safe_decorator
    def land(self):
        try:
            rospy.loginfo("Sending land signal")
            land_resp = self.land_srv(0.0, 0.0, 0.0, 0.0, 0.0)
            rospy.loginfo(land_resp.success)
            rospy.loginfo(land_resp.result)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
