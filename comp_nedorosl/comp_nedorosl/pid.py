import cv2
import numpy as np
import matplotlib.pyplot as plt

# ROS 2 библиотеки
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

from math import pi


class Controller(Node):
    def __init__(self):
        super().__init__('pid')

        # Объявление и инициализация параметров
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp', 0.002),
                ('Ki', 0.0001),
                ('Kd', 0.0),
                ('desiredV', 0.25),
            ]
        )

        # Инициализация переменных состояния
        self.obst = False
        self.PidUp = True
        self.light = False
        self.target = None
        self.right = None
        self.man = False
        self.after_car = False
        self.prev_dist = 10000.0
        self.dir = True

        # PID переменные состояния
        self.len_stack = 15  # Длина интегральной ошибки
        self.E = [0] * self.len_stack  # Кумулятивная ошибка
        self.old_e = 0  # Предыдущая ошибка

        # Счетчики
        self.count_max = 4
        self.count_obst_max = 5
        self.counter = self.count_max
        self.obst_counter = self.count_obst_max

        # Параметры области обнаружения светофоров
        self.distance = 0.57

        # ROS объекты коммуникации
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float64, '/detect/line', self.move_Controller, 5)
        self.subscript_traffic = self.create_subscription(Image, '/color/image', self.traffic_light, 1)
        self.yolo_traffic = self.create_subscription(Int32, '/comand', self.sign_detect, 10)
        self.finish_pub = self.create_publisher(String, '/robot_finish', 1)

        self.br = CvBridge()
        self.twist = Twist()

    def sign_detect(self, msg):
        """
        Обрабатывает сообщения о знаках дорожного движения.
        """
        if msg.data == 1:  # Поворот налево
            self.right = False
        elif msg.data == 2:  # Поворот направо
            self.right = True
        elif msg.data == 3:  # Завершение программы
            self.PidUp = False
            self.finish_pub.publish(String(data='nedoROSl'))
            rclpy.shutdown()
        elif msg.data == 5:  # Ручной режим
            self.man = True
        else:
            self.right = None

    def traffic_light(self, image):
        """
        Обнаруживает зеленый сигнал светофора.
        """
        if not self.light:
            cv_image = self.br.imgmsg_to_cv2(image, "bgr8")  # Конвертация ROS Image в OpenCV Image
            weight, high = cv_image.shape[1], cv_image.shape[0]

            # Определение области интереса для зеленого света
            k_w, k_h, delta = 11 / 15, 5 / 8.5, 5
            green_pred = cv_image[
                int(high * k_h) - delta:int(high * k_h) + delta,
                int(weight * k_w) - delta:int(weight * k_w) + delta
            ]
            green_pred = np.mean(green_pred, axis=(0, 1))

            # Проверка совпадения с зеленым цветом
            if green_pred[0] in range(0, 21) and green_pred[1] in range(90, 120) and green_pred[2] in range(0, 21):
                self.light = True

    def move_Controller(self, msg):
        """
        Управляет движением робота на основе обнаруженной линии и других входных данных.
        """
        if self.light:
            if (self.right is not None and self.counter == self.count_max) or (0 < self.counter < self.count_max):
                # Выполнение поворота на основе знака дорожного движения
                w = ((-1) if self.right else (1)) * pi / 6
                x = 0.085
                self.twist.linear.x = x
                self.twist.angular.z = float(w)
                self.publisher_.publish(self.twist)
                self.counter -= 1
                self.old_e = 0
                self.E = [0] * self.len_stack

            if self.counter == 0:
                self.counter = self.count_max

            if self.PidUp and self.counter == self.count_max:
                self.iteratePID(msg)

    def iteratePID(self, msg):
        """
        Рассчитывает и применяет PID управление для угловой скорости.
        """
        # Извлечение параметров PID
        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
        self.desiredV = self.get_parameter("desiredV").get_parameter_value().double_value

        # Инициализация целевого значения
        if self.target is None:
            self.target = msg.data + 40

        # Расчет ошибок PID
        err = self.target - msg.data
        e_P = err
        e_I = sum(self.E) + err
        e_D = err - self.old_e

        # Расчет угловой скорости
        w = self.Kp * e_P + self.Ki * e_I + self.Kd * e_D
        w_max, k_w_max = 1.0, 0.5
        w = max(min(w, w_max), -w_max)

        # Обновление истории ошибок
        self.E.pop(0)
        self.E.append(err)
        self.old_e = err

        # Обновление и публикация сообщения Twist
        if self.PidUp:
            self.twist.linear.x = self.desiredV * (1 - k_w_max * abs(w) / w_max)
            self.twist.angular.z = float(w)
            self.publisher_.publish(self.twist)


def main(args=None):
    """
    Основная функция для запуска ноды Controller.
    """
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

