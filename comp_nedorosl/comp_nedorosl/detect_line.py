import os

# ROS 2 библиотеки
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

# OpenCV и NumPy
import cv2
import numpy as np

# OpenCV Bridge для работы с ROS Image
from cv_bridge import CvBridge


class DetectLane(Node):
    def __init__(self):
        super().__init__('line')

        # Инициализация паблишера и сабскрайбера
        self.publisher = self.create_publisher(Float64, '/detect/line', 5)
        self.subscription = self.create_subscription(
            Image,
            '/color/image_projected_compensated',
            self.cbFindLane,
            5
        )

        # Инициализация CvBridge для конвертации ROS Image -> OpenCV Image
        self.br = CvBridge()

        # Переменные для хранения предыдущих значений центроидов
        self.last_cx_white = 0
        self.last_cx_yellow = 600

    def cbFindLane(self, image_msg):
        """
        Callback для обработки изображений из камеры и детекции линий разметки.
        """
        # Конвертация изображения из ROS Image в OpenCV Image
        cv_image = self.br.imgmsg_to_cv2(image_msg, image_msg.encoding)
        height, width, _ = cv_image.shape

        # Создаем ограничивающую маску для правой половины экрана
        right_half_mask = np.zeros_like(cv_image[:, :, 0])
        right_half_mask[:, int(width // 5):] = 255

        # Определяем диапазоны для белого и жёлтого цветов в HSV пространстве
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        lower_white = np.array([0, 0, 250])
        upper_white = np.array([0, 0, 255])

        # Преобразование изображения в HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Создание масок для белого и жёлтого цветов
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Применение ограничивающей маски для белого цвета
        mask_white = cv2.bitwise_and(mask_white, right_half_mask)

        # Объединение масок белого и жёлтого цветов
        mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

        # Наложение объединенной маски на исходное изображение
        masked_image = cv2.bitwise_or(cv_image, cv_image, mask=mask_combined)

        # Вычисление моментов для белых и жёлтых масок
        moments_white = cv2.moments(mask_white)
        moments_yellow = cv2.moments(mask_yellow)

        # Вычисление центроидов с использованием моментов
        if moments_white['m00'] != 0 and moments_yellow['m00'] != 0:
            cx_white = int(moments_white['m10'] / moments_white['m00'])
            cx_yellow = int(moments_yellow['m10'] / moments_yellow['m00'])
            self.last_cx_white = cx_white
            self.last_cx_yellow = cx_yellow
        elif moments_white['m00'] == 0 and moments_yellow['m00'] != 0:
            cx_white = self.last_cx_white + 5
            cx_yellow = int(moments_yellow['m10'] / moments_yellow['m00'])
        elif moments_yellow['m00'] == 0 and moments_white['m00'] != 0:
            cx_white = int(moments_white['m10'] / moments_white['m00'])
            cx_yellow = self.last_cx_yellow - 5
        else:
            cx_white = self.last_cx_white
            cx_yellow = self.last_cx_white

        # Отображение изображения с наложенной маской
        # cv2.imshow('camera', masked_image)
        # cv2.waitKey(1)

        # Публикация значения средней линии между жёлтой и белой разметкой
        msg_desired_center = Float64()
        if cx_yellow < cx_white:
            msg_desired_center.data = (cx_yellow + cx_white) / 2
        self.publisher.publish(msg_desired_center)


def main(args=None):
    """
    Основная функция для запуска ноды DetectLane.
    """
    rclpy.init(args=args)

    # Создание объекта ноды
    robot_app = DetectLane()

    # Запуск ROS 2 ноды
    rclpy.spin(robot_app)

    # Очистка ресурсов перед завершением
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

