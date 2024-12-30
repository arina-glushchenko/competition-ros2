import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from ultralytics import YOLO
from sensor_msgs.msg import LaserScan, Image
import os
from ament_index_python.packages import get_package_share_directory
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Controller(Node):
    '''Класс реализует детекцию объектов в реальном времени и 
       публикует классы обнаруженных объектов в топик.'''

    def __init__(self):
        super().__init__('detect')
        # Инициализация сообщений
        self.br = CvBridge()
        self.twist = Twist()
        self.sign = Int32()

        # Создание паблишеров для обработки
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.yolo_publisher = self.create_publisher(Int32, '/comand', 10)
        self.finish_pub = self.create_publisher(String, '/robot_finish', 1)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Создание подписчиков для топиков с камерой
        color_sub = Subscriber(self, Image, '/color/image')
        depth_sub = Subscriber(self, Image, '/depth/image')

        # Загрузка модели YOLO
        self.work_direct = self.declare_parameter('model_path', '').get_parameter_value().string_value
        weight_direct = self.work_direct + '/best.pt' 

        package_name = 'comp_nedorosl'
        package_share_dir = get_package_share_directory(package_name)
        weights_path = os.path.join(package_share_dir, 'weights', 'best.pt')
        self.model = YOLO(weights_path)

        # Создание синхронизатора сообщений
        ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.callback)

        self.color_image, self.depth_image = None, None

    def yolo_detect(self):
        """
        Выполняет детекцию объектов на основе RGB-изображения с использованием модели YOLO.
        """
        # Преобразование изображения в формат RGB и передача его модели YOLO
        image_rgb = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB) 
        results = self.model(image_rgb)

        # Обработка результатов детекции
        detections = results[0].boxes.data.cpu().numpy()
        results_dict = {}

        for detection in detections:
            x_min, y_min, x_max, y_max, conf, cls = detection
            results_dict[int(cls)] = ((x_min, y_min, x_max, y_max), conf)

        return results_dict
    
    def lidar_callback(self, msg):
        """
        Обработчик данных от лидара.
        """
        self.lidar_data = msg.ranges
        self.process_lidar_data()

    def process_lidar_data(self):
        """
        Обрабатывает данные от лидара и выполняет логику на их основе.
        """
        if self.lidar_data is None:
            return

        front_index = 0  # Точка спереди
        left_index = 89   # Точка слева
        right_index = 269  # Точка справа
        front_point = self.lidar_data[front_index]
        left_point = self.lidar_data[left_index]
        right_point = self.lidar_data[right_index]

        self.adjust_movement_with_lidar(front_point, left_point, right_point)

    def adjust_movement_with_lidar(self, front_point, left_point, right_point):
        """
        Регулирует движение робота на основе данных от лидара.
        """
        if front_point < 0.2:
            self.sign.data = 3
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            self.finish_pub.publish(String(data='nedoROSl'))
            rclpy.shutdown()

    def callback(self, color_msg, depth_msg):
        """
        Обработчик изображений от цветной и глубинной камеры.
        """
        self.color_image = self.br.imgmsg_to_cv2(color_msg, color_msg.encoding)
        self.depth_image = self.br.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        if (self.color_image is not None) and (self.depth_image is not None):

            classes_bbox = self.yolo_detect()
            decoder_class = {'left': 5, 'right': 8}
            coder_class = {5: 'left', 8: 'right'}

            # Словари для хранения данных о дистанции и достоверности объектов
            class_distances = {}
            class_conf = {}

            # Размеры изображения
            img_height, img_width = self.depth_image.shape[:2]

            # Обработка каждого класса
            for class_name, (bbox, conf) in classes_bbox.items():

                x_min, y_min, x_max, y_max = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

                # Извлечение bbox из изображения глубины
                depth_bbox = self.depth_image[y_min:y_max, x_min:x_max]

                if class_name in coder_class:
                    name = coder_class[class_name]
                else:
                    name = "unknown"

                if conf > 0.65:
                    cv2.rectangle(self.color_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.putText(self.color_image, f'{name}: {conf}', (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX,  1.0, (0, 255, 0), 2)

                # Проверяем, есть ли пиксели в depth_bbox
                if depth_bbox.size > 0:
                    depth_bbox = np.where(np.isfinite(depth_bbox), depth_bbox, np.nan)
                    mean_val = np.nanmean(depth_bbox)
                    depth_bbox = np.where(np.isnan(depth_bbox), mean_val, depth_bbox)

                    # Вычисляем среднее значение глубины
                    mean_depth = np.mean(depth_bbox)

                    # Добавляем результат в словарь
                    class_distances[class_name] = mean_depth
                    class_conf[class_name] = conf
                else:
                    self.get_logger().info(f'bbox для {class_name} не содержит пикселей.')
                    
            cv2.imshow('YOLOv11', self.color_image)
	    cv2.waitKey(1)

            # Определяем действия на основе объектов
            if (decoder_class['right'] in class_distances) and (class_conf[decoder_class['right']] > 0.65) and \
                    (class_distances[decoder_class['right']] <= 0.65) and (class_distances[decoder_class['right']] >= 0.45):
                self.sign.data = 2

            elif (decoder_class['left'] in class_distances) and (class_conf[decoder_class['left']] > 0.65) and \
                    (class_distances[decoder_class['left']] <= 0.75) and (class_distances[decoder_class['left']] >= 0.525):
                self.sign.data = 1

            else:
                self.sign.data = 0

            self.yolo_publisher.publish(self.sign)

        # Сброс изображений
        self.color_image = None
        self.depth_image = None


def main(args=None):
    rclpy.init(args=args)
    robot_app = Controller()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
