from ultralytics import YOLO
import cv2
import numpy as np


class TrafficSignDetector:
    def __init__(self, model_path, img_size=640, conf_threshold=0.5):
        """
        Initializes the traffic sign detector.

        :param model_path: Path to the YOLO model.
        :param img_size: Input image size for the model (in pixels).
        :param conf_threshold: Confidence threshold for object detection.
        """
        # Load the YOLO model from the specified path
        self.model = YOLO(model_path)
        self.img_size = img_size
        self.conf_threshold = conf_threshold

    def detect_traffic_signs(self, img):
        """
        Detects traffic signs using the YOLO model.

        :param img: Input image (can be in RGB or BGR format).
        :return: The detected traffic sign class.
        """
        # Perform inference on the input image
        results = self.model.predict(source=img, imgsz=self.img_size, conf=self.conf_threshold)
        predictions = []

        # Process the detection results
        for result in results:
            for box in result.boxes:
                # Extract information for each detected object
                class_id = int(box.cls.cpu().numpy())  # Detected class ID
                confidence = float(box.conf.cpu().numpy())  # Confidence score
                bbox = box.xyxy.cpu().numpy().astype(int).flatten()  # Bounding box coordinates

                # Store the detection information
                predictions.append((class_id, confidence, bbox))

        # Identify the class with the highest confidence
        detected_class = "none"
        if predictions:
            # Select the prediction with the maximum confidence
            best_prediction = max(predictions, key=lambda x: x[1])
            detected_class = best_prediction[0]

        return detected_class

    def recognition(self, img):
        """
        High-level function for traffic sign recognition.

        :param img: Input image.
        :return: The detected traffic sign class.
        """
        return self.detect_traffic_signs(img)
