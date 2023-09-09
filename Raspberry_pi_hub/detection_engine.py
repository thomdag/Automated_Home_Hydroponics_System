from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils

class DetectionEngine:
    
    def __init__(self, model_path, model_resolution=(300, 300), coral_tpu=False, threadcount=1, score_threshold=0.5):
        self.model_resolution = model_resolution
        base_options = core.BaseOptions(file_name=model_path, use_coral=coral_tpu, num_threads=threadcount)
        detection_options = processor.ClassificationOptions(score_threshold=score_threshold)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector = vision.ObjectDetector.create_from_options(options)

    def run_inference(self, image):
        resized_image = utils.image.resize(image, self.model_resolution)
        tensor_image = vision.TensorImage.create_from_array(resized_image)
        inference_results = self.detector.detect(tensor_image)
        return inference_results

    def visualize_results(self, image, inference_results):
        return utils.visualize(image, inference_results)

    def __del__(self):
        if hasattr(self, 'detector'):
            del self.detector  # Delete the detector instance

