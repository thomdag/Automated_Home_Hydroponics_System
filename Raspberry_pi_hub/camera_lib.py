import cv2
import sys

class Camera:
    def __init__(self, camera_id=0, width=640, height=480):
        self.cam = cv2.VideoCapture(camera_id)
        self.resolution = (width, height)

    def capture_image(self):
        success, image = self.cam.read()
        if not success:
            sys.exit('Capture Image failed - ERROR')
        return image

    def set_resolution(self):
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

    def release_camera(self):
        self.cam.release()

    def __del__(self):
        self.release_camera()

