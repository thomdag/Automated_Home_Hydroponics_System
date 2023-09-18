import time
import RPi.GPIO as GPIO
import cv2
import smbus
import detection_engine_tfl
import cam_lib
import serial
import smtplib
import streaming_sender
from threading import Thread

alert_led = 20
usb_dir = "/dev/ttyUSB0"
log_dir = "/hydroponic/log_notes.txt"

error_tally_max = 5
error_overtime_max = 25

smtp_server = "smtp.gmail.com"
smtp_port = 587
gmail_username = ""
gmail_password = ""
gmail_receive = ""

default_reply = "///*temp*tds*ph*///"
default_request = "///REQUEST///"

# plant features, tds and ph for current: lettuce
ph_upper = 6.0
ph_lower = 5.5
ph_variance = 1
tds_upper = 840
tds_lower = 560
tds_variance = 150
temp_upper = 30
temp_lower = 5

modeldir = ""
error_tags = []

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(alert_led, GPIO.OUT)
GPIO.output(alert_led, GPIO.LOW)

class HydroStation():
    
    def __init__(self):
        self.cam = cam_lib.camera_class
        self.detector = detection_engine_tfl.detection_engine(coral_tpu=False)
        self.serial_connection = serial.Serial(usb_dir, baudrate=9600, parity=serial.PARITY_NONE,
                                               stopbits=serial.STOPBITS_ONE)
        self.error_counts = {"connection": 0,"sensor": 0,"image": 0,}
        self.error_flag = {"connection": "SAFE","sensor": "SAFE","image": "SAFE",}
        #self.image_sender = streaming_sender.image_pub
    
    def __del__(self):
        self.cam.release_camera()
        GPIO.cleanup()
        
    def ImageDetection(self):
        image = self.cam.capture_image()
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        return self.detector.run_inference(rgb_image)
        
    def PrimaryLoop(self):
        while True:
            image_result = self.ImageDetection() # runs inferences
            sensor_buffer = self.GetSensorData() # gets sensor info
            if self.error_flag["connection"] == "SAFE": # if connection to sensor unit is fine, checks if value is fine
                self.CheckSensorError()  
            elif self.error_flag["connection"] == "ERROR":
                self.error_flag["sensor"] = "ERROR"
            self.CheckImageError(image_result)
            if sensor_buffer[3] == 1:
                self.ErrorAddition()
                self.AlertUserCheck()
            self.LogRecord(sensor_buffer, image_result)
            self.error_flag = {"connection": "SAFE","sensor": "SAFE","image": "SAFE",}
    
    def AlertUserCheck(self):
        if self.error_count["sensor"] > 15 or error_count["connection"] > 15 or self.error_count["image"] > 15:
            self.SendAlert()

    def LogRecord(self, sen_data, infer_data):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log = ("{} - sensors:{},{},{}, inferdata:{} error flags:{},{},{}".format(timestamp,sen_data[0],sen_data[1],sen_data[2], sen_data[3],infer_data, self.error_flag))
        with open(log_dir, "a") as writer:  # Use "a" to append log entries
            writer.write(log)

    def ErrorAddition(self, state):
        if state == self.error_flag["sensor"] = "ERROR":
            self.error_count["sensor"] = self.error_count.get("sensor") + 1
        elif state == self.error_flag["sensor"] = "SAFE" and self.error_count.get("sensor") > 0:
            self.error_count["sensor"] = self.error_count.get("sensor") - 1
        elif state == self.error_flag["sensor"] = "SAFE" and self.error_count.get("sensor") = 0:
            self.error_count["sensor"] = self.error_count.get("sensor")

        if state == self.error_flag["connection"] = "ERROR":
            self.error_count["connection"] = self.error_count.get("connection") + 1
        elif state == self.error_flag["connection"] = "SAFE" and self.error_count.get("connection") > 0:
            self.error_count["connection"] = self.error_count.get("connection") - 1
        elif state == self.error_flag["connection"] = "SAFE" and self.error_count.get("connection") = 0:
            self.error_count["connection"] = self.error_count.get("connection")

        if state == self.error_flag["image"] = "ERROR":
            self.error_count["image"] = self.error_count.get("image") + 1
        elif state == self.error_flag["image"] = "SAFE" and self.error_count.get("image") > 0:
            self.error_count["image"] = self.error_count.get("image") - 1
        elif state == self.error_flag["image"] = "SAFE" and self.error_count.get("image") = 0:
            self.error_count["image"] = self.error_count.get("image")


            
    def CheckSensorError(self, sensor_data):
        if sensor_data[0] < temp_lower or temp_upper <  sensor_data[0]:
            self.error_flag["sensor"] = "ERROR"
        if  sensor_data[1] < tds_lower or  sensor_data[1] < tds_upper:
            self.error_flag["sensor"] = "ERROR"
        if  sensor_data[2] < ph_lower or  sensor_data[2] < ph_upper:
            self.error_flag["sensor"] = "ERROR"
        
    def CheckImageError(self, inference_results):
        if any(x in error_tags for x in inference_results):
            self.error_flag["image"] = "ERROR"
        elif not inference_results:
            self.error_flag["image"] = "ERROR"
        
    def GetSensorData(self):
        self.RequestReading()
        time_elasped = 0
        while self.serial_connection.in_waiting == 0:
            time.sleep(0.2)
            time_elasped = time_elasped + 0.2
            if time_elasped > 60:
                self.error_flag["connection"] = "ERROR"
                return 0
        return self.readReply()
    
        
    def RequestReading(self):
        self.serial_connection.write(default_request.encode())  # Convert string to bytes
    
    def ReadReply(self):
        message = self.serial_connection.readline().decode().strip()  # Decode bytes to string
        sensor_readings = message.split("*")
        temp_buffer = []
        if sensor_readings[0] != "///":
            self.error_flag["connection"] = "ERROR"
            return 0
        try:
            for i in range(4):
                temp_buffer[i] = float(sensor_readings[i + 1])
            self.error_flag["connection"] = "SAFE"
            return temp_buffer
        except ValueError:
            self.error_flag["connection"] = "ERROR"
            return 0
                
    def SendAlert(self):
        self.error_counts = {"connection": 0,"sensor": 0,"image": 0,}
        headers = [
            "From: " + gmail_username,
            "Subject: Hydroponic system ALERT",
            "To: " + gmail_receive,
            "MIME-Version: 1.0",
            "Content-type: text/html"
        ]
        headers = "\r\n".join(headers)
        
        connection = smtplib.SMTP(smtp_server, smtp_port)
        connection.ehlo()
        connection.starttls()
        connection.ehlo()
        
        connection.login(gmail_username, gmail_password)
        connection.sendmail(gmail_username, gmail_receive, headers + "\r\n\r\n" + "The hydroponics system has detected an issue and requires attention.")
        connection.quit()

# Create an instance of HydroStation
hub = HydroStation()
time.sleep(5)  # Wait for initialization
hub.PrimaryLoop()