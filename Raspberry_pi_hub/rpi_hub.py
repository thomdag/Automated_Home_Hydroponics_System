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
        self.serial_buffer = [0.0, 0.0, 0.0, 0.0]  # Initialize serial_buffer
        self.error_counts = {"sensor": 0,"image": 0,"overtime": 0,}
        self.image = None
        self.detection_results = None
        self.sensor_results = None
        #self.image_sender = streaming_sender.image_pub
    
    def __del__(self):
        self.cam.release_camera()
        GPIO.cleanup()
        
    def ImageDetection(self):
        self.image = self.cam.capture_image()
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.detection_results = self.detector.run_inference(rgb_image)
        
    def PrimaryLoop(self):
        while True:
            log_entry = []
            self.ImageDetection()
            self.sensor_results = self.GetSensorData()
            if self.error_flag[0] == error_states["SAFE"]:
                error_status_sensor = self.CheckSensorError()
                log_entry.append(error_status_sensor)  # Append error status
            elif self.error_flag[0] == error_states["ERROR"]:
                log_entry.append(self.sensor_results)
            inference_status = self.CheckImageError(self.detection_results)
            log_entry.append(inference_status)  # Append inference status
            if log_entry[0] == error_states["SAFE"] and log_entry[1] == error_states["SAFE"]:
                self.ErrorCount(error_states["SAFE"])
            else:
                self.ErrorCount(error_states["ERROR"])
            self.LogRecord(log_entry)
            GPIO.output(alert_led, GPIO.HIGH)

    def LogRecord(self, log):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        with open(log_dir, "a") as writer:  # Use "a" to append log entries
            writer.write(f"{timestamp}, {', '.join(log)}\n")

    def ErrorCount(self, state):
        if state == error_states["SAFE"] and self.error_over_time == 0:
            self.error_tally = 0
        elif state == error_states["SAFE"] and self.error_over_time != 0:
            self.error_tally = 0
            self.error_over_time -= 1  # Decrement error_over_time
        else:
            self.error_over_time += 1  # Increment error_over_time
            self.error_tally += 1
        if self.error_over_time > error_overtime_max or self.error_tally == error_tally_max:
            self.SendAlert()
            
    def CheckSensorError(self):
        if tds_upper < self.serial_buffer[1] < tds_lower:
            return error_states["SAFE"]
        elif temp_lower < self.serial_buffer[0] < temp_upper:
            return error_states["SAFE"]
        elif ph_lower < self.serial_buffer[2] < ph_upper:
            return error_states["SAFE"]
        else:
            return error_states["ERROR"]
        
    def CheckImageError(self, inference_results):
        if any(x in error_tags for x in inference_results):
            self.error_flag[1] = 1
            return "error_image"
        elif not inference_results:
            self.error_flag[1] = 1
            return "error_image"
        else:
            return error_states["SAFE"]
        
    def GetSensorData(self):
        self.RequestReading()
        while self.serial_connection.in_waiting == 0:
            time.sleep(0.2)
        return self.ReadReply()
        
    def RequestReading(self):
        self.serial_connection.write(default_request.encode())  # Convert string to bytes
    
    def ReadReply(self):
        message = self.serial_connection.readline().decode().strip()  # Decode bytes to string
        sensor_readings = message.split("*")
        if sensor_readings[0] != "///":
            self.error_flag[0] = 1
            return "error_handshake"
        try:
            for i in range(3):
                self.serial_buffer[i] = float(sensor_readings[i + 1])
            return error_states["SAFE"]
        except ValueError:
            self.error_flag[0] = 1
            return "error_valuetype"
                
    def SendAlert(self):
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