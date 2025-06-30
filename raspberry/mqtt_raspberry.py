import time
import paho.mqtt.client as mqtt
import json
import struct
import serial

class MQTTClient:
    def __init__(self, client_id='imu_client', broker='localhost', port=1883):
        self.client = mqtt.Client()
        self.broker = broker
        self.port = port
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        #arduino serial connection
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        time.sleep(3)
        self.ser.reset_input_buffer()
        print("[MQTTClient] Serial connection to Arduino OK")

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Connected with result code {rc}")
        client.subscribe("imu/acceleration", qos=0)   

    def connect_(self):
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
    
    def disconnect_(self):
        self.client.loop_stop()
        self.client.disconnect()

    def publish_(self, topic, data: dict):
        payload = json.dumps(data)
        self.client.publish(topic, payload, qos=0)

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode().strip()
            if payload.startswith("servo:"):
                pwm_servo = int(payload.split(":")[1])
                if 660 <= pwm_servo <= 1600:
                    packed = struct.pack('<B H', 0x01, pwm_servo)
                    self.ser.write(packed)
                    #print(f"Sent to Arduino [servo]: {pwm_servo}")
                #else:
                    #print("Servo PWM out of range")
            
            elif payload.startswith("esc:"):
                pwm_ESC = int(payload.split(":")[1])
                if 1500 <= pwm_ESC <= 2000:
                    packed = struct.pack('<B H', 0x02, pwm_ESC)
                    self.ser.write(packed)
                    #print(f"Sent to Arduino [ESC]: {pwm_ESC}")
                else:
                    packed = struct.pack('<B H', 0x02, 0)
                    self.ser.write(packed)
            #else:
                #print("Invalid message prefix")

        except ValueError:
            print("Invalid PWM value received")

if __name__ == "__main__":
    client_ = MQTTClient(client_id="imu_data", broker='localhost', port=1883) 
    client_.connect_()
    client_.client.subscribe('servo_esc_cmd')

    try:
        while True:
            accel_data = {
            "x": 0,
            "y": 0,
            "z": 0
            }
            client_.publish_("imu", accel_data)
            print(f"Sent: {accel_data}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping...")
        client_.ser.close()
        client_.disconnect_()