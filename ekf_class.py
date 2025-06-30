import numpy as np
if __name__ == "__main__":
    import paho.mqtt.client as mqtt
    import json
    import time
    import pygame
    import matplotlib.pyplot as plt

import numpy as np

class kinematic_ekf:
    """
    8-state EKF with inertial kinematics + gyro & accel biases.

        x = [X, Y, Vx, Vy, psi,  b_ax, b_ay, b_r]ᵀ
    inputs = raw IMU:  ax_m, ay_m           (m/s²) ,
                       r_m   = psi̇_m       (rad/s)
    measurements = yaw angle  psi_m (rad),
                   commanded body speed vx_cmd (m/s)
    """
    def __init__(self, Ts,
                 # initial state & covariance
                 x0=None, P0=None,                        
                 sigma_psi = np.deg2rad(0.1),              
                 sigma_vcmd = 0.001,                        
                 sigma_ax = 0.001, sigma_ay = 0.001,         
                 sigma_r  = 0.001,                         
                 sigma_bax = 0.001,  sigma_bay = 0.001,
                 sigma_br  = 0.001):
        self.Ts = Ts

        # ---------- state & covariance ----------
        if x0 is None:
            x0 = np.zeros(8)
        if P0 is None:
            P0 = np.diag([1e-6, 1e-6,
                          1e-6, 1e-6,
                          1e-6,
                          0.2**2, 0.2**2, np.deg2rad(2)**2])
        self.x = x0.astype(float)
        self.P = P0.astype(float)

        # ---------- process-noise matrix Q ----------
        T  = Ts
        T2 = T*T;  T3 = T2*T;  T4 = T2*T2
        q_ax = sigma_ax**2
        q_ay = sigma_ay**2
        q_r  = sigma_r**2
        q_bax = sigma_bax**2
        q_bay = sigma_bay**2
        q_br  = sigma_br**2

        self.Q = np.diag([
            0.25*T4*q_ax,     # X
            0.25*T4*q_ay,     # Y
            T2*q_ax,          # Vx
            T2*q_ay,          # Vy
            T2*q_r,           # psi
            T*q_bax,          # b_ax   
            T*q_bay,          # b_ay   
            T*q_br            # b_r    
        ])

        # ---------- measurement-noise matrix ----------
        self.R = np.diag([sigma_psi**2, sigma_vcmd**2])

    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------
    def predict(self, ax_m, ay_m, r_m):                      
        X, Y, Vx, Vy, psi, b_ax, b_ay, b_r = self.x
        T  = self.Ts
        c, s = np.cos(psi), np.sin(psi)

        # de-bias the raw IMU -----------------------------------------
        ax = ax_m - b_ax
        ay = ay_m - b_ay
        r  = r_m  - b_r

        # helper terms
        A = ax*c - ay*s
        B = ax*s + ay*c

        # propagate state ---------------------------------------------
        X  += Vx*T + 0.5*T*T * A
        Y  += Vy*T + 0.5*T*T * B
        Vx += T * A
        Vy += T * B
        psi = self._wrap(psi + r*T)
        self.x = np.array([X, Y, Vx, Vy, psi, b_ax, b_ay, b_r])
        
        
        # ------------------------------------------------------------------
        # Jacobian F_k  (8×8)
        # ------------------------------------------------------------------
        F = np.eye(8)
        # wrt original states (same as before)
        F[0, 2] = T
        F[0, 4] = -0.5*T*T * B
        F[1, 3] = T
        F[1, 4] =  0.5*T*T * A
        F[2, 4] = -T * B
        F[3, 4] =  T * A
        # wrt accel biases                                           
        F[0, 5] = -0.5*T*T * c           
        F[0, 6] =  0.5*T*T * s           
        F[1, 5] = -0.5*T*T * s           
        F[1, 6] = -0.5*T*T * c          
        F[2, 5] = -T * c                 
        F[2, 6] =  T * s                 
        F[3, 5] = -T * s                 
        F[3, 6] = -T * c                 
        F[4, 7] = -T                     

        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------
    def update(self, psi_meas, vx_cmd):
        X, Y, Vx, Vy, psi = self.x[:5]         
        c, s = np.cos(psi), np.sin(psi)

        # measurement prediction
        h = np.array([psi,
                      c*Vx + s*Vy])

        z = np.array([psi_meas, vx_cmd])
        y = z - h
        y[0] = self._wrap(y[0])                

        # Jacobian H_k  (2×8) -----------------------------------------
        H = np.zeros((2, 8))
        H[0, 4] = 1                            
        H[1, 2] = c
        H[1, 3] = s
        H[1, 4] = -s*Vx + c*Vy                 

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.x[4] = self._wrap(self.x[4])      

        I_KH = np.eye(8) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    @staticmethod
    def _wrap(angle):
        """wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2*np.pi) - np.pi


if __name__ == "__main__":

        # MQTT Setup
    broker_address = "192.168.1.59"
    data_topic = "imu"
    cmd_topic = "servo_esc_cmd"
    prev_time = None
    ax_imu, ay_imu, r_imu, yaw_imu = 0.0, 0.0, 0.0, 0.0
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # MQTT Callbacks
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code", rc)
        client.subscribe(data_topic, qos=1)

    def on_message(client, userdata, msg):
        global prev_time, ax_imu, ay_imu, r_imu, yaw_imu
        payload = json.loads(msg.payload.decode())
        timestamp_ns = payload.get("timestamp", None)
        if timestamp_ns:
            timestamp_s = timestamp_ns / 1e9
            if prev_time is not None:
                interval = timestamp_s - prev_time
                #print(f"[From IMU] Interval: {interval:.5f} seconds (~{1/interval:.2f} Hz)")
            prev_time = timestamp_s
        ax_imu = payload["acc"]["x"]
        ay_imu = payload["acc"]["y"]
        yaw_imu = payload["orientation"]["yaw"]
        r_imu = payload["gyro"]["z"]

    # MQTT Client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address, 1883, 60)
    client.loop_start()

    def map_range(val, in_min, in_max, out_min, out_max):
        return int((val - in_min) / (in_max - in_min) * (out_max - out_min) + out_min)

    Ts = 0.01
    ekf = kinematic_ekf(Ts=Ts)  
    next_t = time.perf_counter()
    log = []            
    states_ekf = []
    try:
        while True:
            t0 = time.perf_counter()  
            pygame.event.pump()
            left_x = joystick.get_axis(0)  # Left stick X-axis for steering
            rt = joystick.get_axis(5)      # Right Trigger (RT)
            rt = (rt + 1) / 2

            # SERVO (700-1500)
            servo_pwm = map_range(left_x, -1.0, 1.0, 660, 1600)
            #ESC (1580-2000)
            if rt > 0.01:
                esc_pwm = map_range(rt, 0, 1, 1500, 2000)
                vx_cmd = 1.32105055e-02*esc_pwm -2.02096193e+01
            else:
                vx_cmd = 0
                esc_pwm = 0      
            ekf.predict(ax_imu, ay_imu, r_imu)
            ekf.update(yaw_imu, vx_cmd)
            client.publish(cmd_topic, f"servo:{servo_pwm}", qos=1)
            client.publish(cmd_topic, f"esc:{esc_pwm}", qos=1)
            X, Y, Vx, Vy, psi, bx, by, br = ekf.x
            states_ekf.append([ekf.x])
            print(X, Y)
            log.append(time.perf_counter() - t0)
            next_t += Ts
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                print(f"overrun {-sleep*1e3:.2f} ms")
                next_t = time.perf_counter()   
    
    except KeyboardInterrupt:
        client.publish(cmd_topic, f"esc:{0}", qos=1)
        print("Shutting down...")
        client.disconnect()
        np.save("test_ekf.npy", np.array(states_ekf))
        trajectory = np.array(states_ekf)
        print(trajectory.shape)
        X = trajectory[:, :, 0]  
        Y = trajectory[:, :, 1] 
        psi = trajectory[:, :, 4]
        plt.figure(figsize=(10, 6))
        plt.plot(X, Y, 'b-', label='Robot Trajectory')
        plt.scatter(X[0], Y[0], color='green', marker='o', label='Start')
        plt.scatter(X[-1], Y[-1], color='red', marker='x', label='End')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Robot Trajectory in X-Y Plane')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  
        plt.show()
        plt.plot(psi, label='Yaw Angle')
        plt.show()
        plt.plot(Y)
        plt.title('Y trajectory')
        plt.show()
