import board
import busio
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
from scipy.spatial.transform import Rotation as R 

class IMU:
    """ Class for BNO085 IMU  """

    def __init__(self, sda=board.SDA, scl=board.SCL, i2c=None, imu=None):
        if i2c is None:
            i2c = busio.I2C(scl, sda)
        if imu is None:
            self._imu = BNO08X_I2C(i2c)
        else:
            self._imu = imu

        self._imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        self._imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self._imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self._roll_bias = 0
        self._pitch_bias = 0
        self._gyr_bias = [0.0, 0.0, 0.0]
        self._acc_bias = [0.0, 0.0, 0.0]
        self._quat_bias_inv = None 

    def calibrate(self, samples=1000):
        " Calibrate and store quaternion, gyro, and accelerometer biases "
        gyr_total = [0.0, 0.0, 0.0]
        acc_total = [0.0, 0.0, 0.0]
        quats = []

        for _ in range(samples):
            q = self._imu.quaternion
            quats.append(q)

            gx, gy, gz = self._imu.gyro
            ax, ay, az = self._imu.acceleration
            gyr_total[0] += gx
            gyr_total[1] += gy
            gyr_total[2] += gz
            acc_total[0] += ax
            acc_total[1] += ay
            acc_total[2] += az

        self._gyr_bias = [x / samples for x in gyr_total]
        self._acc_bias = [x / samples for x in acc_total]

        q_avg = tuple(sum(x) / samples for x in zip(*quats))
        self._quat_bias_inv = R.from_quat([q_avg[0], q_avg[1], q_avg[2], q_avg[3]]).inv()

    @property
    def acceleration(self):
        ax, ay, az = self._imu.acceleration
        '''
        return (
            ax - self._acc_bias[0],
            ay - self._acc_bias[1],
            az - self._acc_bias[2]
        )
        '''
        return (
            ax,
            ay,
            az
        )
        

    @property
    def gyro(self):
        gx, gy, gz = self._imu.gyro
        '''
        return (
            gx - self._gyr_bias[0],
            gy - self._gyr_bias[1],
            gz - self._gyr_bias[2]
        )
        '''
        return (
            gx,
            gy,
            gz
        )
        
    @property
    def quaternion(self):
        return self._imu.quaternion  # (i, j, k, real)

    def _unbiased_rotation(self):
        quat = self.quaternion
        r_meas = R.from_quat([quat[0], quat[1], quat[2], quat[3]])
        return self._quat_bias_inv * r_meas
        
    @property
    def rp_deg(self):
        """ Return Roll, Pitch, Yaw in degrees from corrected quaternion """
        corrected = self._unbiased_rotation()
        roll, pitch, yaw = corrected.as_euler('xyz', degrees=False)
        return roll, pitch, yaw