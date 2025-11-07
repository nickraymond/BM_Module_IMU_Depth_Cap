# import time
# import math
# from adafruit_bno055 import BNO055_I2C
#
# class IMU:
#     def __init__(self, i2c, calibration_offsets=None):
#         """
#         Initialize the IMU with an I2C object and optional calibration offsets.
#         """
#         self.bno = BNO055_I2C(i2c)
#         self.calibration_offsets = calibration_offsets or {
#             "magnetometer": (-682, -420, -354),
#             "gyroscope": (0, -3, 0),
#             "accelerometer": (-25, 83, -35),
#         }
#         #self.q_offset = (0.697754, 0.052124, -0.713684, 0.0333252)  # first attempt
#         self.q_offset = (0.689575, 0.122559, -0.707336, 0.0952759)  # calibrated 2025/03/09
#
#
#         time.sleep(2)  # Allow sensor to initialize
#         self.apply_calibration_offsets()
#         time.sleep(2)  # Wait for calibration to take effect
#
#     def apply_calibration_offsets(self):
#         """Apply predefined calibration offsets to the IMU."""
#         print("\n🛠 Applying IMU Calibration Offsets...")
#         self.bno.offsets_magnetometer = self.calibration_offsets["magnetometer"]
#         self.bno.offsets_gyroscope = self.calibration_offsets["gyroscope"]
#         self.bno.offsets_accelerometer = self.calibration_offsets["accelerometer"]
#         print("✅ Calibration Offsets Applied Successfully!\n")
#
#     @staticmethod
#     def quaternion_multiply(q1, q2):
#         """Perform quaternion multiplication."""
#         w1, x1, y1, z1 = q1
#         w2, x2, y2, z2 = q2
#         return (
#             w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
#             w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
#             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
#             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
#         )
#
#     @staticmethod
#     def quaternion_conjugate(q):
#         """Compute the conjugate of a quaternion."""
#         w, x, y, z = q
#         return (w, -x, -y, -z)
#
#     @staticmethod
#     def quaternion_normalize(q):
#         """Ensure quaternion is unit length."""
#         w, x, y, z = q
#         norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
#         if norm == 0:
#             return (1.0, 0.0, 0.0, 0.0)  # Return identity quaternion if norm is zero
#         return (w / norm, x / norm, y / norm, z / norm)
#
#
#     def get_corrected_raw_acceleration_00(self):
#         """Get corrected raw acceleration values in X, Y, Z using quaternion rotation."""
#         raw_accel = self.bno.acceleration  # Use calibrated raw acceleration including gravity
#         raw_quaternion = self.bno.quaternion  # Read IMU orientation quaternion
#         q_conjugate = self.quaternion_conjugate(raw_quaternion)
#
#         # Apply calibration offsets to acceleration
#         ax = raw_accel[0] - self.calibration_offsets["accelerometer"][0]
#         ay = raw_accel[1] - self.calibration_offsets["accelerometer"][1]
#         az = raw_accel[2] - self.calibration_offsets["accelerometer"][2]
#
#         # Convert acceleration to quaternion (w=0, x=ax, y=ay, z=az)
#         accel_quaternion = (0.0, ax, ay, az)
#
#         # Apply quaternion transformation: q * accel * q_conjugate
#         rotated_accel_q = self.quaternion_multiply(
#             self.quaternion_multiply(raw_quaternion, accel_quaternion), q_conjugate
#         )
#
#         # Return only the vector part (x, y, z) as world-frame acceleration
#         return rotated_accel_q[1], rotated_accel_q[2], rotated_accel_q[3]
#
#
#     def get_corrected_orientation(self):
#         """Get the corrected Euler angles from the IMU."""
#         raw_quaternion = self.bno.quaternion  # Read raw quaternion
#         corrected_quaternion = self.quaternion_multiply(raw_quaternion, self.quaternion_conjugate(self.q_offset))
#         roll, pitch, yaw = self.quaternion_to_euler(corrected_quaternion)
#
#         # Calculate tilt from corrected roll and pitch
#         tilt = self.calculate_tilt(roll, pitch)
#
#         return roll, pitch, yaw, tilt
#
#     def calculate_tilt(self, roll, pitch):
#         """
#         Calculate the absolute tilt angle relative to vertical using roll and pitch.
#         """
#         roll_rad = math.radians(roll)
#         pitch_rad = math.radians(pitch)
#         tilt_rad = math.acos(math.sqrt(math.cos(roll_rad)**2 * math.cos(pitch_rad)**2))
#         return math.degrees(tilt_rad)
#
#
#     def get_corrected_linear_acceleration(self):
#         """Get corrected linear acceleration values in X, Y, Z using quaternion rotation."""
#         raw_accel = self.bno.linear_acceleration  # Use gravity-free acceleration
#         raw_quaternion = self.bno.quaternion  # Read raw quaternion
#         q_conjugate = self.quaternion_conjugate(raw_quaternion)
#
#         # Convert acceleration to quaternion (w=0, x=ax, y=ay, z=az)
#         accel_quaternion = (0.0, raw_accel[0], raw_accel[1], raw_accel[2])
#
#         # Rotate acceleration using correct quaternion transformation: q * accel * q_conjugate
#         rotated_accel_q = self.quaternion_multiply(
#             self.quaternion_multiply(raw_quaternion, accel_quaternion), q_conjugate
#         )
#
#         # Return only the vector part (x, y, z)
#         return rotated_accel_q[1], rotated_accel_q[2], rotated_accel_q[3]
#
#     def quaternion_to_euler(self, q):
#         """Convert quaternion to Euler angles (Roll, Pitch, Yaw)."""
#         w, x, y, z = q
#         ysqr = y * y
#
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + ysqr)
#         roll = math.degrees(math.atan2(t0, t1))
#
#         t2 = +2.0 * (w * y - z * x)
#         t2 = max(min(t2, 1.0), -1.0)
#         pitch = math.degrees(math.asin(t2))
#
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (ysqr + z * z)
#         yaw = math.degrees(math.atan2(t3, t4))
#
#         return roll, pitch, yaw
#
#
#
#     def get_corrected_raw_acceleration(self):
#         """Get corrected raw acceleration values in X, Y, Z using quaternion rotation."""
#         raw_accel = self.bno.acceleration  # Read sensor-frame acceleration
#         raw_quaternion = self.bno.quaternion  # Read IMU orientation
#         q_offset_conjugate = self.quaternion_conjugate(self.q_offset)  # Get conjugate of offset
#
#         # Step 1: Apply offset correction to IMU orientation
#         corrected_quaternion = self.quaternion_multiply(raw_quaternion, q_offset_conjugate)
#         corrected_quaternion = self.quaternion_normalize(corrected_quaternion)  # Normalize
#
#         print(f"Raw Quaternion: {raw_quaternion}")
#         print(f"Corrected Quaternion: {corrected_quaternion}")
#         print(f"Raw Acceleration (sensor frame): {raw_accel}")
#
#         # Step 2: Convert acceleration to quaternion form (w=0, x=ax, y=ay, z=az)
#         accel_quaternion = (0.0, raw_accel[0], raw_accel[1], raw_accel[2])
#
#         # Step 3: Apply quaternion transformation: q_corrected * accel * q_corrected^*
#         q_corrected_conjugate = self.quaternion_conjugate(corrected_quaternion)
#         rotated_accel_q = self.quaternion_multiply(
#             self.quaternion_multiply(corrected_quaternion, accel_quaternion), q_corrected_conjugate
#         )
#         # Reverse the direction of the rotation, some do it differently
#         #rotated_accel_q = self.quaternion_multiply(
#         #    self.quaternion_conjugate(corrected_quaternion),
#         #   self.quaternion_multiply(accel_quaternion, corrected_quaternion)
#         #)
#
#         # Step 4: Extract world-frame acceleration
#         #corrected_accel = (rotated_accel_q[1], rotated_accel_q[2], rotated_accel_q[3])
#
#         # first attempt, uses North-East-Down (NED)
#         corrected_accel = (rotated_accel_q[1], rotated_accel_q[2], rotated_accel_q[3])
#
#         # second attemp, uses East-North-Up (ENU)
#         #corrected_accel = (rotated_accel_q[1], -rotated_accel_q[2], -rotated_accel_q[3])
#
#
#         print(f"Corrected Acceleration (world frame): {corrected_accel}")
#
#         return corrected_accel
#
#     def get_swapped_acceleration(self):
#         """Convert sensor-frame acceleration to world-frame using axis swapping."""
#         raw_accel = self.bno.acceleration  # Read sensor-frame acceleration
#         world_accel = (-raw_accel[2], raw_accel[1], raw_accel[0])  # Apply axis swap
#         print(f"Raw Acceleration (sensor frame): {raw_accel}")
#         print(f"Corrected Acceleration(world frame): {world_accel}")
#
#         return world_accel
#
#     def get_swapped_linear_acceleration(self):
#         """Convert sensor-frame acceleration to world-frame using axis swapping."""
#         raw_linear_accel = self.bno.linear_acceleration  # Read sensor-frame acceleration
#         world_linear_accel = (-raw_linear_accel[2], raw_linear_accel[1], raw_linear_accel[0])  # Apply axis swap
#         #print(f"Raw Linear Acceleration (sensor frame): {raw_linear_accel}")
#         #print(f"Corrected Linear Acceleration(world frame): {world_linear_accel}")
#
#         return world_linear_accel
#
#
# imu_module.py — BNO055 + quaternion-offset corrected orientation (mag north via NDOF)
import time, math
from adafruit_bno055 import BNO055_I2C

# ---- Quaternion helpers ----
def _q_norm(q):
    w, x, y, z = q
    n = (w*w + x*x + y*y + z*z) ** 0.5
    if n == 0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)

def _q_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def _q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

def _q_from_euler_rpy(r_deg, p_deg, y_deg):
    # roll (X), pitch (Y), yaw (Z) → quaternion
    r = math.radians(r_deg); p = math.radians(p_deg); y = math.radians(y_deg)
    cr, sr = math.cos(r/2.0), math.sin(r/2.0)
    cp, sp = math.cos(p/2.0), math.sin(p/2.0)
    cy, sy = math.cos(y/2.0), math.sin(y/2.0)
    return _q_norm((
        cy*cp*cr + sy*sp*sr,   # w
        cy*cp*sr - sy*sp*cr,   # x
        sy*cp*sr + cy*sp*cr,   # y
        sy*cp*cr - cy*sp*sr,   # z
    ))

def _euler_from_q_rpy(q):
    # quaternion → roll(X), pitch(Y), yaw(Z) degrees
    w, x, y, z = q
    # roll (x-axis)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # pitch (y-axis)
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi/2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    # yaw (z-axis)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return (roll, pitch, yaw)

class IMU:
    """
    BNO055 wrapper that:
      - applies sensor offset tuples (mag/gyro/accel),
      - runs in NDOF (magnetic north) by default,
      - maintains a quaternion offset (q_offset),
      - reports corrected (R,P,Y) derived from corrected quaternion.
    """
    MODE_CONFIG       = 0x00
    MODE_IMUPLUS      = 0x08  # gyro+accel fusion (relative yaw; use if mag uncalibrated)
    MODE_NDOF         = 0x0C  # full fusion (absolute yaw to mag north)

    def __init__(self, i2c, calibration_offsets=None, q_offset=None, use_ndof=True):
        self.bno = BNO055_I2C(i2c)
        time.sleep(0.05)  # let I2C stabilize

        # Choose fusion mode: NDOF (abs yaw) or IMUPLUS (relative yaw)
        try:
            self.bno.mode = self.MODE_NDOF if use_ndof else self.MODE_IMUPLUS
        except Exception:
            pass

        # Offsets (tuples): (x,y,z) for mag/gyro/accel as per Adafruit driver
        self.calibration_offsets = calibration_offsets or {
            "magnetometer":   (-682, -420, -354),
            "gyroscope":      (0, -3, 0),
            "accelerometer":  (-25, 83, -35),
        }

        # Quaternion offset (w,x,y,z): identity by default
        self._q_off = _q_norm(tuple(q_offset)) if q_offset else (1.0, 0.0, 0.0, 0.0)

        # Apply sensor offsets to BNO
        self.apply_calibration_offsets()
        time.sleep(0.05)

    # ---- Sensor offsets (into the BNO) ----
    def apply_calibration_offsets(self):
        try:
            self.bno.offsets_magnetometer  = self.calibration_offsets["magnetometer"]
            self.bno.offsets_gyroscope     = self.calibration_offsets["gyroscope"]
            self.bno.offsets_accelerometer = self.calibration_offsets["accelerometer"]
            print("✅ IMU: calibration tuples applied to BNO")
        except Exception as e:
            print("⚠️ IMU: failed to apply calibration tuples:", e)

    def get_calibration_status(self):
        """Return (sys, gyro, accel, mag). In NDOF, yaw is sane only when mag==3."""
        try:
            return self.bno.calibration_status
        except Exception:
            return (0, 0, 0, 0)

    # ---- Quaternion offset (external pose alignment) ----
    def set_quaternion_offset(self, q_tuple):
        """Set/replace the pose offset quaternion (w,x,y,z)."""
        self._q_off = _q_norm(tuple(q_tuple))

    def get_quaternion_offset(self):
        return self._q_off

    def compute_offset_for_target_euler(self, r_deg, p_deg, y_deg):
        """
        Given the sensor's current quaternion q_now and a target Euler (r,p,y),
        compute q_offset s.t. corrected = conj(q_offset) * q_now ≈ q_target.
        => q_offset = q_now * conj(q_target)
        """
        q_now = self.get_quaternion()
        q_tgt = _q_from_euler_rpy(r_deg, p_deg, y_deg)
        return _q_norm(_q_mul(q_now, _q_conj(q_tgt)))

    # ---- Raw quaternion from device ----
    def get_quaternion(self):
        """Return normalized (w,x,y,z) from BNO; never returns identity unless sensor does."""
        try:
            q = self.bno.quaternion  # (w,x,y,z) or None
            if not q:
                return (1.0, 0.0, 0.0, 0.0)
            return _q_norm((float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        except Exception:
            return (1.0, 0.0, 0.0, 0.0)

    # ---- Corrected orientation (what the rest of your app uses) ----
    def get_corrected_orientation(self):
        """
        Returns (roll, pitch, yaw, tilt) where R,P,Y are computed from the
        corrected quaternion:
            q_corr = conj(q_offset) * q_now     (Convention A)
        R,P,Y are NOT taken from bno.euler (so your offset affects yaw too).
        """
        q_now  = self.get_quaternion()
        q_corr = _q_mul(_q_conj(self._q_off), q_now)
        r, p, y = _euler_from_q_rpy(_q_norm(q_corr))

        # simple tilt metric from roll/pitch
        tilt = math.sqrt(r*r + p*p)
        return (r, p, y, tilt)

    # ---- Linear acceleration (world-frame swap used by your code) ----
    def get_swapped_linear_acceleration(self):
        """
        Return gravity-free linear accel with your previous axis swap
        (sensor->world): (-Z, +Y, +X). Keeps behavior consistent with your app.
        """
        try:
            ax, ay, az = self.bno.linear_acceleration  # may be None
            if ax is None:
                return (0.0, 0.0, 0.0)
            # Your historical swap:
            return (-az, ay, ax)
        except Exception:
            return (0.0, 0.0, 0.0)
