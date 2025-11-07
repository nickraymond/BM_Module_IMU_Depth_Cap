# /lib/orientation_align.py — Quaternion target/zero alignment for CircuitPython
import math

def q_conj(q):
    w,x,y,z = q
    return (w, -x, -y, -z)

def q_mul(a, b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def euler_to_q(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg) / 2.0
    p = math.radians(pitch_deg) / 2.0
    y = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        sy*cp*sr + cy*sp*cr,
        sy*cp*cr - cy*sp*sr
    )

class OrientationAligner:
    def __init__(self, *, read_json, write_json_atomic, cal_json_path, ack_fn):
        self.read_json = read_json
        self.write_json_atomic = write_json_atomic
        self.cal_json_path = cal_json_path
        self.ack = ack_fn
        self.q_offset = (1.0, 0.0, 0.0, 0.0)

    def load_from_file(self, cal_defaults: dict):
        cal = self.read_json(self.cal_json_path, cal_defaults)
        q = (((cal.get("orientation") or {}).get("q_offset")) or None)
        if q and len(q) == 4:
            self.q_offset = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        return self.q_offset

    def save_to_file(self, cal_defaults: dict):
        cal = self.read_json(self.cal_json_path, cal_defaults)
        cal.setdefault("orientation", {})["q_offset"] = [float(v) for v in self.q_offset]
        ok = self.write_json_atomic(self.cal_json_path, cal)
        if ok:
            self.ack("ORIENT: q_offset saved")
        else:
            self.ack("ORIENT ERR: save failed")
        return ok

    def zero_now(self, q_now):
        # Make the current pose the reference
        self.q_offset = q_now
        self.ack("ORIENT: zeroed")

    def set_target(self, roll_deg, pitch_deg, yaw_deg, q_now):
        # We want: q_corrected = q_now * conj(q_offset) ≈ q_target
        # ⇒ q_offset ≈ q_now * conj(q_target)
        q_target = euler_to_q(roll_deg, pitch_deg, yaw_deg)
        from_quat = q_now
        self.q_offset = q_mul(from_quat, q_conj(q_target))
        self.ack("ORIENT: target set [{},{},{}]".format(roll_deg, pitch_deg, yaw_deg))

    def apply(self, q_raw):
        return q_mul(q_raw, q_conj(self.q_offset))
