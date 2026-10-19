"""Experimental planar wheel-increment integration with an IMU attitude source.

No ROS, world-pose observations or accumulated wheel XY are output positions.
Quaternions use xyzw order; IMU orientation is reference-to-sensor attitude.
"""
from bisect import bisect_left
from collections import deque
from math import atan2, cos, hypot, isfinite, sin


def wrap(a):
    return atan2(sin(a), cos(a))


def quaternion(q):
    if len(q) != 4 or not all(isfinite(v) for v in q):
        raise ValueError('non-finite quaternion')
    n = hypot(*q)
    if abs(n - 1) > .001:
        raise ValueError('quaternion must have unit norm')
    return tuple(v / n for v in q)


def multiply(a, b):
    x,y,z,w=a; X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def heading(q, mount, reference_yaw):
    q = quaternion(q); m = quaternion(mount)
    x,y,z,w = multiply(q, (-m[0],-m[1],-m[2],m[3]))
    return wrap(atan2(2*(w*z+x*y), 1-2*(y*y+z*z)) - reference_yaw)


def timestamp(t):
    if type(t) is not int or t < 0:
        raise ValueError('timestamp must be nonnegative integer nanoseconds')


class HeadingOdometry:
    """Bounded interpolation; one output per advancing wheel source stamp."""
    def __init__(self, max_imu_gap_ns=100000000, max_wheel_gap_ns=100000000):
        self.imu = deque(maxlen=400)
        self.previous = None
        self.xy = (0., 0.)
        self.max_imu_gap = max_imu_gap_ns
        self.max_wheel_gap = max_wheel_gap_ns

    def add_imu(self, t, yaw):
        timestamp(t)
        if not isfinite(yaw):
            raise ValueError('non-finite heading')
        if self.imu and t <= self.imu[-1][0]:
            raise ValueError('IMU timestamps must advance')
        self.imu.append((t, wrap(yaw)))

    def at(self, t):
        timestamp(t)
        rows=list(self.imu);i=bisect_left([r[0] for r in rows], t)
        if i < len(rows) and rows[i][0] == t:
            return rows[i][1]
        if not i or i == len(rows):
            raise LookupError('IMU does not bracket wheel stamp')
        a,b=rows[i-1],rows[i]
        if b[0]-a[0] > self.max_imu_gap:
            raise ValueError('IMU interpolation gap exceeds limit')
        return wrap(a[1]+(t-a[0])/(b[0]-a[0])*wrap(b[1]-a[1]))

    def update(self, t, x, y, yaw):
        timestamp(t)
        if not all(isfinite(v) for v in (x,y,yaw)):
            raise ValueError('non-finite wheel pose')
        h1=self.at(t)
        if self.previous is None:
            if hypot(x,y) > .05 or abs(wrap(yaw)) > .05:
                raise ValueError('estimator requires a fresh wheel-odom origin')
            self.previous=(t,x,y,yaw)
            return (t,*self.xy,h1)
        old=self.previous
        if not 0 < t-old[0] <= self.max_wheel_gap:
            raise ValueError('wheel timestamps repeated/backwards or gap too large')
        h0=self.at(old[0])
        rows=list(self.imu)
        if any(b[0]-a[0] > self.max_imu_gap for a,b in zip(rows,rows[1:])
               if b[0] > old[0] and a[0] < t):
            raise ValueError('IMU gap inside wheel interval exceeds limit')
        dx,dy=x-old[1],y-old[2]
        mid_raw=old[3]+wrap(yaw-old[3])/2
        forward=dx*cos(mid_raw)+dy*sin(mid_raw)
        length=hypot(dx,dy)
        ds=length if forward >= 0 else -length
        mid_imu=h0+wrap(h1-h0)/2
        result=(self.xy[0]+ds*cos(mid_imu),self.xy[1]+ds*sin(mid_imu))
        if not all(isfinite(v) for v in result):
            raise ValueError('estimated position overflow')
        self.xy=result;self.previous=(t,x,y,yaw)
        return (t,*result,h1)
