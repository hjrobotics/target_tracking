import math

class quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    
    def to_euler_angle(self):
        w = self.w
        x = self.x
        y = self.y
        z = self.z
        
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        
        return X, Y, Z
    
    def to_axis_angle(self):
        w = self.w
        angle = 2*math.acos(w)
        ax = angle * self.x / math.sqrt(1-w*w)
        ay = angle * self.y / math.sqrt(1-w*w)
        az = angle * self.z / math.sqrt(1-w*w)
        return [ax, ay, az]

    @staticmethod
    def axis_angle_to_quaternion(a = None):
        ax, ay, az = a[0], a[1], a[2]
        angle = math.sqrt(ax*ax + ay*ay + az*az)
        return quaternion(math.cos(angle/2), 
                          ax*math.sin(angle/2)/angle, 
                          ay*math.sin(angle/2)/angle, 
                          az*math.sin(angle/2)/angle)
    
    def conjugate(self):
        return quaternion(self.w, -self.x, -self.y, -self.z)
    
    def multiply(self, q0):
        # self*q0
        w0, x0, y0, z0 = q0.w, q0.x, q0.y, q0.z
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        return quaternion(-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                          x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                          -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                          x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0)

    def rotate_vector(self, v):
        # self*vector*conjugate(self)
        assert len(v) == 3
        half = self.multiply(quaternion(0, v[0], v[1], v[2]))
        full = half.multiply(self.conjugate())
        return full.x, full.y, full.z
    
    def delta(self, q0):
        # self*conjugate(q0)
        return self.multiply(q0.conjugate())

