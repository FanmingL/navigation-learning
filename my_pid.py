import numpy
import time


class MyPid:
    """
    p: argument P
    i: argument I
    d: argument D
    integrate_limit: integrate limit value
    out_limit: output limit
    low_pass_hz: band width of low path filter (Hz) (-1 for disable low pass filter for output)
    """
    def __init__(self, p, i, d, integrate_limit, out_limit, low_pass_hz):
        self.t0 = time.time()
        self.t1 = self.t0
        self.t2 = self.t0
        self.argP = p
        self.argI = i
        self.argD = d
        self.ILimit = integrate_limit
        self.OLimit = out_limit
        self.LPBW = low_pass_hz
        self.enable_lpf = True
        self.error_old = 0
        self.error_i = 0
        self.out = 0
        self.out_d = 0
        if self.LPBW == -1:
            self.enable_lpf = False

    def step(self, feedback, expect=0, period=-1):
        if period == -1:
            self.t2 = self.t1
            self.t1 = time.time()
            period = self.t1 - self.t2
        error = expect - feedback
        error_d = (error - self.error_old) * self.safe_div(period)
        differential = self.argD * error_d
        if self.enable_lpf:
            self.out_d = self.low_pass_filter1(period, differential, self.out_d)
        else:
            self.out_d = differential
        self.error_i = self.error_i + error
        self.error_i = self.limit(self.error_i, -self.ILimit, self.ILimit)
        self.out = self.argP * error + self.out_d + self.argI * self.error_i
        self.error_old = error
        self.out = self.limit(self.out, -self.OLimit, self.OLimit)
        return self.out

    def safe_div(self, _val):
        if _val == 0:
            return 0
        return 1.0/_val

    def limit(self, _in,  _min, _max):
        if _min > _in:
            return _min
        elif _max < _in:
            return _max
        return _in

    def low_pass_filter1(self, t, _in, _out):
        out = _out + 1 / (1 + 1 / (self.LPBW * 6.28 * t)) * (_in - _out)
        return out


if __name__ == '__main__':
    pid = MyPid(p=10, i=1, d=1, integrate_limit=100, out_limit=1000, low_pass_hz=-1)
    for i in range(100):
        print pid.step(feedback=10, expect=0)

