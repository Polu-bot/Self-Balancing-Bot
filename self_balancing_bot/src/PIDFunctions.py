import math

class PID_Controller(object):
    def __init__(self, Kp, Ki, Kd):
        #Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        #State Variables
        self.prevError = 0
        self.Stdt = 0
        self.t = 0

    def tune(self, newKp, newKi, newKd):
        self.Kp = newKp
        self.Ki = newKi
        self.Kd = newKd

    def getCorrection(self, target, current, dt = 1):
        error = target - current

        if self.t > 0:
            #derror/dt
            derrordt = (error - self.prevError)/dt
            self.Stdt += error*dt
        else:
            derrordt = 0
            self.Stdt = 0

        correction = self.Kp*error + self.Ki*self.Stdt + self.Kd*derrordt

        self.t += 1
        self.prevError = error

        return correction
        