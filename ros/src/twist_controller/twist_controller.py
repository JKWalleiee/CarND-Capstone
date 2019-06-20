import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
        wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, min_speed):
        self.yaw_controller = YawController(
            wheel_base=wheel_base,
            steer_ratio=steer_ratio,
            min_speed=min_speed,
            max_lat_accel=max_lat_accel,
            max_steer_angle=max_steer_angle)

        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        #self.fuel_capacity = fuel_capacity
        self.wheel_radius = wheel_radius

        self.pid = PID(kp=0.8, ki=0.1, kd=0.0, mn=decel_limit, mx=0.2)
        #self.pid = PID(kp=5, ki=0.5, kd=0.5, mn=decel_limit, mx=accel_limit)
        #self.pid = PID(0.15, 0.0, 0.09, mn=decel_limit, mx=accel_limit)
        self.s_lpf = LowPassFilter(tau = 3, ts = 1)
        self.t_lpf = LowPassFilter(tau = 3, ts = 1)
        self.vel_lpf = LowPassFilter(tau = 0.5, ts = 0.2)

    def reset(self):
        self.pid.reset()
        self.s_lpf.reset()
        self.t_lpf.reset()
        self.vel_lpf.reset()

    def control(self, current_velocity, dbw_enabled, linear_vel, ang_vel, del_time):
        
        if not dbw_enabled:
            self.reset()
            return 0.,0.,0.
        
        lin_vel = abs(linear_vel)
        vel_err = lin_vel - current_velocity

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_velocity)
        next_steer = self.s_lpf.filt(next_steer)

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_lpf.filt(acceleration)

        if acceleration > 0.009:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration

            if deceleration < self.brake_deadband:
                deceleration = 0.0

            brake = deceleration * (self.vehicle_mass* GAS_DENSITY) * self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, next_steer