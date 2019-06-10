import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
        wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # YawController -> steering Output
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, 
                        max_steer_angle)
        
        # throttle_controller -> throttle -> Output
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. #Minimun Throtthle
        mx = 0. #Mximun Throtthle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2*pi*tau) = cutoff frequency
        ts = 0.2
        self.vel_lpf = LowPassFilter(tau, ts)

        # throttle pass filter
        self.low_pass_filter = LowPassFilter(12.0, 1)

        self.vehicle_mass=vehicle_mass
        self.fuel_capacity=fuel_capacity
        self.brake_deadband=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius

        #self.last_time = rospy.get_time()




    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, sample_time):
        
        # Check manual/automatic control with dbw_enabled
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.low_pass_filter.reset()
            self.vel_lpf.reset()
            return 0.,0.,0.
        
        #Filter the Input Velocity
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        brake = 0.
        throttle = 0.


        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        throttle = self.throttle_controller.step(vel_error, sample_time)

        #filtvalue = self.low_pass_filter.filt(throttle)
        #if self.low_pass_filter.ready:
        #    throttle = self.low_pass_filter.get()

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        
        return throttle, brake, steering
