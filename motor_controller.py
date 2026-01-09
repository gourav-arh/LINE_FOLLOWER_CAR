# motor_controller.py - OPTIMIZED FOR SPEED
import RPi.GPIO as GPIO
import time
import numpy as np

class HighPerformanceMotorController:
    def __init__(self):
        # Motor pins (BCM numbering)
        self.LEFT_PWM = 13
        self.LEFT_IN1 = 19
        self.LEFT_IN2 = 26
        self.RIGHT_PWM = 12
        self.RIGHT_IN1 = 20
        self.RIGHT_IN2 = 21
        
        # PWM Frequency - CRITICAL for smoothness
        self.PWM_FREQ = 20000  # 20kHz for silent operation
        
        # PID Parameters (TUNE THESE!)
        self.kp = 0.8    # Proportional
        self.ki = 0.01   # Integral  
        self.kd = 0.25   # Derivative
        
        # Feed-forward compensation
        self.ff_gain = 0.3
        
        # Anti-windup
        self.integral_max = 100
        
        self.setup()
    
    def setup(self):
        GPIO.setmode(GPIO.BCM)
        # Setup all pins
        for pin in [self.LEFT_IN1, self.LEFT_IN2, 
                   self.RIGHT_IN1, self.RIGHT_IN2]:
            GPIO.setup(pin, GPIO.OUT)
        
        # PWM Setup
        GPIO.setup(self.LEFT_PWM, GPIO.OUT)
        GPIO.setup(self.RIGHT_PWM, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(self.LEFT_PWM, self.PWM_FREQ)
        self.right_pwm = GPIO.PWM(self.RIGHT_PWM, self.PWM_FREQ)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def apply_pid_ff(self, error, dt):
        """PID with Feed-Forward for aggressive tracking"""
        # Proportional
        p = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        i = self.ki * self.integral
        
        # Derivative
        d_error = (error - self.prev_error) / dt if dt > 0 else 0
        d = self.kd * d_error
        self.prev_error = error
        
        # Feed-forward based on curve prediction
        ff = self.ff_gain * np.tanh(error * 2)
        
        return p + i + d + ff
    
    def sharp_turn_90(self, direction='right'):
        """Optimized 90° turn sequence"""
        base_speed = 40
        turn_speed = 60
        
        if direction == 'right':
            # Pivot turn
            self.set_motors(-turn_speed, turn_speed)
            time.sleep(0.35)  # TUNE THIS!
            # Correction
            self.set_motors(30, 30)
            time.sleep(0.1)
        else:
            self.set_motors(turn_speed, -turn_speed)
            time.sleep(0.35)
            self.set_motors(30, 30)
            time.sleep(0.1)
    
    def set_motors(self, left_speed, right_speed):
        # Clip speeds
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # Set direction
        # Left motor
        if left_speed >= 0:
            GPIO.output(self.LEFT_IN1, GPIO.HIGH)
            GPIO.output(self.LEFT_IN2, GPIO.LOW)
        else:
            GPIO.output(self.LEFT_IN1, GPIO.LOW)
            GPIO.output(self.LEFT_IN2, GPIO.HIGH)
        
        # Right motor
        if right_speed >= 0:
            GPIO.output(self.RIGHT_IN1, GPIO.HIGH)
            GPIO.output(self.RIGHT_IN2, GPIO.LOW)
        else:
            GPIO.output(self.RIGHT_IN1, GPIO.LOW)
            GPIO.output(self.RIGHT_IN2, GPIO.HIGH)
        
        # Set PWM
        self.left_pwm.ChangeDutyCycle(abs(left_speed))
        self.right_pwm.ChangeDutyCycle(abs(right_speed))
