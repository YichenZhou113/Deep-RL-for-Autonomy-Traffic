import RPi.GPIO as GPIO
import time
from jetbot import Robot
import rospy
from std_msgs.msg import String
#rospy.init_node('main_control')

class Jetbot:
    def __init__(self):
        self.robot = Robot()
        self.time_since_start = 0.0
        self.time_before_activation = 30
        self.low_speed_factor = 0.8
        self.time_before_speedup = 90
        self.speedup_factor = 1.0
        self.speed_left = 0.0
        self.speed_right = 0.0
        self.dir_ = 1
        self.red_right = 0.0
        self.red_left = 0.0
        self.stopped = True
        self.headway_min = 0.0
        self.headway_max = 25.0
        self.headway_break = 1.0
        self.start_speed_mult = 1.0
        self.start_accel = 0.7
        self.start_reaction_time = 800
        self.speed_mult_rl = 1.0
        self.ratio = 1.0
        self.time_ = time.time()
        self.ultra_data = 0.0
        self.GPIO_TRIGGER = 31
        self.GPIO_ECHO = 33

    def callback(self,data):
        self.ultra_data = float(data.data)


    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(15, GPIO.IN)
        GPIO.setup(16, GPIO.IN)

    def is_left_black(self):
        if GPIO.input(15) == GPIO.LOW:
            return True
        else:
            return False

    def is_right_black(self):
        if GPIO.input(16) == GPIO.LOW:
            return True
        else:
            return False

    def is_left_white(self):
        return not self.is_left_black()

    def is_right_white():
        return not self.is_right_black()

    def set_speed(self,left_speed,right_speed):
        left_speed /= 130.0
        right_speed /= 130.0
        self.robot.left_motor.value = left_speed
        self.robot.right_motor.value = right_speed

    def move(self,direction,speed):
        if direction == 1:
            self.set_speed(speed,speed)
            return True
        if direction == 2:
            self.set_speed(-speed,-speed)
            return True
        if direction == 3:
            self.set_speed(-speed,speed)
            return True
        if direction == 4:
            self.set_speed(speed,-speed)
            return True
        return False

    def ultra(self):
        GPIO.setup(31, GPIO.OUT)
        GPIO.output(31, False)
        time.sleep(0.0001)
        GPIO.output(31, True)
        time.sleep(0.0001)
        GPIO.output(31, False)
        cnt = 0
        GPIO.setup(31, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        time0 = time.time()
        count = time.time()
        while GPIO.input(31) == 0 and time.time() - count < 0.1:
            time0 = time.time()
        #print("anticipate:",(time0-count)*340.0)
        count = time.time()
        while GPIO.input(31) == 1 and time.time() - count < 0.1:
            cnt = cnt + 1
        return cnt
    
    def sr04(self):

        GPIO.setup(self.GPIO_TRIGGER,GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO,GPIO.IN)
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.0001)
        GPIO.output(self.GPIO_TRIGGER,False)
        count = time.time()
        StartTime = time.time()
        StopTime = time.time()
        while GPIO.input(self.GPIO_ECHO) == 0 and time.time() - count < 0.01:
            StartTime = time.time()
        while GPIO.input(self.GPIO_ECHO) == 1 and time.time() - count < 0.01:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300)/2.0
        return max(distance,0.0)
    
    def run(self):
        dt = time.time() - self.time_
        self.time_ = time.time()
        self.time_since_start += dt
        if(self.time_since_start <= self.time_before_activation):
            self.speed_mult_rl = 1.0
        elif(self.time_since_start < self.time_before_speedup):
            self.speed_mult_rl = self.low_speed_factor
        else:
            self.speed_mult_rl = self.speedup_factor
   
        #rospy.Subscriber("ultrasonic", String, self.callback)
        headway = self.sr04()
        speed_mult = 1.0
        if (headway <= self.headway_max):
            if (headway >= self.headway_min):
                speed_mult = (headway - self.headway_min) / (self.headway_max - self.headway_min)
            if (headway <= self.headway_break):
                self.stopped = True
                self.speed_left = 0.0
                self.speed_right = 0.0
        if headway > self.headway_break:
            if self.stopped:
                time.sleep(self.start_reaction_time/1000.0)
                self.stopped = False
                self.start_speed_mult = 0.0
            if self.start_speed_mult < 1.0:
                self.start_speed_mult += self.start_accel * dt
            if self.start_speed_mult > 1.0:
                self.start_speed_mult = 1.0
            left_ok = self.is_left_black()
            right_ok = self.is_right_black()
            if(left_ok and right_ok):
                self.dir_ = 0
            elif(left_ok and (not right_ok)):
                if self.dir_ != 1:
                    self.red_right += 1
                self.dir_ = 1
            elif((not left_ok) and right_ok):
                if self.dir_ != 2:
                    self.red_left += 1
                self.dir_ = 2
            else:
                self.dir_ = 3
            if self.dir_ == 0:
                self.speed_left = 85.0 - self.red_right + self.red_left
                self.speed_right = 100.0
            if self.dir_ == 1:
                self.speed_left = 50.0
                self.speed_right = 100.0
            if self.dir_ == 2:
                self.speed_left = 100.0
                self.speed_right = 50.0
            if self.dir_ == 3:
                self.speed_left = -50.0
                self.speed_right = -50.0
            self.speed_left = min(100.0,max(-100.0,self.speed_left))
            self.speed_right = min(100.0,max(-100.0,self.speed_right))
            self.speed_left *= speed_mult * self.start_speed_mult * self.speed_mult_rl
            self.speed_right *= speed_mult * self.start_speed_mult * self.speed_mult_rl
        self.set_speed(self.speed_left,self.speed_right)
        #print("direction:",dir_,"Setspeed:",speed_left,speed_right)
    
    def loop(self):
        self.setup()
        print('Setup Success')
        while True:
            self.run()


jbot = Jetbot()
jbot.loop()