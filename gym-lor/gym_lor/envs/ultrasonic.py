import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
count = time.time()
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('ultrasonic', String, queue_size=1)
rospy.init_node('ultrasonic')
r = rospy.Rate(30) # 10hz
while not rospy.is_shutdown():
    GPIO.setup(31, GPIO.OUT)
    GPIO.output(31,False)
    time.sleep(0.0001)
    GPIO.output(31,True)
    time.sleep(0.0001)
    GPIO.output(31,False)
    cnt = 0
    GPIO.setup(31, GPIO.IN,pull_up_down=GPIO.PUD_UP)
    time0 = time.time()
    count = time.time()
    while GPIO.input(31)==0 and time.time()-count<0.1:
        time0 = time.time()
    count = time.time()
    while GPIO.input(31)==1 and time.time()-count<0.1:
        cnt=cnt+1
    time1 = time.time()
    print('Distance: %f cm'%cnt)
    pub.publish(str(cnt))
    r.sleep()