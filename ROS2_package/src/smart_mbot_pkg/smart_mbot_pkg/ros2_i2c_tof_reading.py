# ROS2 Libs.
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Int32MultiArray
from sensor_msgs.msg import Range
# I2C_VL53L0X_ToF Libs.
import time
import sys
sys.path.append("/home/ubuntu/smart_mbot_ws/src/smart_mbot_pkg/smart_mbot_pkg/lib/VL53L0X_rasp_python/python") # for VL53L0X lib

import VL53L0X
import RPi.GPIO as GPIO

arr_tof_data = [0]*8

class I2C_VL53L0X_ToF(Node):
    def __init__(self):
        super().__init__('reading_i2c_VL53L0X_tof')

        self.pub_i2c_tof_reader = self.create_publisher(Int32MultiArray, 'reading_tof_array', 10)
        self.tof_sensor_setting()
        # Setting timer for pulbisher
        pub_i2c_tof_reader_timer_period = 0.001  # seconds
        self.pub_i2c_tof_reader_timer = self.create_timer(pub_i2c_tof_reader_timer_period, self.pub_i2c_VL53L0X_tof_callback)

        #self.pub_range_i2c_tof_reader = self.create_publisher(Range, 'reading_tof_range', 10)
        #pub_range_i2c_tof_reader_timer_period = 0.001
        #self.pub_range_i2c_tof_reader_timer = self.create_timer(pub_range_i2c_tof_reader_timer_period, self.pub_range_i2c_tof_reader_callback)

    def tof_sensor_setting(self):
        # GPIO for Sensor 1 shutdown pin
        sensor1_shutdown = 17
        sensor2_shutdown = 15
        sensor3_shutdown = 14
        sensor4_shutdown = 18
        sensor5_shutdown = 23
        sensor6_shutdown = 24
        sensor7_shutdown = 22
        sensor8_shutdown = 27
        GPIO.setwarnings(False)

        # Setup GPIO for shutdown pins on each VL53L0X
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(sensor1_shutdown, GPIO.OUT)
        GPIO.setup(sensor2_shutdown, GPIO.OUT)
        GPIO.setup(sensor3_shutdown, GPIO.OUT)
        GPIO.setup(sensor4_shutdown, GPIO.OUT)
        GPIO.setup(sensor5_shutdown, GPIO.OUT)
        GPIO.setup(sensor6_shutdown, GPIO.OUT)
        GPIO.setup(sensor7_shutdown, GPIO.OUT)
        GPIO.setup(sensor8_shutdown, GPIO.OUT)

        # Set all shutdown pins low to turn off each VL53L0X
        GPIO.output(sensor1_shutdown, GPIO.LOW)
        GPIO.output(sensor2_shutdown, GPIO.LOW)
        GPIO.output(sensor3_shutdown, GPIO.LOW)
        GPIO.output(sensor4_shutdown, GPIO.LOW)
        GPIO.output(sensor5_shutdown, GPIO.LOW)
        GPIO.output(sensor6_shutdown, GPIO.LOW)
        GPIO.output(sensor7_shutdown, GPIO.LOW)
        GPIO.output(sensor8_shutdown, GPIO.LOW)

        # Keep all low for 500 ms or so to make sure they reset
        time.sleep(0.50)

        # Create one object per VL53L0X passing the address to give to
        self.tof = VL53L0X.VL53L0X(address=0x2B)
        self.tof1 = VL53L0X.VL53L0X(address=0x2D)
        self.tof2 = VL53L0X.VL53L0X(address=0x30)
        self.tof3 = VL53L0X.VL53L0X(address=0x31)
        self.tof4 = VL53L0X.VL53L0X(address=0x32)
        self.tof5 = VL53L0X.VL53L0X(address=0x33)
        self.tof6 = VL53L0X.VL53L0X(address=0x34)
        self.tof7 = VL53L0X.VL53L0X(address=0x35)

        # Set shutdown pin high for the first VL53L0X then
        # call to start ranging
        tof_mode = VL53L0X.VL53L0X_BETTER_ACCURACY_MODE

        GPIO.output(sensor1_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof.start_ranging(tof_mode)

        GPIO.output(sensor2_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof1.start_ranging(tof_mode)

        GPIO.output(sensor3_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof2.start_ranging(tof_mode)

        GPIO.output(sensor4_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof3.start_ranging(tof_mode)

        GPIO.output(sensor5_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof4.start_ranging(tof_mode)

        GPIO.output(sensor6_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof5.start_ranging(tof_mode)

        GPIO.output(sensor7_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof6.start_ranging(tof_mode)

        GPIO.output(sensor8_shutdown, GPIO.HIGH)
        time.sleep(0.50)
        self.tof7.start_ranging(tof_mode)

    def tof_sensor_stop(self):
        self.tof.stop_ranging()
        GPIO.output(sensor1_shutdown, GPIO.LOW)
        
        self.tof1.stop_ranging()
        GPIO.output(sensor2_shutdown, GPIO.LOW)
        
        self.tof2.stop_ranging()
        GPIO.output(sensor3_shutdown, GPIO.LOW)
        
        self.tof3.stop_ranging()
        GPIO.output(sensor4_shutdown, GPIO.LOW)
        
        self.tof4.stop_ranging()
        GPIO.output(sensor5_shutdown, GPIO.LOW)
        
        self.tof5.stop_ranging()
        GPIO.output(sensor6_shutdown, GPIO.LOW)
        
        self.tof6.stop_ranging()
        GPIO.output(sensor7_shutdown, GPIO.LOW)
        
        self.tof7.stop_ranging()
        GPIO.output(sensor8_shutdown, GPIO.LOW)


    def tof_sensor_reading(self):
        wonsu = [0]*8
        distance = self.tof.get_distance()
        if (distance > 0):
            wonsu[self.tof.my_object_number] = distance
        else:
            wonsu[self.tof.my_object_number] = 0

        distance = self.tof1.get_distance()
        if (distance > 0):
            wonsu[self.tof1.my_object_number] = distance
        else:
            wonsu[self.tof1.my_object_number] = 0

        distance = self.tof2.get_distance()
        if (distance > 0):
            wonsu[self.tof2.my_object_number] = distance
        else:
            wonsu[self.tof2.my_object_number] = 0

        distance = self.tof3.get_distance()
        if (distance > 0):
            wonsu[self.tof3.my_object_number] = distance
        else:
            wonsu[self.tof3.my_object_number] = 0

        distance = self.tof4.get_distance()
        if (distance > 0):
            wonsu[self.tof4.my_object_number] = distance
        else:
            wonsu[self.tof4.my_object_number] = 0

        distance = self.tof5.get_distance()
        if (distance > 0):
            wonsu[self.tof5.my_object_number] = distance
        else:
            wonsu[self.tof5.my_object_number] = 0

        distance = self.tof6.get_distance()
        if (distance > 0):
            wonsu[self.tof6.my_object_number] = distance
        else:
            wonsu[self.tof6.my_object_number] = 0

        distance = self.tof7.get_distance()
        if (distance > 0):
            wonsu[self.tof7.my_object_number] = distance
        else:
            wonsu[self.tof7.my_object_number] = 0
        
        return wonsu

    def pub_i2c_VL53L0X_tof_callback(self):
        '''
        [    0 degree ToF,  45 degree ToF,  90 degree ToF, 135 degree ToF,  
           180 degree ToF, 225 degree ToF, 270 degree ToF, 315 degree ToF  ]
        '''
        msg = Int32MultiArray()
        arr_tof_data = self.tof_sensor_reading()

        msg.data = arr_tof_data
        self.pub_i2c_tof_reader.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pub_i2c_VL53L0X_tof = I2C_VL53L0X_ToF()

    try:
        rclpy.spin(pub_i2c_VL53L0X_tof)

    except KeyboardInterrupt:
        print('XXXXXXX: repeater stopped cleanly')
        pub_i2c_VL53L0X_tof.tof_sensor_stop()

    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        pub_i2c_VL53L0X_tof.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


