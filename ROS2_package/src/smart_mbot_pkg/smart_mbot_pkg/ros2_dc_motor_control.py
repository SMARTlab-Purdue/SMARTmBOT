# ROS2 lib.
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

# ROS2 Msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32MultiArray, Int32MultiArray

# Raspberry Pi lib.
import sys
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


robot_wheel_radius = 2 # unit: cm
robot_wheel_dist = 9.5 # unit: cm

class Motor:
    def __init__(self, pinForward, pinBackward, pinControl):
        """ Initialize the motor with its control pins and start pulse-width
             modulation """

        self.pinForward = pinForward
        self.pinBackward = pinBackward
        self.pinControl = pinControl

        GPIO.setup(self.pinForward, GPIO.OUT)
        GPIO.setup(self.pinBackward, GPIO.OUT)
        GPIO.setup(self.pinControl, GPIO.OUT)

        self.pwm_forward = GPIO.PWM(self.pinForward, 100)
        self.pwm_backward = GPIO.PWM(self.pinBackward, 100)

        self.pwm_forward.start(0)
        self.pwm_backward.start(0)

        GPIO.output(self.pinControl,GPIO.HIGH) 

    def forward(self, speed):
        """ pinForward is the forward Pin, so we change its duty
             cycle according to speed. """
        self.pwm_backward.ChangeDutyCycle(0)
        self.pwm_forward.ChangeDutyCycle(speed)

    def backward(self, speed):
        """ pinBackward is the forward Pin, so we change its duty
             cycle according to speed. """
        self.pwm_forward.ChangeDutyCycle(0)
        self.pwm_backward.ChangeDutyCycle(speed)

    def stop(self):
        """ Set the duty cycle of both control pins to zero to stop the motor. """
        self.pwm_forward.ChangeDutyCycle(0)
        self.pwm_backward.ChangeDutyCycle(0)

class MotorControl(Node):
    def __init__(self):
        super().__init__('writing_dc_motor_vel')

        self.declare_parameter('w_right_motor', 1.0)
        self.declare_parameter('w_left_motor', 1.0)

        self.right_vel_weight = self.get_parameter('w_right_motor').value
        self.left_vel_weight = self.get_parameter('w_left_motor').value

        if self.right_vel_weight < 0:
            self.right_vel_weight = 0
            print("please set DC motor parameter from 0 to 1")
        if self.right_vel_weight > 1:
            self.right_vel_weight = 1
            print("please set DC motor parameter from 0 to 1")

        if self.left_vel_weight < 0:
            self.left_vel_weight = 0
            print("please set DC motor parameter from 0 to 1")
        if self.left_vel_weight > 1:
            self.left_vel_weight = 1
            print("please set DC motor parameter from 0 to 1")

        self.motor1 = Motor(19, 13, 26)
        self.motor2 = Motor(21, 20, 16)

        self.sub_dc_wheel_vel = self.create_subscription(Float32MultiArray, 'writing_dc_motor_vel', self.sub_dc_wheel_vel_callback, 10)
        self.sub_dc_cmd_vel =  self.create_subscription(Twist, 'writing_dc_cmd_vel', self.sub_dc_cmd_vel_callback, 10)

    def sub_dc_cmd_vel_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        #print("Linear / Angular: ", lin_vel, ang_vel)
        
        
        left_velocity  = (lin_vel - ang_vel * robot_wheel_dist / 2) / robot_wheel_radius
        right_velocity = (lin_vel + ang_vel * robot_wheel_dist / 2) / robot_wheel_radius

        updated_right_vel = right_velocity * self.right_vel_weight
        updated_left_vel  = left_velocity  * self.left_vel_weight

        ## For test
        updated_right_vel = updated_right_vel * 100
        updated_left_vel = updated_left_vel * 100


        if updated_right_vel > 100:
            updated_right_vel = 100
        
        if updated_right_vel < -100:
            updated_right_vel = -100
        
        if updated_left_vel > 100:
            updated_left_vel = 100
        
        if updated_left_vel < -100:
            updated_left_vel = -100

        if updated_right_vel >= 0:
            self.motor1.forward(int(updated_right_vel))
        else:
            self.motor1.backward(int(-1*updated_right_vel))

        if updated_left_vel >= 0:
            self.motor2.forward(int(updated_left_vel))
        else:
            self.motor2.backward(int(-1*updated_left_vel))

        print("left / Right: ", updated_left_vel, updated_right_vel)
        

    def sub_dc_wheel_vel_callback(self, msg):
        # [left_vel, left_vel_weight , right_vel, right_vel_weight]
        #self.right_vel_weight = self.get_parameter('w_right_motor').value
        #self.left_vel_weight  = self.get_parameter('w_left_motor').value

        left_velocity = msg.data[0]
        right_velocity = msg.data[1]

        updated_right_vel = right_velocity * self.right_vel_weight
        updated_left_vel  = left_velocity  * self.left_vel_weight

        if updated_right_vel > 100:
            updated_right_vel = 100
        if updated_right_vel < -100:
            updated_right_vel = -100
        if updated_left_vel > 100:
            updated_left_vel = 100
        if updated_left_vel < -100:
            updated_left_vel = -100

        if updated_right_vel >= 0:
            self.motor1.forward(int(updated_right_vel))
        else:
            self.motor1.backward(int(-1*updated_right_vel))

        if updated_left_vel >= 0:
            self.motor2.forward(int(updated_left_vel))
        else:
            self.motor2.backward(int(-1*updated_left_vel))

def main(args=None):
    rclpy.init(args=args)
    sub_motor = MotorControl()

    try:
        rclpy.spin(sub_motor)

    except KeyboardInterrupt:
        print('repeater stopped cleanly')
        sub_motor.motor1.stop()
        sub_motor.motor2.stop()

    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:
        sub_motor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
