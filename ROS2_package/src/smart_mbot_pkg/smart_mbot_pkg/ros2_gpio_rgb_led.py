import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Int32MultiArray

# RGB CMD5050 GPIO Libs.
import RPi.GPIO as GPIO
import sys, time

P_RED = 25     # adapt to your wiring
P_GREEN = 6   # ditto

P_BLUE = 0    # ditto

fPWM = 50      # Hz (not higher with software PWM)


def led_gpio_setup():
    global pwmR, pwmG, pwmB
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)    

    GPIO.setup(P_RED, GPIO.OUT)
    GPIO.setup(P_GREEN, GPIO.OUT)
    GPIO.setup(P_BLUE, GPIO.OUT)

    GPIO.output(P_RED, GPIO.LOW)
    GPIO.output(P_GREEN, GPIO.LOW)
    GPIO.output(P_BLUE, GPIO.LOW)

    pwmR = GPIO.PWM(P_RED, fPWM)
    pwmG = GPIO.PWM(P_GREEN, fPWM)
    pwmB = GPIO.PWM(P_BLUE, fPWM)

    pwmR.start(0)
    pwmG.start(0)
    pwmB.start(0)
    time.sleep(0.2)
    
    pwmR.ChangeDutyCycle(0)
    pwmG.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    time.sleep(0.2)

def setColor(r, g, b):
    pwmR.ChangeDutyCycle(int(r / 255 * 100))
    pwmG.ChangeDutyCycle(int(g / 255 * 100))
    pwmB.ChangeDutyCycle(int(b / 255 * 100))


class GPIO_SMD5050_RGB_LED(Node):
    def __init__(self):
        super().__init__('writing_gpio_smd5050_led')
        led_gpio_setup()
        self.rec_vel = self.create_subscription(Int32MultiArray, 'writing_gpio_smd5050_led', self.sub_gpio_smd5050_control_callback, 10)

    def sub_gpio_smd5050_control_callback(self,msg):
        #[r,g,b,intensity[from 0.0 to 1.0], mode, wait time for mode 1&2]
        r = msg.data[0]
        g = msg.data[1]
        b = msg.data[2]
        if r < 0 or r > 255 or g < 0 or g > 255 or b < 0 or b > 255:
            print("Please use a proper data from 0 to 255")
            pass
        else:
            setColor(r, g, b)
        
        
def main(args=None):
    rclpy.init(args=args)

    sub_gpio_smd5050_led = GPIO_SMD5050_RGB_LED()

    rclpy.spin(sub_gpio_smd5050_led)

    try:
        rclpy.spin(sub_gpio_smd5050_led)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
        self.setColor(0,0,0)

    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        sub_gpio_smd5050_led.destroy_node()
        rclpy.shutdown() 




    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    sub_gpio_smd5050_led.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
