import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32MultiArray


# RGB strip Libs.
import neopixel
import board
import time

# Setup for RGB strip
pixel_pin = board.D12
num_pixels = 27
ORDER = neopixel.GRB
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False, pixel_order=ORDER)

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    if pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    pos -= 170
    return (pos * 3, 0, 255 - pos * 3)

def color_chase(color, wait):
    for i in range(num_pixels):
        pixels[i] = color
        time.sleep(wait)
        pixels.show()


def rainbow_cycle(wait):
    for j in range(255):
        for i in range(num_pixels):
            rc_index = (i * 256 // num_pixels) + j
            pixels[i] = wheel(rc_index & 255)
        pixels.show()
        time.sleep(wait)

#### Need to add Service

class WS2813_RGB_LED(Node):
    def __init__(self):
        super().__init__('reading_ws2813_rgb_strip')

        pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False, pixel_order=ORDER)
        for j in range(0, num_pixels, 1):
            pixels[j] = (0, 255, 00)
        pixels.show()

        self.rec_vel = self.create_subscription(Float32MultiArray, 'writing_ws2813_rgb_strip', self.sub_rgb_control_callback, 10)

    def sub_rgb_control_callback(self,msg):
        #[r, g, b, led_gap[from 1 to num_pixels], intensity[from 0.0 to 1.0], mode, wait time for mode 1&2]
        #Example: [255.0, 0.0, 0.0, 1.0, 0.5, 0.0, 1.0]
        color = (int(msg.data[0]), int(msg.data[1]), int(msg.data[2]))
        led_gaps = int(msg.data[3]) #(from 1 to num_pixels)
        led_brightness = msg.data[4] #(from 0.0 to 1.0)
        led_mode = int(msg.data[5])
        led_wait = int(msg.data[6])

        if led_brightness > 1.0 or led_brightness < 0.0:
            print("please use a proper brightness from 0.0 to 1.0 (topic/data[3]]")
        if led_gaps < 1 or led_gaps > num_pixels:
            print("please use a ropoer led gaps from 0 to 27")

        else:
            pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=led_brightness, auto_write=False, pixel_order=ORDER)

            if led_mode == 0:
                for x in range(0, num_pixels, led_gaps):
                    pixels[x] = color
                pixels.show()

            elif led_mode == 1:
                #Chase
                color_chase(color, led_wait)

            elif led_mode == 2:
                #Rainbow
                rainbow_cycle(led_wait)

            elif led_mode == 3:
                # For  LED off
                for x in range(0, num_pixels):
                   pixels[x] = (0,0,0)
                pixels.show()

            else:
                #error or add more modes
                print("please select a proper mode from [0, 1, and 2];(topic/data[5]]")
                pass

def main(args=None):
    rclpy.init(args=args)
    sub_rgb_led = WS2813_RGB_LED()

    try:
        rclpy.spin(sub_rgb_led)

    except KeyboardInterrupt:
        pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False, pixel_order=ORDER)
        for i in range(0, num_pixels, 1):
            pixels[i] = (0, 0, 0)
        pixels.show()

    
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:
        pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.5, auto_write=False, pixel_order=ORDER)
        for i in range(0, num_pixels, 1):
            pixels[i] = (0, 0, 0)
        pixels.show()

        sub_rgb_led.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
