

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist, Vector3

import busio
import board
import digitalio

import adafruit_mcp3xxx.mcp3008 as MCP
import RPi.GPIO as GPIO
from adafruit_mcp3xxx.analog_in import AnalogIn

from time import sleep


# used pins; SCK, MISO, MOSI, GPIO_05, 
class SMARTMobile_ADC_SPI_Reader(Node):
    def __init__(self):
        super().__init__('reading_spi_adc')
        # Initialize the SPI module
        self.spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        self.cs = digitalio.DigitalInOut(board.D5)
 
        self.mcp = MCP.MCP3008(self.spi, self.cs)

        self.linesensor = 5
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.linesensor, GPIO.OUT)
        GPIO.output(self.linesensor, GPIO.HIGH)
        
        
        # Initialize and define Publishers
        self.pub_adc_spi_reader = self.create_publisher(Float32MultiArray, 'reading_spi_adc', 10)
        # Setting timer for pulbisher
        pub_adc_spi_reader_timer_period = 0.001  # seconds
        self.pub_adc_spi_reader_timer = self.create_timer(pub_adc_spi_reader_timer_period, 
            self.pub_adc_spi_reader_timer_callback)
        
    def pub_adc_spi_reader_timer_callback(self):               
        msg = Float32MultiArray()
        chan_1 = AnalogIn(self.mcp, MCP.P0)
        chan_2 = AnalogIn(self.mcp, MCP.P1)
        chan_3 = AnalogIn(self.mcp, MCP.P2)
        chan_4 = AnalogIn(self.mcp, MCP.P3)
        chan_5 = AnalogIn(self.mcp, MCP.P4)
        chan_6 = AnalogIn(self.mcp, MCP.P5)
        chan_7 = AnalogIn(self.mcp, MCP.P6)
        chan_8 = AnalogIn(self.mcp, MCP.P7)

        msg.data = [chan_1.voltage, chan_2.voltage, chan_3.voltage, chan_4.voltage, chan_5.voltage, chan_6.voltage, chan_7.voltage, chan_8.voltage] 
        #print(msg.data)
        self.pub_adc_spi_reader.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pub_adc_spi_reader = SMARTMobile_ADC_SPI_Reader()

    try:
        rclpy.spin(pub_adc_spi_reader)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
   
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        pub_adc_spi_reader.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()


