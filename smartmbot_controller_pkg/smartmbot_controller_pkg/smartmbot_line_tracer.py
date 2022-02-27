import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray

import sys, math
import numpy as np

class controller_irb_cctv_line_follower(Node):
    def __init__(self):
        super().__init__('robot_irb_leader_follower_node')
        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        self.declare_parameter('robot_name', 'smartmbot_4')
        self.declare_parameter('moving_speed', 30)
        self.declare_parameter('adc_threshold', 1.5)
        self.declare_parameter('adc_threshold_back', 1.5)

        self.param_robot_ID = self.get_parameter('robot_name').value
        self.param_robot_speed = self.get_parameter('moving_speed').value
        self.param_adc_threshold = self.get_parameter('adc_threshold').value
        self.param_adc_threshold_back = self.get_parameter('adc_threshold_back').value
        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        self.human_positions = np.zeros(6)
        self.all_smartmbot_positions = np.zeros((20, 6))
        self.smartmbot_spi_adc_data = np.zeros(8)
        self.smartmbot_tof_array_data = np.zeros(8)
        self.cctv_experiment_command = 0

        self.robot_direction = True # True = Forward, False = Backward

        self.robot_ID_array = ['smartmbot_1', 'smartmbot_2', 'smartmbot_3', 'smartmbot_4', 'smartmbot_5', 'smartmbot_6', 'smartmbot_7', 'smartmbot_8', 'smartmbot_9', 'smartmbot_10', 'smartmbot_11', 'smartmbot_12', 'smartmbot_13', 'smartmbot_14', 'smartmbot_15', 'smartmbot_16', 'smartmbot_17', 'smartmbot_18', 'smartmbot_19', 'smartmbot_20']
        
        #====================================================#
        ####  Subscription Section                        ####
        #====================================================#
        # Reading Human Pose
        #self.sub_human_pose = self.create_subscription(Float32MultiArray, '/smartmbot_host/reading_pose/human', self.sub_human_pose_callback, 10)
        # Reading All Robot Pose
        #self.sub_all_robot_pose = self.create_subscription(Float32MultiArray, '/smartmbot_host/reading_pose/robots', self.sub_all_robot_pose_callback, 10)
       

        # Sub:: SMARTmBOT Robots
        self.sub_smartmbot_reading_spi_adc = self.create_subscription(Float32MultiArray, '/'+self.param_robot_ID+'/reading_spi_adc', self.sub_smartmbot_reading_spi_adc_callback, 10)
        self.sub_smartmbot_reading_tof_array = self.create_subscription(Int32MultiArray, '/'+ self.param_robot_ID+'/reading_tof_array', self.sub_smartmbot_reading_tof_array_callback, 10)
        
        # Sub:: User Study Status
        self.sub_cctv_experiment_command = self.create_subscription(Int32, '/cctv_GUI/control_state', self.sub_cctv_experiment_command_callback, 10)
        #Example: self.pub_cctv_GUI_control_state.publish(Int32(data=1))
        # data = 0 ------>  Start the mission and go to Prep. session
        # data = 1 ------>  Showing CCTV cameras
        # data = 2 ------>  SAM Question
        # data = 3 ------>  NASA-TLX Question
        # data = 4 ------>  End the mission


        #=======================
        # =============================#
        ####  Publisher Section                           ####
        #====================================================#
        # Pub:: User Study Status

        # Pub:: SMARTmBOT Robot 1
        print('/'+ self.param_robot_ID+'/writing_dc_motor_vel')
        self.pub_smartmbot_writing_dc_motor_vel = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_dc_motor_vel', 10)
        self.pub_smartmbot_writing_gpio_smd5050_led = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_gpio_smd5050_led', 10)
        self.pub_smartmbot_writing_ws2813_rgb_strip = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_ws2813_rgb_strip', 10)

        smartmbot_controller_timer_period = 0.01  # seconds
        self.smartmbot_controller_timer = self.create_timer(smartmbot_controller_timer_period, self.smartmbot_controller_timer_callback)       


    def smartmbot_controller_timer_callback(self):
        follow_robot_ID_index = self.robot_ID_array.index(self.param_robot_ID)
        
        user_study_state = self.cctv_experiment_command

        if user_study_state == 0:
            print("Ready, and current PWMs are: ", self.param_robot_speed)
            # self.cctv_experiment_command == 0  --> Ready to begin the study      
            self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0., 0., 255.0, 3., 0.5, 0., 1.]))
            vL_pwm = 0
            vR_pwm = 0     

        elif user_study_state == 1:
            # self.cctv_experiment_command == 1  --> Start the study
            self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0., 0., 0., 1., 0.5, 0., 1.]))

            spi_left = self.smartmbot_spi_adc_data[0]
            spi_right = self.smartmbot_spi_adc_data[1]
            
            spi_right_back = self.smartmbot_spi_adc_data[2]
            spi_left_back = self.smartmbot_spi_adc_data[3]


            tof_front = self.smartmbot_tof_array_data[0]
            tof_right = self.smartmbot_tof_array_data[2]
            tof_back = self.smartmbot_tof_array_data[4]
            tof_left = self.smartmbot_tof_array_data[6]
            
            tof_max_distance = 500

            robot_moving_speed = self.param_robot_speed
            adc_threshold = 1.4

            if spi_right > adc_threshold and spi_left > adc_threshold: #Go Front
                vL_pwm = robot_moving_speed
                vR_pwm = robot_moving_speed
            elif spi_right < adc_threshold and spi_left > adc_threshold: #Go Left
                vL_pwm = robot_moving_speed
                vR_pwm = 0
            elif spi_right > adc_threshold and spi_left < adc_threshold: #Go Right
                vL_pwm = 0
                vR_pwm = robot_moving_speed
            elif spi_right < adc_threshold and spi_left < adc_threshold: # Stop
                if tof_left > tof_max_distance:
                    vL_pwm = 0
                    vR_pwm = 0
                    print("DONE_DONE_DONE_DONE")
                else: 
                    return
                #self.pub_cctv_experiment_command.publish(Int32(data=0))

                
            print("speed:", robot_moving_speed, vL_pwm, vR_pwm, spi_left, spi_right)

        elif user_study_state == 2 or user_study_state == 9999:
            # self.cctv_experiment_command == 2  --> Stop during the study
            self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[255.0, 0, 0, 1, 0.5, 0, 1]))
            vL_pwm = 0
            vR_pwm = 0

        else:
            pass

        #Publish dc motor vels
        msg = Float32MultiArray()        
        msg.data = [float(vL_pwm), float(vR_pwm)] # PWM: []
        self.pub_smartmbot_writing_dc_motor_vel.publish(msg)


    # Sub: For reading sensors of the SMARTmBOT  
    def sub_smartmbot_reading_spi_adc_callback(self, msg):
        try:
            self.smartmbot_spi_adc_data = msg.data
        except:            
            pass
    def sub_smartmbot_reading_tof_array_callback(self, msg):
        try:
            self.smartmbot_tof_array_data = msg.data
        except:
            pass
    
    def sub_cctv_experiment_command_callback(self, msg):
        try:
            self.cctv_experiment_command = msg.data
            # self.cctv_experiment_command == 0  --> Ready to begin the study            
            # self.cctv_experiment_command == 1  --> Stop during the study
            # self.cctv_experiment_command == 2  --> Stop during the study

        except:
            pass
        

def main(args=None):
    rclpy.init(args=args)

    robot_irb_leader_follower_node = controller_irb_cctv_line_follower()

    try:
        while rclpy.ok():
            rclpy.spin(robot_irb_leader_follower_node)
        
        robot_irb_leader_follower_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))


    except KeyboardInterrupt:
        robot_irb_leader_follower_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        print('repeater stopped cleanly')
        
    except BaseException:
        robot_irb_leader_follower_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:      
        robot_irb_leader_follower_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        robot_irb_leader_follower_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
