import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from vicon_receiver.msg import Position



import sys, math
import numpy as np


class Go_to_Goal_Controller(Node):
    def __init__(self):
        super().__init__('robot_go_to_goal_control_node')
        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        self.declare_parameter('robot_name', 'smartmbot_1')
        self.param_robot_ID = self.get_parameter('robot_name').value
        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        #self.human_positions = np.zeros(6)
        self.smartmbot_spi_adc_data = np.zeros(8)
        self.smartmbot_tof_array_data = np.zeros(8)
        self.is_back = 0
        self.right_heavy_for = 0
        self.left_heavy_for = 0
        self.right_heavy_back = 0
        self.left_heavy_back = 0
 
        ####  Robot ID and positions
        self.current_robot_pose = np.zeros(3)
        self.error_robot_pose = np.zeros(3)

        self.current_target_pose = np.zeros(3)
        self.error_target_pose = np.zeros(3)

        #====================================================#
        ####  Subscription Section                        ####
        #====================================================#
        # Reading Human Pose
        #self.sub_human_pose = self.create_subscription(Float32MultiArray, '/smartmbot_host/reading_pose/human', self.sub_human_pose_callback, 10)

        # Reading All Robot Pose
        self.sub_robot_pose = self.create_subscription(Position, '/vicon/'+self.param_robot_ID+'/'+self.param_robot_ID, self.sub_robot_pose_callback, 10)
        
        # Reading All Robot Pose
        target_name = "cup"
        self.sub_target_pose = self.create_subscription(Position, '/vicon/'+target_name+'/'+target_name, self.sub_target_pose_callback, 10)


        # Sub:: SMARTmBOT Robot 1
        self.sub_smartmbot_reading_spi_adc = self.create_subscription(Float32MultiArray, '/'+ self.param_robot_ID+'/reading_spi_adc', self.sub_smartmbot_reading_spi_adc_callback, 10)
        self.sub_smartmbot_reading_tof_array = self.create_subscription(Int32MultiArray, '/'+ self.param_robot_ID+'/reading_tof_array', self.sub_smartmbot_reading_tof_array_callback, 10)


        #====================================================#
        ####  Publisher Section                           ####
        #====================================================#
        # Pub:: SMARTmBOT Robot 1
        print('/'+ self.param_robot_ID+'/writing_dc_motor_vel')
        self.pub_smartmbot_writing_dc_motor_vel = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_dc_motor_vel', 10)
        self.pub_smartmbot_writing_gpio_smd5050_led = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_gpio_smd5050_led', 10)
        self.pub_smartmbot_writing_ws2813_rgb_strip = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_ws2813_rgb_strip', 10)

        self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0.0, 255.0, 0.0, 1.0, 0.5, 0.0, 1.0]))

        smartmbot_controller_timer_period = 0.01  # seconds
        self.smartmbot_controller_timer = self.create_timer(smartmbot_controller_timer_period, self.smartmbot_controller_timer_callback)       
    
    def smartmbot_controller_timer_callback(self):

        # Currnet Robot pose::: [x, y, yaw]  - unit: [mm, mm, rad]
        smartmbot_currnt_pose = self.current_robot_pose
        
        
        # for test
        #print("current:", smartmbot_currnt_pose)
        
        k_linear = 0.02
        k_angular = 0.8
        
        
        (linear_vel, angular_vel)= self.go_to_goal_controller(smartmbot_currnt_pose, self.current_target_pose, k_linear, k_angular)
        
        ### Robot Controller
        wheel_base = 90 # unit: mm   
        wheel_radius = 20 # unit: mm    

        if linear_vel == 0:
            offset = 0
        else:
            offset = 20 # PWM

        print(linear_vel, angular_vel)

        (vL_pwm, vR_pwm) = self.robot_unicycle_controller(linear_vel, angular_vel, wheel_base, wheel_radius, offset)



        if vL_pwm > 100:
            vL_pwm = 100
        if vL_pwm < -100:
            vL_pwm = -100

        if vR_pwm > 100:
            vR_pwm = 100
        if vR_pwm < -100:
            vR_pwm = -100

        #For test
        print(vL_pwm, vR_pwm)
        #vL_pwm = 0.0
        #vR_pwm = 0.0


        #Publish dc motor vels
        msg = Float32MultiArray()

        msg.data = [float(vL_pwm), float(vR_pwm)] # PWM: []
        self.pub_smartmbot_writing_dc_motor_vel.publish(msg)
   

    def go_to_goal_controller(self, current_pos, desired_pos, linear_gain = 1, angular_gain = 2):

        (current_x, current_y, current_theta) = current_pos # Unit: mm
        (goal_x, goal_y, goal_theta) = desired_pos # Unit: radian


        # Go-To-Goal Algorithm        
        distance = math.sqrt(math.pow((current_x-goal_x), 2) + math.pow((current_y-goal_y), 2))
        print("distance:", distance)

        linear = distance * linear_gain
        
        diff_angle = round(math.atan2(goal_y - current_y, goal_x - current_x), 4)
        angular = angular_gain * (diff_angle - current_theta)

        # When robot arrived the trageted position
        if distance <= 150:
            linear = 0
            print("arrived the goal")

           
        else:
            self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0.0, 0.0, 255.0, 1.0, 0.5, 0.0, 1.0]))
        

        linear_velocity = round(linear, 4)
        angular_velocity = 0 #round(angular, 4)

        #print("is back?:", self.is_back)

        return linear_velocity, angular_velocity

   
    def robot_unicycle_controller(self, linear_vel, angular_vel, wheel_base = 90, wheel_radius = 20, offset_pwm = 20):

        left_speed = linear_vel - angular_vel * wheel_base/2
        right_speed = linear_vel + angular_vel * wheel_base/2
        
        left_speed_pwm = int(left_speed + offset_pwm)
        right_speed_pwm = int(right_speed + offset_pwm)
    
        if self.left_heavy_for == 1:
            right_speed_pwm *= 1.2
        elif self.left_heavy_back == 1:
            right_speed_pwm *= 1.2
        elif self.right_heavy_for == 1:
            left_speed_pwm *= 1.2
        elif self.right_heavy_back == 1:
            left_speed_pwm *= 1.2 

        return left_speed_pwm, right_speed_pwm

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw #unit: radian

      

    def sub_target_pose_callback(self, msg):
        # Currnet Robot pose::: [x, y, yaw]  - unit: [mm, mm, rad]
        if msg.x_trans != 0:
            self.current_target_pose[0] = msg.x_trans
            self.current_target_pose[1] = msg.y_trans

            input_quat_data = [msg.x_rot, msg.y_rot, msg.z_rot, msg.w]
            [roll, pitch, yaw] = self.euler_from_quaternion(input_quat_data)
            self.current_target_pose[2] = yaw
            self.error_target_pose = self.current_target_pose
        else:
            self.current_target_pose = self.error_target_pose

    def sub_robot_pose_callback(self, msg):
        # Currnet Robot pose::: [x, y, yaw]  - unit: [mm, mm, rad]
        if msg.x_trans != 0:
            self.current_robot_pose[0] = msg.x_trans
            self.current_robot_pose[1] = msg.y_trans

            input_quat_data = [msg.x_rot, msg.y_rot, msg.z_rot, msg.w]
            [roll, pitch, yaw] = self.euler_from_quaternion(input_quat_data)
            self.current_robot_pose[2] = yaw
            self.error_robot_pose = self.current_robot_pose

        else:
            self.current_robot_pose = self.error_robot_pose

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


def main(args=None):
    rclpy.init(args=args)

    robot_go_to_goal_control_node = Go_to_Goal_Controller()

    try:
        while rclpy.ok():
            pass
            rclpy.spin(robot_go_to_goal_control_node)

    except KeyboardInterrupt:
        print('repeater stopped cleanly')
        
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:        
        robot_go_to_goal_control_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
