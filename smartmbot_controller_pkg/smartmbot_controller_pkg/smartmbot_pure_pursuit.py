import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Int32MultiArray, Float32MultiArray

import sys, math, random
import numpy as np
import matplotlib.pyplot as plt
from vicon_receiver.msg import Position


# Pure Pursuit Parameters
k = 0.1  # look forward gain
Lfc = 0.3 # [m] look-ahead distance
Kp = 10.0  # speed proportional gain
dt = 0.01  # [s] self.time tick
WB = 0.15 # [m] wheel base of vehicle # was 0.1
Ang_kp = 5 # was 3 

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta): ## need to read vicon positions
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def calc_distance_back(self, point_x, point_y):
        dx = point_x - self.x
        dy = point_y - self.y
        return math.hypot(dx, dy)
class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            if ind >= (self.cx.size - 1):
                pass

            else:
                distance_this_index = state.calc_distance(self.cx[ind],
                                                        self.cy[ind])
                while True:
                    distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                            self.cy[ind + 1])
                    if distance_this_index < distance_next_index:
                        break
                    ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                    distance_this_index = distance_next_index
                self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf   
class TargetCourse_back:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [icx for icx in self.cx - state.x]
            dy = [icy for icy in self.cy - state.y]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                try:
                    if ind > len(self.cx):
                        continue 
                    else:               
                        distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                                self.cy[ind + 1])
                        if distance_this_index < distance_next_index:
                            break
                        ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                        distance_this_index = distance_next_index
                    
                except:
                    #print("error")
                    pass
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf  
class controller_pure_pursuit(Node):
    def __init__(self):
        super().__init__('robot_irb_leader_follower_node')
        #
        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        self.declare_parameter('robot_name', 'smartmbot_8')
        self.param_robot_ID = self.get_parameter('robot_name').value
        self.declare_parameter('target_speed', 0.1)
        self.param_target_speed = self.get_parameter('target_speed').value

        self.declare_parameter('pwm_max', 60)
        self.param_pwm_max = self.get_parameter('pwm_max').value

        self.declare_parameter('pwm_min', 30)
        self.param_pwm_min = self.get_parameter('pwm_min').value
        self.declare_parameter('x_start', 311.1026916503906/1000) #unit: meter
        self.param_x_start = self.get_parameter('x_start').value
        self.declare_parameter('x_end', 3.962386474609375) #unit: meter
        self.param_x_end = self.get_parameter('x_end').value
        self.declare_parameter('y_start', 0.7749718627929688) #unit: meter
        self.param_y_start = self.get_parameter('y_start').value 
        self.declare_parameter('y_end', 0.7458865966796875) #unit: meter
        self.param_y_end = self.get_parameter('y_end').value
        self.declare_parameter('adc_threshold', 1.5)
        self.param_adc_threshold = self.get_parameter('adc_threshold').value



        #====================================================#
        ####  Global Variables in Class                   ####
        #====================================================#
        self.smartmbot_spi_adc_data = np.zeros(8)
        self.smartmbot_tof_array_data = np.zeros(8)
        self.cctv_experiment_command = 0

        self.robot_direction = True # True = Forward, False = Backward
      
        # Currnet Robot pose::: [x, y, yaw]  - unit: [mm, mm, rad]
        self.current_robot_pose = np.zeros(3)
        self.error_robot_pose = np.zeros(3)

        #====================================================#
        ####  Subscription Section                        ####
        #====================================================#
        # Reading Human Pose        
        # Reading All Robot Pose

        self.sub_robot_pose = self.create_subscription(Position, '/vicon/'+self.param_robot_ID+'/'+self.param_robot_ID, self.sub_robot_pose_callback, 10)
        

        
        # Sub:: User Study Status
        self.sub_cctv_experiment_command = self.create_subscription(Int32, '/cctv_GUI/control_state', self.sub_cctv_experiment_command_callback, 10)
        #Example: self.pub_cctv_GUI_control_state.publish(Int32(data=1))

        # data = 0 ------>  Ready for connecting sensors or another participant to join the study.
        # data = 1 ------>  Start the mission and go to Prep. session
        # data = 2 ------>  Showing CCTV cameras (main experiment)
        # data = 3 ------>  SAM Question
        # data = 4 ------>  ISA Question
        # data = 5 ------>  NASA-TLX Question
        # data = 6 ------>  End the mission


        #=======================
        # =============================#
        ####  Publisher Section                           ####
        #====================================================#
        # Pub:: User Study Status
        # Pub:: SMARTmBOT Robot 1
        #print('/'+ self.param_robot_ID+'/writing_dc_motor_vel')
        self.pub_smartmbot_writing_dc_motor_vel = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_dc_motor_vel', 10)
        
        self.pub_smartmbot_writing_ws2813_rgb_strip = self.create_publisher(Float32MultiArray, '/'+ self.param_robot_ID+'/writing_ws2813_rgb_strip', 10)
        smartmbot_controller_timer_period = 0.01  # seconds
        self.smartmbot_controller_timer = self.create_timer(smartmbot_controller_timer_period, self.smartmbot_controller_timer_callback)       

    

        #====================================================#
        ####  Robot's Variables in Class                  ####
        #====================================================#
        
        random_vel_timer_period = 2  # seconds
        self.random_vel_timer_timer = self.create_timer(random_vel_timer_period, self.random_vel_timer_callback)   
        #  target course
        self.cx = np.arange(self.param_x_start, self.param_x_end, smartmbot_controller_timer_period)
        self.cy = np.arange(self.param_y_start, self.param_y_end, (self.param_y_end- self.param_y_start)/len(self.cx))

        self.cx_back = np.arange(self.param_x_end, self.param_x_start, -smartmbot_controller_timer_period)
        self.cy_back = np.arange(self.param_y_end, self.param_y_start, (self.param_y_start- self.param_y_end)/len(self.cx_back))


    
        # initial self.state
        self.state = State(x=self.param_x_start, y=self.param_y_start, yaw=0.0, v=0.0)

        self.lastIndex = len(self.cx) -  1
        self.time = 0.0
        self.states = States()
        self.states.append(self.time, self.state)
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)
    

        
    def random_vel_timer_callback(self):
        #  Random Velocity
        random_vel_list = [40, 50, 60, 70, 80]
        #random.choice(mylist, weights=[0.2, 0.2, 0.6], k=10)

        if self.param_pwm_max == 9999:
            self.smartmbot_speed = random.choice(random_vel_list)

        else:
            self.smartmbot_speed = self.param_pwm_max

        #print(self.smartmbot_speed)


    def smartmbot_controller_timer_callback(self):
        user_study_state = self.cctv_experiment_command
        #user_study_state = 1 # For test

        smartmbot_current_pose = self.current_robot_pose
        
        if user_study_state == 2:
            # self.cctv_experiment_command == 1  --> Start the study
            #self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0., 0., 0., 1., 0.5, 0., 1.]))
            
            self.state.x = smartmbot_current_pose[0]/1000 # mm to m
            self.state.y = smartmbot_current_pose[1]/1000 # mm to m
            self.state.yaw = smartmbot_current_pose[2]

            self.show_animation = False #True
            
            
            if self.robot_direction:
                # forward
                self.target_course = TargetCourse(self.cx, self.cy)
                self.target_ind, _ = self.target_course.search_target_index(self.state)

                if self.lastIndex  > self.target_ind:
                    #print("forward",self.target_ind) 
                    #self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[0., 0., 255., 1., 0.5, 0., 1.]))
                    #self.get_logger().info('self.target_ind=="%s", self.lastIndex=="%s"' % (self.target_ind, self.lastIndex))

                    self.forward_speed = self.param_target_speed# m/s
                    ai = self.proportional_control(self.forward_speed, self.state.v)
                    #ai = ai*np.random.rand() # For noise            
                    di, self.target_ind = self.pure_pursuit_steer_control(self.state, self.target_course, self.target_ind)

                    if abs(ai) > 0.1: # For curve line.            
                        di =  di*0.5

                    self.state.update(ai, di)  # Control vehicle
                    # Need to update vicon information

                    left_speed = self.state.v - di * WB/2
                    right_speed = self.state.v + di * WB/2

                    if self.lastIndex - 10 <= self.target_ind:
                        left_speed =  float(np.interp(left_speed,[-self.forward_speed, self.forward_speed],[-40, 40]))
                        right_speed = float(np.interp(right_speed,[-self.forward_speed, self.forward_speed], [-40, 40]))
                    else:        
                        left_speed =  float(np.interp(left_speed,[-self.forward_speed, self.forward_speed],[-self.smartmbot_speed, self.smartmbot_speed]))
                        right_speed = float(np.interp(right_speed,[-self.forward_speed, self.forward_speed], [-self.smartmbot_speed, self.smartmbot_speed]))


                    left_speed = round(left_speed, 2)                    
                    right_speed = round(right_speed, 2)

                    #publish left, right wheel.
                    #print(smartmbot_current_pose)
                
                    self.states.append(self.time, self.state) #For trajectory
                    
                   
                    #print("last::::", self.lastIndex , self.target_ind)
                    vL_pwm = left_speed
                    vR_pwm = right_speed
                    #publish left, right wheel.

                else:              
                    self.target_course_back = TargetCourse_back(self.cx_back, self.cy_back)
                    self.target_ind = 0
                    self.state.v = 0
                    vL_pwm = 0
                    vR_pwm = 0
                    self.robot_direction = False # change mode  
                    #print("changed forward to back::", self.lastIndex , self.target_ind)
            else:
                # backward
                #self.target_ind, _ = self.target_course_back.search_target_index(self.state)                
                
                if self.lastIndex  > self.target_ind:
                    #print("backward")
                    #self.pub_smartmbot_writing_ws2813_rgb_strip.publish(Float32MultiArray(data=[255., 0., 0., 1., 0.5, 0., 1.]))
                    self.backward_speed = -self.param_target_speed # m/s

                    ai = self.proportional_control(self.backward_speed, self.state.v)

                    di, self.target_ind = self.pure_pursuit_steer_control_back(self.state, self.target_course_back, self.target_ind)
                   
                    if abs(ai) > 0.1: # For curve line.            
                        di =  di*0.5

                    self.state.update(ai, di)  # Control vehicle
                    # Need to update vicon information
                    left_speed = (self.state.v + di * WB/2)
                    right_speed = (self.state.v - di * WB/2)

                    if self.lastIndex - 10 <= self.target_ind:
                        left_speed =  float(np.interp(left_speed,[self.backward_speed, -self.backward_speed],[-40, 40]))
                        right_speed = float(np.interp(right_speed,[self.backward_speed, -self.backward_speed], [-40, 40]))
                    else:        
                        left_speed =  float(np.interp(left_speed,[self.backward_speed, -self.backward_speed], [-self.smartmbot_speed, self.smartmbot_speed]))

                        right_speed = float(np.interp(right_speed,[self.backward_speed, -self.backward_speed], [-self.smartmbot_speed, self.smartmbot_speed]))


                    left_speed = round(left_speed, 2)                    
                    right_speed = round(right_speed, 2)


                    #print(left_speed, right_speed)
                    #publish left, right wheel.
                    
                
                    self.states.append(self.time, self.state) #For trajectory
                    
                    
                    #print("last::::", self.lastIndex , self.target_ind)
                    vL_pwm = left_speed
                    vR_pwm = right_speed
                    #publish left, right wheel.

                  
                else:
                    self.target_ind = 0
                    self.state.v = 0
                    vL_pwm = 0
                    vR_pwm = 0
                    self.robot_direction = True # change mode  
                    #print("changed back to forward::", self.target_ind)
        else:
            vL_pwm = 0
            vR_pwm = 0
        
        #print("speed:", robot_moving_speed, vL_pwm, vR_pwm, spi_left, spi_right)
                
        #Publish dc motor vels
        msg = Float32MultiArray()

        #print(vL_pwm, vR_pwm, smartmbot_current_pose[0]/1000 , smartmbot_current_pose[1]/1000 ) ## For test
 
        #### For test
        #vL_pwm = 0.0  ## for test
        #vR_pwm = 0.0  ## for test

        msg.data = [float(vL_pwm), float(vR_pwm)] # PWM: []
        self.pub_smartmbot_writing_dc_motor_vel.publish(msg)

    def proportional_control(self, target, current):
        a = Kp * (target - current)
        return a
   

    def pure_pursuit_steer_control(self, state, trajectory, pind):
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = Ang_kp*(math.atan2(ty - state.y, tx - state.x) - state.yaw)
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        return delta, ind

    def pure_pursuit_steer_control_back(self, state, trajectory, pind):
        ind, Lf = trajectory.search_target_index(state)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = Ang_kp*(state.yaw - math.atan2(state.y - ty, state.x - tx))
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

        return delta, ind    


    def plot_arrow(self, x, y, yaw, length=0.2, width=0.2, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)




    # Sub: For reading sensors of the SMARTmBOT  
    '''
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
    '''

    def sub_cctv_experiment_command_callback(self, msg):
        try:
            self.cctv_experiment_command = msg.data
            # self.cctv_experiment_command == 0  --> Ready to begin the study            
            # self.cctv_experiment_command == 1  --> Stop during the study
            # self.cctv_experiment_command == 2  --> Stop during the study
        except:
            pass

    # Functions for this code    
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


    def sub_robot_pose_callback(self, msg):
        # Currnet Robot pose::: [x, y, yaw]  - unit: [mm, mm, rad]
        if msg.x_trans is not 0:
            self.current_robot_pose[0] = msg.x_trans
            self.current_robot_pose[1] = msg.y_trans

            input_quat_data = [msg.x_rot, msg.y_rot, msg.z_rot, msg.w]
            [roll, pitch, yaw] = self.euler_from_quaternion(input_quat_data)
            self.current_robot_pose[2] = yaw
            self.error_robot_pose = self.current_robot_pose

        else:
            self.current_robot_pose = self.error_robot_pose



def main(args=None):
    rclpy.init(args=args)

    controller_pure_pursuit_node = controller_pure_pursuit()

    try:
        while rclpy.ok():
            rclpy.spin(controller_pure_pursuit_node)
        
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))


    except KeyboardInterrupt:
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))

        print('repeater stopped cleanly')
        
    except BaseException:
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))

        print('exception in repeater:', file=sys.stderr)
        raise

    finally:      
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))
        controller_pure_pursuit_node.pub_smartmbot_writing_dc_motor_vel.publish(Float32MultiArray(data=[0, 0]))

        controller_pure_pursuit_node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
