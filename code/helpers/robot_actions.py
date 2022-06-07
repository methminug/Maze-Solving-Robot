import rospy
import math
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RobotActions:

    def __init__(self):
        self.__progress_state = 0
        self.__robot_position = Point()
        self.__robot_yaw = 0
        self.__robot_front = 0
        self.__robot_left = 0
        self.__robot_front_left = 0
        self.__robot_front_right = 0
        self.__robot_right = 0
        self.__found_wall = False  
        self.__stuck_count = 0
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.YAW_ACCURACY = math.pi / 90
        self.DISTANCE_ACCURACY = 0.2

    @property
    def progress_state(self):
        return self.__progress_state
        
    @property
    def robot_position(self):
        return self.__robot_position

    @property
    def robot_front(self):
        return self.__robot_front

    @property
    def robot_left(self):
        return self.__robot_left

    @property
    def robot_front_left(self):
        return self.__robot_front_left

    @property
    def robot_front_right(self):
        return self.__robot_front_right
    
    @property
    def robot_right(self):
        return self.__robot_right
                  
    def change_progress_state(self, new_state):
        self.__progress_state = new_state
    
    def stop(self):
        command = Twist()
        command.linear.x = 0
        command.angular.z = 0
        self.cmd_vel.publish(command)

    def odom_callback(self, msg):
        self.__robot_position = msg.pose.pose.position
        odom_ori = msg.pose.pose.orientation
        euler = euler_from_quaternion([odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w])
        self.__robot_yaw = euler[2]

    def scan_callback(self, msg):
        self.__robot_front = min(min(msg.ranges[0:5]), min(msg.ranges[355:]))
        self.__robot_front_left = min(msg.ranges[14:60])
        self.__robot_left = min(msg.ranges[74:105])
        self.__robot_right = min(msg.ranges[268:271])
        self.__robot_front_right = min(msg.ranges[299:345]) 

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (angle * 2 * math.pi) / (math.fabs(angle))
        return angle

    def correct_yaw(self, destination):
        destination_yaw = math.atan2(destination.y - self.__robot_position.y, destination.x - self.__robot_position.x)
        yaw_error = self.normalize_angle(destination_yaw - self.__robot_yaw)
        command = Twist()
        if math.fabs(yaw_error) > self.YAW_ACCURACY:
            command.angular.z = 0.3 if yaw_error > 0 else -0.3
        self.cmd_vel.publish(command)
        if math.fabs(yaw_error) <= self.YAW_ACCURACY:
            self.__progress_state = 1

    def move_directly(self, destination):
        destination_yaw = math.atan2(destination.y - self.__robot_position.y, destination.x - self.__robot_position.x)
        yaw_error = destination_yaw - self.__robot_yaw
        err_pos = math.sqrt(pow(destination.y - self.__robot_position.y, 2) + pow(destination.x - self.__robot_position.x, 2))
        if err_pos > self.DISTANCE_ACCURACY:
            command = Twist()
            command.linear.x = 0.2 
            command.angular.z = 0.2 if yaw_error > 0 else -0.2 
            self.cmd_vel.publish(command)
        else:
            # One area is over, robot is moving to next area
            self.__progress_state = 2 
        if math.fabs(yaw_error) > self.YAW_ACCURACY:
            self.__progress_state = 0

    def follow_wall(self, distance, command):
        while(not self.__found_wall and not rospy.is_shutdown()): 
            if(self.__robot_front > distance and self.__robot_front_right > distance and self.__robot_front_left > distance):
                command.angular.z = -0.1
                command.linear.x = 0.22
            elif(self.__robot_front_left < distance):
                self.__found_wall = True
            else:
                command.angular.z = -0.25
                command.linear.x = 0.0

            self.cmd_vel.publish(command)
        else:
            if(self.__robot_front > distance):
                self.__stuck_count = 0
                if(self.__robot_front_right < (distance / 2)):
                    print("Wall-following - Too close, reversing")
                    command.angular.z = 1.2 
                    command.linear.x = -0.1
                elif(self.__robot_front_right > (distance * 0.75)):
                    print("Wall-following - Turning left")
                    command.angular.z = -0.8 
                    command.linear.x = 0.22 
                else:
                    print("Wall-following - Turning right")
                    command.angular.z = 0.8 
                    command.linear.x = 0.22 
            elif(self.__robot_front_right < (distance * 1.5)): 
                #If robot is in this position for longer than 5 seconds, it is stuck while turning
                self.__stuck_count = self.__stuck_count +1
                if(self.__stuck_count > 20):
                    command.angular.z = 0.0 
                    command.linear.x = 50
                    self.cmd_vel.publish(command)
                print("Obstacle ahead. Turning away.")
                command.angular.z = 1.0  
                command.linear.x = 0.0
                self.cmd_vel.publish(command)
                while(self.__robot_front < 0.3 and not rospy.is_shutdown()):
                    self.cmd_vel.publish(command)
            self.cmd_vel.publish(command)

