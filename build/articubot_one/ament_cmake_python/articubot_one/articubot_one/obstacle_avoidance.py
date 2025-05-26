#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


# i am writing down this piece of code with paramters 
# Paramters are for the node , it enables modularity and you can modify the parameters when executing the node 
# declare_parameter takes into account (name of parameter, value)
# ex : ros2 run my_robot_package avoidance_node --ros-args -p avoidance_angular_speed:=1.0
class Obstacle(Node):
    def __init__(self) :
        super().__init__("Laser_Obstacle_Avoidance")
        
        self.latest_data=None
        self.obstacle_detected=None

        self.declare_parameter("threshold_distance",0.5)  # i can use 0.5m or 1m as threshold distance
        self.threshold=self.get_parameter("threshold_distance").value
        
        self.declare_parameter("laser_scan_topic","/scan")
        self.laser_topic=self.get_parameter("laser_scan_topic").value
        
        self.declare_parameter("cmd_vel_topic","/cmd_vel")
        self.cmd_topic=self.get_parameter("cmd_vel_topic").value

        # Angular range in front of the robot to check for obstacles (in degrees).
        #  60.0 means checking from -30 to +30 degrees relative to the robot's front.
        self.declare_parameter('frontal_detection_angle_deg', 60.0)
        self.frontal_detection_angle_rad = math.radians(self.get_parameter('frontal_detection_angle_deg').value)
        # create a subscriber to the laser
        self.laser_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.laser_callback,
            10
        )
        self.get_logger().info(f'Subscribing to laser scan on: {self.laser_topic}')
        #create a publisher to the /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist,self.cmd_topic, 10)
        self.get_logger().info(f'Publishing velocity commands on: {self.cmd_topic}')
       

    def laser_callback(self,msg):

        twist = Twist()
        scan_size = len(msg.ranges)
        #to facilitate testing
        self.latest_data=msg 
        
        if not(scan_size) :
            #no laser data 
            return 
        # the lidar provides angles,angle step (common difference of the angle sequence), distances in meter
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        min_distance_in_frontal_sector = float('inf')
        current_angle = angle_min
        
        # range is the distance
        # ---1. Calculate the minimum distance in the sector -30° and 30°

        #for every angle in this sector, we get a valid distance and compare it with the angle_(i+1) in the sector so as to get the minimum distance
        #we get an angle, we get it's corresponding distance and compare it's distance with a min distance
        # At each round, we get the minimum distance since our objectif is to get the closest distance to an obstacle 

        # we are sweeping across all the angles with the increment 
        # while sweeping we compare  only the ranges with the ones in the defined angle 
        # here since we use abs, the comparaison is done between 0 and 30°

        for i in range(scan_size):
            # Normalize angle to be within [-pi, pi) for easier comparison
            normalized_angle = math.atan2(math.sin(current_angle), math.cos(current_angle))

            # Check if the current angle is within the frontal detection sector
            # Use absolute angle to check if it's within half the frontal range from 0.
            if abs(normalized_angle) <= self.frontal_detection_angle_rad / 2.0:
                 # If the range reading is valid (not inf or nan)
                 if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]):
            
                     min_distance_in_frontal_sector = min(min_distance_in_frontal_sector, msg.ranges[i])

            current_angle += angle_increment

        # --- 2. Determine Action Based on Obstacle Detection ---

        self.obstacle_detected = min_distance_in_frontal_sector < self.threshold

        #  To stop the robot from moving ahead when ever it meets an obstacle, it will just go back 
        if self.obstacle_detected :
           self.get_logger().info(f'Obstacle detected at {min_distance_in_frontal_sector:.2f}m. Deciding to stop.')
           twist.linear.x = 0.0
           twist.angular.z =0.0
           self.cmd_vel_pub.publish(twist)
        else:
           self.get_logger().info(f'No obstacle detected') 

def main(args=None):
    rclpy.init(args=args)
    node = Obstacle()
    try:
        rclpy.spin(node) # Keep the node running and process callbacks
    except KeyboardInterrupt:
        node.get_logger().info('Smart Obstacle Avoider Node stopped cleanly via KeyboardInterrupt.')
    finally:
        # Ensure the robot stops when the node is shut down
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        try:
            node.cmd_vel_pub.publish(stop_twist)
            node.get_logger().info('Published stop command before shutdown.')
        except Exception as e:
            node.get_logger().error(f'Failed to publish stop command: {e}')

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
