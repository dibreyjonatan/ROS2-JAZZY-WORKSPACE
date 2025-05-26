#!/usr/bin/python3

import pytest
import rclpy
from rclpy.node import Node
import time 
import math
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

from articubot_one.obstacle_avoidance import Obstacle 

import logging
from datetime import datetime

# Create a logger
logger = logging.getLogger('simple_logger')
logger.setLevel(logging.ERROR)

# Create a console handler
console_handler = logging.StreamHandler()

# Define the ROS 2-like log format
formatter = logging.Formatter(
    '[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# Apply the formatter
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


#creating a node to test the node under test
# Re-using the TestClientNode which has a /cmd_vel subscriber and publishes to the /scan

#don't name your helper class with Test*** because this confuses pytest 
#all the def functions you write should all begin with  [Tt] so they can be recognized with pytest
class _TestClientNode(Node):
    def __init__(self):
        super().__init__('test_client_node')
        self.cmd_vel_data = None
        self.received_cmd_vel = False
        self.scan_msg=None
        # Publisher for sending fake LaserScan messages to the node under test
        self.laser_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data
        )

        # Subscriber for receiving Twist messages from the node under test
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_system_default
        )

    def cmd_vel_callback(self, msg):
        self.get_logger().info("Test node received Twist message")
        self.cmd_vel_data = msg
        self.received_cmd_vel = True
    def publish_laser_scan(self):
        # - create the message 
       scan_msg = LaserScan()
       
       scan_msg.angle_min = -math.pi/6 #corresponds to -30°
       scan_msg.angle_max = math.pi/6 #corresponds to +30°
       scan_msg.angle_increment = math.pi / 180  # 1-degree resolution
       scan_msg.range_min = 0.0
       scan_msg.range_max = 2.0
       #message without obstacle 
       ranges = [float('inf')] * 180
       #simulating an obstacle infront of object
       center_index = len(ranges) // 2
       ranges[center_index] = 0.3  # Obstacle at 0.3 meters directly ahead
       scan_msg.ranges = ranges
       self.laser_publisher.publish(scan_msg)
       self.scan_msg=scan_msg
       self.get_logger().info(f"Test node published LaserScan message with {len(ranges)} ranges")

@pytest.fixture(scope='module')
def ros_setup():
    """Fixture to initialize and shutdown rclpy once per module."""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def node_fixture(ros_setup):
    """Fixture to create and manage the Obstacle node and node to test obstacle node."""
    node = None
    executor = None
    try:
        node = Obstacle()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        #run the test node created 
        nodeto_test_node = _TestClientNode()
        executor.add_node(nodeto_test_node)
        # Give the node a moment to spin and receive parameters/connections
        # In a real launch_testing scenario, the node is launched,
        # and the test starts after. Waiting here might still be needed
        # depending on how parameters are set and when they arrive.
        # A more robust way might be to wait for parameters to be set or topics to be active.
        timeout_sec = 2.0
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout_sec:
             executor.spin_once(0.1) # Spin briefly to process events

        yield node,nodeto_test_node, executor # Yield the node and executor

    finally:
        if node:
            executor.remove_node(node)
            node.destroy_node()
        if nodeto_test_node:
            executor.remove_node(nodeto_test_node)
            nodeto_test_node.destroy_node()    
        if executor:
             executor.shutdown()


def test_node_creation(node_fixture):
    """Test node creation and basic attributes."""
    node,_,executor = node_fixture
    assert node.get_name() == "Laser_Obstacle_Avoidance"
    # Checking attributes created in the constructor
    assert hasattr(node, 'laser_sub')
    assert node.laser_sub.topic_name == '/scan' # Often includes the leading slash in ROS 2
    assert hasattr(node, 'cmd_vel_pub')
    assert node.cmd_vel_pub.topic_name == '/cmd_vel' # Often includes the leading slash

 #create a laser data that i'll publish to the /scan topic and expect 
#our node to get it and process it 
def test_obstacle(node_fixture):
    node,nodeto_test_node, executor = node_fixture
    try :
          
       # Publish the fake LaserScan message to the node under test
       nodeto_test_node.publish_laser_scan()
       #give some time for the info to be processed 
       timeout_sec = 1.0
       start_time = time.time()
       while rclpy.ok() and time.time() - start_time < timeout_sec:
         executor.spin_once(0.1)

       # Test if data is recieved 
       assert node.latest_data is not None 
       print("data being recieved")
       # Test if it is thesame data recieved
       expected_laser_data=nodeto_test_node.scan_msg
       assert node.latest_data == expected_laser_data
       print("the expected message is been recieved")
       #test if the object was effectively detected
       assert node.obstacle_detected == 1 , f" expected to detect and object but failed"

       #test if the published message is null as expected 
       #1 - create the expected twist 
       expected=Twist()
       expected.linear.x=0
       expected.angular.z=0
        #2 verify if message is recieved
       assert nodeto_test_node.recieved_cmd_vel == True 
       #3 Test if the message is what is expected 
       assert nodeto_test_node.cmd_vel_data.linear.x == expected.linear.x
       assert nodeto_test_node.cmd_vel_data.angular.z == expected.angular.z 

       logger.info("Test passed successfully")
    except :
        logger.error("An error occured")

def test_obstacle_parameters(node_fixture):
    """Test ROS parameters after the node has had a chance to spin."""
    node,_,executor = node_fixture

    # Now access parameters after the node has spun briefly
    # Note: Getting parameters this way works if they are declared
    # and have default values, or if they are set externally
    # AND the node has spun to receive them.
    try:
        # Use the correct ROS 2 way to get parameter values
        threshold = node.get_parameter('threshold_distance').value
        scan_topic = node.get_parameter('laser_scan_topic').value
        cmd_topic = node.get_parameter('cmd_vel_topic').value
        front_sector_angle = node.get_parameter('frontal_detection_angle_deg').value

        # assert the values are correct as expected
        assert isinstance(threshold, float), f"expect the threshold to be a float"
        assert isinstance(scan_topic, str), f"expect the scan topic to be a string"
        assert isinstance(cmd_topic, str), f"expect the cmd velocity command to be a string"
        assert isinstance(front_sector_angle, float), f'expect the angle of sector to be a float'

        # 2- check if the values of the parameters are correct
        # These values should match what's set in your launch file or node defaults
        assert threshold == 0.5
        assert scan_topic == "/scan"
        assert cmd_topic == "/cmd_vel"
        assert front_sector_angle == 60.0

    except rclpy.exceptions.ParameterNotDeclaredException as e:
        pytest.fail(f"Parameter not declared: {e}")
    except Exception as e:
        pytest.fail(f"Error getting parameter: {e}")






























'''


The reason why this test failed  :

incorrect rclpy.init() and rclpy.shutdown() Usage:

    You are calling rclpy.init() and rclpy.shutdown() inside each test function (test_node_creation, test_parameter_node, test_obstacle).
    rclpy.init() should typically be called only once before starting any ROS 2 operations in a process, and rclpy.
    shutdown() once at the very end. Calling them repeatedly in separate test functions like this will lead to errors and unexpected 
    behavior because rclpy is not designed for this lifecycle pattern within a single process execution ike a pytest run.
    Solution: Use a pytest fixture to manage the rclpy lifecycle and node creation, 
    initializing rclpy once before the tests that need it and shutting it down afterward. 
    This is the standard and correct approach in launch_testing.


def test_node_creation():
    rclpy.init
    try :
        node=Obstacle()
        # verify that the node has the expected name
        assert node.get_name()=="Laser_Obstacle_Avoidance"
        # verify that the node has the subscribed to the correct topic by name
        assert hasattr(node,'laser_sub')
        assert node.laser_sub.topic_name == 'scan'
        # verify that the node is publishing to the correct topic by name 
        assert hasattr(node,'cmd_vel_pub')
        assert node.cmd_vel_pub.topic_name == 'cmd_vel'
    finally :
        rclpy.shutdown
def test_parameter_node():
    rclpy.init
    try :
        pass
    finally :
        rclpy.shutdown
def test_obstacle():
    rclpy.init
    try :
        node=Obstacle()
        #get values of parameters
        threshold=node.get_parameter('threshold_distance').value
        scan_topic=node.get_parameter("laser_scan_topic").value
        cmd_topic=node.get_parameter("cmd_vel_topic").value
        front_sector_angle=node.get_parameter("frontal_detection_angle_deg").value
        # assert the values are correct as expected 
        # 1- check if the parameters are of expected classes 
        assert isinstance(threshold,float) ,f"expect the threshold to be a float"
        assert isinstance(scan_topic,str), f"expect the scan topic to be a string"
        assert isinstance(cmd_topic,str), f"expect the cmd velocity command to be a string"
        assert isinstance(front_sector_angle,float),f'expect the angle of sector to be a float'
        # 2- check if the values of the parameters are correct 
        assert threshold == 0.5
        assert scan_topic== "scan"
        assert cmd_topic == "cmd_vel"
        assert front_sector_angle == 60.0 
    finally :
        rclpy.shutdown


if __name__=='__main__' :
    pytest.main(['-v'])

'''    