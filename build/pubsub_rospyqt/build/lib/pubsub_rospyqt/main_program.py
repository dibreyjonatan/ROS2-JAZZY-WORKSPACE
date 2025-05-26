#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys 

## you write .utils here for it to be recognized during build and exec by ros
from .utils import Ui_MainWindow    
from PyQt5.QtWidgets import QApplication,QMainWindow
from PyQt5.QtCore import QTimer

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
        self.setWindowTitle("Publish Subscriber ROS2 interface")

        #creating the publisher node 
        self.pub_node=Node("Publisher_node")
        #The publisher
        self.publisher=self.pub_node.create_publisher(Int32,'/send_digit',10)
        #The aim is to sent a digit each second 
        timer_period=1
        self.timer=self.pub_node.create_timer(timer_period,self.timer_callback)
        self.i=0
        #i need to spin the publisher 
        self.time_pub=QTimer(self)
        self.time_pub.timeout.connect(self.spin_ros_pub_once)
        self.time_pub.setInterval(10)  # Spin every 10 ms
        
        
        #creating the subscriber node 
        self.sub_node=Node("Subscriber_node")
        #he subscriber 
        self.subscriber=self.sub_node.create_subscription(Int32,'/send_digit',self.listener_callback,10)
        #Create a timer to spin the node periodically
        self.time= QTimer(self)
        self.time.timeout.connect(self.spin_ros_once)
        self.time.setInterval(10)  # Spin every 10 ms

        #spin the nodes at thesame time 

        self.time.start()
        self.time_pub.start()

    def spin_ros_pub_once(self):
        rclpy.spin_once(self.pub_node,timeout_sec=0)
    def spin_ros_once(self):
        rclpy.spin_once(self.sub_node, timeout_sec=0)

    def timer_callback(self):
        #update the label of the publisher node 
        self.label_2.setText(str(self.i))
        
        msg=Int32()
        msg.data=self.i
        self.publisher.publish(msg)
        self.pub_node.get_logger().info('Publishing : "%s" '% msg.data)
        self.i+=1
        
    def listener_callback(self,msg):
       self.sub_node.get_logger().info('I heard : "%s"' % msg.data)
        #update the label of the subscriber node 
       self.label_4.setText(str(msg.data))    

def main(args=None):
    rclpy.init(args=args)
    app=QApplication(sys.argv)
    window=MainWindow()
    window.show()

    # Shutdown ROS cleanly when the Qt app quits
    app.aboutToQuit.connect(lambda: rclpy.shutdown())

    app.exec_()

if __name__== '__main__' :
    main()               



#NB : you could spin both nodes without creating a QTimer object 
# so there is a problem of spining nodes when using pyqt 

