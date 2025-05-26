#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys 

## you write .utils here for it to be recognized during build and exec by ros
from .utils import Ui_MainWindow    
from PyQt5.QtWidgets import QApplication,QMainWindow
from PyQt5.QtCore import QTimer

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
        self.setWindowTitle("Simple Interface to implement ROS2")

        self.node=Node("Publish_subscriber_node")
        #The publisher
        self.publisher=self.node.create_publisher(String,'/send_msg',10)
        #The subscriber 
        self.subscriber=self.node.create_subscription(String,'/send_msg',self.listener_callback,10)

        ##connecting the button that will publish 
        self.pushButton.clicked.connect(self.publish_msg)

        #Create a timer to spin the node periodically
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros_once)
        self.timer.start(10)  # Spin every 10 ms
        
    def spin_ros_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)
    def publish_msg(self):
        text=self.lineEdit.text()
        msg=String()
        msg.data=text
        self.publisher.publish(msg)
        self.lineEdit.clear()
        
    def listener_callback(self,msg):
       self.node.get_logger().info('I heard : "%s"' %msg.data)
       self.output_text.setText(msg.data)    

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