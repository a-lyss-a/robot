#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
import xml.etree.ElementTree as ET
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import uuid
import os


class Controller(Node):

    def __init__(self):
        super().__init__('urdf_prefix')
        self.prefix = self.declare_parameter('prefix', '')
        self.filename = self.declare_parameter('filename', 'robot')
        self.urdf = self.declare_parameter('urdf', '')

        self.sub = self.create_subscription(String, 'in_robot_description', self.sub_callback, 1)

        if len(self.urdf.value):
            self.get_logger().info('urdf parameter is set, using that')
            self.create_file(self.urdf.value)



    def sub_callback(self, msg):
        self.create_file(msg.data)


    def create_file(self, data):
        self.get_logger().info('Got robot description urdf, will write to /tmp/%s.urdf' % self.filename.value)
        tree = ET.ElementTree(ET.fromstring(data))
        root = tree.getroot()

        for i in root.iter():
            #if i.tag in ['link', 'joint', 'sensor', 'camera']:
            if i.tag in ['link', 'joint']:
                i.attrib['name'] = self.prefix.value + i.attrib['name']

            elif i.tag in ['parent', 'child']:
                i.attrib['link'] = self.prefix.value + i.attrib['link']

            elif i.tag == 'gazebo':
                if 'reference' in i.attrib:
                    i.attrib['reference'] = self.prefix.value + i.attrib['reference']

            elif i.tag == 'plugin':
                if i.attrib['filename'] == 'libgazebo_ros_ray_sensor.so':
                    x = i.find('frame_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_force_based_move.so':
                    x = i.find('robot_base_frame')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_p3d.so':
                    x = i.find('body_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_joint_state_publisher.so':
                    x = i.find('joint_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_lifter.so':
                    x = i.find('joint_name')
                    x.text = self.prefix.value + x.text

                elif i.attrib['filename'] == 'libgazebo_ros_camera.so':
                    x = i.find('frame_name')
                    if x != None:
                        x.text = self.prefix.value + x.text

        # To eliminate a possible race condition where the RSP helper sees and reads the file before
        # is is comletely written, we write the file with a unique id, ensure it is written, then 
        # rename the file to the desired name. This works because rename is an atomic file operation.
        unique_filename = str(uuid.uuid4())
        with open('/tmp/%s' % unique_filename, 'wb') as f:
            tree.write(f)
            f.flush()
            os.fsync(f.fileno())

        time.sleep(1)
        self.get_logger().info('Moving file to /tmp/%s.urdf' % self.filename.value)
        os.rename('/tmp/%s' % unique_filename, '/tmp/%s.urdf' % self.filename.value)

        # msg.data = '<?xml version="1.0" ?>' + ET.tostring(root, encoding='unicode', method='xml')
        # self.pub.publish(msg)

        time.sleep(1)
        self.destroy_node()
        exit(0)
        

def main():

    rclpy.init()

    controller = Controller()
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
    
    



