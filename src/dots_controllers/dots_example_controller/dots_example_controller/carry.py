#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

import sensor_msgs.msg
import nav_msgs.msg
#from sensor_msgs import point_cloud2 as pc2

import geometry_msgs.msg
import sys
import numpy as np
import random

import struct
import operator
import py_trees
import py_trees_ros


from py_trees.composites import Parallel as Par
from py_trees.composites import Sequence as Seq
from py_trees.composites import Selector as Sel
from py_trees.behaviours import Periodic
from py_trees.behaviours import Running

print(sys.path)

class Check_collision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Check_collision, self).__init__(name)
    
    def setup(self):
        pass

    def initialise(self):
        pass

    def update(self):
        pass

    def terminate(self):
        pass
        
class Process_irtof(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_irtof, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='irtof', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='obstacle', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='collision', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='new_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='first_cmd_vel', access=py_trees.common.Access.WRITE)

        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5

        # Start with forward motion
        cv                          = geometry_msgs.msg.Twist()
        cv.linear.x                 = self.speed
        self.blackboard.first_cmd_vel = cv


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print(e)


    def update(self):
        # Return running if the blackboard entry does not yet exist
        try:
            irtof = self.blackboard.irtof
        except KeyError:
            return py_trees.common.Status.RUNNING

        # There are 16 sensors, make somewhere to store data from each one. The Gazebo
        # version of the laser scanner doesn't send data for points with no return detected
        # but the hardware does, so fill in the maximum range for each entry to start with.
        msg = irtof
        data        = np.zeros((16))
        data[:]     = self.max_range
        step        = math.pi / 8.0
        prox        = np.zeros(2)
        collision   = False
        for i in range(msg.width):
            # Points within the pointcloud are actually locations in 3D space in the scanner
            # frame. Each is a float32 taking 12 bytes. 
            # Extract point
            [x, y, z]   = struct.unpack('fff', bytes(msg.data[i * msg.point_step : i * msg.point_step + 12]))
            # Get angle and see which sensor it was
            th          = math.atan2(y, x)
            if th < 0.0:
                th += 2 * math.pi
            idx         = int(round(th / step))
            # Get the distance and save it in the array
            dist        = math.sqrt(x**2 + y**2)
            data[idx]   = dist
            # Check if there is a collision
            if dist < self.collision_range:
                collision = True
            # Calculate a vector pointing towards the nearest obstacle
            nm          = np.array([x, y]) / dist
            nm_inv_len  = 1 - (dist - self.min_range) / (self.max_range - self.min_range)
            nm          = nm * nm_inv_len
            prox        += nm 

        self.blackboard.obstacle    = prox
        self.blackboard.collision   = collision

        cv = geometry_msgs.msg.Twist()

        coll_vector     = -prox / np.linalg.norm(prox)
        cv.linear.x     = coll_vector[0] * self.speed
        cv.linear.y     = coll_vector[1] * self.speed
        cv.angular.z    = random.uniform(-math.pi / 2.0, math.pi / 2.0)

        self.blackboard.new_cmd_vel = cv

        #self.node.get_logger().info('%s' % irtof)
        return py_trees.common.Status.SUCCESS



def create_root():
    root = py_trees.composites.Parallel(
        name    = "Simple BT controller",
        policy  = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    topics2bb   = py_trees.composites.Sequence("Topics2BB")

    irtof2bb    = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'irtof2bb',
        topic_name              = 'sensor/scan',
        topic_type              = sensor_msgs.msg.PointCloud2,
        qos_profile             = qos_profile_sensor_data,
        blackboard_variables    = {'irtof' : None}
    )

    odom2bb     = py_trees_ros.subscribers.ToBlackboard(
        name                    = 'odom2bb',
        topic_name              = 'odom',
        topic_type              = nav_msgs.msg.Odometry,
        qos_profile             = qos_profile_system_default,
        blackboard_variables    = {'odom' : None}
    )

    battery2bb  = py_trees_ros.battery.ToBlackboard(
        name                    = "Battery2BB",
        topic_name              = "battery_state",
        qos_profile             = qos_profile_sensor_data,
        threshold               = 30.0
    )

    priorities      = py_trees.composites.Sequence(name="Tasks", memory=False)
    first_cmd_vel = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'first_cmd_vel'
    )
    start_moving    = py_trees.decorators.OneShot(name='Start moving', child=first_cmd_vel)
    proc_irtof      = Process_irtof(name="Proc irtof")

    coll_avoid  = py_trees.composites.Selector("Coll avoid")
    coll_check  = py_trees.behaviours.CheckBlackboardVariableValue(name="Clear of obstacles?",
        check   = py_trees.common.ComparisonExpression(
            variable    = 'collision', 
            value       = False, 
            operator    = operator.eq)
    )
    avoid_cmd_vel = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'new_cmd_vel'
    )

    idle = py_trees.behaviours.Running(name="Idle")



    root.add_child(topics2bb)
    topics2bb.add_child(irtof2bb)
    topics2bb.add_child(odom2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(start_moving)
    priorities.add_child(proc_irtof)
    priorities.add_child(coll_avoid)
    priorities.add_child(idle)
    coll_avoid.add_child(coll_check)
    coll_avoid.add_child(avoid_cmd_vel)

    return root




class Controller(Node):

    def __init__(self):
        # Create the controller node
        super().__init__('controller')

        # Publish to one topic, the command velocity
        self.cmd_vel_pub        = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to the laser time of flight sensors topic. 
        # This must be set to the qos sensor profile since the data is sent as best effort, and the 
        # default qos is reliable, which produces no data traffic
        self.irtof_sub          = self.create_subscription(PointCloud2, 'sensor/scan', self.scan_callback, 
                                    qos_profile=qos_profile_sensor_data)


        self.command            = Twist()
        self.heading            = random.uniform(0, math.pi)


        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5

        # Collision
        self.collision          = False
        self.coll_vector        = []

        # Start off with forward motion
        self.command.linear.x   = self.speed

        # Start a timer to run the control loop at 10Hz
        self.timer              = self.create_timer(0.1, self.control_loop_callback)

        # Build a behaviour tree
        self.tree = py_trees_ros.trees.BehaviourTree(create_root(), unicode_tree_debug=True)
        self.tree.setup()        



    def scan_callback(self, msg):
        # This function gets called with every new message on the laser scan topic.
        # Extract the data and work out if there is a collision, i.e range is less
        # than some amount, if so, select a new direction.

        # There are 16 sensors, make somewhere to store data from each one. The Gazebo
        # version of the laser scanner doesn't send data for points with no return detected
        # but the hardware does, so fill in the maximum range for each entry to start with.
        data        = np.zeros((16))
        data[:]     = self.max_range
        step        = math.pi / 8.0
        prox        = np.zeros(2)
        self.collision   = False
        for i in range(msg.width):
            # Points within the pointcloud are actually locations in 3D space in the scanner
            # frame. Each is a float32 taking 12 bytes. 
            # Extract point
            [x, y, z]   = struct.unpack('fff', bytes(msg.data[i * msg.point_step : i * msg.point_step + 12]))
            # Get angle and see which sensor it was
            th          = math.atan2(y, x)
            if th < 0.0:
                th += 2 * math.pi
            idx         = int(round(th / step))
            # Get the distance and save it in the array
            dist        = math.sqrt(x**2 + y**2)
            data[idx]   = dist
            # Check if there is a collision
            if dist < self.collision_range:
                self.collision = True
            # Calculate a vector pointing towards the nearest obstacle
            nm          = np.array([x, y]) / dist
            nm_inv_len  = 1 - (dist - self.min_range) / (self.max_range - self.min_range)
            nm          = nm * nm_inv_len
            prox        += nm

        if self.collision:
            # If too close to something, get the unit vector pointing away from the 
            # nearest obstacle and use that to set the velocity, with a bit and random
            # angular velocity added
            self.coll_vector        = -prox / np.linalg.norm(prox)





    def control_loop_callback(self):

        self.tree.tick()


        if self.collision:
            self.command.linear.x   = self.coll_vector[0] * self.speed
            self.command.linear.y   = self.coll_vector[1] * self.speed
            self.command.angular.z  = random.uniform(-math.pi / 2.0, math.pi / 2.0)

        self.cmd_vel_pub.publish(self.command)
        

def main():
    rclpy.init()

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree.setup()
    tree.tick_tock(period_ms=100.0)
    rclpy.spin(tree.node)

    #controller = Controller()
    #rclpy.spin(controller)


if __name__ == '__main__':
    main()
    
    



