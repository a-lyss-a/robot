#!/usr/bin/env python3

import sys
print(sys.path)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import math

import tf2_ros
import sensor_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import rosgraph_msgs.msg
#from sensor_msgs import point_cloud2 as pc2
import dots_interfaces.msg
import geometry_msgs.msg
import numpy as np
import random

import struct
import operator
import py_trees
import py_trees_ros

class G:
    # Global values
    max_linear_velocity     = 0.5
    max_angular_velocity    = 3.0

    # P controllers
    Pv                      = 2.0
    Pw                      = 2.0
    carriers                = [100, 101, 102, 103, 104]


class Pick_random_direction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Pick_random_direction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='random_cmd_vel', access=py_trees.common.Access.WRITE)
    def update(self):
        cv              = geometry_msgs.msg.Twist()
        angle           = random.uniform(-math.pi, math.pi)
        cv.linear.x     = G.max_linear_velocity * math.cos(angle)
        cv.linear.y     = G.max_linear_velocity * math.sin(angle)
        cv.angular.z    = 0.0
        self.blackboard.random_cmd_vel = cv
        return py_trees.common.Status.SUCCESS


class Process_irtof(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_irtof, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='irtof', access=py_trees.common.Access.READ)
        self.blackboard.register_key(key='obstacle', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='collision', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='avoid_cmd_vel', access=py_trees.common.Access.WRITE)

        self.max_range          = 2.0
        self.min_range          = 0.13
        self.collision_range    = 0.3
        self.speed              = 0.5


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
        cv.linear.x     = coll_vector[0] * G.max_linear_velocity
        cv.linear.y     = coll_vector[1] * G.max_linear_velocity
        cv.angular.z    = random.uniform(-G.max_angular_velocity / 2.0, G.max_angular_velocity / 2.0)

        self.blackboard.avoid_cmd_vel = cv

        #self.node.get_logger().info('%s' % irtof)
        return py_trees.common.Status.SUCCESS

###########################################
# - This is where the behaviour tree is defined in terms of node types and relationships between them
###########################################
def create_root():
	#######
	# this part defines the node types
	#######
    root = py_trees.composites.Parallel(
        name    = 'Simple BT controller example',
        policy  = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    obstacle_check = py_trees.composites.Sequence(
        name = 'Obstacle check',
        memory = True
    )

    proc_obstacle_check      = Process_irtof(name='Proc seq')

    obstacle_qu  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Obstacles?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'collision', 
            value       = True, 
            operator    = operator.eq)
    )
    
    avoid_ob  = py_trees_ros.publishers.FromBlackboard(
        name                = 'Avoid obstacle',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'zero_cmd_vel'
    )


    random_wander   = py_trees.composites.Sequence('Wander', memory=True)
    random_wander_sir = py_trees.decorators.SuccessIsRunning(random_wander)

    pick_direction  = Pick_random_direction('Pick direction')
    random_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        name                = 'random cmd_vel',
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'random_cmd_vel'
    )
    wander_delay    = py_trees.behaviours.TickCounter(name='Wander delay', duration=50)
	
	#######
	# this part defines the node relationships
	#######
	
    root.add_child(obstacle_check)
    obstacle_check.add_child(obstacle_qu)
    obstacle_check.add_child(avoid_ob)
    root.add_child(random_wander_sir)
    random_wander.add_child(pick_direction)
    random_wander.add_child(random_cmd_vel)
    random_wander.add_child(wander_delay)

    return root




        

def main():
    rclpy.init()

    #py_trees.logging.level = py_trees.logging.level.DEBUG
    root = create_root()
    #tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    tree.tick_tock(period_ms=100.0)
    rclpy.spin(tree.node)



if __name__ == '__main__':
    main()
    
    



