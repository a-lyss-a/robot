#!/usr/bin/env python3


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

import geometry_msgs.msg
import sys
import numpy as np
import random

import struct
import operator
import py_trees
import py_trees_ros

# The behaviour of sequence without memory in the current py-trees is not 
# semantically correct, it should be the inverse of sel without mem. 
# Subclassing and adding inverters makes a fake version with the correct semantics
class FakeSequence(py_trees.decorators.Inverter):
    def __init__(self, **kwargs):
        super(FakeSequence, self).__init__(child=py_trees.composites.Selector(**kwargs))
    def add_child(self, child):
        self.decorated.add_child(py_trees.decorators.Inverter(child))


class G:
    # Global values
    max_linear_velocity     = 0.5
    max_angular_velocity    = 3.0

    # P controllers
    Pv                      = 3.0
    Pw                      = 3.0
    carriers                = [100, 101, 102, 103, 104]

    pre_dock                = geometry_msgs.msg.Transform()



def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

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
        
class Process_vision(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Process_vision, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(key='cam0_tags', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='under_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='centre_cmd_vel', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='seen_carrier', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key='align_cmd_vel', access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            self.node = None
            print(e)

        # Listen to the lists of tags seen by each camera
        self.cam0_tags_sub = self.node.create_subscription(std_msgs.msg.Int32MultiArray, 
                            'cam0_tags', self.cam0_tags_callback, qos_profile_system_default)
        self.cam4_tags_sub = self.node.create_subscription(std_msgs.msg.Int32MultiArray, 
                            'cam4_tags', self.cam4_tags_callback, qos_profile_system_default)

        # test sub to clock
        #self.clock_sub = self.node.create_subscription(rosgraph_msgs.msg.Clock, '/clock', 
        #                    self.clock_callback, qos_profile_system_default)

        # Transform listener
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer, self.node)


        self.centre_transform = None
        self.align_transform = None


    # def clock_callback(self, msg):
    #     #self.node.get_logger().info('Got %s' % msg)
    #     pass

    def get_past(self, delta):
        # Look slightly into the past for the transforms. This seems to fail sometimes
        # with a negative time. Presumably a callback arrives before the node has got
        # a time from /clock or from the system (only seen happen in simulation)
        now = self.node.get_clock().now()
        d = rclpy.duration.Duration(seconds=delta)
        try:
            past = now - d
        except ValueError as e:
            past = now
        return past


    def cam0_tags_callback(self, msg):
        #self.node.get_logger().info('Got cam0_tags %s' % msg.data)

        past = self.get_past(0.1)
        self.blackboard.seen_carrier = False
        transforms = []
        for x in msg.data:
            if x in G.carriers:
                try:
                    transform = self.tfbuffer.lookup_transform(
                        'robot_deadbeef_base_link', 
                        'robot_deadbeef_cam0_fid%03d' % x, past)
                except (tf2_ros.LookupException, 
                        tf2_ros.ConnectivityException, 
                        tf2_ros.ExtrapolationException) as e:
                    pass
                    #self.node.get_logger().info('Failed tf lookup %s' % e)
                else:
                    transforms.append(transform)
                    #self.node.get_logger().info('%s' % transform)
        # All the valid IDs get appended, choose the closest one
        if len(transforms):
            transforms.sort(key=lambda t: np.linalg.norm(np.array(( t.transform.translation.x, 
                                                                    t.transform.translation.y, 
                                                                    t.transform.translation.z))))
            t = transforms[0]
            #self.node.get_logger().info('%s % 8f % 8f % 8f' % (transforms[0].child_frame_id, t.x, t.y, t.z))



            self.align_transform = transforms[0]
            self.blackboard.seen_carrier = True



    def cam4_tags_callback(self, msg):
        #self.node.get_logger().info('Got cam4_tags %s %s' % (msg.data, self.node.get_clock().now()))

        past = self.get_past(0.1)
        self.blackboard.under_carrier = False
        if len(msg.data) == 1:
            try:
                self.centre_transform = self.tfbuffer.lookup_transform(
                    'robot_deadbeef_base_link', 
                    'robot_deadbeef_cam4_fid%03d' % msg.data[0], past)
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                #self.node.get_logger().info('Failed tf lookup %s' % e)
                pass
            else:
                self.blackboard.under_carrier = True



    def update(self):
        if self.centre_transform != None:
            # We're under a carrier, calculate the velocity to centre
            r,p,y   = euler_from_quaternion(self.centre_transform.transform.rotation)
            tr      = self.centre_transform.transform.translation

            self.node.get_logger().info('% 8f % 8f % 8f : % 8f % 8f % 8f' % (tr.x, tr.y, tr.z, r, p, y))

            # Simple P controller to move to centre of carrier
            cmd_vel = geometry_msgs.msg.Twist()
            cmd_vel.linear.x    = G.Pv * tr.x
            cmd_vel.linear.y    = G.Pv * tr.y
            cmd_vel.angular.z   = G.Pw * y
            # Limit velocities
            cmd_vel.linear.x    = np.clip(cmd_vel.linear.x, -G.max_linear_velocity, G.max_linear_velocity)
            cmd_vel.linear.y    = np.clip(cmd_vel.linear.y, -G.max_linear_velocity, G.max_linear_velocity)
            cmd_vel.angular.z   = np.clip(cmd_vel.angular.z, -G.max_angular_velocity, G.max_angular_velocity)

            self.blackboard.centre_cmd_vel = cmd_vel

        if self.align_transform != None:
            # We've seen a carrier with an appropriate ID, align in preparation 
            # for docking and centering
            r,p,y   = euler_from_quaternion(self.align_transform.transform.rotation)
            tr      = self.align_transform.transform.translation
            #self.node.get_logger().info('% 8f % 8f % 8f : % 8f % 8f % 8f' % (tr.x, tr.y, tr.z, r, p, y))
            


        return py_trees.common.Status.SUCCESS

        
def test():
    root = py_trees.composites.Selector(memory=False)
    root.add_child(py_trees.behaviours.Failure())
    #seq = py_trees.composites.Sequence(memory=False)
    seq = FakeSequence(memory=False, name='fake')
    seq.add_child(py_trees.behaviours.TickCounter(duration=10))
    seq.add_child(py_trees.behaviours.Success())
    root.add_child(seq)

    return root


def create_root():
    root = py_trees.composites.Parallel(
        name    = 'Simple BT controller',
        policy  = py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )

    topics2bb   = py_trees.composites.Sequence(
        name    = 'Topics2BB',
        memory  = True
    )

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

    #priorities      = py_trees.composites.Sequence(name='Priorities', memory=False)
    priorities      = FakeSequence(name='FS Priorities', memory=False)

    perform_tasks   = py_trees.composites.Selector(name='Tasks', memory=False)


    proc_irtof      = Process_irtof(name='Proc irtof')
    proc_vision     = Process_vision(name='Proc vision')

    centre_if_under = py_trees.composites.Sequence('Centre if under', memory=True)
    check_if_under  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Under carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'under_carrier', 
            value       = True, 
            operator    = operator.eq)
    )
    centre_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'centre_cmd_vel'
    )

    align_if_found  = py_trees.composites.Sequence('Dock if found', memory=True)
    check_if_found  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Seen carrier?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'seen_carrier',
            value       = True,
            operator    = operator.eq)
    )
    align_cmd_vel   = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'align_cmd_vel'
    )


    avoid_collision  = py_trees.composites.Sequence('Coll avoid', memory=True)
    coll_check  = py_trees.behaviours.CheckBlackboardVariableValue(
        name    = 'Obstacles?',
        check   = py_trees.common.ComparisonExpression(
            variable    = 'collision', 
            value       = True, 
            operator    = operator.eq)
    )
    avoid_cmd_vel = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'avoid_cmd_vel'
    )

    random_wander   = py_trees.composites.Sequence('Wander', memory=True)
    pick_direction  = Pick_random_direction('Pick direction')
    random_cmd_vel  = py_trees_ros.publishers.FromBlackboard(
        topic_name          = 'cmd_vel',
        topic_type          = geometry_msgs.msg.Twist,
        qos_profile         = qos_profile_system_default,
        blackboard_variable = 'random_cmd_vel'
    )
    delay           = py_trees.behaviours.TickCounter(name='Wander delay', duration=10)


    idle = py_trees.behaviours.Success(name='Idle')



    root.add_child(topics2bb)
    topics2bb.add_child(irtof2bb)
    topics2bb.add_child(odom2bb)
    root.add_child(priorities)
    priorities.add_child(proc_irtof)
    priorities.add_child(proc_vision)
    priorities.add_child(perform_tasks)
    priorities.add_child(idle)

    perform_tasks.add_child(centre_if_under)
    centre_if_under.add_child(check_if_under)
    centre_if_under.add_child(centre_cmd_vel)

    perform_tasks.add_child(avoid_collision)
    avoid_collision.add_child(coll_check)
    #avoid_collision.add_child(avoid_cmd_vel)

    perform_tasks.add_child(align_if_found)
    align_if_found.add_child(check_if_found)
    align_if_found.add_child(align_cmd_vel)

    perform_tasks.add_child(random_wander)
    random_wander.add_child(pick_direction)
    #random_wander.add_child(random_cmd_vel)
    random_wander.add_child(delay)




    return root




        

def main():
    rclpy.init()

    #py_trees.logging.level = py_trees.logging.level.DEBUG
    root = create_root()
    #root = test()
    #tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup()
    tree.tick_tock(period_ms=100.0)
    rclpy.spin(tree.node)



if __name__ == '__main__':
    main()
    
    



