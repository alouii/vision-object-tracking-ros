#!/usr/bin/env python3
"""Spawn models into Gazebo and move them on simple trajectories.

This node spawns models from the `models/` folder using `gazebo_msgs/SpawnModel`
and then periodically sets their poses with `gazebo_msgs/SetModelState`.
"""

import rospy
import rospkg
import time
import os
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion


class SpawnerMover:
    def __init__(self):
        rospy.init_node('spawn_and_move')
        rp = rospkg.RosPack()
        pkg_path = rp.get_path('vision_object_tracking')

        # read params
        self.rate = rospy.get_param('~rate', 30)
        self.sphere_name = rospy.get_param('~sphere_name', 'red_sphere')
        self.person_name = rospy.get_param('~person_name', 'person_cylinder')
        self.sphere_radius = rospy.get_param('~sphere_radius', 0.06)
        self.path_radius = rospy.get_param('~path_radius', 1.0)
        self.path_speed = rospy.get_param('~path_speed', 0.5)
        self.person_speed = rospy.get_param('~person_speed', 0.2)

        self.sphere_sdf = open(os.path.join(pkg_path, 'models', 'red_sphere', 'model.sdf')).read()
        self.person_sdf = open(os.path.join(pkg_path, 'models', 'person_cylinder', 'model.sdf')).read()

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # spawn models
        self._safe_delete(self.sphere_name)
        self._safe_delete(self.person_name)

        rospy.loginfo('Spawning models...')
        self.spawn_model(self.sphere_name, self.sphere_sdf, Pose(position=Point(x=0.0, y=-1.0, z=0.1)))
        self.spawn_model(self.person_name, self.person_sdf, Pose(position=Point(x=1.5, y=0.0, z=0.8)))

        self.start_time = rospy.Time.now().to_sec()
        self.loop()

    def _safe_delete(self, name):
        try:
            self.delete_srv(name)
        except Exception:
            pass

    def spawn_model(self, model_name, model_xml, initial_pose):
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = ''
        req.initial_pose = initial_pose
        req.reference_frame = 'world'
        try:
            self.spawn_srv(req)
            rospy.loginfo('Spawned model: %s', model_name)
        except Exception as e:
            rospy.logerr('Spawn failed for %s: %s', model_name, e)

    def set_pose(self, name, x, y, z, yaw=0.0):
        state = ModelState()
        state.model_name = name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        # quaternion from yaw only
        q = self._yaw_to_quaternion(yaw)
        state.pose.orientation = q
        try:
            self.set_state_srv(state)
        except Exception as e:
            rospy.logerr('Failed to set state for %s: %s', name, e)

    def _yaw_to_quaternion(self, yaw):
        # return geometry_msgs/Quaternion from yaw
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*q)

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.start_time
            # sphere moves in circle in front of the robot
            x = self.path_radius * math.cos(self.path_speed * t)
            y = self.path_radius * math.sin(self.path_speed * t) - 0.5
            z = 0.06
            self.set_pose(self.sphere_name, x, y, z)

            # person walks along x axis back and forth
            period = max(0.1, 4.0 / self.person_speed)
            px = 0.5 * math.sin(2 * math.pi * t / period) + 1.0
            py = 0.0
            pz = 0.8
            self.set_pose(self.person_name, px, py, pz, yaw=0.0)

            rate.sleep()


if __name__ == '__main__':
    try:
        SpawnerMover()
    except rospy.ROSInterruptException:
        pass
