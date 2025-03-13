#!/usr/bin/python3
import rospy
import numpy as np
import math
import time

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, PolygonStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg

class ROSNode():
    def __init__(self):
        self.TOPIC_VEL = "/cmd_vel"
        self.TOPIC_GLOBAL_PLAN = "/move_base/TrajectoryPlannerROS/global_plan"
        self.TOPIC_LOCAL_PLAN = "/move_base/TrajectoryPlannerROS/local_plan"
        self.TOPIC_ODOM = "/odometry/filtered"
        self.TOPIC_CLOUD = "/front/odom/cloud"
        self.TOPIC_MAP_CLOUD = "/map/cloud"
        
        self.pub_vel = rospy.Publisher(self.TOPIC_VEL, Twist, queue_size=1, latch=True)
        
        self.sub_odometry = rospy.Subscriber(self.TOPIC_ODOM, Odometry, self.callback_odom)
        self.sub_global_plan = rospy.Subscriber(self.TOPIC_GLOBAL_PLAN, Path, self.callback_global_plan)
        self.sub_local_plan = rospy.Subscriber(self.TOPIC_LOCAL_PLAN, Path, self.callback_local_plan)
        self.sub_cloud = rospy.Subscriber(self.TOPIC_CLOUD, PointCloud2, self.callback_cloud)
        self.sub_map_cloud = rospy.Subscriber(self.TOPIC_MAP_CLOUD, PointCloud2, self.callback_map_cloud)

        self.projector = lg.LaserProjection()
        self.cmd_vel = Twist()
        self.odometry = Odometry()
        self.global_plan = Path()
        self.local_plan = Path()

        self.state_dim = 720
        self.action_dim = 2
        self.hidden_dim = 128
        self.actor = Actor(self.state_dim, self.action_dim, self.hidden_dim)
        self.critic = Critic(self.state_dim, self.hidden_dim)
        
    def callback_cloud(self, data):
        point_generator = pc2.read_points(data)
        self.obs_x = []
        self.obs_y = []
        for point in point_generator:
            self.obs_x.append(point[0])
            self.obs_y.append(point[1])

    def callback_map_cloud(self, data):
        point_generator = pc2.read_points(data)
        self.map_x = []
        self.map_y = []
        for point in point_generator:
            self.map_x.append(point[0])
            self.map_y.append(point[1])


    def callback_odom(self, data):
        print("odom data: ", data)
        self.odometry = data

    def callback_global_plan(self, data):
        print("global plan data: ", data)
        self.global_plan = data

    def callback_local_plan(self, data):
        print("local plan data: ", data)
        self.local_plan = data

    def publish_velocity(self, v_opt, w_opt):
        vel = Twist()
        vel.linear.x = v_opt
        vel.angular.z = w_opt
        self.pub_vel.publish(vel)

    def run(self):
        self.publish_velocity(0,0)


if __name__ =="__main__":
    rospy.init_node("a2c_node")
    rospy.loginfo("A2C Node running")
    node = ROSNode()
    pause = rospy.Rate(10)
    time.sleep(1)
    while not rospy.is_shutdown():
        node.run()
        pause.sleep()