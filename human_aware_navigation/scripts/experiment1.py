#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

class Experiment(object):
    def __init__(self):
        # ROS
        rospy.init_node('experiment_hallway')
        # Goal to RVIZ
        self.pub_point = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_start = rospy.Publisher('experiment/start', Bool, queue_size=10)
        self.pub_finish = rospy.Publisher('experiment/finish', Bool, queue_size=10)
        self.rate = rospy.Rate(1)
        self.tolerance = rospy.get_param('~tolerance')

        # [x, y, z, w]. 0, 1 - position, [2, 3] - orientation        
        # Experimnet in the room - horizontal   
        self.positions_hor = [[0.008839154762, 0.672228568674, -0.887309733908, 0.46117397597],
                          [-0.639815248709, -0.431307852741, 0.387353165649, 0.921931410172],
                          [0.008839154762, 0.672228568674, -0.887309733908, 0.46117397597],
                          [-0.639815248709, -0.431307852741, 0.387353165649, 0.921931410172],
                          [0.008839154762, 0.672228568674, -0.887309733908, 0.46117397597],
                          [-0.639815248709, -0.431307852741, 0.387353165649, 0.921931410172],
                          [0.008839154762, 0.672228568674, -0.887309733908, 0.46117397597],
                          [-0.639815248709, -0.431307852741, 0.387353165649, 0.921931410172],
                          [0.008839154762, 0.672228568674, -0.887309733908, 0.46117397597]]

        # Experimnet in the room - vertical
        self.positions_vert = [[-1.13505337871, 1.13245020905, 0.98825523176, 0.152812293022],
                          [1.32935637573, -0.532676737586, 0.953918905788, 0.30006452836],
                          [-1.13505337871, 1.13245020905, 0.98825523176, 0.152812293022],
                          [1.32935637573, -0.532676737586, 0.953918905788, 0.30006452836],
                          [-1.13505337871, 1.13245020905, 0.98825523176, 0.152812293022],
                          [1.32935637573, -0.532676737586, 0.953918905788, 0.30006452836],
                          [-1.13505337871, 1.13245020905, 0.98825523176, 0.152812293022],
                          [1.32935637573, -0.532676737586, 0.953918905788, 0.30006452836]]

        self.experiment_case = rospy.get_param('~experiment')
        if self.experiment_case == 'horizontal':
            self.positions = self.positions_hor
        else:
            self.positions = self.positions_vert

    def start_experiment_dist(self):
        id = 0
        self.rate.sleep()
        rospy.logwarn('START')
        self.pub_start.publish(True)
        for point_list in self.positions:
            point = PoseStamped()
            point.pose.position.x = point_list[0]
            point.pose.position.y = point_list[1]
            point.pose.orientation.z = point_list[2]
            point.pose.orientation.w = point_list[3]
            point.header.seq = id
            point.header.frame_id = "map"
            now = rospy.get_rostime()
            point.header.stamp.secs = now.secs
            point.header.stamp.nsecs = now.nsecs
            self.pub_point.publish(point)
            self.rate.sleep()

            goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            bewore_start_len = len(goals_array.status_list)
            after_start_len = len(goals_array.status_list)
            while bewore_start_len == after_start_len:
                self.pub_point.publish(point)
                goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
                after_start_len = len(goals_array.status_list)
            goal_id = goals_array.status_list[-1].status
            rospy.logwarn('The id of point is ' + str(id))

            amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)   
            robot_x = amcl_pose.pose.pose.position.x
            robot_y = amcl_pose.pose.pose.position.y
            dist = math.sqrt((robot_x - point_list[0])**2 + (robot_y - point_list[1])**2)
            while dist >= self.tolerance:
                if goal_id == 4:
                    rospy.logwarn('The point with id = {:-2} is unreachable. Move to next point'.format(id))
                    break
                goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
                goal_id = goals_array.status_list[-1].status   
                amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)   

                robot_x = amcl_pose.pose.pose.position.x
                robot_y = amcl_pose.pose.pose.position.y
                dist = math.sqrt((robot_x - point_list[0])**2 + (robot_y - point_list[1])**2) 

            id = id +1
        rospy.logwarn('FINISH')
        self.pub_finish.publish(True)

    def start_pose(self):
        point = PoseStamped()
        point.pose.position.x = self.start_position[0]
        point.pose.position.y = self.start_position[1]
        point.pose.orientation.z = self.start_position[2]
        point.pose.orientation.w = self.start_position[3]
        point.header.seq = 0
        point.header.frame_id = "map"
        now = rospy.get_rostime()
        point.header.stamp.secs = now.secs
        point.header.stamp.nsecs = now.nsecs
        self.pub_point.publish(point)

        self.rate.sleep()

        goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
        bewore_start_len = len(goals_array.status_list)
        after_start_len = len(goals_array.status_list)
        while bewore_start_len == after_start_len:
            self.pub_point.publish(point)
            goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            after_start_len = len(goals_array.status_list)
        goal_id = goals_array.status_list[-1].status
        rospy.logwarn('The start position is sent')

        while goal_id!=3:
            if goal_id == 4:
                rospy.logwarn('The start point is unreachable')
                break
            self.rate.sleep()
            goals_array = rospy.wait_for_message('/move_base/status', GoalStatusArray)
            goal_id = goals_array.status_list[-1].status  
        rospy.logwarn('The robot is on the start position')
 
if __name__ == '__main__':
    path_node = Experiment()
    path_node.start_experiment()
    rospy.spin()
