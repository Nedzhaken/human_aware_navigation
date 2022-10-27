#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from people_msgs.msg import People
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry
from os.path import dirname, exists
from os import makedirs

class Metrics(object):
    def __init__(self):
        # ROS
        rospy.init_node('metrics')
        rospy.Subscriber('/people_tracker/trajectory_acc', People, self.callback_tracker)
        rospy.Subscriber('experiment/start', Bool, self.callback_start)
        rospy.Subscriber('experiment/finish', Bool, self.callback_finish)
        #Goal for the move_base
        self.get_plan = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        self.rate = rospy.Rate(1)
        #Variables for the time during
        self.time_during = 0
        self.finish_flag = False
        #Variables for the frequency
        self.people_arr_all = []
        self.people_freq_counter = []
        self.out_soc_zone = rospy.get_param('~social_zone')
        self.inner_soc_zone = rospy.get_param('~close_zone')
        #Variables for the mean velocities near the people
        self.velocities = []
        self.mean_velocity = 0
        self.usual_velocity = rospy.get_param('~speed')
        #Variables for the length of the path
        self.path_points_real = []
        self.length_real = 0
        #Variables for the writing the metrics to the file
        self.flag_log = False
        self.end_node_flag = True
        self.dir_name = dirname(__file__)

    def callback_start(self, callback_start):
        rospy.logwarn('Metrics - start of the experiment')
        #Update variables
        self.path_points_real = []
        self.length_real = 0

        #Get the start time
        start_time = rospy.get_time()
        #Get the start position

        amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        self.path_points_real.append(amcl_pose)

        while not self.finish_flag:
            amcl_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
            self.path_points_real.append(amcl_pose)
        finish_time = rospy.get_time()  

        #Calculate the finish time
        self.time_during = finish_time - start_time

        #Calculate the length of the real distance
        self.calculate_dist()
        self.calculate_mean_velo()

        self.write_metric()
        self.flag_log = True

        rospy.logwarn('Metrics - end of the experiment')
        rospy.loginfo('The time of the path is ' + str(self.time_during))

    def callback_finish(self, callback_finish):
        self.finish_flag = True

    def calculate_dist(self):

        for index in range(1, len(self.path_points_real)):
            x1 = self.path_points_real[index - 1].pose.pose.position.x
            y1 = self.path_points_real[index - 1].pose.pose.position.y
            x2 = self.path_points_real[index].pose.pose.position.x
            y2 = self.path_points_real[index].pose.pose.position.y
            dist = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
            self.length_real = self.length_real + dist
        
    def callback_tracker(self, people_trac):
        people_arr = people_trac.people
        for person in people_arr:
            id = int(person.name)

            if not (id in self.people_arr_all):
                self.people_arr_all.append(id)
                self.people_freq_counter.append([0, 0])
                
            dist = ((person.position.x)**2 + (person.position.y)**2)**(0.5)
            # rospy.loginfo('The distance to person is ' + str(dist))
            if dist <= self.out_soc_zone:
                rospy.loginfo('The close distance to person is ' + str(dist))
                index = self.people_arr_all.index(id)
                self.people_freq_counter[index][0] = self.people_freq_counter[index][0] + 1
                #Add the velocities to the list of the velocities, when the robot is near to the people
                odom = rospy.wait_for_message('/odometry/filtered', Odometry)
                x_vel = odom.twist.twist.linear.x
                y_vel = odom.twist.twist.linear.y
                self.velocities.append([x_vel, y_vel])

                if dist <= self.inner_soc_zone:
                    rospy.loginfo('Very close distance to person is ' + str(dist))
                    self.people_freq_counter[index][1] = self.people_freq_counter[index][1] + 1

    def calculate_mean_velo(self):
        #Calculate the sum of the velocities
        for i in self.velocities:
            x_vel = i[0]
            y_vel = i[1]
            dist = math.sqrt((x_vel)**2 + (y_vel)**2)
            self.mean_velocity = self.mean_velocity + dist
        #Calculate mean velocities, if the list of velocities has a values
        if self.velocities:
            self.mean_velocity = self.mean_velocity / len(self.velocities)

    def write_metric(self, folder_name = 'metrics', file_name = '1.txt'):
        rospy.logwarn('Write')
        #Create interesting name of the file for the metrics
        name = dirname(__file__) + '/' + folder_name + '/' + file_name
        #Check the folders
        path_foldr = dirname(__file__) + '/' + folder_name
        if not exists(path_foldr):
            makedirs(path_foldr)
        #Check the file. If the file exist, create a new with next id
        while exists(name):
            file_id = int(file_name.split('.')[0])
            file_name = str(file_id + 1) + '.txt'
            name = dirname(__file__) + '/' + folder_name + '/' + file_name
        #The write metrics values to the file
        with open(name, 'w') as f:
            f.write('The time of the last path is: ' + str(self.time_during) + '\n')
            f.write('The id of the pedestrians: ' + str(self.people_arr_all) + '\n')
            f.write('The frequency of the pedestrians: ' + str(self.people_freq_counter) + '\n')
            f.write('The length of the real path: ' + str(self.length_real) + '\n')
            f.write('The ration of the mean velocities near of the pedestrians and the usual velocity: ' + 
            str(self.mean_velocity / self.usual_velocity) + '\n')

    def shutdown_func(self):
        if self.end_node_flag:
            self.calculate_mean_velo()
            if not self.flag_log:
                self.write_metric()
            self.end_node_flag = False

if __name__ == '__main__':
    metrics_node = Metrics() 
    while not rospy.is_shutdown():
        try:
            rospy.on_shutdown(metrics_node.shutdown_func)
        except rospy.ROSInterruptException:
            pass
