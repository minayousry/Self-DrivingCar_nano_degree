#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
from scipy.spatial import KDTree
import os
from shutil import rmtree


STATE_COUNT_THRESHOLD = 3

class DataCollecter(object):
    def __init__(self):
        rospy.init_node('data_collecter')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        
        rospy.loginfo('data collection')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()

        self.waypoints_2d = None
        self.waypoints_tree = None

        self.current_directory = os.getcwd()
        
        self.all_images_counter = 1
        self.red_images_counter = 1

        self.light_status_dir = os.path.join(self.current_directory , "LightStatusData")
        self.red_light_dir = os.path.join(self.current_directory , "RedLightDistanceData")

        try:
            rmtree(self.light_status_dir)
            rmtree(self.red_light_dir)
        except:
            pass

        try:
            os.mkdir(self.light_status_dir)
            os.mkdir(os.path.join(self.light_status_dir, "IMG"))
            os.mkdir(self.red_light_dir)
            os.mkdir(os.path.join(self.red_light_dir, "IMG"))
        except:
            pass

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state, distance = self.process_traffic_lights()

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        name = str(self.all_images_counter) + '.jpg'
        cv2.imwrite(os.path.join(self.light_status_dir, "IMG", name), cv_image)
        self.all_images_counter += 1

        if distance <= 300.0:
            with open(os.path.join(self.light_status_dir, "data.csv"), "a+") as log:
                log.write("{0},{1}\n".format('IMG/' + name, str(state if state == TrafficLight.RED else -1)))
        else:
            with open(os.path.join(self.light_status_dir, "data.csv"), "a+") as log:
                log.write("{0},{1}\n".format('IMG/' + name, str(-1)))

        if (state == TrafficLight.RED) and (distance <= 300.0):
            name = str(self.red_images_counter) + '.jpg'
            cv2.imwrite(os.path.join(self.red_light_dir, "IMG", name), cv_image)
            with open(os.path.join(self.red_light_dir , "data.csv"), "a+") as log:
                log.write("{0},{1}\n".format('IMG/' + name, str(distance)))
            self.red_images_counter += 1


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #cv2.imshow("Image Window", cv_image)
        #cv2.waitKey(1)

        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)


            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
                if d>= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
            
        if closest_light:
            state = self.get_light_state(closest_light)

            return line_wp_idx, state, diff
        
        return -1, TrafficLight.UNKNOWN, None

    
if __name__ == '__main__':
    try:
        DataCollecter()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

