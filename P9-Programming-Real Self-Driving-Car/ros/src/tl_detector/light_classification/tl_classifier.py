from styx_msgs.msg import TrafficLight
import h5py
from keras.models import load_model
from keras.layers import Flatten, Dense, Lambda, Convolution2D, Dropout, Cropping2D, Activation,Conv2D
import cv2
import tensorflow as tf
from keras.models import Sequential
import os
import rospy

class TLClassifier(object):
    def __init__(self):
        # load classifier
	# Model reconstruction from JSON file
	#with open('./light_classification/model_architecture.json', 'r') as f:
    	   #self.model = model_from_json(f.read())

	# Load weights into the new model
	#self.model = Sequential()
        #self.model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(600, 800, 3)))
    	#self.model.add(Conv2D(24,(5,5),strides = (2,2),padding = "VALID"))
        #self.model.add(Activation('relu'))

        #self.model.add(Conv2D(36,(5,5),strides = (2,2),padding = "VALID"))
        #self.model.add(Activation('relu'))

        #self.model.add(Conv2D(48,(5,5),strides = (2,2),padding = "VALID"))
        #self.model.add(Activation('relu'))
        #self.model.add(Dropout(0.5))
        #self.model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
        #self.model.add(Activation('relu'))
	#self.model.add(Dropout(0.5))
        #self.model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
        #self.model.add(Activation('relu'))
        #self.model.add(Flatten())
        #self.model.add(Dense(100))
        #self.model.add(Dense(50))
        #self.model.add(Dense(10))

        #self.model.add(Dense(1))

	#yaml_file = open('./light_classification/yaml_design.yaml', 'r')
	#self.model = model_from_yaml(yaml_file.read())

	self.model = load_model('./light_classification/model.h5')
	#self.model.load_weights('./light_classification/model_weights6.h5')

    
        self.graph = tf.get_default_graph()


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        with self.graph.as_default():
            light_state = (self.model.predict(image[None,:,:,:], batch_size=1))

            if light_state < -0.5:
		        #rospy.loginfo("UNKNOWN")
                return -1
                
            else:
                #rospy.loginfo("RED")
                return 0
