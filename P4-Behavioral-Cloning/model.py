# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 12:10:26 2019

@author: msamy
"""

import csv
import cv2
import numpy as np
from keras.models import Sequential,Model
from keras.layers import Flatten,Dense,Cropping2D,Lambda,Conv2D,Dropout,Activation
from sklearn.model_selection import train_test_split
import sklearn
import math
from sklearn.utils import shuffle
import matplotlib.pyplot as plt
import random


#all training data
X_data = []
y_data = []

visualize_data = False


def show_images(images, cols, titles):
    """Display a list of images in a single figure with matplotlib.
    
    Parameters
    ---------
    images: List of np.arrays compatible with plt.imshow.
    
    cols (Default = 1): Number of columns in figure (number of rows is 
                        set to np.ceil(n_images/float(cols))).
    
    titles: List of titles corresponding to each image. Must have
            the same length as titles.
    """
    assert((titles is None)or (len(images) == len(titles)))
    n_images = len(images)
    
    
    fig = plt.figure()
    for n, (image, title) in enumerate(zip(images, titles)):
        a = fig.add_subplot(cols, np.ceil(n_images/float(cols)), n + 1)
        plt.imshow(image)
        a.set_title(title)
    fig.set_size_inches(np.array(fig.get_size_inches()) * n_images)
    plt.show()



def data_ganerator(samples,batch_size = 32):
        
    num_samples = len(samples)
    
    while 1:
        shuffle(samples)
        for offset in range(0,num_samples,batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            measurements = []
                
            for batch_sample in batch_samples:
                    images.append(batch_sample[0])
                    measurements.append(batch_sample[1])
                    
                    yield shuffle(np.array(images), np.array(measurements))
    

            
            

def prepare_training_data():

    global X_data,y_data,visualize_data

    
    correction = 0.3
    
    print("preparing data for trainning \n")
    lines = []
    with open("./../../../opt/data/driving_log.csv") as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            lines.append(line)
    
    #remove labels
    lines = lines[1:]
    
    

    if visualize_data is True:
        random_num = np.random.randint(len(lines))
        img_num = 0
    
        dataset_example_images = []
        dataset_example_titles = []
        mirror_example_images = []
        mirror_example_titles = []
    
    
    

    random_num = random.randint(1,len(lines))
    
  
    line_no = 0

    
    for line in lines:
         
        
        for i in range(3):
            source_path = line[i]
            file_name = source_path.split("/")[-1]
            current_path = "./../../../opt/data/IMG/" + file_name
        
            image = cv2.cvtColor(cv2.imread(current_path),cv2.COLOR_BGR2RGB)
            steering_angle = float(line[3])

            
            
            if (visualize_data is True) and (line_no == random_num):
                dataset_example_images.append(image)
                
                if i == 0:
                    dataset_example_titles.append("center_view")
                elif i == 1:
                    dataset_example_titles.append("left_view")
                    
                    mirror_example_images.append(image)
                    mirror_example_titles.append("left_view")
                    
                    left_flipped_image = cv2.flip(image,1)
                    mirror_example_images.append(left_flipped_image)
                    mirror_example_titles.append("flipped_left_view")
                    
                    
                else:
                    dataset_example_titles.append("right_view")
            
            
            #if it's left image
            if(i == 1):
                steering_angle += correction
                    
                
            elif(i == 2):
                steering_angle -= correction
                
            X_data.append(image)
            y_data.append(steering_angle)
            
        
            #flip image horizontally
            X_data.append(cv2.flip(image,1))
            y_data.append(steering_angle*-1)
            

            
            
        
        line_no = line_no + 1

    if visualize_data is True:
        print("total no of examples is ",len(X_data))
        training_exapmles = int(0.8* len(X_data))
        print("no of training examples is ",training_exapmles)
        print("no of testing examples is ",len(X_data) - training_exapmles)
        print("image input shape",np.shape(X_data[0]))
        show_images(dataset_example_images,1,dataset_example_titles)
        show_images(mirror_example_images,1,mirror_example_titles)
        
        
    


    
def train_network(network_id = 0):

    
    global X_data,y_data
    
    print("start trainning  \n")
    
    
    #setup the model
    model = Sequential()
    
    #normalize images using Lambda
    model.add(Lambda(lambda d: (d/255.0) - 0.5,input_shape=(160,320,3) ))
    
    #crop unwanted top an bottom sections fo clear images
    #74 rows pixels from the top of the image
    #20 rows pixels from the bottom of the image
    #60 columns of pixels from the left of the image
    #60 columns of pixels from the right of the image
    model.add(Cropping2D(cropping = ((74,20), (60,60)),input_shape=(160, 320, 3)))
    
    
    #define netwrok architecture here
    if network_id == 0:    
       
    
        #add the images to the netowks
        model.add(Flatten())
        
        
    elif network_id == 1:
        model.add(Conv2D(24,(5,5),strides = (2,2),padding = "VALID"))
        model.add(Activation('relu'))
        model.add(Dropout(0.5))
        model.add(Conv2D(36,(5,5),strides = (2,2),padding = "VALID"))
        model.add(Activation('relu'))
        model.add(Dropout(0.5))
        model.add(Conv2D(48,(5,5),strides = (2,2),padding = "VALID"))
        model.add(Activation('relu'))
        model.add(Dropout(0.5))
        model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
        model.add(Activation('relu'))
        model.add(Dropout(0.5))
        model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
        model.add(Activation('relu'))
        model.add(Flatten())
        model.add(Dense(100))
        model.add(Dense(50))
        model.add(Dense(10))

    model.add(Dense(1))
    
    #prepare Training,validation data
    batch_szie = 32
    samples = list(zip(X_data, y_data))
    
    training_samples,validation_samples = train_test_split(np.array(samples),test_size = 0.2)
    
    no_of_training_batches = math.ceil(len(training_samples)/batch_szie)
    no_of_validation_batches = math.ceil(len(validation_samples)/batch_szie)
    
    training_samples_gen = data_ganerator(training_samples,batch_szie)
    validation_samples_gen = data_ganerator(validation_samples,batch_szie)


    #mean square error because we are using regression
    model.compile(loss = 'mse',optimizer = 'adam')
    
    model.fit_generator(training_samples_gen,
                        steps_per_epoch = no_of_training_batches,
                        validation_data = validation_samples_gen,
                        validation_steps = no_of_validation_batches,
                        epochs = 5,verbose = 1)
                        
    
    print("trainning is done  \n")

    model.save('./model.h5')

def pipeline():
    

    prepare_training_data()

    network_id = 1
    train_network(network_id)

pipeline()