import csv
import cv2
import numpy as np
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Convolution2D, Dropout, Cropping2D, Activation,Conv2D
from keras.optimizers import Adam
import sklearn
from sklearn.model_selection import train_test_split
import os
from math import ceil
from random import shuffle
import h5py
import pandas as pd
from keras.models import model_from_json


# Here is the directory of the ready made data
#required_data_folder = 'RedLightDistanceData'
required_data_folder = 'set2'
current_dir = os.getcwd()



filesDir = os.path.join(current_dir, required_data_folder+'/LightStatusData')

# Here we can estract information from the csv file to prepare the images and measurments.
samples = []


with open(os.path.join(filesDir, 'data.csv')) as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = os.path.join(filesDir, 'IMG', batch_sample[0].split('/')[-1])
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[1])
                images.append(center_image)
                angles.append(center_angle)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)


# This is Nvidia model after adding some other layers as the normalization layer, 
# cropping layer and drop-out layer
def cnn_nividia(input_shape=(600, 800, 3)):

    
    model = Sequential()
    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=input_shape))
    #model.add(Cropping2D(cropping=( (70,25), (0,0) ) ))
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), activation="relu"))
    model.add(Convolution2D(64, 3, 3, activation="relu"))
    model.add(Convolution2D(64, 3, 3, activation="relu"))
    model.add(Flatten())
    model.add(Dropout(0.5))
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))
#    model = Sequential()
#    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=(600, 800, 3)))
#    model.add(Conv2D(24,(5,5),strides = (2,2),padding = "VALID"))
#    model.add(Activation('relu'))
#
#    model.add(Conv2D(36,(5,5),strides = (2,2),padding = "VALID"))
#    model.add(Activation('relu'))
#
#    model.add(Conv2D(48,(5,5),strides = (2,2),padding = "VALID"))
#    model.add(Activation('relu'))
#    model.add(Dropout(0.5))
#    model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
#    model.add(Activation('relu'))
#    model.add(Dropout(0.5))
#    model.add(Conv2D(64,(3,3),strides = (2,2),padding = "SAME"))
#    model.add(Activation('relu'))
#    model.add(Flatten())
#    model.add(Dense(100))
#    model.add(Dense(50))
#    model.add(Dense(10))
#
#    model.add(Dense(1))

    return model


batch_size = 64


epochs = 5


train_samples, validation_samples = train_test_split(samples, test_size=0.2)
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

# Create model using Nvidia model function
model = cnn_nividia()
# Use Adam optimizer with learning rate of e-3
adam = Adam(lr=0.001)
model.compile(optimizer=adam, loss='mse')
model.summary()

#X_train, X_test, y_train, y_test = train_test_split(X_train, y_train, test_size = 0.10, random_state = 100)
#model.fit(x=X_train, y=y_train, epochs=epochs, batch_size=batch_size, validation_split=0.2, shuffle=True)
model.fit_generator(train_generator, steps_per_epoch=ceil(len(train_samples)/batch_size), 
    validation_data=validation_generator, validation_steps=ceil(len(validation_samples)/batch_size), 
    epochs=epochs, verbose=1)


#model.save_weights('model_weights6.h5')
model.save('model.h5')
#yaml_model = model.to_yaml()
#yaml_file = open('yaml_design.yaml', 'w')
#yaml_file.write(yaml_model)

