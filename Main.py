#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 28 21:04:00 2017

@author: lukaszpietrasik
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
import pandas as pd
import numpy as np
import csv
from sklearn.metrics import accuracy_score
import argparse
import sys
import tftables
import h5py

# Starting an interactive session
sess = tf.InteractiveSession()

#Load data from 5hdf format
train = pd.read_hdf("data/train.h5", "train")
test = pd.read_hdf("data/test.h5", "test")

#Transforming read data into the matrix form of numpy array
test_matrix = pd.DataFrame.as_matrix(test)
train_matrix = pd.DataFrame.as_matrix(train)

# Initializetion
x = tf.placeholder(tf.float32, shape=[None, 100]) # 100 features
y_ = tf.placeholder(tf.float32, shape=[None, 5]) # 5 output classes: 0,1,2,3,4

#Initialize variables
W = tf.Variable(tf.zeros([100,5]))
b = tf.Variable(tf.zeros([5]))
sess.run(tf.global_variables_initializer())

#Create the function for y
y = tf.matmul(x,W) + b
cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(y, y_))

# Declar training method
train_step = tf.train.GradientDescentOptimizer(0.5).minimize(cross_entropy)

'''
Training - in this section training is made in batches of size n/divider.
Firstly I have found that the smaller batches have given better results but
obviously take longer to train.
'''
# Training patameters
divider = 1259 #max = 1259, another 36
batch_size = 45324/divider

#train neaural networks in batches
for i in range(0,divider):
    train_x = train_matrix[batch_size*i:batch_size*(i+1),1:101].reshape(batch_size,100)
    # tf.one_hot changes integers into an array of 5 binary values: eg. 3 = [0 0 0 1 0]
    train_y = tf.one_hot(
            indices = train_matrix[batch_size*i:batch_size*(i+1),0],
            depth = 5,
            on_value = 1.0,
            off_value = 0.0,
            axis = -1).eval().reshape(batch_size,5)
    print(batch_size*(i+1),'/45324') #print for tracking the progress
    
    train_step.run(feed_dict={x: train_x, y_: train_y})

'''
Prediction
'''
prediction = sess.run(tf.argmax(y, 1), # this gives the most probable class
                      feed_dict={x: test_matrix[:,0:100].reshape(8137,100)})

# Writing the data to file
with open('data/sample.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=",")
    writer.writerow( ('Id', 'y') )
    for i in range(0,prediction.shape[0]):
        index = 45324+i
        writer.writerow( (index, prediction[i]) )