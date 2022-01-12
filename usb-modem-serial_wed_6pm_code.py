#!/usr/bin/env python
# coding: utf-8

# In[32]:


import serial

import time
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np

import re
import pandas as pd
import dask.dataframe as dd
import streamz.dataframe as sdf
# from streamz.dataframe import StreamingDataFrame

# import threading, queue
import multiprocess as mp
# from multiprocessing import Process, Pipe
import signal
import os
from subprocess import check_output

from scipy.ndimage import gaussian_filter1d


# In[9]:


portAddr = "/dev/cu.usbmodem14101"
baud = 115200


# In[10]:


ser = serial.Serial(
    port=portAddr,
    baudrate=baud,
    bytesize=8,
    timeout=2,
    stopbits=serial.STOPBITS_ONE
)
ser.flushInput()


# In[11]:


# plot_window = 20
# y_var = np.array(np.zeros([plot_window]))

# plt.ion()
# fig, ax = plt.subplots()
# line, = ax.plot(y_var)


# In[12]:


def process_serial_out(ascii_string):
    ascii_string += ','
    findall = re.findall(r'(?P<var>.*?):(?P<out>.*?),', ascii_string)
    extracted = {k.strip(): v.strip() for k,v in findall}
    return extracted


# In[ ]:





# In[13]:



def arduino_worker(queue):
    print("worker starting...")
    while True:
        try:
            ser_bytes = ser.readline()
            try:
                print("got", ser_bytes)
                decoded_bytes = ser_bytes.decode('ascii')
                row = process_serial_out(decoded_bytes)
                print(decoded_bytes)
                queue.put(row)
            except:
                print_exc()
                print("exception reading one line from arduino")
                continue
        except:
            print_exc()
            print("Keyboard Interrupt")
            break


# In[14]:


import random

def stride_boundary(row, this_stride):
#     just a dummy code. We need to implement the logic.
    if (bool(random.getrandbits(1))):
        return True
    else:
        return False


# In[34]:


class Stride:
    def __init__(self, df_walk_data, start_idx, stop_idx):
        self.start_idx = start_idx
        self.stop_idx = stop_idx
        self.df_walk_data = df_walk_data

        total_disp, integral1, integral2 = get_displacement_of_walk_segment (df_walk_data, start_idx, stop_idx)
        self.horizontal_length = abs(total_disp)
        self.time_duration = (df_walk_data.time[self.stop_idx] - df_walk_data.time[self.start_idx])/1000
        self.speed = self.horizontal_length/self.time_duration #m/s


def get_displacement_of_walk_segment(df_data, start_idx, end_idx):
    # first integration
    integral_0 = df_data.accel_z[start_idx:end_idx]

    # first integral
    cumsum = 0
    integral_1 = []
    for idx, elem in enumerate(integral_0):
        delta_time = (df_data.time[start_idx+idx+1] - df_data.time[start_idx+idx] ) / 1000
        cumsum += elem * delta_time
        integral_1.append(cumsum)
    
    # second integral
    cumsum = 0
    integral_2 = []
    for idx, elem in enumerate(integral_1):
        delta_time = (df_data.time[start_idx+idx+1] - df_data.time[start_idx+idx] ) / 1000
        cumsum += elem * delta_time
        integral_2.append(cumsum)
    
    # total_displacement, first_integral, second_integral
    return cumsum, integral_1, integral_2

segment_length = 100
temp_segment_compute_limit_start = 0
temp_segment_compute_limit_end = 400
# temp_segment_compute_limit_end = len(readings)
total_walk = []
strides = []

# Algorithm Constants
start_quiet_noise_idx = 0
end_quiet_noise_idx = 100 # 1 second sigma calibration
STRIDE_TIMEOUT = 500
num_of_stddev = 3
gaussian_filter_sigma = 5


# In[ ]:


# scaffolding outside loop 

# def process_calculate_strides(total_data):
#     print("starting calculate_strides")
#     while True:
#     walk_segment = total_data.get()
#     print("the segment: ", walk_segment)


#     for k in walk_segment: total_walk.append(k) 
#         df_walk = pd.DataFrame(total_walk)
#         df_walk.time = df_walk.time - df_walk.time[0] # reset time from start of stream

#         # Filter out accel_z
#         accel_z_smooth = gaussian_filter1d(df_walk.accel_z, gaussian_filter_sigma)

#         # Define Baseline Noise

#         accel_z_std = np.std(df_walk.accel_z[start_quiet_noise_idx:end_quiet_noise_idx], axis=0)
#         accel_z_mean = np.mean(df_walk.accel_z[start_quiet_noise_idx:end_quiet_noise_idx])
#         upper_limit = accel_z_mean + num_of_stddev * accel_z_std
#         lower_limit = accel_z_mean - num_of_stddev * accel_z_std

#         # Determining Stride
#         # Stride Class with auto-compute all chracterisitcs
#         stride_under = False
#         stride_over = False
#         start_stride_idx, end_stride_idx = None, None


# In[ ]:




#worker to process one stride boundary, sort data into buckets
def stride_processor_worker(total_data, raw_stream):
    this_segment = []
    print("stride_processor starting...")
    while True:
        #Infinitely 1. appends to this_segment, then when boundary is met,
        #appends that stride to total_data, reinitializes this_segment
        #and loops forever.
        try:
            row = raw_stream.get()
            print ("decoded: ", row)
            if len(this_segment) > segment_length:
                #put away the current stride in total data as one stride
                total_data.put(this_segment)
                print("finished writing this one stride, size: ", len(this_segment))
                #reinitialize the stride queue
                this_segment = []
            else:
                this_segment.append(row)
        except:
            print_exc()
            print("exception in stride processor worker")
            
    


# In[16]:


# overall structure:
# total_data = [stride1, stride2,,,,,,stride500]
# stride1 = [{r1}, {r2}, {r3}]


# In[17]:


######### start 'main' ########


# In[18]:


pool = mp.Pool()
manager = mp.Manager()
usb_reading_queue = mp.SimpleQueue()
total_data = mp.SimpleQueue()


# In[19]:


#  set up worker threads


# In[20]:


stride_process = mp.Process(target=stride_processor_worker, args=[total_data, usb_reading_queue])


# In[31]:


arduino_proc = mp.Process(target=arduino_worker, args=[usb_reading_queue])


# In[ ]:





# In[ ]:





# In[ ]:





# In[22]:


arduino_proc.start()


# In[23]:


stride_process.start()


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[24]:


# arduino_proc.join()
# stride_process.join()
arduino_proc.terminate()
stride_process.terminate()


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[25]:


total_data


# In[26]:


total_data.empty()


# In[29]:


stride_1 = total_data.get()


# In[30]:


stride_1


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




