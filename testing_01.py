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
from multiprocessing import Process, Queue
import multiprocessing as mp




portAddr = "/dev/cu.usbmodem14101"
baud = 115200


# In[117]:


ser = serial.Serial(
    port=portAddr,
    baudrate=baud,
    bytesize=8,
    timeout=2,
    stopbits=serial.STOPBITS_ONE
)
ser.flushInput()

# In[138]:



def arduino_worker(queue):
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
            print("Keyboard Interrupt")
            break


# In[141]:


pool = mp.Pool()
manager = mp.Manager()
usb_reading_queue = mp.SimpleQueue()
total_data = mp.SimpleQueue()


usb_reading_queue.put("hi")
usb_reading_queue.empty()
x = usb_reading_queue.get()
x
# arduino_res = pool.apply_async(arduino_worker, (usb_reading_queue))
