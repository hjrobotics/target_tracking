
# coding: utf-8

# In[1]:


import sys
sys.path.append('../library')
sys.path.append('../library/rtde')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time
import math
import subprocess
import os


# In[2]:


#logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '192.168.253.1'
ROBOT_PORT = 30004
config_filename = '../config/config_path_reading.xml'

reached_initial_point = False
keep_running = True

dist_thresh = 0.001

logging.getLogger().setLevel(logging.INFO)

wayp_file = open("../data/waypoint.txt", "r")


# In[9]:


conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')
button_names, button_types = conf.get_recipe('button')
is_joint_names, is_joint_types = conf.get_recipe('is_joint')


# In[4]:


con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)
button = con.send_input_setup(button_names, button_types)
is_joint = con.send_input_setup(is_joint_names, is_joint_types)


# In[5]:


def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

def button_to_list(button):
    list = []
    for i in range(0,2):
        list.append(button.__dict__["input_int_register_%i" % (i+1)])
    return list

def list_to_button(button, list):
    for i in range (0,2):
        button.__dict__["input_int_register_%i" % (i+1)] = list[i]
    return button

def is_joint_to_list(is_joint):
    list = []
    for i in range(0,1):
        list.append(is_joint.__dict__["input_int_register_%i" % (i+3)])
    return list

def list_to_is_joint(is_joint, list):
    for i in range (0,1):
        is_joint.__dict__["input_int_register_%i" % (i+3)] = list[i]
    return is_joint


# In[6]:


# Initialization
# setp
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# button
button.input_int_register_1 = 0
button.input_int_register_2 = 0

# is_joint
is_joint.input_int_register_3 = 0
  
# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0


# In[7]:


#start data synchronization
if not con.send_start():
    sys.exit()


# In[8]:


# read initial point from file
target_init_state = wayp_file.readline()
target_init_state = [float(s) for s in target_init_state.split(',')]

# control loop
while not reached_initial_point:
    # receive the current state
    state = con.receive()
    
    # If connection is broken break
    if state is None:
        break;
    
    # distance to target
    rmse = math.sqrt(sum([(t-s)**2 for s, t in zip(state.actual_q, target_init_state)]))
    if rmse < dist_thresh:
        reached_initial_point = True
    
    list_to_setp(setp, target_init_state)
    
    # send new setpoint        
    con.send(setp)
    
    # send is_joint
    new_is_joint = [1]
    list_to_is_joint(is_joint, new_is_joint)
    con.send(is_joint)
    
    # kick watchdog
    con.send(watchdog)

print("initialization finished!!!! :)")
# control loop
while keep_running:
    # receive the current state
    state = con.receive()
    
    # If connection is broken break
    if state is None:
        break;
    
    # read new set point
    new_setp = wayp_file.readline()
    if new_setp == '':
        break;
    new_setp = [float(s) for s in new_setp.split(',')]
    
    list_to_setp(setp, new_setp)
    
    # send new setpoint        
    con.send(setp)
    
    # Get button data
    new_button = [0, 0]
    list_to_button(button, new_button)
    
    # Send button data
    con.send(button)

    
    # send is_joint
    new_is_joint = [0]
    list_to_is_joint(is_joint, new_is_joint)
    con.send(is_joint)
    
    # kick watchdog
    con.send(watchdog)


# In[ ]:


wayp_file.close()
con.send_pause()
con.disconnect()
sys.stdout.flush()

