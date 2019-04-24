'''
Created on 2018/08/03

@author: gong

This module contains the time related functions used in the simulator.
The input and output time-stamps are all string type; the input stamps are converted to datetime type firstly.
the output stamps are converted to string types before returned.
'''
import time,datetime


def get_Pre_Timestamp(cur_time,timestep):
    # this function returns the previous time stamp which is "timestep" less than current time stamp
    # cur_time is the current time stamp, string type
    cur_time=datetime.datetime.strptime(cur_time,"%Y-%m-%d %H:%M:%S")# change the type to datetime from string    
    timestep=float(timestep)    
    timestep=datetime.timedelta(seconds=timestep)
    pre_timestamp=cur_time-timestep
    pre_timestamp=str(pre_timestamp)[:19]
    return pre_timestamp# output is string type in format of yyyy-mm-dd hh:mm:ss
def get_Nex_Timestamp(cur_time,timestep):
    # this function returns the next time stamp which is "timestep" more than current time stamp
    # cur_time is the current time stamp, string type
    cur_time=datetime.datetime.strptime(cur_time,"%Y-%m-%d %H:%M:%S")# change the type to datetime from string    
    timestep=float(timestep)    
    timestep=datetime.timedelta(seconds=timestep)
    nex_timestamp=cur_time+timestep
    nex_timestamp=str(nex_timestamp)[:19]
    return nex_timestamp# output is string type in format of yyyy-mm-dd hh:mm:ss
def cal_Time(date1,date2): #This function used to calculate time difference between two time stamp, and result are in seconds
    # input date should be string type
    date1=time.strptime(date1,"%Y-%m-%d %H:%M:%S")
    date2=time.strptime(date2,"%Y-%m-%d %H:%M:%S")
    date1=datetime.datetime(date1[0],date1[1],date1[2],date1[3],date1[4],date1[5])
    date2=datetime.datetime(date2[0],date2[1],date2[2],date2[3],date2[4],date2[5])
    return ((date2-date1).days*86400+(date2-date1).seconds)# in seconds
def add_Timestamp(cur_time,timestep):
    # this function returns the a new time stamp which is "timestep" more than current time stamp
    # cur_time is the current time stamp, in the format of yyyy-mm-dd hh:mm:ss
    # timestep is in seconds
    # the returned output is also in the format of 'yyyy-mm-dd hh:mm:ss' in the string type
    cur_time=datetime.datetime.strptime(cur_time,"%Y-%m-%d %H:%M:%S")
    timestep=float(timestep)
    timestep=datetime.timedelta(seconds=timestep)
    new_timestamp=cur_time+timestep
    new_timestamp=str(new_timestamp)[:19]
    return new_timestamp# the output is string type

import numpy as np
import math

def start_Time_Normal_Distri(*para):
    # this function returns a list of time stamps showing their occurrence time in a normal distribution
    # the mean is the middle value of difference between start_time_stamp and end_time_stamp
    if len(para)==3:
        start_time_stamp=para[0]
        end_time_stamp  =para[1]
        num_of_vehicles =para[2]
        
        diff_time_stamps=cal_Time(start_time_stamp, end_time_stamp)# in seconds
        mu              =int(diff_time_stamps/2.0)
        sigma           =int(mu/5.0)
    elif len(para)==4:
        start_time_stamp=para[0]
        end_time_stamp  =para[1]
        num_of_vehicles =para[2]
        
        mu              =para[3]
        sigma=int(mu/5.0)
    elif len(para)==5:
        start_time_stamp=para[0]
        end_time_stamp  =para[1]
        num_of_vehicles =para[2]
        mu              =para[3]
        sigma           =para[4]
    
    # set seeds
    np.random.seed(10)
    s=np.random.normal(mu,sigma,num_of_vehicles)
    list_time_stamps=[]# initialize the output list of time stamps
    #convert items in s to int
    for i in range(0,len(s)):
        s[i]=int(math.ceil(s[i]))
        if s[i]<=0:
            s[i]=1
        timestamp=get_Nex_Timestamp(start_time_stamp, s[i])
        list_time_stamps.append(timestamp)
    list_time_stamps.sort()
    return list_time_stamps


'''
# set the default of time step
timestep=1# unit is seconds

nowtime=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
print nowtime,type(nowtime)
print 'now change to datetime type'
nowtime=datetime.datetime.strptime(nowtime,'%Y-%m-%d %H:%M:%S')
print nowtime,type(nowtime)
date2="2018-08-07 11:31:40"
print date2[:19]

#print Caltime(nowtime, date2),"OK,checked"
print add_timestamp(str(nowtime), timestep)
'''
