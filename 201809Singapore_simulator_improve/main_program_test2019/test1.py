'''
Created on 2019/02/05

@author: gong
'''
import numpy as np
import math


from c_TimeCalculation.time_calculation import * 
def start_Time_Normal_Distri(start_time_stamp,end_time_stamp,num_of_vehicles):
    
    # this function returns a list of time stamps showing their occurrence time in a normal distribution
    # the mean is the middle value of difference between start_time_stamp and end_time_stamp
    diff_time_stamps=cal_Time(start_time_stamp, end_time_stamp)# in seconds
    mu=int(diff_time_stamps/2.0)
    sigma=int(mu/5.0)
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



start="2019-02-07 07:00:00"
end="2019-02-07 07:29:59"
num=1800

list_time_stamps=start_Time_Normal_Distri(start, end, num)
print list_time_stamps