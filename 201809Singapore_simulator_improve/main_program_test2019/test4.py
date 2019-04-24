'''
Created on 2019/02/07

@author: gong
'''
import numpy as np
np.random.seed(10)

def driver_Value_Time_Gen(median_value_time):
    
    # this function generates the value-of-time of the driver
    # a normal distribution is used to randomly generate the value-of-time for each driver
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
    #   further complicated function for predicting value of time can be done   #
    # e.g. value of time decided by the income
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#    
    mu=median_value_time
    sigma=mu/5.0    
    s = np.random.normal(mu, sigma, 1)    
    return s[0] 

for i in range(0,10):
    print driver_Value_Time_Gen(50)
    

