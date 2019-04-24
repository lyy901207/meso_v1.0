'''
Created on 2019/02/05

@author: gong
'''
import numpy as np
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

list_s=[]

sigma=10
mu=50

for i in range(0,2000):
    d=driver_Value_Time_Gen(50)
    list_s.append(d)

import matplotlib.pyplot as plt
count, bins, ignored = plt.hist(list_s, 30, density=True)
plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi))*np.exp( - (bins - mu)**2 / (2 * sigma**2) ),linewidth=2, color='r')
plt.show()
