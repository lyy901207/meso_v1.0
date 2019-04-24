'''
Created on 2019/02/07

@author: gong
'''
import numpy as np
mu, sigma = 0, 1 # mean and standard deviation
s = np.random.normal(mu, sigma, 1800)
print type(s)
print sum(s)
print np.mean(s),np.std(s)
print s