'''
Created on 2018/07/09

@author: gong
'''
import csv,random


def cal_Dist(p1,p2):
    # the input two points are nodes defined with same features in node class
    # p1, p2 are two dictionaries which have keys of coor-X and coor-Y
    return(((p1['coor-X']-p2['coor-X'])**2+ (p1['coor-Y']-p2['coor-Y'])**2)**(0.5))   

class Node(object):
    def __init__(self,line):
        #record=line.split(",")
        record=line
        self.nodeid=str(record[0])# node ID
        self.nodetype=int(record[1])# node type: centroid, signalized intersection(long duration or short duration, unsignalized intersection;currently only centroid or not is used
        self.coortype=int(record[2])# coordinate system type, either Cartesian or geographic, the later needs to be transformed to the former for calculation
        self.coorX=int(record[3])# X coordinate, can be transformed from longitude-latitude
        self.coorY=int(record[4])# Y coordinate, can be transformed from longitude-latitude       
        self.delay=float(record[5])# preset average delay at this intersection. current file uses 1.5 sec as preset value



                    
    
    