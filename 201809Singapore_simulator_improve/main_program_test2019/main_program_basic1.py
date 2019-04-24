'''
Last tested on 2019/01/30

@author: gong

This test is to check the performance of lane-based toll-collection system.
Bus can change to a faster lane without external cost. 
Private car changes to a faster lane depending on the value-time, driver's preference type.

OD matrices are increasing from 10% to 100% with a step of 10% on both bus and private car.

output:
1) travel time from each OD pair at each simulation step
   average travel time for bus and private vehicles by OD pair
2) composition of private car's decision : changed lane, not changed lane, in whole simulation period and every 5 minutes
3) density of each lane at each simulation step
4) start-time,end-time,saved-time if changing the lane of each vehicle

revised before 2019/2/20, added the initial selection of a fast-lane system.
'''
import time,datetime
import copy,math
import random
import numpy as np
import csv

from a_ReadRoadNetwork.readgeometry import *
from b_ODPackage.class_vehicle import *
from c_TimeCalculation.time_calculation import *

from b_ODPackage.ReadOD import *
from h_OtherFunctions.dictionary_cal import *
from z_ExistingPackages.Dijkstra4 import *


# set the rule of the road, can be either "Left" or "Right"
rule_of_road="Left"

# set the location of the output results
#%%%%%%%%%%%%%%%%%%%%%for checking the simulation logs
outfile              =open(r"D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_process_during_simulation.csv","ab")
writer=csv.writer(outfile)#%%%%%%%%%%%%%%%%%%%%%for checking the results
#%%%%%%%%%%%%%%%%%%%%%for storing the status of lanes and vehicles
outfile_lane_features=open(r'D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_lane_features.csv',"ab")
writer_lane_features=csv.writer(outfile_lane_features)

outfile_veh_features =open(r'D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_veh_features.csv',"ab")
writer_veh_features=csv.writer(outfile_veh_features)

outfile_OD_features =open(r'D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_OD_features.csv',"ab")
writer_OD_features=csv.writer(outfile_OD_features)

outfile_statistic1   =open(r'D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_statistics_along_timestamps.csv',"ab")
writer_statistic_along_timestamps=csv.writer(outfile_statistic1)

outfile_statistic2   =open(r'D:\03Work during PD\31ERP2_in_COI\meso_by_python\results_201901\outfile_statistics_whole_simulation.csv',"ab")
writer_statistic_whole_simulation=csv.writer(outfile_statistic2)
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


def check_Position_NotInUsage(p0,p1,p2):
    # this function checks the distance between point p0 to the line (connected from p1 to p2).
    # it is used to check whether the vehicle (update location is p0) is following the lane (line from p1 to p2)
    # if the distance is more than a threshold, it means the vehicle did not follow the lane and some error exists in the locate function.
    # input three points are in class of node {'coor-X':coor_X,'coor-Y':coor_Y,....}
    dummy_result=0
    
    return dummy_result

def locate_no_Delay(veh_id,curr_timestamp):    
    # time_stamp is the current time, yyyy-mm-dd hh:mm:ss
    # this function update a vehicle's location at current time in case of no delay at intersections (i.e. nodes)
    # in case of receiving continuously instant GPS signal, it returns the map-matched coordinates(matched to the central line of lane (not included in this function yet). 
    # in case of a simulated circumstance, the current location is calculated based on the location of previous time stamp and the speed on related links. 
    pre_timestamp=get_Pre_Timestamp(curr_timestamp, timestep)# previous time stamp
    dummy_vanishing_veh=0# an indicator to show at current time stamp whether this vehicle will arrive its destination or not: 0, has not arrive; 1, has arrived.
    #print "veh-id, current time, previous time", veh_id,time_stamp,pre_timestamp#****************for checking only
    #print "list-timestamps:", dic_vehicles[veh_id]['list-timestamps']#****************for checking only
    #print "locations are:", dic_vehicles[veh_id]['dic-locations']#****************for checking only
    #print "routes are:", dic_vehicles[veh_id]['dic-routes']#****************for checking only
    
    pre_location=dic_vehicles[veh_id]['dic-locations'][pre_timestamp]# location at previous stamp
    pre_route=dic_vehicles[veh_id]['dic-routes'][pre_timestamp]# best route at previous stamp,lane-based routes
    
    # no output of this function; it just update the feature of the vehicle
    curr_location={}# initialize the location at current time stamp
        
    ###---------- calculate the location at current time stamp ------------###
    lane_id=pre_location['lane-id']    
    speed_lane=dic_lanes[lane_id]['speed']
    speed_veh=dic_vehicles[veh_id]['max-speed']
    speed=min([speed_lane,speed_veh])
    #nodeid1=dic_lanes[lane_id]['node-id1']# node1 of the lane focused on
    nodeid2=dic_lanes[lane_id]['node-id2']# node2 of the lane focused on
    dist_to_id2=cal_Dist(pre_location,dic_nodes[nodeid2])/1000.0# in km, make sure each item has a dictionary structure containing keys of "coor-X" and "coor-Y"
    time_to_id2=dist_to_id2*1.0/float(speed)*3600.0
    time_to_travel=copy.deepcopy(timestep)# the time that vehicle can move
    while time_to_travel>=time_to_id2:## check until vehicle cannot reach the end of a lane on the best path.
        if nodeid2==dic_vehicles[veh_id]['destination']:# this vehicle arrives its destination
            coor_X=dic_nodes[nodeid2]['coor-X']
            coor_Y=dic_nodes[nodeid2]['coor-Y']
            list_van_vehids.append(veh_id)# put its id into list of vanishing veh-ids
            dic_vehicles[veh_id]['end-timestamp']=curr_timestamp
            dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=speed
            # update its location to id2
            curr_location['coor-X']=coor_X
            curr_location['coor-Y']=coor_Y
            curr_location['lane-id']='' 
            dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=curr_location
            dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=''
            dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
            dummy_vanishing_veh=1
            
            dic_vehicles[veh_id]['lane-type']=""
            dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=""            
            break
        else:
            # if true, update lane to the next along the best route at previous time stamp;update the location of vehicle to the start node of updated lane (or end node of previous lane)                     
            time_to_travel=time_to_travel-time_to_id2
            # update lane to the next lane of current lane along the best route at the previous time stamp
            lane_id=pre_route[lane_id]
            dic_vehicles[veh_id]['list-laneid-used'].append(lane_id)
            # update the location of vehicle to the start node of next lane (end node of previous lane); then use updated pre_location to re-calculate the time_to_id2
            pre_location={'coor-X':dic_nodes[nodeid2]['coor-X'],'coor-Y':dic_nodes[nodeid2]['coor-Y'],'lane-id':lane_id}
            #nodeid1=dic_lanes[lane_id]['node-id1']
            nodeid2=dic_lanes[lane_id]['node-id2']
            speed=dic_lanes[lane_id]['speed']
            dist_to_id2=cal_Dist(pre_location,dic_nodes[nodeid2])/1000.0
            time_to_id2=dist_to_id2*1.0/float(speed)*3600             
    if dummy_vanishing_veh==0:
        # now veh-id cannot arrive node-id2 at the end of this simulation step        
        dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=speed#!!!!!!!!!!if the time step is long enough for vehicle to pass several links, this is the speed of the final link        
        #if speed<=1:
            #speed=1# set the min speed for traveling to avoid dead congestion
        X1=pre_location['coor-X']# coordinate X of previous location
        Y1=pre_location['coor-Y']# coordinate Y of previous location
        X2=dic_nodes[nodeid2]['coor-X']# coordinate X of end node of focused lane
        Y2=dic_nodes[nodeid2]['coor-Y']# coordinate Y of end node of focused lane
        coor_X=X1+(X2-X1)*time_to_travel/3600.0*speed*1000.0/cal_Dist(dic_nodes[nodeid2],pre_location)# in meter, coordinate X of current position of vehicle
        coor_Y=Y1+(Y2-Y1)*time_to_travel/3600.0*speed*1000.0/cal_Dist(dic_nodes[nodeid2],pre_location)# in meter, coordinate Y of current position of vehicle
        #cur_location={'coor-X':coor_X,'coor-Y':coor_Y,'lane-id':lane_id}
        curr_location['coor-X']=coor_X
        curr_location['coor-Y']=coor_Y
        curr_location['lane-id']=lane_id    
        # update the dictionaries of location and list of time stamps
        dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
        dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=curr_location
        ###------------- best route is updated again if it decides to change lanes -----------------###
        # best route of current time stamp is set as the same as previous time stamp
        dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=copy.deepcopy(pre_route)
        veh_lane_type=copy.deepcopy(dic_vehicles[veh_id]['dic-lane-type'][pre_timestamp])
        dic_vehicles[veh_id]['lane-type']=veh_lane_type
        dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=veh_lane_type
    return ()
# strategy of generating delay at the intersection;
# currently 4 strategies are included in the module, as follows

def delay_by_Vol_Simple(nodeid,pre_timestamp,aver_delay):
    # this function returns the delay at previous timestamp of current time at intersection of nodeid \
    # \by calculating the real-time delay with the traffic volume at the links connected by the intersection at the same time stamp.
    # this function assumes that vehicles in the links connected by the nodeid have same delay.
    # the delay is depending on the total number of vehicles and the preset parameter of aver_delay
    # aver_delay is the average delay due to whole number of vehicles, which has to be calibrated.
      
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
    #   further complicated function for calculating delay can be done          #
    # e.g. dynamic aver_delay depending on the total number of vehicles in nodeid  
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#   
   
    # initialize the lists or variables used in delay calculation in this function    
    list_lanes_in_intersection=[]   # the list, storing number of lanes coming into this intersection
    list_lanes_out_intersection=[]  # the list, storing number of lanes coming out of this intersection    
    list_vehs_in_intersection=[]    # the list, storing number of vehicles on the lanes coming into this intersection
    list_vehs_out_intersection=[]   # the list, storing number of vehicles on the lanes coming out of this intersection
    list_linkids=[]                 # the list of link-ids one nodeid of whose is nodeid   
    for laneid in dic_lanes:
        if dic_lanes[laneid]['node-id1']==nodeid:# this lane starting from nodeid
            list_lanes_out_intersection.append(laneid)
            if not dic_lanes[laneid]['link-id'] in list_linkids:
                list_linkids.append(dic_lanes[laneid]['link-id'])
            list_vehs_out_intersection.append(float(dic_lanes[laneid]['dic-counts'][pre_timestamp]))
        if dic_lanes[laneid]['node-id2']==nodeid:# this lane ending at nodeid
            list_lanes_in_intersection.append(laneid)
            if not dic_lanes[laneid]['link-id'] in list_linkids:
                list_linkids.append(dic_lanes[laneid]['link-id'])
            list_vehs_in_intersection.append(float(dic_lanes[laneid]['dic-counts'][pre_timestamp]))
    count_lanes_in_intersection =len(list_lanes_in_intersection)
    count_lanes_out_intersection=len(list_lanes_out_intersection)
    count_legs_intersection     =len(list_linkids)# the total number of legs of this intersection
    count_vehs_in_intersection  =sum(list_vehs_in_intersection)
    count_vehs_out_intersection =sum(list_vehs_out_intersection)
    aver_veh=np.mean([count_vehs_in_intersection/count_lanes_in_intersection,count_vehs_out_intersection/count_lanes_out_intersection])
    delay=aver_delay*aver_veh
    return delay 

def delay_by_Vol_in_a_Dist(nodeid,pre_timestamp,delay_at_least,delay_at_most,effect_distance):
    # this function returns the delay at a node (intersection) at curr_timestamp
    # it assumes a linear relationship between average delay and the number of vehicles
    # minimum and maximum delay exist in this method
    
    ################### set default values used in the linear calculation of delay  ##########################
    count_bottom=5 # minimum number of count of vehicles (in pcu) which has the least delay (i.e. delay_bottom)
    count_top=20   # maximum number of count of vehicles (in pcu) which has the largest delay (i.e. delay_top)
    delay_bottom=delay_at_least # the least delay
    delay_top=delay_at_most   # the largest delay
    effect_dist=effect_distance # the distance in which a vehicle to a node will influence the delay
    ##########################################################################################################
    delay=0
    # the delay is dependent on the traffic volume (i.e. vehicles locating less than 50 meters to the node) coming into the intersection.
    # vehicles arriving the intersection only.
    # only vehicles in a distance less than 50 meters to the node
    # simple linear function used, minimum delay is 1 seconds, maximum delay is 10 seconds, which have to adjusted due to the network
    count=0# initialize the vehicles in the nearby area of intersection as 0
    for veh_id in list_running_vehids:##################### make sure the vehicles have to be in the running vehicle list
        if dic_vehicles[veh_id]['dic-routes'][pre_timestamp]<>'':# the vehicle has not arrived its destination
            curr_location=dic_vehicles[veh_id]['dic-locations'][pre_timestamp]
            lane_id=curr_location['lane-id']
            if dic_lanes[lane_id]['node-id2']==nodeid:# the vehicle is moving to the intersection of nodeid
                dist=cal_Dist(curr_location,dic_nodes[nodeid])#coordinates are in meter, which means the distance is also in meter
                if dist<=effect_dist:
                    count+=1
       
    if count<=count_bottom:# less than 5 pcu
        delay=delay_bottom# in seconds
    elif count>=count_top: #more than 20 pcu
        delay=delay_top# in seconds
    else:
        delay=int(delay_bottom+1.0*(delay_top-delay_bottom)*(count-count_bottom)/(count_top-count_bottom))
    return delay
def delay_by_Random(delay_at_least,delay_at_most):
    # this function returns an average delay of a vehicle as a random value in an interval between delay_at_least and delay_at_most
    delay=random.randint(delay_at_least,delay_at_most)# in seconds, integer
    return delay

def update_Node_delay(strategy,curr_timestamp,aver_delay=0.05,delay_at_least=0,delay_at_most=3,effect_distance=50):
    # this function is used to update the delay at each node (intersection) if a strategy of dynamic delay is selected.
    # delay is a feature of a node and it is updated due to different delay strategy.
    # if a delay at a node is necessary in the algorithm, just visit the feature of delay at a node. 
    # a defect of using this function is updating and calculating the delay at each simulation step
    pre_timestamp=get_Pre_Timestamp(curr_timestamp, timestep)
    for node_id in dic_nodes:
        if strategy=="vol-simple":
            delay=delay_by_Vol_Simple(node_id,pre_timestamp,aver_delay)
        elif strategy=="vol-dist":
            delay=delay_by_Vol_in_a_Dist(node_id, pre_timestamp, delay_at_least,delay_at_most,effect_distance)
        elif strategy=="random":
            delay=delay_by_Random(delay_at_least,delay_at_most)
        dic_nodes[node_id]['delay']=delay
    
    return()
def locate_with_Delay(veh_id,curr_timestamp):
    # checked on 2018/9/30 for queue propagation and dissipation
    # check confirms that previous location function can include the queue propagation and dissipation
    # the queue issue was solved by when the density inside the lane reaches its jam-density, the locate function stops working and make the vehicle in its previous location.
    # if the speed is less than 0.001 km/h, the vehicle will stay at previous location, neither moving forward or stepping to next link.
    
    # improved on 2019/1/4 by adding a new feature of vehicle show whether this vehicle is currently being stopped by the intersection delay.
    # if so, the vehicle will update its location to its previous location
    # last edited on 2019/1/20    
    
    pre_timestamp=get_Pre_Timestamp(curr_timestamp, timestep)# previous time stamp
    dummy_vanishing_veh=0# an indicator to show at current time stamp whether this vehicle will arrive its destination or not: 0, did not arrive; 1, arrived.
    
    pre_location=dic_vehicles[veh_id]['dic-locations'][pre_timestamp]# location at previous stamp
    pre_route=dic_vehicles[veh_id]['dic-routes'][pre_timestamp]# best route at previous stamp,lane-based routes
    
    # no output of this function; it just update the feature of vehicles
    curr_location={}# initialize the location at current time stamp
        
    ###---------- calculate the location at current time stamp ------------###
    lane_id=pre_location['lane-id']    
    speed_lane=dic_lanes[lane_id]['speed']
    speed_veh=dic_vehicles[veh_id]['max-speed']
    speed=min([speed_lane,speed_veh])
    #nodeid1=dic_lanes[lane_id]['node-id1']# node1 of the lane focused on
    nodeid2=dic_lanes[lane_id]['node-id2']# node2 of the lane focused on
    dist_to_id2=cal_Dist(pre_location,dic_nodes[nodeid2])/1000.0# in km, make sure each item has a dictionary structure containing keys of "coor-X" and "coor-Y"
    time_to_id2=dist_to_id2*1.0/float(speed)*3600.0
    time_to_travel=copy.deepcopy(timestep)# the time that vehicle can move
    
    while time_to_travel>=time_to_id2:## check until vehicle cannot reach the end of a lane on the best path. 
        if nodeid2==dic_vehicles[veh_id]['destination']:# this vehicle arrives its destination
            coor_X=dic_nodes[nodeid2]['coor-X']
            coor_Y=dic_nodes[nodeid2]['coor-Y']
            
            dic_vehicles[veh_id]['end-timestamp']=curr_timestamp
            dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=speed
            # update its location to id2
            curr_location['coor-X']=coor_X
            curr_location['coor-Y']=coor_Y
            curr_location['lane-id']='D' # this vehicle has arrived its destination
            dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=curr_location
            dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=''
            dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
            dummy_vanishing_veh=1

            dic_vehicles[veh_id]['lane-type']=""
            dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=""               
            
            break
        else:# the vehicle-moving-forward node of current lane is not its destination
            # if true, update lane to the next lane along the best route at previous time stamp;update the location of vehicle to the start node of updated lane (or end node of previous lane)
            # checked on 2018/10/1 and confirmed that code before Sept. 2018 can reflect the queue propagation and dissipation.
             
            # On 2019/1/20, added the intersection delay.
            # intersection delay is added by the function of Delay.
            # if delay is more than time_to_travel, the vehicle needs to stop at the intersection after arriving it \
            # \ and not move forward until the delay finishes; the location will keep the same as the intersection.
            # two new features were added to vehicle in order to accommodate the delay
            delay=dic_nodes[nodeid2]['delay'] #the delay at the intersection of nodeid2 at pre_timestamp
            time_to_travel=time_to_travel-time_to_id2-delay# unit, all in sec
            lane_id=pre_route[lane_id]# update lane to the next lane of current lane along the best route at the previous time stamp
            dic_vehicles[veh_id]['list-laneid-used'].append(lane_id)
            pre_location={'coor-X':dic_nodes[nodeid2]['coor-X'],'coor-Y':dic_nodes[nodeid2]['coor-Y'],'lane-id':lane_id}# update the location of vehicle to the start node of next lane (end node of previous lane)
            if time_to_travel>=0:# not delayed at node-id2
                #nodeid1=dic_lanes[lane_id]['node-id1']
                nodeid2=dic_lanes[lane_id]['node-id2']
                speed_lane=dic_lanes[lane_id]['speed']
                speed_veh=dic_vehicles[veh_id]['max-speed']
                speed=min([speed_lane,speed_veh])
                dist_to_id2=cal_Dist(pre_location,dic_nodes[nodeid2])/1000.0
                time_to_id2=dist_to_id2*1.0/float(speed)*3600    
            else:# delayed at node-id2
                dic_vehicles[veh_id]['delay-status']=1# turn the vehicle delay-status to be positive, meaning this vehicle is being delayed at this intersection
                dic_vehicles[veh_id]['delay-time-until']=get_Nex_Timestamp(curr_timestamp,math.ceil(abs(time_to_travel)))# set the time until when the vehicle is delayed at this intersection
                
                # if delayed, the newly updated pre_location will be the vehicle's current location
                dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=copy.deepcopy(pre_location)
                dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
                dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=copy.deepcopy(pre_route)
                dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=copy.deepcopy(speed)
                
                veh_lane_type=copy.deepcopy(dic_vehicles[veh_id]['dic-lane-type'][pre_timestamp])
                dic_vehicles[veh_id]['lane-type']=veh_lane_type
                dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=veh_lane_type                
                break
    if dummy_vanishing_veh==0 and dic_vehicles[veh_id]['delay-status']<>1:# make sure this vehicle has not arrived its destination and not being delayed in an intersection 
        # now veh-id cannot arrive node-id2 at the end of this simulation step; update its location (it cannot arrive at the end node of current lane, either).        
        #dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=speed#!!!!!!!!!!if the time step is long enough for vehicle to pass several links, this is the speed of the final link        
        #if speed<=1:
            #speed=1# set the min speed for traveling to avoid dead congestion
        X1=pre_location['coor-X']# coordinate X of previous location
        Y1=pre_location['coor-Y']# coordinate Y of previous location
        X2=dic_nodes[nodeid2]['coor-X']# coordinate X of end node of focused lane
        Y2=dic_nodes[nodeid2]['coor-Y']# coordinate Y of end node of focused lane
        coor_X=X1+(X2-X1)*time_to_travel/3600.0*speed*1000.0/cal_Dist(dic_nodes[nodeid2],pre_location)# in meter, coordinate X of current position of vehicle
        coor_Y=Y1+(Y2-Y1)*time_to_travel/3600.0*speed*1000.0/cal_Dist(dic_nodes[nodeid2],pre_location)# in meter, coordinate Y of current position of vehicle
        #cur_location={'coor-X':coor_X,'coor-Y':coor_Y,'lane-id':lane_id}
        curr_location['coor-X']=coor_X
        curr_location['coor-Y']=coor_Y
        curr_location['lane-id']=lane_id    
        # update the dictionaries of location and list of time stamps
        dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
        dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=curr_location
        ###------------- best route is updated again if it decides to change lanes -----------------###
        # best route of current time stamp is set as the same as previous time stamp
        dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=copy.deepcopy(pre_route)
        
        # update the lane-type of vehicle is being on
        #print "pre-timestamp is ", pre_timestamp, ", for vehicle", veh_id, "which starts from ",dic_vehicles[veh_id]['start-timestamp']
        #print dic_vehicles[veh_id]['dic-lane-type']
        veh_lane_type=copy.deepcopy(dic_vehicles[veh_id]['dic-lane-type'][pre_timestamp])
        dic_vehicles[veh_id]['lane-type']=veh_lane_type
        dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=veh_lane_type        
       
    return ()

def density_Speed(density,freespeed,lane_id):
    # this function returns the speed on a lane when inputting a density
    # negative linear relationship between density and speed is used
    # additional parameters of free speed and jam  density are needed.
    
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
    #   further complicated function for speed-density can be done              #
    # e.g.   
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#     
    speed=0.1
    if density>=jamdensity:# to avoid getting 0 or minus speed
        speed=0.001# km/h
    else:
        speed=freespeed*(1.0-1.0*density/jamdensity)    
    # make sure the lanes directing to the destination nodes have a minimum speed of 5 or 10 km/h
    if speed<5 and lane_id in list_lane_low_connect_to_Ds:
        speed=5
        if not lane_id in list_lane_speed_revised:
            list_lane_speed_revised.append(lane_id)
    elif speed<10 and lane_id in list_lane_high_connect_to_Ds:
        speed=10
        if not lane_id in list_lane_speed_revised:
            list_lane_speed_revised.append(lane_id)
    elif speed<3 and not lane_id in list_lane_low_connect_to_Ds and not lane_id in list_lane_high_connect_to_Ds:
        speed=speed_Adjacent_to_Lanes(lane_id)              
        if speed<3:            
            key1="lane-id:"+lane_id+"--speed:"+str(speed)+"--"+curr_timestamp
            value1=check_Speed_Adjacent_to_Lanes(lane_id)
            dic_laneid_timestamp_adjacent_speeds[key1]=value1
            if not lane_id in list_lane_speed_revised:
                list_lane_speed_revised.append(lane_id)  
            speed=3
    return speed

def speed_Adjacent_to_Lanes(lane_id):
    # this function returns the average speed of adjacent lanes sharing the same node of the given lane
    # regarding shared node, adjacent lanes starts from this node while given lane ends at this node
    # this function intends to solve the problem of queue propagation of the given lane when it reaches jam density 
    aver_speed=0.1 #initialize the average speed of adjacent lanes    
    share_node_id=dic_lanes[lane_id]['node-id2']
    lane_type=dic_lanes[lane_id]['lane-type']
    list_speed=[]
    for item in dic_lanes.keys():
        if dic_lanes[item]['node-id1']==share_node_id and dic_lanes[item]['lane-type']==lane_type:
            list_speed.append(dic_lanes[item]['speed'])
    if len(list_speed)>0:
        aver_speed=sum(list_speed)/len(list_speed)    
    return aver_speed
def check_Speed_Adjacent_to_Lanes(lane_id):
    # this function returns the individual speed of adjacent lanes sharing the same node of the given lane
    # regarding shared node, adjacent lanes starts from this node while given lane ends at this node       
    share_node_id=dic_lanes[lane_id]['node-id2']
    lane_type=dic_lanes[lane_id]['lane-type']
    list_laneid_and_speed=[]
    for item in dic_lanes.keys():
        if dic_lanes[item]['node-id1']==share_node_id and dic_lanes[item]['lane-type']==lane_type:
            list_laneid_and_speed.append(item)
            list_laneid_and_speed.append(dic_lanes[item]['speed'])
    
    return list_laneid_and_speed
def update_Lanes(curr_timestamp):
    # this function update features of all lanes in the dic_lanes, dic_graph_low and dic_graph_high at a time stamp.
    # the updated features include counts (the number of vehicles, PCU, i.e. passenger car unit).
    # then the density, speed, and travel time are updated according to the counts.    
    
    # initialize the counts on each lane is 0
    for lane_id in dic_lanes.keys():
        dic_lanes[lane_id]['counts']=0
        id1=dic_lanes[lane_id]['node-id1']
        id2=dic_lanes[lane_id]['node-id2']
        lane_type=dic_lanes[lane_id]['lane-type']
        if lane_type==0:
            dic_graph_low[id1][id2]['counts']=0
        elif lane_type==1:
            dic_graph_high[id1][id2]['counts']=0
        else:
            print('sth wrong when initialize counts on the lanes')
    
    # check which lane each running vehicle is on and count them    
    for veh_id in list_running_vehids:
        # check the vehicle type
        veh_type=dic_vehicles[veh_id]['veh-type']
        pcu=0
        if veh_type==0 or veh_type=='car':# car
            pcu=1.0
        elif veh_type==1 or veh_type=='bus':# bus
            pcu=3.5
        elif veh_type==2 or veh_type=='truck':# truck
            pcu=3.5
        else:
            print ("ERROR, no corresponding pcu value to the vehicle type")            
        
        lane_id=dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id']# lane id where this running vehicle is on
        dic_lanes[lane_id]['counts']+=pcu
        lane_type=dic_lanes[lane_id]['lane-type']
        id1=dic_lanes[lane_id]['node-id1']
        id2=dic_lanes[lane_id]['node-id2']        
        if lane_type==0:
            dic_graph_low[id1][id2]['counts']+=pcu
        elif lane_type==1:
            dic_graph_high[id1][id2]['counts']+=pcu
        else:
            print("sth wrong when counting the vehicles on the lanes")
    # update the density and speed on each lane based on the counts on the lane
    for lane_id in dic_lanes.keys():
        # update density, speed, travel-time in dic_lanes
        density=dic_lanes[lane_id]['density']=dic_lanes[lane_id]['counts']/dic_lanes[lane_id]['length']# density=counts/length, unit is pcu/km
        #speed_adja_to_lane=speed_Adjacent_to_Lanes(lane_id)
        speed=dic_lanes[lane_id]['speed']=density_Speed(density,dic_lanes[lane_id]['free-speed'],lane_id)# three input, real-time density, free-speed, jam density, unit is km/h
        #speed=max([speed_adja_to_lane,speed_by_density])
        # real-time travel-time=length/real-time speed, plus the delay at the end-node of this lane, unit is sec
        travel_time=dic_lanes[lane_id]['travel-time']=dic_lanes[lane_id]['length']*3600.0/speed+dic_nodes[dic_lanes[lane_id]['node-id2']]['delay']
        dic_lanes[lane_id]['dic-speeds'][curr_timestamp]=copy.deepcopy(speed)
        dic_lanes[lane_id]['dic-counts'][curr_timestamp]=copy.deepcopy(dic_lanes[lane_id]['counts'])
        dic_lanes[lane_id]['dic-densities'][curr_timestamp]=copy.deepcopy(density)        
        # obtain lane type and id1 and id2 to update the lane features in the other two lane dictionaries
        lane_type=dic_lanes[lane_id]['lane-type']
        id1=dic_lanes[lane_id]['node-id1']
        id2=dic_lanes[lane_id]['node-id2']
        # update density,speed,travel-time in two other lane dictionaries
        if lane_type==0:
            dic_graph_low[id1][id2]['density']=copy.deepcopy(density)
            dic_graph_low[id1][id2]['speed']=copy.deepcopy(speed)
            dic_graph_low[id1][id2]['travel-time']=copy.deepcopy(travel_time)
            dic_graph_low[id1][id2]['dic-speeds'][curr_timestamp]=copy.deepcopy(speed)
            dic_graph_low[id1][id2]['dic-counts'][curr_timestamp]=copy.deepcopy(dic_lanes[lane_id]['counts'])
            dic_graph_low[id1][id2]['dic-densities'][curr_timestamp]=copy.deepcopy(density)  
        elif lane_type==1:
            dic_graph_high[id1][id2]['density']=copy.deepcopy(density)
            dic_graph_high[id1][id2]['speed']=copy.deepcopy(speed)
            dic_graph_high[id1][id2]['travel-time']=copy.deepcopy(travel_time)
            dic_graph_high[id1][id2]['dic-speeds'][curr_timestamp]=copy.deepcopy(speed)
            dic_graph_high[id1][id2]['dic-counts'][curr_timestamp]=copy.deepcopy(dic_lanes[lane_id]['counts'])
            dic_graph_high[id1][id2]['dic-densities'][curr_timestamp]=copy.deepcopy(density)            
    return()

def best_Route_in_Time(veh_id,curr_timestamp):
    # this function returns the best route by travel-time    
    # this function search the best route for a vehicle at a time stamp, based on the lane features at previous time stamp
    # make sure this function is used after the previous function (Update_Lanes) which updates the latest travel time on each lane.    
    curr_location=dic_vehicles[veh_id]['dic-locations'][curr_timestamp]
    lane_id=curr_location['lane-id']
    nodeid1=dic_lanes[lane_id]['node-id1']    
    nodeid2=dic_lanes[lane_id]['node-id2']# it causes excluding the current lane (and this lanes' ids) in the best route search results. Manually include them in the results.  
    
    results={}
    D=dic_vehicles[veh_id]['destination']
    # search best route on low-slow lane-network
    D_low,P_low=shortest_Path_in_Node(dic_graph_low, nodeid2, D)# travel time and best route, in the format of dictionary whose key and value is node-id pairs
    best_route_lanes_l=convert_PPath_to_PathIDs(P_low,dic_graph_low,nodeid2,D)# best route, in the format of dictionary whose key and value is lane-id pairs
    results['best-route-lanes-low']=best_route_lanes_l
    results['time-to-destination-low']=D_low[D]
    results['best-route-nodes-low']=P_low
    # add current lane information into the above results    
    next_nodeid=P_low[nodeid2]
    next_laneid=dic_graph_low[nodeid2][next_nodeid]['lane-id']
    results['best-route-nodes-low'][nodeid1]=nodeid2
    results['best-route-lanes-low'][lane_id]=next_laneid
    
    # search best route on high-fast lane-network
    D_high,P_high=shortest_Path_in_Node(dic_graph_high, nodeid2, D)
    best_route_lanes_h=convert_PPath_to_PathIDs(P_high, dic_graph_high, nodeid2, D)
    results['best-route-lanes-high']=best_route_lanes_h
    results['time-to-destination-high']=D_high[D]
    results['best-route-nodes-high']=P_high      
    # add current lane information into the above results
    next_nodeid=P_high[nodeid2]
    next_laneid=dic_graph_high[nodeid2][next_nodeid]['lane-id']
    results['best-route-nodes-high'][nodeid1]=nodeid2
    results['best-route-lanes-high'][lane_id]=next_laneid   
    return results # in dictionary; it includes the best routes (in the form of lane-ids and node-ids) on both fast-lane network and slow-lane network
def best_Route_in_Stress1(veh_id):
    # this function returns the best route searching results based on the minimization of stress
    # based on graph_stress structure 1
    # input is only the vehicle id, and no temporal input is required, as currently stress function is not assumed to be correlated to time
    O=dic_vehicles[veh_id]['origin']
    D=dic_vehicles[veh_id]['destination']
    dic_node_stress,dic_route_in_node_pair=shortest_Path_in_Node_Stress1(dic_graph_low,dic_graph_stress1,O,D)        
    return dic_node_stress,dic_route_in_node_pair # route does not include the virtual origin and destination nodes
def best_Route_in_Stress2(veh_id):
    # this function returns the best route searching results based on the minimization of stress
    # based on graph_stress structure 2
    # input is only the vehicle id, and no temporal input is required, as currently stress function is not assumed to be correlated to time
    O=dic_vehicles[veh_id]['origin']
    D=dic_vehicles[veh_id]['destination']
    dic_lane_stress,dic_route_in_lane_pair=shortest_Path_in_Node_Stress2(dic_graph_low,dic_graph_stress1,O,D)        
    return dic_lane_stress,dic_route_in_lane_pair # route does not include the virtual links; to obtain the final total-stress, use the key including the string of "-D"


def decision_Make(veh_id,curr_timestamp):
    # this function returns a result of whether to change to a faster and more expensive lane-network at a current time stamp after obtaining its current location.
    # this function is only used when the vehicle is not in the last section to its destination; this is an if-condition in the main program
    # revised on 2019/1/25 by integrating the function of lane_Change
    chan_lane=0# code of whether changing lanes to fast-expensive network: 1 is to change and 0 is not to change
    #start_time=dic_vehicles[veh_id]['start-timestamp']
    expect_endtime=dic_vehicles[veh_id]['expected-endtime']
    #print veh_id,dic_vehicles[veh_id]['start-timestamp'],dic_vehicles[veh_id]['origin'],dic_vehicles[veh_id]['destination'],"current time and expect end time are:", curr_timestamp,expect_endtime
    left_time_budget=cal_Time(curr_timestamp,expect_endtime)
    best_route_results=best_Route_in_Time(veh_id, curr_timestamp)
    curr_location=dic_vehicles[veh_id]['dic-locations'][curr_timestamp]
    lane_id=curr_location['lane-id']
    nodeid2=dic_lanes[lane_id]['node-id2']
    
    ### assume changing lanes completes immediately
    # speed of vehicle if it continues to run on the slow lane of current link    
    speed_low=min([dic_vehicles[veh_id]['max-speed'],dic_lanes[lane_id]['speed']])
    # speed of vehicle if it changes to the fast lane of current link
    speed_high=min([dic_vehicles[veh_id]['max-speed'],dic_lanes[lane_id]['speed']])
    # calculate the time needed from current location to its destination in case of running on the slow lane network and fast lane network
    needed_time_low=best_route_results['time-to-destination-low']+cal_Dist(curr_location,dic_nodes[nodeid2])/1000.0/speed_low*3600.0
    needed_time_high=best_route_results['time-to-destination-high']+cal_Dist(curr_location,dic_nodes[nodeid2])/1000.0/speed_high*3600.0
    # check the left time budget and the value of time to decide the lane-changing probability
    if left_time_budget<needed_time_low:
        if dic_vehicles[veh_id]['veh-type']=='bus':
            chan_lane=1
        elif dic_vehicles[veh_id]['veh-type']=='car':
            if dic_vehicles[veh_id]['value-time']>=median_value_time:
                dic_vehicles[veh_id]['lane-chan-p']=dic_vehicles[veh_id]['lane-chan-p']*2
                if dic_vehicles[veh_id]['lane-chan-p']>1:
                    dic_vehicles[veh_id]['lane-chan-p']=1
            # conclude the decision based on the lane-changing probability
            if dic_vehicles[veh_id]['lane-chan-p']>=0.7:
                chan_lane=1# change lane
            else:
                chan_lane=0# do not change lane                
            if left_time_budget<needed_time_high:
                #######################print("even changing to the faster-expensive lane networks cannot arrive on time")
                chan_lane=0# do not change lane; need further study regarding loss
                dic_vehicles[veh_id]['lane-changed']=[9,curr_timestamp]# changing lane still cannot meet the time budget
    if chan_lane==1: # if the vehicle decides to change lane:
        id1=dic_lanes[lane_id]['node-id1']
        id2=dic_lanes[lane_id]['node-id2']
        fast_lane_id=dic_graph_high[id1][id2]['lane-id']
        low_lane_id=dic_graph_low[id1][id2]['lane-id']   
        
        best_route_h=best_route_results['best-route-lanes-high']
        #print "veh-id is ",veh_id, " and its destination is: ",dic_vehicles[veh_id]['destination'],".it starts at ",dic_vehicles[veh_id]['start-timestamp'],";now it is approaching node of ", nodeid2
        #print dic_vehicles[veh_id]['dic-lane-type']
        #print lane_id,low_lane_id,fast_lane_id
        #print best_route_h,best_route_results['best-route-nodes-high']
        #print best_route_results['best-route-lanes-low'],best_route_results['best-route-nodes-low']
        # one more pair should be included in the best_route_h, as lane-id-before-changing:next-lane-id has been included while lane-id-after-changing:next-lane-id has not been. the latter should also be included.
        next_lane_id=best_route_h[low_lane_id]
        best_route_h[fast_lane_id]=next_lane_id   
        
        ##################################################
        # lane-change is assumed to be instantly, which means the location and best-route should be updated simultaneously 
        # update the location and best route of the vehicle
        dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id']=fast_lane_id
        dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=best_route_h
        dic_vehicles[veh_id]['lane-type']=1
        dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=1
        dic_vehicles[veh_id]['lane-changed']=[1,curr_timestamp]
        dic_vehicles[veh_id]['list-laneid-used'].append(fast_lane_id)
        dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=dic_lanes[fast_lane_id]['speed']    
    return ()
def OD_Travel_Time(curr_timestamp):
    # this function is for obtaining the travel time between each OD pair at current time stamp; the travel time will be stored at the dictionary
    dic_O_D_low_travel_time[curr_timestamp]={}
    dic_O_D_high_travel_time[curr_timestamp]={}
    for O in list_O:
        dic_O_D_low_travel_time[curr_timestamp][O]={}
        dic_O_D_high_travel_time[curr_timestamp][O]={}

        for D in list_D:
            D_low,P_low=shortest_Path_in_Node(dic_graph_low, O, D)
            travel_time_low=D_low[D]
            D_high,P_high=shortest_Path_in_Node(dic_graph_high, O, D)
            travel_time_high=D_high[D]
            
            dic_O_D_low_travel_time[curr_timestamp][O][D]=travel_time_low
            dic_O_D_high_travel_time[curr_timestamp][O][D]=travel_time_high
    return ()

###########################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
###########=============== Main Program =================##################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
###########################################################################

#--------------------------------------------------------------------------------------------------------------#
#************************** this part set the parameters for scenario test ************************************#
#
# set the delay strategy
scenario_test_list_delay_strategy=["no-delay","vol-dist","random"]#"preset","vol-simple",
delay_strategy="random"

# set the vehicle generation distribution method
# "in the whole generation period" violates the OD demand in each 5-minute time slot in the demand files; here using these just tries to show the applicability.
scenario_test_list_veh_gene_distri_method=['uniform-in-whole-generation-period','random-in-whole-generation-period',"normal-in-whole-generation-period"]#'uniform-in-ODpair-VehType-TimeSlot','random-in-ODpair-VehType-TimeSlot',
veh_gen_distri="uniform-in-whole-generation-period"
# set the sigma of distribution of vehicle generation time stamps
mu_veh_time_gen=900# mean value of normal distribution of vehicle generation time stamps
scenario_test_list_sigma_veh_time_gen=[100,200]
sigma_veh_time_gen=100# std of normal distribution of vehicle generation time stamps

# set the multiple times of basic traffic demand
scenario_test_list_multi_OD_para=[1,2,3,4,5]#
multi_OD_para=2 # multiple times of basic OD matrices, a parameter to increase the travel demand

# set the charging rate of fast-lane
scenario_test_list_fast_lane_charge=[0.1,1,10]# unit in SGD per kilometers, this is the fixed type of charge as it does not vary due to density on lanes
fast_lane_charge=1

'''
####################### combine the above parameters for the test #####################
scenario_test_list_para_combinations=[]# a combination (in the form of list, in sameorder of the above parameters) is an item in this list
writer_statistic_whole_simulation.writerow(["following is the parameter specification of scenario test"])
for delay_strategy in scenario_test_list_delay_strategy:
    for veh_gen_distri in scenario_test_list_veh_gene_distri_method:
        for sigma_veh_time_gen in scenario_test_list_sigma_veh_time_gen:
            for multi_OD_para in scenario_test_list_multi_OD_para:
                if veh_gen_distri<>"normal-in-whole-generation-period":
                    mu_veh_time_gen="--"
                    sigma_veh_time_gen="--"
                else:
                    mu_veh_time_gen=copy.deepcopy(mu_veh_time_gen_preset)
                item_para_combination=[delay_strategy,veh_gen_distri,sigma_veh_time_gen,mu_veh_time_gen,multi_OD_para]
                if not item_para_combination in scenario_test_list_para_combinations:
                    scenario_test_list_para_combinations.append(item_para_combination)
                    print item_para_combination
                    writer_statistic_whole_simulation.writerow(item_para_combination)
print "totally there are ",len(scenario_test_list_para_combinations)," scenarios for the test"
'''
#*************************************************************************#
#                        initialize key parameters                        #
#*************************************************************************# 
# set the starting time stamp for simulation, 7am of running day
today_date=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')[:10]
simu_start_time_stamp=today_date+' '+'07:00:00'
# set whole simulation duration
total_steps=4000# 
# set time step used in the simulation, unit in seconds
timestep=1# less than 1 hour simulation is implemented, time-step is 1 seconds and totally 3600 steps are implemented

# set jam density used in this research
jamdensity=124# 200 pcu/mile=124 pcu/km. from:https://lost-contact.mit.edu/afs/eos.ncsu.edu/info/ce400_info/www2/flow1.html
# set/calculate the median of value-of-time used in the study
median_value_time=50# the median of value-of-time in SG, used to the check the person
  
# set random seeds
# NOTE: random seeds should be re-implemented at the beginning of each scenario test to make sure each scenario has the same features of drivers etc.
random.seed(10)
np.random.seed(10)



#************************* this is the end of setting parameters for scenario test ****************************#
#--------------------------------------------------------------------------------------------------------------#


# obtain the date when the simulation starts
today_date=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')[:10]


# set the list of heads for output
list_para_combination_heads=["delay-strategy","veh-generation-method","sigma-norm-distri-veh-gener","demand-times","--"]
# set the list of parameter-combination for output
list_para_combination_values=[delay_strategy,veh_gen_distri,sigma_veh_time_gen,multi_OD_para,"--"]


#*************************************************************************#
#                        initialize key parameters                        #
#*************************************************************************# 
# set the starting time stamp for simulation, 7am of running day

simu_start_time_stamp=today_date+' '+'07:00:00'
# set whole simulation duration
total_steps=4000# 
# set time step used in the simulation, unit in seconds
timestep=1# less than 1 hour simulation is implemented, time-step is 1 seconds and totally 3600 steps are implemented

# set jam density used in this research
jamdensity=124# 200 pcu/mile=124 pcu/km. from:https://lost-contact.mit.edu/afs/eos.ncsu.edu/info/ce400_info/www2/flow1.html
# set/calculate the median of value-of-time used in the study
median_value_time=50# the median of value-of-time in SG, used to the check the person
  
# set random seeds
# NOTE: random seeds should be re-implemented at the beginning of each scenario test to make sure each scenario has the same features of drivers etc.
random.seed(10)
np.random.seed(10)


#************************* this is the end of setting parameters for scenario test ****************************#
#--------------------------------------------------------------------------------------------------------------#


# initialize vehicle-id; it is a global variable, as there may be many OD-matrix
vehicle_id=0# the first vehicle id is 1 and is increased subsequently with a step of 1


dic_nodes={}
dic_graph_low={}# all lanes belonging to the low fee type
dic_graph_high={}# all lanes belonging to the high fee type
dic_lanes={}
dic_turns={}
dic_graph_stress1={}
dic_graph_stress2={}

dic_vehicles={}# all vehicles; key is the vehicle id and a dictionary containing other features is used as the value. {vehid1:{'veh-type':veh_type,'max-speed':max_speed,....},vehid2:{... ...},vehid3:....} 
dic_gen_time_vehids={}# all vehicle-ids which are grouped by generation time stamp; key is a time stamp for generation, value is a list containing all vehicles' ids generated on that time stamp. {time1:[id1,id2,...],time2:[id5,id6,...],..}
list_running_vehids=[]# list containing all vehicle-ids that are running on the network at current time stamp
list_gen_vehids=[]# list containing vehicle ids generated at current time stamp
list_van_vehids=[]# list containing the vehicle ids vanishing at each seconds


list_D=[]# list containing all destination nodes, no duplicate
list_lane_low_connect_to_Ds= []# list containing lane ids of low level which connects to the destination nodes
list_lane_high_connect_to_Ds=[]# list containing lane ids of high level which connects to the destination nodes
list_lane_speed_revised=[]# to check the lanes whose speed are revised due to the connection
dic_laneid_timestamp_adjacent_speeds={}# dictionary containing the speed values of adjacent lanes which start from the same node as laneid; it is used to check the surrounding speed when laneid see it blocked. key is laneid+curr_timestamp,value is a list containing the adjacent laneid and its speed value


# dictionaries used for exporting statistics; information in the following dictionaries will be output to csv files
dic_O_D_low_travel_time={}# dictionary of collecting travel time from each OD pair on low level lane network at each simulation step; format: {t1:{O1:{D1:time1,D2:time2,...},O2:{},...},t2:
dic_O_D_high_travel_time={}# similar to the above, but for lane network of high level
dic_timestamp_veh_running={}# dictionary of collecting running vehicle ids at each simulation step; format: {t1:count1,t2:count2,....}
dic_timestamp_veh_vanish={}# dictionary of collecting vanishing vehicle ids at each simulation step; format: {t1:count1,t2:count2,....}
dic_timestamp_veh_changed_lane={}# dictionary of vehicle changing lanes at each simulation step; format:{t1:count1 of vehicle changed lane;t2:count2 of vehicle changed lane,...}
# information in dic_lanes and dic_vehicles will also be exported to csv files
dic_car_composition_decision={"changed":0,"not-changed":0,"not-changed-as-no-improve":0}
time_stamp_no_running_veh=""# the time stamp when all vehicles have already arrived their destination; it is an output
list_count_running_vehicle=[]# list of number of running vehicles at each time stamp, in the order of list of time stamps 
#*************************************************************************#
#                      read geometry from csv files                       #
#*************************************************************************# 
########## 
infile_node=open(r"D:\03Work during PD\31ERP2_in_COI\Sioux Falls network\nodes-SiouxFalls_gong.csv","r")
infile_node.readline()# jump the first row of column head
infile_lane=open(r"D:\03Work during PD\31ERP2_in_COI\Sioux Falls network\lanes-SiouxFalls_gong.csv","r")
infile_lane.readline()# jump the first row of column head

dic_nodes=read_Nodes(infile_node,dic_nodes)
dic_graph_high,dic_graph_low,dic_lanes=read_Lanes(infile_lane,dic_graph_high,dic_graph_low,dic_lanes)

# calculate the length of lanes
dic_graph_low =length_Cal(dic_graph_low, dic_nodes)
dic_graph_high=length_Cal(dic_graph_high,dic_nodes)
dic_lanes=length_Cal2(dic_lanes,dic_nodes)

# update the fixed charge rate of fast-lane
dic_graph_high=fixed_Charge_Update(dic_graph_high,fast_lane_charge)# assign the fast lane charge rate to each lane

#*************************************************************************#
#                       generate graph for stress                         #
#*************************************************************************# 
dic_turns=turn_Gen(dic_nodes,dic_lanes,rule_of_road)
dic_graph_stress1,dic_graph_stress2=graph_Stress(dic_turns,dic_lanes,rule_of_road)

# start running and outputting the process in the writer file
writer.writerow(["************************---------------following is the read node--------------********************************"])
list_heads_node=["node-id","a-series-of-node-features"]
writer.writerow(list_heads_node)
for node_id in dic_nodes:
    writer.writerow([node_id,dic_nodes[node_id]])
writer.writerow(["****************************************************************************************************************"])

writer.writerow(["************************---------------following is the read lanes--------------********************************"])
list_heads_graph=["graph-type-by-speed","lane-id","a-series-of-lane-features"]
writer.writerow(list_heads_graph)
for item in dic_lanes:
    id1=dic_lanes[item]['node-id1']
    id2=dic_lanes[item]['node-id2']
    lane_ty=dic_lanes[item]['lane-type']
    writer.writerow(["dic-lanes",item, dic_lanes[item]])
    if lane_ty==0:
        writer.writerow(["dic-graph-low", dic_graph_low[id1][id2]['lane-id'],dic_graph_low[id1][id2]])
    else:
        writer.writerow(['dic-graph-high',dic_graph_high[id1][id2]['lane-id'],dic_graph_high[id1][id2]])

writer.writerow(["following is the read turning information"])
list_heads_turn=['from-lane-id','to-lane-id','from-lane-nodeid1','from-lane-nodeid2','to-lane-nodeid1','to-lane-nodeid2','direction','abs-degree-turned','original-turned-degree']
writer.writerow(list_heads_turn)
for lane1 in dic_turns:
    for lane2 in dic_turns[lane1]:
        turns=dic_turns[lane1][lane2]
        record=[lane1,lane2,dic_lanes[lane1]['node-id1'],dic_lanes[lane1]['node-id2'],dic_lanes[lane2]['node-id1'],dic_lanes[lane2]['node-id2'],turns['direction'],turns['abs degree'],turns['turned-degree']]
        writer.writerow(record)
writer.writerow(["following is the read stress-structure 1"])
list_heads_graph_stress1=["at-node-id","to-node-id","from-node-id","quantified-stress","direction","turned-degree"]
writer.writerow(list_heads_graph_stress1)
for node2 in dic_graph_stress1:
    for node3 in dic_graph_stress1[node2]:
        for node1 in dic_graph_stress1[node2][node3]:
            stress=dic_graph_stress1[node2][node3][node1]
            lane12id=dic_graph_low[node1][node2]['lane-id']
            lane23id=dic_graph_low[node2][node3]['lane-id']
            dire=dic_turns[lane12id][lane23id]['direction']
            degree_turned=dic_turns[lane12id][lane23id]['turned-degree']
            record=[node2,node3,node1,stress,dire,degree_turned]
            writer.writerow(record)
writer.writerow(["following is the read stress-structure 2"])
list_heads_graph_stress2=["from-lane-id","to-lane-id","quantified-stress","direction","turned-degree"]
writer.writerow(list_heads_graph_stress2)
for lane_from in dic_graph_stress2:
    for lane_to in dic_graph_stress2[lane_from]:
        stress=dic_graph_stress2[lane_from][lane_to]
        direction=dic_turns[lane_from][lane_to]['direction']
        degree_turned=dic_turns[lane_from][lane_to]['turned-degree']
        record=[lane_from,lane_to,stress,direction,degree_turned]        
        writer.writerow(record)
        
writer.writerow(["the geometry has been read completely"])
writer.writerow(["****************************************************************************************************************"])
########## read vehicles from OD-matrix in the csv files
folderpath=r"D:\03Work during PD\31ERP2_in_COI\meso_by_python\OD_data"# the folder where the OD files are stored
# obtain the OD matrices and simulation starting time from a series of csv files of traffic demand in OD pairs
OD_matrices,list_O,list_D=read_OD(folderpath,multi_OD_para)
dic_vehicles,dic_gen_time_vehids,vehicle_id=vehicle_Gen(OD_matrices, veh_gen_distri,vehicle_id,dic_graph_low,dic_graph_high,median_value_time,dic_nodes,dic_vehicles,dic_gen_time_vehids,mu_veh_time_gen,sigma_veh_time_gen)
# put lane ids to the list_lane_low_connect_to_Ds and list_lane_high_connect_to_Ds
for lane_id in dic_lanes:
    if dic_lanes[lane_id]['node-id2'] in list_D and dic_lanes[lane_id]['lane-type']==0 and not lane_id in list_lane_low_connect_to_Ds:
        list_lane_low_connect_to_Ds.append(lane_id)
    elif dic_lanes[lane_id]['node-id2'] in list_D and dic_lanes[lane_id]['lane-type']==1 and not lane_id in list_lane_high_connect_to_Ds:
        list_lane_high_connect_to_Ds.append(lane_id)   



writer.writerow(["following is the read OD matrix"])
for item in OD_matrices:
    writer.writerow([OD_matrices[item]])
writer.writerow(["****************************************************************************************************************"])
writer.writerow(["***********-------------------------following is the read vehicles---------------------------*******************"])
for item in dic_vehicles:
    writer.writerow([dic_vehicles[item]])
writer.writerow(["****************************************************************************************************************"])

writer.writerow(["following is the vehicles that should be generated at pre-set timestamps"])

total_veh=0
for item in dic_gen_time_vehids:    
    writer.writerow([item,len(dic_gen_time_vehids[item]),dic_gen_time_vehids[item]])
    total_veh+=len(dic_gen_time_vehids[item])
#!print "total generated vehicles are:",total_veh,len(dic_vehicles)
writer.writerow(["total generated vehicles are:",total_veh,len(dic_vehicles)])
writer.writerow(["****************************************************************************************************************"])
        
######### update vehicles and geometry at every simulation step
# initialize current time stamp as the starting time stamp of the simulation
curr_timestamp=copy.deepcopy(simu_start_time_stamp)# string type
update_Lanes(curr_timestamp)# update lanes in case of the delay strategy is preset in the node delay feature

for i in range(0,total_steps):# each simulation step i    
    ####################################################################################################################
    #### generate vehicles,update vehicles' locations,  update vanishing vehicles, and finally update lane features ####
    ####################################################################################################################
    
    print "this is ", i, "th step in simulation at time-stamp of ", curr_timestamp#%%%%%%%%%%%% for checking
    writer.writerow(["this is ", i, "th step in simulation at time-stamp of ", curr_timestamp])
    # collect the travel time between each OD pair at each simulation step, just for output statistics
    OD_Travel_Time(curr_timestamp)
    # update the dynamic delay at each node, if necessary
    if not delay_strategy in ["no-delay","preset"]:# if delay is not pre-set or 0-delay, update to get the dynamic delay
        if i==0:
            for node_id in dic_nodes:
                dic_nodes[node_id]['delay']=0
        else:  
            update_Node_delay(delay_strategy, curr_timestamp)

    #!!!!!!!!!!!!!!!!!!!!! calculation for this time stamp begins !!!!!!!!!!!!!!!!!!!!#
    # step1--------------------------------------------------------------------------------------------------------------------------------------------------------------
    ######**** regarding newly generated vehicles
    if dic_gen_time_vehids.has_key(curr_timestamp):# this time stamp generates vehicles           
        list_gen_vehids=copy.deepcopy(dic_gen_time_vehids[curr_timestamp])
        writer.writerow(["The time stamp--",curr_timestamp,"--generates vehicles. Totally newly generated vehicles are ", len(list_gen_vehids)])
        # update their best route by previous time-stamp
        for veh_id in list_gen_vehids:# not generated at the initial time of simulation; need update their best route at first
            if curr_timestamp<>simu_start_time_stamp:# vehicles generated not at simulation-start-timestamp need to update their features due to current timestamp
                writer.writerow(["following is the generated vehicles that are not generated at the simulation-starting time stamp"])
                O=dic_vehicles[veh_id]['origin']
                D=dic_vehicles[veh_id]['destination']
                dic_graph={}
                if dic_vehicles[veh_id]['lane-type']==1:
                    dic_graph=copy.deepcopy(dic_graph_high)
                else:
                    dic_graph=copy.deepcopy(dic_graph_low)
                D_graph,P_graph=shortest_Path_in_Node(dic_graph, O, D)
                D_shortest_travel_time=D_graph
                P_shortest_path_in_nodes=P_graph
                best_route_lanes=convert_PPath_to_PathIDs(P_shortest_path_in_nodes,dic_graph,O,D)
                time_budget=D_shortest_travel_time[D]

                # re-set and update the features of this vehicle: best route and time budget are a result of current traffic status 
                dic_vehicles[veh_id]['dic-routes']={}
                dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=best_route_lanes
                dic_vehicles[veh_id]['time-budget']=time_budget
                
                next_nodeid=P_shortest_path_in_nodes[O]
                lane_id=dic_graph[O][next_nodeid]['lane-id']                
                
                if time_budget>36000:
                    writer.writerow(["the time budget for vehicle of ",veh_id," is more than 10hr. ERROR"])
                    print "the time budget for vehicle---",veh_id,"---is more than 10hr. ERROR"
                dic_vehicles[veh_id]['expected-endtime']=add_Timestamp(curr_timestamp,time_budget)               
                               
                dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id']=lane_id
                dic_vehicles[veh_id]['list-laneid-used'].append(lane_id)
                dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=dic_lanes[lane_id]['speed']
                # update list-of-used-lanes and dic-speeds
            writer.writerow(["the newly generated vehicles have the following features"])
            writer.writerow([dic_vehicles[veh_id]])
    else:# this time stamp does not generate vehicles
        list_gen_vehids=[]
        writer.writerow(["this time stamp does not generate new vehicles"])
    
    writer.writerow(["****************************************************************************************************************"])
    # step2-------------------------------------------------------------------------------------------------------------------------------------------------------------
    ######**** regarding the existing vehicles: update their locations by previous time stamp and make a decision, collecting the vanishing vehicles' ids simultaneously.
    list_van_vehids=[]# initialize the list-of-vanishing-vehicles as blank
    if len(list_running_vehids)>0:# there are vehicles running in the road network
        writer.writerow([curr_timestamp,"still has running vehicles on the network"])
        writer.writerow(["there are ",len(list_running_vehids)," vehicles running on the network"])
        #!print "there are ",len(list_running_vehids)," vehicles running on the network"
        writer.writerow(["their ids are",list_running_vehids])        
        for veh_id in list_running_vehids:  
            # only update the vehicles not being delayed
            if dic_vehicles[veh_id]['delay-status']<>1:              
                # set the location with or without delay; if the former, select the delay strategy; check whether this vehicle will arrive its destination (vanishing vehicle)
                if delay_strategy=="no-delay":
                    locate_no_Delay(veh_id, curr_timestamp)# obtain the current location of vehicles
                else:
                    locate_with_Delay(veh_id, curr_timestamp)
                if dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id']=='D':# this vehicle has arrived its destination
                    list_van_vehids.append(veh_id)# put it into the list of vanishing vehicles
                if dic_vehicles[veh_id]['lane-type']==0 and not veh_id in list_van_vehids:# only vehicles on slow lanes are making decisions
                    veh_cur_laneid=dic_vehicles[veh_id]['dic-locations'][curr_timestamp]['lane-id']
                    if dic_lanes[veh_cur_laneid]['node-id2']<>dic_vehicles[veh_id]['destination']:
                        decision_Make(veh_id, curr_timestamp)                        
                    else:
                        writer.writerow([veh_id, "is at the last section to its destination, node--", dic_vehicles[veh_id]["destination"]])   
            
            # being delayed vehicle copies its previous location and speed
            else:
                # if time reaches to the delay end, then turn the vehicle status from 1 to 0
                #print "vehicle of ", veh_id," was being delayed; now check whether it can be put back to normal running vehicle"
                #print veh_id," is set to be delayed until", dic_vehicles[veh_id]['delay-time-until'],"current time is: ",curr_timestamp
                if dic_vehicles[veh_id]['delay-time-until']==curr_timestamp:# delayed vehicle updates its location from curr_timestamp + 1 time-step   
                    #print "vehicle of ", veh_id, " has finished delayed status"                 
                    dic_vehicles[veh_id]['delay-status']=0         
                    dic_vehicles[veh_id]['delay-time-until']=""
                # use previous location information as the location of curr_timestamp by copying
                pre_timestamp=get_Pre_Timestamp(curr_timestamp,timestep)
                dic_vehicles[veh_id]['dic-locations'][curr_timestamp]=copy.deepcopy(dic_vehicles[veh_id]['dic-locations'][pre_timestamp])                
                dic_vehicles[veh_id]['dic-speeds'][curr_timestamp]=copy.deepcopy(dic_vehicles[veh_id]['dic-speeds'][pre_timestamp])
                dic_vehicles[veh_id]['dic-routes'][curr_timestamp]=copy.deepcopy(dic_vehicles[veh_id]['dic-routes'][pre_timestamp])
                dic_vehicles[veh_id]['dic-lane-type'][curr_timestamp]=copy.deepcopy(dic_vehicles[veh_id]['dic-lane-type'][pre_timestamp])
                dic_vehicles[veh_id]['list-timestamps'].append(curr_timestamp)
    else:
        writer.writerow([curr_timestamp,"does not have running vehicles on the network"])
        
    #print "vanishing vehicles are", list_van_vehids#%%%%%%%%%%%% for checking
    #print "running vehicles are", list_running_vehids#%%%%%%%%%%%% for checking
    writer.writerow(["after locating positions, there are ", len(list_van_vehids)," vanishing vehicles. their IDs are", list_van_vehids])
    #!print "after locating positions, there are ", len(list_van_vehids)," vanishing vehicles."
    dic_timestamp_veh_vanish[curr_timestamp]=len(list_van_vehids)
    # step3.1-----------------------------------------------------------------------------------------------------------------------------------------------------------
    #####**** regarding the list of running vehicles: remove the vanishing vehicle ids from the running vehicle ids
    writer.writerow(["now removing the vanishing vehicles from the running vehicles"])
    list_running_vehids=list(set(list_running_vehids).difference(set(list_van_vehids)))    
    writer.writerow(["after removing vanishing vehicles, the running vehicles are", len(list_running_vehids)])#%%%%%%%%%%%% for checking
    
    # step3.2-----------------------------------------------------------------------------------------------------------------------------------------------------------
    #####**** regarding the list of running vehicles: combine running vehicle ids with newly generated vehicle ids
    writer.writerow(["now combining the newly generated vehicles with the running vehicles"])
    list_running_vehids=list(set(list_gen_vehids).union(set(list_running_vehids)))
    #print "newly generated vehicles are", list_gen_vehids#%%%%%%%%%%%% for checking
    writer.writerow(["after combining with generated vehicles, the running vehicles are", len(list_running_vehids)])#%%%%%%%%%%%% for checking
    writer.writerow(["****************************************************************************************************************"])
   
    #!print "newly generated ", len(list_gen_vehids),"vehicles"
    #!print "after vanishing and generating, there are ", len(list_running_vehids)," vehicles on the network"
    dic_timestamp_veh_running[curr_timestamp]=len(list_running_vehids)
    list_count_running_vehicle.append(len(list_running_vehids))
    # step4------------------------------------------------------------------------------------------------------------------------------------------------------------
    ######**** regarding the lanes: update the features
    writer.writerow(["now updating the lane features"])
    update_Lanes(curr_timestamp)
    writer.writerow(["****************************************************************************************************************"])

    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! move to the next time step in the simulation !!!!!!!!!!!!!!!!!!!!!!!!!!!!#
    writer.writerow(["move to the next simulation step"])
    curr_timestamp=add_Timestamp(curr_timestamp, timestep)
    writer.writerow(["****************************************************************************************************************"])

########################################################################################################################################################################

######################################################################
######### output the statistics results  #############################
######################################################################
print "calculation finished; now output the results"

# order the features by time stamp
list_laneids=[]
list_vehids=[]
for lane_id in dic_lanes:
    list_laneids.append(lane_id)
for veh_id in dic_vehicles:
    list_vehids.append(veh_id)
list_laneids.sort
list_vehids.sort
list_timestamps=[]
for i in range(0,total_steps):
    timestamp=add_Timestamp(simu_start_time_stamp, i)
    list_timestamps.append(timestamp)
    dic_timestamp_veh_changed_lane[timestamp]=0


columnheads1=list_para_combination_heads+["lane-id","lane-type"]+list_timestamps
#columnheads2=list_para_combination_heads+["veh-id","veh-type"]+list_timestamps
columnheads3=list_para_combination_heads+["veh-id","veh-type","value-time","lane-chan-p","origin","destination","time-budget","start-time","expected-endtime","end-time","lane-changed"]

# calculate the time stamp when all the running vehicles arrive their destinations
count_from_end=0
print "list_count_running_vehicle: ",list_count_running_vehicle
while list_count_running_vehicle[-1]==0:
    count_from_end+=1
    list_count_running_vehicle=list_count_running_vehicle[:-1]
time_stamp_no_running_veh=list_timestamps[-1*count_from_end]
# export the time stamp when all vehicles arrive their destinations
writer_statistic_whole_simulation.writerow(["total has run "+str(i)+" step and all vehicle arrive their destination at "+ str(time_stamp_no_running_veh)])

##########################################################################################################
###########          output features changes along the time stamp           ##############################
columnhead_of_network_along_timestamp=list_para_combination_heads+["statistic-general","statistic-detail"]+list_timestamps
writer_statistic_along_timestamps.writerow(columnhead_of_network_along_timestamp)
writer_OD_features.writerow(columnhead_of_network_along_timestamp)
# vehicle generated,vanished and running at each time stamp
row_veh_gen         =list_para_combination_values+["vehicles","veh-generated"]
row_veh_van         =list_para_combination_values+["vehicles","veh-vanished"]
row_veh_run         =list_para_combination_values+["vehicles","veh-running"]
row_veh_changed_lane=list_para_combination_values+["vehicles","changed-lane"]


# output average speed and density of lane-network (low level and high level in a separate way) at each time stamp
row_low_lane_aver_speed   =list_para_combination_values+["speed","low-lane-speed"]
row_high_lane_aver_speed  =list_para_combination_values+["speed","high-lane-speed"]
row_low_lane_aver_density =list_para_combination_values+["density","low-lane-density"]
row_high_lane_aver_density=list_para_combination_values+["density","high-lane-density"]

for timestamp in list_timestamps:
    list_low_lane_speed=[]# list of speeds of all low-level lanes at time-stamp
    list_high_lane_speed=[]# list of speeds of all high-level lanes at time-stamp
    list_low_lane_density=[]# list of densities of all low-level lanes at time-stamp
    list_high_lane_density=[]# list of densities of all high-level lanes at time-stamp
    for lane_id in dic_lanes:
        if dic_lanes[lane_id]['lane-type']==0:
            list_low_lane_speed.append(dic_lanes[lane_id]['dic-speeds'][timestamp])
            list_low_lane_density.append(dic_lanes[lane_id]['dic-densities'][timestamp])
        elif dic_lanes[lane_id]['lane-type']==1:
            list_high_lane_speed.append(dic_lanes[lane_id]['dic-speeds'][timestamp])
            list_high_lane_density.append(dic_lanes[lane_id]['dic-densities'][timestamp])
    aver_list_low_lane_speed=sum(list_low_lane_speed)*1.0/len(list_low_lane_speed)
    aver_list_high_lane_speed=sum(list_high_lane_speed)*1.0/len(list_high_lane_speed)
    aver_list_low_lane_density=sum(list_low_lane_density)*1.0/len(list_low_lane_density)
    aver_list_high_lane_density=sum(list_high_lane_density)*1.0/len(list_high_lane_density)
    row_low_lane_aver_speed.append(aver_list_low_lane_speed)
    row_high_lane_aver_speed.append(aver_list_high_lane_speed)
    row_low_lane_aver_density.append(aver_list_low_lane_density)
    row_high_lane_aver_density.append(aver_list_high_lane_density)
writer_statistic_along_timestamps.writerow(row_low_lane_aver_speed)
writer_statistic_along_timestamps.writerow(row_high_lane_aver_speed)
writer_statistic_along_timestamps.writerow(row_low_lane_aver_density)
writer_statistic_along_timestamps.writerow(row_high_lane_aver_density)

# output the average travel time over all OD pairs at each time stamp
row_ODpairs_aver_travel_time_low       =list_para_combination_values+["OD-low-travel-time","average-of-all-ODpairs"] # travel time on low-level network
row_ODpairs_aver_travel_time_high      =list_para_combination_values+["OD-high-travel-time","average-of-all-ODpairs"] # travel time on high-level network
row_ODpairs_aver_travel_time_difference=list_para_combination_values+["OD-travel-time-difference","average-of-all-ODpairs"] # average of travel time difference
count_OD_pairs=0

for timestamp in list_timestamps:
    sum_travel_time_low=0
    sum_travel_time_high=0
    count_OD_pairs=0
    for O in list_O:
        for D in list_D:
            count_OD_pairs+=1
            sum_travel_time_low+=dic_O_D_low_travel_time[timestamp][O][D]
            sum_travel_time_high+=dic_O_D_high_travel_time[timestamp][O][D]
    row_ODpairs_aver_travel_time_low.append(sum_travel_time_low*1.0/count_OD_pairs)
    row_ODpairs_aver_travel_time_high.append(sum_travel_time_high*1.0/count_OD_pairs)
    row_ODpairs_aver_travel_time_difference.append((sum_travel_time_low-sum_travel_time_high)*1.0/count_OD_pairs)

writer_statistic_along_timestamps.writerow(row_ODpairs_aver_travel_time_low)
writer_statistic_along_timestamps.writerow(row_ODpairs_aver_travel_time_high)
writer_statistic_along_timestamps.writerow(row_ODpairs_aver_travel_time_difference)

# output the number of vehicles changes along the time stamp        
for veh_id in dic_vehicles:
    veh_change_lane_infor=dic_vehicles[veh_id]['lane-changed']
    if veh_change_lane_infor[0]==1:# changed the lane
        change_lane_time_stamp=veh_change_lane_infor[1]
        dic_timestamp_veh_changed_lane[change_lane_time_stamp]+=1
        
for timestamp in list_timestamps:
    if dic_gen_time_vehids.has_key(timestamp):
        row_veh_gen.append(len(dic_gen_time_vehids[timestamp]))
    else:
        row_veh_gen.append(0)
    if dic_timestamp_veh_changed_lane.has_key(timestamp):
        row_veh_changed_lane.append(dic_timestamp_veh_changed_lane[timestamp])
    else:
        row_veh_changed_lane.append(0)
    
    row_veh_van.append(dic_timestamp_veh_vanish[timestamp])
    row_veh_run.append(dic_timestamp_veh_running[timestamp])
    
writer_statistic_along_timestamps.writerow(row_veh_gen)
writer_statistic_along_timestamps.writerow(row_veh_van)
writer_statistic_along_timestamps.writerow(row_veh_run)
writer_statistic_along_timestamps.writerow(row_veh_changed_lane)

# output travel time between each OD pair at each time stamp to a different file
list_row_OD_low_travel_time=[]
list_row_OD_high_travel_time=[]
list_row_OD_travel_time_difference=[]# travel time difference between low level and high level lane network
for O in list_O:
    for D in list_D:
        if D<>O:
            row_OD_low_travel_time       =list_para_combination_values+["OD-low-travel-time","OD"+O+"-"+D]
            row_OD_high_travel_time      =list_para_combination_values+["OD-high-travel-time","OD"+O+"-"+D]
            row_OD_travel_time_difference=list_para_combination_values+["OD-travel-time-difference","OD"+O+"-"+D]
            for timestamp in list_timestamps:
                row_OD_low_travel_time.append(dic_O_D_low_travel_time[timestamp][O][D])
                row_OD_high_travel_time.append(dic_O_D_high_travel_time[timestamp][O][D])
                row_OD_travel_time_difference.append(dic_O_D_low_travel_time[timestamp][O][D]-dic_O_D_high_travel_time[timestamp][O][D])
            list_row_OD_low_travel_time.append(row_OD_low_travel_time)
            list_row_OD_high_travel_time.append(row_OD_high_travel_time)
            list_row_OD_travel_time_difference.append(row_OD_travel_time_difference)
for item in list_row_OD_low_travel_time:
    writer_OD_features.writerow(item)
for item in list_row_OD_high_travel_time:
    writer_OD_features.writerow(item)
for item in list_row_OD_travel_time_difference:
    writer_OD_features.writerow(item)    
######################### end of outputting feature changes along the time stamp    #################
#####################################################################################################

#####################################################################################################
############ output features evaluated per simulation (not per each time stamp) #####################
##### regarding vehicle's decision of whether changing lanes
columnhead_of_vehicles_decision_whole_simu=list_para_combination_heads+["statistic-general","statistic-detail","changed-lane","not-change-lane","not-change-lane-as-no-time-improve"]
writer_statistic_whole_simulation.writerow(columnhead_of_vehicles_decision_whole_simu)
count_car_changed_lane=0
count_car_not_changed=0
count_car_not_changed_no_improve=0
count_bus_changed_lane=0
count_bus_not_changed=0
count_bus_not_changed_no_improve=0

list_car_1_aver_speed=[]# list of average speed of each vehicle falling in this category
list_car_0_aver_speed=[]
list_car_9_aver_speed=[]
list_bus_1_aver_speed=[]
list_bus_0_aver_speed=[]
list_bus_9_aver_speed=[]

for veh_id in dic_vehicles:
    if dic_vehicles[veh_id]['veh-type']=='car' and dic_vehicles[veh_id]['lane-changed'][0]==0:
        count_car_not_changed+=1
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_car_0_aver_speed.append(aver_speed)
    elif dic_vehicles[veh_id]['veh-type']=='car' and dic_vehicles[veh_id]['lane-changed'][0]==1:
        count_car_changed_lane+=1        
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_car_1_aver_speed.append(aver_speed)
    elif dic_vehicles[veh_id]['veh-type']=='car' and dic_vehicles[veh_id]['lane-changed'][0]==9:
        count_car_not_changed_no_improve+=1
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_car_9_aver_speed.append(aver_speed)
    elif dic_vehicles[veh_id]['veh-type']=='bus' and dic_vehicles[veh_id]['lane-changed'][0]==0:
        count_bus_not_changed+=1
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_bus_0_aver_speed.append(aver_speed)
    elif dic_vehicles[veh_id]['veh-type']=='bus' and dic_vehicles[veh_id]['lane-changed'][0]==1:
        count_bus_changed_lane+=1
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_bus_1_aver_speed.append(aver_speed)
    elif dic_vehicles[veh_id]['veh-type']=='bus' and dic_vehicles[veh_id]['lane-changed'][0]==9:
        count_bus_not_changed_no_improve+=1
        aver_speed=sum_Dictionary_Value(dic_vehicles[veh_id]['dic-speeds'])*1.0/len(dic_vehicles[veh_id]['dic-speeds'])
        list_bus_9_aver_speed.append(aver_speed)
    else:
        print "ERROR at categorizing the vehicles regarding the decision-making"
row_car_decision=list_para_combination_values+['decision-made','car',count_car_changed_lane,count_car_not_changed,count_car_not_changed_no_improve]
row_bus_decision=list_para_combination_values+['decision-made','bus',count_bus_changed_lane,count_bus_not_changed,count_bus_not_changed_no_improve]
writer_statistic_whole_simulation.writerow(row_car_decision)
writer_statistic_whole_simulation.writerow(row_bus_decision)

######## regarding vehicle's speed by vehicle type and changing lane per simulation
columnhead_of_vehicles_speed_whole_simu=list_para_combination_heads+["statistic-general","statistic-detail","count-vehicles","min-speed","max-speed","aver-speed"]
writer_statistic_whole_simulation.writerow(columnhead_of_vehicles_speed_whole_simu)
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_car_0_aver_speed)>0:
    count=len(list_car_0_aver_speed)
    min_speed=min(list_car_0_aver_speed)
    max_speed=max(list_car_0_aver_speed)
    aver_speed=sum(list_car_0_aver_speed)*1.0/count
row_car_0_speeds=list_para_combination_values+["average-speed","car-no-changed-lanes",count,min_speed,max_speed,aver_speed]
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_car_1_aver_speed)>0:
    count=len(list_car_1_aver_speed)
    min_speed=min(list_car_1_aver_speed)
    max_speed=max(list_car_1_aver_speed)
    aver_speed=sum(list_car_1_aver_speed)*1.0/count
row_car_1_speeds=list_para_combination_values+["average-speed","car-changed-lanes",count,min_speed,max_speed,aver_speed]
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_car_9_aver_speed)>0:
    count=len(list_car_9_aver_speed)
    min_speed=min(list_car_9_aver_speed)
    max_speed=max(list_car_9_aver_speed)
    aver_speed=sum(list_car_9_aver_speed)*1.0/count
row_car_9_speeds=list_para_combination_values+["average-speed","car-not-changed-lanes-as-no-improve",count,min_speed,max_speed,aver_speed]
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_bus_0_aver_speed)>0:
    count=len(list_bus_0_aver_speed)
    min_speed=min(list_bus_0_aver_speed)
    max_speed=max(list_bus_0_aver_speed)
    aver_speed=sum(list_bus_0_aver_speed)*1.0/count
row_bus_0_speeds=list_para_combination_values+["average-speed","bus-no-changed-lanes",count,min_speed,max_speed,aver_speed]
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_bus_1_aver_speed)>0:
    count=len(list_bus_1_aver_speed)
    min_speed=min(list_bus_1_aver_speed)
    max_speed=max(list_bus_1_aver_speed)
    aver_speed=sum(list_bus_1_aver_speed)*1.0/count
row_bus_1_speeds=list_para_combination_values+["average-speed","bus-changed-lanes",count,min_speed,max_speed,aver_speed]
count=0
min_speed=max_speed=aver_speed="NA"   
if len(list_bus_9_aver_speed)>0:
    count=len(list_bus_9_aver_speed)
    min_speed=min(list_bus_9_aver_speed)
    max_speed=max(list_bus_9_aver_speed)
    aver_speed=sum(list_bus_9_aver_speed)*1.0/count
row_bus_9_speeds=list_para_combination_values+["average-speed","bus-not-changed-lanes-as-no-improve",count,min_speed,max_speed,aver_speed]
writer_statistic_whole_simulation.writerow(row_car_0_speeds)
writer_statistic_whole_simulation.writerow(row_car_1_speeds)
writer_statistic_whole_simulation.writerow(row_car_9_speeds)
writer_statistic_whole_simulation.writerow(row_bus_0_speeds)
writer_statistic_whole_simulation.writerow(row_bus_1_speeds)
writer_statistic_whole_simulation.writerow(row_bus_9_speeds)

#!writer_statistics.writerow(["*******************this is the end of output statistics**************************"])


# output features of lanes along the time stamps
writer_lane_features.writerow(["-----------------------following is the speeds of lanes along the time-stamps-----------------------"])
writer_lane_features.writerow(columnheads1)
for lane_id in list_laneids:
    row=list_para_combination_values+[lane_id,dic_lanes[lane_id]['lane-type']]
    for timestamp in list_timestamps:
        row.append(dic_lanes[lane_id]['dic-speeds'][timestamp])
    writer_lane_features.writerow(row)
    
writer_lane_features.writerow(["-----------------------following is the counts of lanes along the time-stamps-----------------------"])
writer_lane_features.writerow(columnheads1)
for lane_id in list_laneids:
    row=list_para_combination_values+[lane_id,dic_lanes[lane_id]['lane-type']]
    for timestamp in list_timestamps:
        row.append(dic_lanes[lane_id]['dic-counts'][timestamp])
    writer_lane_features.writerow(row)

writer_lane_features.writerow(["-----------------------following is the densities of lanes along the time-stamps-----------------------"])
writer_lane_features.writerow(columnheads1)
for lane_id in list_laneids:
    row=list_para_combination_values+[lane_id,dic_lanes[lane_id]['lane-type']]
    for timestamp in list_timestamps:
        row.append(dic_lanes[lane_id]['dic-densities'][timestamp])
    writer_lane_features.writerow(row)

# output features of vehicles 
writer_veh_features.writerow(["-----------------------following is the features of vehicles----------------------"])
writer_veh_features.writerow(columnheads3)
for veh_id in list_vehids:
    vehicle=dic_vehicles[veh_id]
    row=list_para_combination_values+[veh_id,vehicle['veh-type'],vehicle['value-time'],vehicle['lane-chan-p'],vehicle['origin'],vehicle['destination'],vehicle['time-budget'],vehicle['start-timestamp'],vehicle['expected-endtime'],vehicle['end-timestamp'],vehicle['lane-changed']]
    writer_veh_features.writerow(row)


print "there are ", len(dic_vehicles)," vehicles in total in the simulation"
list_num_of_features_vehicle=[]
for veh_id in dic_vehicles:
    if not len(dic_vehicles[veh_id]) in list_num_of_features_vehicle:
        list_num_of_features_vehicle.append(len(dic_vehicles[veh_id]))
print "list of number of features of vehicles is: ", list_num_of_features_vehicle

