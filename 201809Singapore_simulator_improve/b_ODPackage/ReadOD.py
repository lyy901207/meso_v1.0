'''
Created on 2018/07/02
This module is used to read the OD matrices from csv files.

csv file of OD matrix should be named as the time slot with beginning and ending time;
names following the above rule is to suit the loading of traffic demand to the road network;
all csv files for one road network or a possible scenario should be put in one folder.

The format of OD demand in the csv file consists of the following columns:
time slot, vehicle type, origin ID, destination ID,vehicle volume

currently, we use a time slot of 5 minutes. vehicles generated during this time slot are put randomly onto the road network.

done and checked on 2018/7/4 
module of generating vehicles is also included in this module as vehicle-generation is just the next step of reading OD-matrix; and these two modules cannot be separated.

modified on 2019/1/10
a randomly distributed start-time of vehicle was newly added for sensitivity analysis. 
so far, two distributions of start-time of vehicle are included in the simulator. 

@author: gong
'''
import time,datetime
import random
from class_vehicle import *
import numpy as np
from z_ExistingPackages.Dijkstra4 import *#route search algorithms have been extended and stress-based ones are included from 2019/01/10
from c_TimeCalculation.time_calculation import *
from math import ceil,floor
import csv
import os
from a_ReadRoadNetwork.class_lane import *

# pre-set the maximum speed for each type of vehicle. in km/h
preset_max_speed_interval_car=[70,80]
preset_max_speed_interval_truck=[50,70]
preset_max_speed_interval_bus=[40,60]



def read_OD(folderpath,multi_OD_para):
    # this function returns a dictionary of OD matrices, when given a folder which contains the OD files
    list_file_names=os.listdir(folderpath)#return a list containing all names of all files in the folder(folderpath)
    #print list_file_names
    dic_OD={}
    list_O=[]# list containing the origin nodes
    list_D=[]# list containing the destination nodes
    for eachfile in list_file_names:
        #!print eachfile
        each_file_path=os.path.join('%s%s%s' % (folderpath,"/", eachfile))#each path+file
        infile=open(each_file_path,"r")
        infile.readline()#jump the first row in the csv file, which is the column head
        reader=csv.reader(infile)        
        for row in reader:
            #print row
            timeslot   =row[0]#time slot, every 5 minute
            vehicle    =row[1]#vehicle type
            origin     =row[2]#O,node ID
            destination=row[3]#D,node ID
            if not origin in list_O:
                list_O.append(origin)
            if not destination in list_D:
                list_D.append(destination)
            volume     =multi_OD_para*int(row[4])#vehicle volume from O to D.
            OD=origin+"_to_"+destination
            #print OD
            if dic_OD.has_key(timeslot):
                if dic_OD[timeslot].has_key(OD):
                    if dic_OD[timeslot][OD].has_key(vehicle):
                        print("ATTENTION:duplicate vehicle type for same OD during same time slot")
                        dic_OD[timeslot][OD][vehicle]+=volume
                    else:                        
                        dic_OD[timeslot][OD][vehicle]=volume
                else:                    
                    dic_OD[timeslot][OD]={}
                    dic_OD[timeslot][OD][vehicle]=volume
            else:
                dic_OD[timeslot]={}
                dic_OD[timeslot][OD]={}
                dic_OD[timeslot][OD][vehicle]=volume
        #print dic_OD.keys()
    return dic_OD,list_O,list_D

             
def obtain_Timestamp(timeslot):
    # this function converts the time-slot (in string) to time stamps of the start and ending of the time-slot
    # an example of the original format of time-slot can be '7.10-7.14' which means time slot from 7:10:00-7:14:59, totally 300 seconds
    # this function is used to extract 'yyyy-mm-dd 7:10:00' and 'yyyy-mm-dd 7:14:59' from the string of '7.10-7.14'
    # separate the string into two firstly
    split_times=timeslot.split("-")
    start=split_times[0]
    ending=split_times[-1]
    split_start=start.split(".")
    hour_start=split_start[0]
    minu_start=split_start[-1]
    split_ending=ending.split(".")
    hour_end=split_ending[0]
    minu_end=split_ending[-1]
    start_time=hour_start+":"+minu_start+":"+"00"
    end_time=hour_end+":"+minu_end+":"+"59"
    today=datetime.datetime.now().date()# date of today, used to combine with current time to make a complete combination of date and time
    start=str(today)+" "+str(start_time)# date + time
    start=datetime.datetime.strptime(start,"%Y-%m-%d %H:%M:%S")# change the type to datetime from string
    ending=str(today)+" "+str(end_time)
    ending=datetime.datetime.strptime(ending,"%Y-%m-%d %H:%M:%S")  
    return str(start)[:19],str(ending)[:19] # returned values are in string type

def obtain_O_and_D(OD_label):
    # this function returns the label of O and label D, the code ids of origin and destination when inputting a OD label which is the key of a series of OD-values
    # the output is a list containing origin code ID and destination code ID, both of which are strings
    split_od_label=OD_label.split("_to_")
    O=split_od_label[0]
    D=split_od_label[1]
    return O,D# strings

def vehicle_Maxspeed(veh_type):
    # this function returns the max speed at random value during a given interval according to a given vehicle type
    max_speed_interval_car=preset_max_speed_interval_car
    max_speed_interval_truck=preset_max_speed_interval_truck
    max_speed_interval_bus=preset_max_speed_interval_bus
    max_speed=0
    if veh_type=="car":
        max_speed=random.randint(max_speed_interval_car[0],max_speed_interval_car[1])
    elif veh_type=="truck":
        max_speed=random.randint(max_speed_interval_truck[0],max_speed_interval_truck[1])
    elif veh_type=="bus":
        max_speed=random.randint(max_speed_interval_bus[0],max_speed_interval_bus[1])
    else:
        print("error of inputting vehicle type")                
    return max_speed 

def driver_Ty_Gen():
    # this function randomly generate driver type, 0 or 1 (two types of drivers are optional)
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
    #   further complicated function for predicting driver type can be done   #
    # e.g. type decided by demographic features of drivers
    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
    driver_type=random.randint(0,1)
    return driver_type 

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

def lane_Change_P_Gen(driver_type):
    # this function generates the initial value of lane-change-ability for each driver
    # the probability is assumed to be related to the driver-type
    lane_change_p=0# initialize the lane-change-probability as 0 
    if driver_type==0:
        lane_change_p=random.randint(0,2)/10.0
    elif driver_type==1:
        lane_change_p=random.randint(2,5)/10.0
    else:
        print("input driver-type is error for generating lane-change-probability")
    return(lane_change_p)

def vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids):
    # this function is used in the following function, vehicle_gen; 
    # this is the common parts for generating vehicles due to different distribution of generation;
    # it prepares and returns the features of a vehicle, except the generation time stamp (i.e. start_timestamp) which is generated from a given distribution and is provided as an input of this function
    max_speed=vehicle_Maxspeed(vehicle_type)
    driver_type=driver_Ty_Gen()
    value_time=driver_Value_Time_Gen(median_value_time)
    lane_change_p=lane_Change_P_Gen(driver_type)
    ################  ATTENTION: the time-budget is obtained as the travel time calculated on the low-graph, if all vehicles are generated before putting them on the network,\
    ################             it is the optimal one that is the free-flow travel time on the low-graph.
    ################             Otherwise, if the vehicles are generated with time-steps in simulation, it is the real-time optimal travel time in which there are vehicles on the network.
    veh_lane_type=0
    
    dic_graph={}# newly added on 2019/2/16; it make the driver who has a higher value of time directly select the fast-lane system at the beginning of departure
    if value_time>=median_value_time*1.2:
        dic_graph=copy.deepcopy(dic_graph_high)
        veh_lane_type=1
    else:
        dic_graph=copy.deepcopy(dic_graph_low)
        veh_lane_type=0
    
    
    dic_time_budgets,dic_best_route_nodes=shortest_Path_in_Node(dic_graph, O, D)  
    dic_best_route_lanes=convert_PPath_to_PathIDs(dic_best_route_nodes,dic_graph,O,D)
    next_nodeid=dic_best_route_nodes[O]    
                     
    time_budget=ceil(dic_time_budgets[D])
    vehicle_end_timestamp_expected=add_Timestamp(vehicle_start_timestamp, time_budget)###########!!!make sure time-budget is in seconds
    veh_list_timestamps=[vehicle_start_timestamp]
    lane_id_initial=dic_graph[O][next_nodeid]['lane-id']
    veh_dic_locations={vehicle_start_timestamp:{'coor-X':dic_nodes[O]['coor-X'],'coor-Y':dic_nodes[O]['coor-Y'],'lane-id':lane_id_initial}}
    veh_dic_routes={vehicle_start_timestamp:dic_best_route_lanes}
    veh_dic_lane_type={vehicle_start_timestamp:veh_lane_type}
    
    veh_lane_changed=[0,"","","",""]# dummy-of-change, and time stamp of changing lanes
    veh_list_laneid_used=[lane_id_initial]
    veh_endtime=''
    speed=dic_graph[O][next_nodeid]['speed']
    veh_dic_speeds={vehicle_start_timestamp:speed}
    veh_delay_status=0
    veh_delay_time_to=""
    veh_route_stress={}
    vehicle=Vehicle(vehicle_id,max_speed,vehicle_type,driver_type,value_time,lane_change_p,O,D,time_budget,vehicle_start_timestamp, \
                    vehicle_end_timestamp_expected,veh_list_timestamps, veh_dic_locations,veh_dic_routes,veh_lane_type, \
                    veh_dic_lane_type,veh_lane_changed,veh_list_laneid_used,veh_endtime,veh_dic_speeds, \
                    veh_delay_status, veh_delay_time_to,veh_route_stress)                            
    dic_vehicles,dic_time_vehids=read_Vehicle(vehicle,dic_vehicles,dic_time_vehids)
    return dic_vehicles,dic_time_vehids    


def vehicle_Gen(OD_matrix,gen_distribution,vehicle_id,dic_graph_low,dic_graph_high,median_value_time,dic_nodes,dic_vehicles,dic_time_vehids,mu,sigma):
    # together with previous function, this function generates vehicles needed to be put into the road network based on the OD value
    # this function mainly generates the start-time of vehicles
    # OD_value is a value of a OD pair in OD-matrix
    # gen_distribution is the distribution of vehicle generated and put onto the road network
    # the output are two dictionaries and the updated vehicle_id: \
    # \ one is the dictionary of vehicles which will be used in the whole simulation period;
    # \ another is the dictionary of time-vehicle-ids which indicate at each time stamp, which vehicles should be generated
    # this function is used together with its previous function---vehicle_features_except_time
    # a key feature of a vehicle is the start time stamp

    if gen_distribution=="uniform-in-ODpair-VehType-TimeSlot":
        # "uniform" distribution here is actually is uniform for each OD pair in a small time slot
        # the vehicles are evenly generated/put onto the network during the time slot
        # even generated in a uniform way, the start-time-stamp of vehicle has to be modified a bit to suit the convenience of time-step for simulation
        # a random temporal-skewness is added to the starting time of each OD pair/vehicle type/time slot to avoid the vehicles are generated in a concentrate way
         
        for timeslot in OD_matrix.keys():
            for OD_label in OD_matrix[timeslot].keys():
                for vehicle_type in OD_matrix[timeslot][OD_label]:######******vehicle-type                    
                    OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                    start_timestamp,end_timestamp=obtain_Timestamp(timeslot)# two time stamps are in string type
                    #print start_timestamp,end_timestamp
                    #print type(start_timestamp),type(end_timestamp)
                    duration_timeslot=cal_Time(start_timestamp,end_timestamp)# the duration of time slot of each csv file; unit in seconds
                    random_time_offset=random.randint(0,10)# 0-9 seconds
                    start_timestamp=get_Nex_Timestamp(start_timestamp, random_time_offset)
                    O,D=obtain_O_and_D(OD_label)######*******Origin and Destination
                    if OD_value>0:                
                        aver_interval=1.0*duration_timeslot/OD_value # the interval later will be modified to a up-close integer                       
                        for i in range(0,OD_value):# totally generate OD_value vehicles in this iteration
                            vehicle_id+=1
                            vehicle_start_timestamp=add_Timestamp(start_timestamp,int(floor(aver_interval*i)))######******* the time stamp when the vehicle is put on the network; in string type                                                                                                                                                             
                            ########*********obtain other features of this vehicle*********########
                            dic_vehicles,dic_time_vehids=vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids)
    elif gen_distribution=="uniform-in-whole-generation-period":
        # all vehicles no matter vehicle type or OD pair, or even small time-slot, are generated in a uniform style during the whole vehicle-generation-period
        ########### obtain the total OD value and the time stamps of OD starts and ends: to get the average time interval of vehicle generation
        list_time_start_end_stamps=[]# collect all the time stamps of OD start and end
        total_OD_value=0 # count the total number of vehicles in the OD matrix; not in pcu
        for timeslot in OD_matrix.keys():
            start_timestamp,end_timestamp=obtain_Timestamp(timeslot)# two time stamps are in string type                    
            if not start_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(start_timestamp)
            if not end_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(end_timestamp) 
                           
            for OD_label in OD_matrix[timeslot].keys():
                for vehicle_type in OD_matrix[timeslot][OD_label]:######******vehicle-type                    
                    OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                    total_OD_value+=OD_value
                    
        list_time_start_end_stamps.sort() # re-order the time stamps of OD starts and ends
        start_timestamp=list_time_start_end_stamps[0]
        end_timestamp=list_time_start_end_stamps[-1]

        duration_whole_generation=cal_Time(start_timestamp, end_timestamp)
        if duration_whole_generation>0 and total_OD_value>0:
            aver_interval=1.0*duration_whole_generation/total_OD_value
            i=0
            for timeslot in OD_matrix.keys():
                for OD_label in OD_matrix[timeslot].keys():
                    O,D=obtain_O_and_D(OD_label)
                    for vehicle_type in OD_matrix[timeslot][OD_label]:
                        OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                        for j in range(0,OD_value):
                            vehicle_id+=1                           
                            vehicle_start_timestamp=add_Timestamp(start_timestamp, int(floor(aver_interval*i)))
                            i+=1
                            dic_vehicles,dic_time_vehids=vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids)

    elif gen_distribution=="random-in-ODpair-VehType-TimeSlot":
        # the vehicles are randomly generated/put onto the network during the time slot
        # a random time-stamp during the time slot  
        # the calculated start-time-stamp of vehicle has to be modified a bit to suit the convenience of time-step for simulation
        
        for timeslot in OD_matrix.keys():
            for OD_label in OD_matrix[timeslot].keys():
                for vehicle_type in OD_matrix[timeslot][OD_label]:######******vehicle-type                    
                    OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                    start_timestamp,end_timestamp=obtain_Timestamp(timeslot)# two time stamps are in string type
                    #print start_timestamp,end_timestamp
                    #print type(start_timestamp),type(end_timestamp)
                    duration_timeslot=cal_Time(start_timestamp,end_timestamp)# the duration of time slot of each csv file; unit in seconds
                    O,D=obtain_O_and_D(OD_label)######*******Origin and Destination
                    if OD_value>0:                
                        random_interval=random.randint(0,duration_timeslot-1) # the random interval later will be added to the start-time-stamp of current time slot                       
                        for i in range(0,OD_value):# totally generate OD_value vehicles in this iteration
                            vehicle_start_timestamp=add_Timestamp(start_timestamp,random_interval)######******* the time stamp when the vehicle is put on the network; in string type      
                            vehicle_id+=1                                                                                                                                                       
                            ########*********obtain other features of this vehicle*********########
                            dic_vehicles,dic_time_vehids=vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids)                                   
    
    elif gen_distribution=="random-in-whole-generation-period":
        # the vehicles are randomly generated during the whole period of generation duration in the OD matrix
        ########### obtain the total OD value and the time stamps of OD starts and ends: to get the average time interval of vehicle generation
        list_time_start_end_stamps=[]# collect all the time stamps of OD start and end
        total_OD_value=0 # count the total number of vehicles in the OD matrix; not in pcu
        for timeslot in OD_matrix.keys():
            start_timestamp,end_timestamp=obtain_Timestamp(timeslot)# two time stamps are in string type                    
            if not start_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(start_timestamp)
            if not end_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(end_timestamp)            
            
            for OD_label in OD_matrix[timeslot].keys():
                for vehicle_type in OD_matrix[timeslot][OD_label]:######******vehicle-type                    
                    OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                    total_OD_value+=OD_value

        list_time_start_end_stamps.sort() # re-order the time stamps of OD starts and ends
        start_timestamp=list_time_start_end_stamps[0]
        end_timestamp=list_time_start_end_stamps[-1]
        duration_whole_generation=cal_Time(start_timestamp, end_timestamp)
                
        if duration_whole_generation>0 and total_OD_value>0:       
            for timeslot in OD_matrix.keys():
                for OD_label in OD_matrix[timeslot].keys():
                    O,D=obtain_O_and_D(OD_label)
                    for vehicle_type in OD_matrix[timeslot][OD_label]:
                        OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                        for i in range(0,OD_value):
                            vehicle_id+=1
                            random_interval=random.randint(0,duration_whole_generation-1)
                            vehicle_start_timestamp=add_Timestamp(start_timestamp,random_interval)######******* the time stamp when the vehicle is put on the network; in string type      
                            dic_vehicles,dic_time_vehids=vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids)                                   
    elif gen_distribution=="normal-in-whole-generation-period":     
        # the vehicles are generated following a normal distribution in the whole vehicle generation period
        list_time_start_end_stamps=[]# collect all the time stamps of OD start and end
        total_OD_value=0 # count the total number of vehicles in the OD matrix; not in pcu
        for timeslot in OD_matrix.keys():
            start_timestamp,end_timestamp=obtain_Timestamp(timeslot)# two time stamps are in string type                    
            if not start_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(start_timestamp)
            if not end_timestamp in list_time_start_end_stamps:
                list_time_start_end_stamps.append(end_timestamp)            
            
            for OD_label in OD_matrix[timeslot].keys():
                for vehicle_type in OD_matrix[timeslot][OD_label]:######******vehicle-type                    
                    OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                    total_OD_value+=OD_value

        list_time_start_end_stamps.sort() # re-order the time stamps of OD starts and ends
        start_timestamp=list_time_start_end_stamps[0]
        end_timestamp=list_time_start_end_stamps[-1]
        
        duration_whole_generation=cal_Time(start_timestamp, end_timestamp)

        
        list_time_stamps_veh_starts=[]# initialize the list containing all the vehicles' start time stamps
        list_time_stamps_veh_starts=start_Time_Normal_Distri(start_timestamp, end_timestamp, total_OD_value,mu,sigma)
        j=0# self-increasing variable whose max is total_OD_value
        
        if duration_whole_generation>0 and total_OD_value>0:       
            for timeslot in OD_matrix.keys():
                for OD_label in OD_matrix[timeslot].keys():
                    O,D=obtain_O_and_D(OD_label)
                    for vehicle_type in OD_matrix[timeslot][OD_label]:
                        OD_value=OD_matrix[timeslot][OD_label][vehicle_type]
                        for i in range(0,OD_value):
                            vehicle_id+=1
                            vehicle_start_timestamp=list_time_stamps_veh_starts[j]                
                            dic_vehicles,dic_time_vehids=vehicle_Features_except_Time(vehicle_id,vehicle_type,median_value_time,dic_graph_low,dic_graph_high,O,D,vehicle_start_timestamp,dic_nodes,dic_vehicles,dic_time_vehids)
                            j+=1                                   
    
    else:
        print("wrong input for vehicle-generation distribution")    
    
    return dic_vehicles,dic_time_vehids,vehicle_id



'''
# follows is the test of this module only  
#######################################################################################################################
###     ######   ######      #####    ######
###       ##     ##        ###          ##
###       ##     #####       ####       ##
###       ##     ##             ###     ##
###       ##     ######     #####       ##
#######################################################################################################################

# set the random see
random.seed(10)

# set/calculate the median of value-of-time used in the study
median_value_time=50# the median of value-of-time in SG, used to the check the person  
# initialize vehicle-id; it is a global variable, as there may be many OD-matrix
vehicle_id=0# the first vehicle id is 1 and is increased subsequently with a step of 1
# initialize the time-step used for update during the simulation
time_step=1# unit in sec
dic_vehicles={}# initialize the dictionary of vehicles
dic_time_vehids={}
##############################################################################################################
# as the vehicle objects need the initialized network information, the geometry needs to be read, too.
from a_ReadRoadNetwork.readgeometry import *
dic_graph_high={}
dic_graph_low={}
dic_lanes={}
dic_nodes={}
# import csv files where the node and lane data are stored
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
print "        "
print "the items of lanes are printed as follows"
for key in dic_lanes:
    print dic_lanes[key]

print "finish printing the features of lanes in the dictionary of dic_lanes"
#############################################################################################################
#update the file path of folder containing the OD matrices as below
folderpath=r"D:\03Work during PD\31ERP2_in_COI\meso_by_python\OD_data"

OD_matrices=read_OD(folderpath)
dic_vehicles,dic_time_vehids,vehicle_id=vehicle_Gen(OD_matrices, 'uniform',vehicle_id,dic_graph_low,median_value_time,dic_nodes,dic_vehicles,dic_time_vehids)
print len(dic_vehicles)

print "now printing the features of generated vehicles"
for key in dic_vehicles:
    print key,dic_vehicles[key]
'''