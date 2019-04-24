'''
Created on 2018/07/04

This module is generate the different types of vehicles with their profiles.
Currently, three types of vehicle can be included, which are car, bus and truck
The profiles of vehicle include max speed, lane-changing probability, time value so far. 

max speed is constant for each type of vehicle, car 60km/h, bus 40km/h, truck 50km/h.
The value of time is set according the average wage per hour online(https://www.payscale.com/research/SG/Location=Singapore/Hourly_Rate) which is from 7 to 41 SD
Lane-changing probability is set as that the driver whose value of time is more than the middle value in the above is definitely to change for a faster lane when the estimated travel time is more than expected.


@author: gong
'''
   
class Vehicle(object):# vehicle has 19 features
    # actually it is a complex of combination of vehicle and driver, although vehicle and driver have a random feature and are combined together
    def __init__(self,vehicle_id,max_speed,veh_type,driver_type,value_time,lane_chan_probability,origin,destination,time_budget, \
                 vehicle_start_timestamp,vehicle_end_timestamp_expected,veh_list_timestamp,veh_dic_locations,veh_dic_routes, \
                 veh_lane_type,veh_dic_lane_type,veh_lane_changed,veh_list_laneid_used,veh_endtime,veh_dic_speeds,veh_delay_status,veh_delay_time_to, \
                 veh_route_stress):
        self.vehicleid=vehicle_id# int
        self.veh_type=veh_type#vehicle type
        self.max_speed=max_speed#the max speed of this type of vehicle can achieve
        self.driver_type=driver_type#driver type, not applicable to bus driver
        self.value_time=value_time#value of time of the driver, not applicable to bus driver
        self.lane_chan_probability=lane_chan_probability#the probability of changing lane,float from 0 to 1
        self.origin=origin# a node's id; has same features as a node,string
        self.destination=destination# a node's id; has same features as a node,string
        self.time_budget=time_budget# seconds,int
        self.starttime=vehicle_start_timestamp# time to put the vehicle into the road network,string type in yyyy-mm-dd hh:mm:ss
        self.expected_endtime=vehicle_end_timestamp_expected# expected end time decided by the start time and the time budget, string, yyyy-mm-dd hh:mm:ss
        # features below are initialized as blank and no need to be imported from outside  
        self.list_timestamp=veh_list_timestamp# 'yyyy-mm-dd hh:mm:ss', store the time stamps from when the vehicle is generated to when it reaches its destinations. \
                                            #    A simulation step is the difference between each pair of consecutive time stamps.
        self.dic_locations=veh_dic_locations# store the locations of vehicle at each time stamp; key is the time stamp and the value is another dictionary \
                                            # {corr-X:corr-X,corr-Y:corr-Y,lane-id:lane-id} of corresponding location.
        self.dic_routes=veh_dic_routes# store the best path at current time stamp; key is the time stamp and the value is a dictionary containing shortest path \
                                    # by lane ids, {laneid1:laneid3,laneid3:laneid2,laneid2:laneid22,laneid22:laneid43,...}
        
        self.lane_type=veh_lane_type# realtime status, the type of lane network the vehicle is driving on: 0-slow and cheap; 1-fast and expensive
        self.dic_lane_type=veh_dic_lane_type # lane type along the time stamps, in a dictionary whose key is the time stamp, the value is either 0 or 1
        self.lane_changed=veh_lane_changed# whether the vehicle changed lane finally:[0/1/9,time/""]--- 0-did not change,1-changed,9-even change cannot arrive on time
        self.list_laneid_used=veh_list_laneid_used# the lane ids that this vehicle used from origin to destination.
        self.endtime=veh_endtime# the time stamp when the vehicle finishes its trip, i.e. the time stamp that the vehicle vanishes
        self.dic_speeds=veh_dic_speeds# collect instant speed at each simulation step of this vehicle (from generation to vanishment): {time-stamp1:speed1,timestamp2:speed2,timestamp3:speed3,...}
        self.delay_status=veh_delay_status# the delay status of a vehicle, 1 means this vehicle is being delayed at an intersection, 0 otherwise
        self.delay_time_to=veh_delay_time_to# "yyyy/mm/dd, hh:mm:ss" the time until when the vehicle is delayed at an intersection
        self.route_stress=veh_route_stress# best route for minimization stress, in the following format: {node1:node2,node2:node3,...} or {lane1:lane2,lane2:lane3,...} depending on the structure of stress graph


def read_Vehicle(vehicle,dic_vehicles,dic_gen_time_vehids):
    # this function read the features of an object vehicle into the dictionary of vehicles, whose key is the feature name and the value is corresponding to the feature-name
    # another task in this function is to include the time stamp when each vehicle is generated in a dictionary (i.e dic_gen_time_vehids)
    # put the vehicle and its features in the dic_vehicles
    if dic_vehicles.has_key(vehicle.vehicleid):
        print("duplicate vehicle IDs, sth wrong")
    else:# vehicle has 19 features
        dic_vehicles[vehicle.vehicleid]={'vehicle-id':vehicle.vehicleid,'veh-type':vehicle.veh_type,'max-speed':vehicle.max_speed,'driver-type':vehicle.driver_type, \
                                              'value-time':vehicle.value_time,'lane-chan-p':vehicle.lane_chan_probability,'origin':vehicle.origin,'destination':vehicle.destination, \
                                              'time-budget':vehicle.time_budget,'start-timestamp':vehicle.starttime,'expected-endtime':vehicle.expected_endtime, \
                                              'list-timestamps':vehicle.list_timestamp,'dic-locations':vehicle.dic_locations,'dic-routes':vehicle.dic_routes,'lane-type':vehicle.lane_type, \
                                              'dic-lane-type':vehicle.dic_lane_type,'lane-changed':vehicle.lane_changed, 'list-laneid-used':vehicle.list_laneid_used,'end-timestamp':vehicle.endtime, \
                                              'dic-speeds':vehicle.dic_speeds,'delay-status':vehicle.delay_status,'delay-time-until':vehicle.delay_time_to,'route-stress':vehicle.route_stress}       
    # put the vehicle-ids in the dic_gen_time_vehids
    if dic_gen_time_vehids.has_key(vehicle.starttime):
        if vehicle.vehicleid in dic_gen_time_vehids[vehicle.starttime]:
            print ("duplicate vehicle ids generated at the same time stamp,ERROR")
        else:
            dic_gen_time_vehids[vehicle.starttime].append(vehicle.vehicleid)
    else:
        dic_gen_time_vehids[vehicle.starttime]=[vehicle.vehicleid]
    
    return dic_vehicles,dic_gen_time_vehids

'''
# initialize the dictionary of vehicles
dic_vehicles={}# all vehicles; key is the vehicle id and a dictionary containing other features is used as the value. {vehid1:{'veh-type':veh_type,'max-speed':max_speed,....},vehid2:{... ...},vehid3:....} 
dic_gen_time_vehids={} # vehicle ids with their generation time. {time1:[vehid1,vehid2....],time2:[vehid5,vehid6,...],....}
# read vehicle one by one and included in the dictionary of vehicles           
            
'''        
        