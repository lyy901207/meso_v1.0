'''
Created on 2018/07/10

@author: gong
'''
from class_node import Node
from class_lane import Lane
import csv,copy,datetime
from math import *

def cal_Dist(p1,p2):
    # the input two points are nodes defined with same features in node class
    # p1, p2 are two dictionaries which have keys of coor-X and coor-Y
    return(((p1['coor-X']-p2['coor-X'])**2+ (p1['coor-Y']-p2['coor-Y'])**2)**(0.5))   



def read_Nodes(nodefile,dic_nodes):
    # this function reads nodes from csv file to the dic_nodes
    reader=csv.reader(nodefile)
    for row in reader:
        node=Node(row)
        if not dic_nodes.has_key(node.nodeid):
            dic_nodes[node.nodeid]={'node-type':node.nodetype,'coor-type':node.coortype,'coor-X':node.coorX,'coor-Y':node.coorY,'delay':node.delay}
        else:
            print("duplicate node id, ATTENTION")
    return dic_nodes

def read_Lanes(lanefile,dic_graph_high,dic_graph_low,dic_lanes):
    # update the dictionary of graphs and lanes. How many dictionary of graphs depends on how many types of lanes used in the analysis.
    # this function reads lanes from csv file to the dic_graph_low and dic_graph_high depending on the lane type
    reader=csv.reader(lanefile)
    count0=0# used to check whether the total number of read lanes is equal to the csv file
    count1=0
    for row in reader:
        lane=Lane(row)
        # include lanes to dic_graph_low and dic_graph_high depending on the lane category
        # lane has 18 features
        if int(lane.lanetype)==0:# the low-speed-type lane
            if dic_graph_low.has_key(lane.nodeid1):
                if not dic_graph_low[lane.nodeid1].has_key(lane.nodeid2):                    
                    count0+=1
                    dic_graph_low[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                               'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                               'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                               'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                               'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                               'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}
                else:
                    print('duplicate lanes in dic-graph-low, ATTENTION',lane.nodeid1,lane.nodeid2)
            else:
                count0+=1
                dic_graph_low[lane.nodeid1]={}
                dic_graph_low[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                           'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                           'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                           'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                           'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                           'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}    
        elif int(lane.lanetype)==1:# the high-speed-type lane
            if dic_graph_high.has_key(lane.nodeid1):
                if not dic_graph_high[lane.nodeid1].has_key(lane.nodeid2):
                    count1+=1
                    dic_graph_high[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                               'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                               'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                               'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                               'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                               'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}
                else:
                    print('duplicate lanes in dic-graph-high, ATTENTION',lane.nodeid1,lane.nodeid2)
            else:
                count1+=1
                dic_graph_high[lane.nodeid1]={}
                dic_graph_high[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                           'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                           'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                           'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                           'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                           'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}              
        # include the lane into dic_lanes
        if dic_lanes.has_key(lane.laneid):
            print('duplicate lane ids in dic-lanes, error in the data set',lane.laneid)
        else:
            dic_lanes[lane.laneid]={'lane-type':lane.lanetype,'node-id1':lane.nodeid1,'node-id2':lane.nodeid2, 'link-id':lane.linkid, \
                                   'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                   'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                   'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                   'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                   'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}
    # check the number of items in the dictionaries
    #print('count of 1st type lane is ',count0)
    #print("count of 2nd type lane is ",count1)
    return dic_graph_high,dic_graph_low,dic_lanes

def read_Lanes2(lanefile,dic_graph,dic_lanes):
    #similar to ReadLane, just one dic_graph; this is for a scenario without distinguishing lanes with different specifications
    # this function reads lanes from csv file to the dic_graph
    reader=csv.reader(lanefile)
    count0=0# used to check whether the total number of read lanes is equal to the csv file    
    for row in reader:
        lane=Lane(row)
        # include lanes to dic_graph_low and dic_graph_high depending on the lane category
        # lane has 18 features
        if int(lane.lanetype)==0:
            if dic_graph.has_key(lane.nodeid1):
                if not dic_graph[lane.nodeid1].has_key(lane.nodeid2):                    
                    count0+=1
                    dic_graph[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                               'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                               'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                               'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                               'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                               'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}
                else:
                    print('duplicate lanes in dic-graph-low, ATTENTION',lane.nodeid1,lane.nodeid2)
            else:
                count0+=1
                dic_graph[lane.nodeid1]={}
                dic_graph[lane.nodeid1][lane.nodeid2]={'lane-type':lane.lanetype,'lane-id':lane.laneid,'link-id':lane.linkid, \
                                                           'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                                           'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                                           'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                                           'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                                           'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}    
              
            # include the lane into dic_lanes
            if dic_lanes.has_key(lane.laneid):
                print('duplicate lane ids in dic-lanes, error in the data set',lane.laneid)
            else:
                dic_lanes[lane.laneid]={'lane-type':lane.lanetype,'node-id1':lane.nodeid1,'node-id2':lane.nodeid2, 'link-id':lane.linkid, \
                                       'link-type':lane.linktype,'length':lane.length,'free-speed':lane.freespeed, \
                                       'free-time':lane.freetraveltime,'speed':lane.speed,'density':lane.density, \
                                       'travel-time':lane.traveltime,'fixed-charge':lane.fixedcharge, \
                                       'dynamic-charge':lane.charge,'counts':lane.counts,'dic-speeds':lane.dic_speeds, \
                                       'dic-counts':lane.dic_counts,'dic-densities':lane.dic_densities}
    # check the number of items in the dictionaries
    #print('count of 1st type lane is ',count0)
    #print("count of 2nd type lane is ",count1)
    return dic_graph,dic_lanes

def length_Cal(dic_graph,dic_nodes):
    # this function calculate and update the length of lanes inside a graph with the node id coordinates
    # calculate the length of lanes in dic_graph which uses node-ids as the key 
    # update the free-travel-time and travel-time in condition of 0 density
    for key1 in dic_graph:
        for key2 in dic_graph[key1]:
            dic_graph[key1][key2]['length']=cal_Dist(dic_nodes[key1],dic_nodes[key2])/1000.0# coordinates are in meter; length is in km
            dic_graph[key1][key2]['free-time']=dic_graph[key1][key2]['length']*1.0/dic_graph[key1][key2]['free-speed']*3600.0# travel time is in seconds
            free_time=copy.deepcopy(dic_graph[key1][key2]['free-time'])
            dic_graph[key1][key2]['travel-time']=free_time*1.25
    return dic_graph

def length_Cal2(dic_lanes,dic_nodes):
    # this function calculate and update the length of lanes inside a graph with the node id coordinates 
    # calculate the length of lanes in dic_lanes which uses lane-id as the key
    # update the free-travel-time and travel-time in condition of 0 density
    for key1 in dic_lanes:
        dic_lanes[key1]['length']=cal_Dist(dic_nodes[dic_lanes[key1]['node-id1']],dic_nodes[dic_lanes[key1]['node-id2']])/1000.0# coordinates are in meter; length is in km
        dic_lanes[key1]['free-time']=dic_lanes[key1]['length']*1.0/dic_lanes[key1]['free-speed']*3600.0# travel time is in seconds
        free_time=copy.deepcopy(dic_lanes[key1]['free-time'])
        dic_lanes[key1]['travel-time']=free_time*1.25
    return dic_lanes
def direction_Cal_Not_In_Usage(point1,point2,point):
    # currently not in usage
    # this function calculates whether point is at the left side or right side of line connected by point1 and point2
    # line has direction from point1 to point2(pointing to point2)
    
    # The coordinates of the nodes/points should be Cartesian coordinates; cannot be geographical coordinates.
    x1=1.0*point1.coorX
    y1=1.0*point1.coorY
    x2=1.0*point2.coorX
    y2=1.0*point2.coorY
    x =1.0*point.coorX
    y =1.0*point.coorY
    
    direction="" 
    tmp=(y1-y2)*x+(x2-x1)*y+x1*y2-x2*y1
    if tmp>0:# point is at left side of the line
        direction="left"
    elif tmp<0:# point is at the right side of the line
        direction="right"
    else:# the point is on the line
        direction="on"
    return tmp, direction


def degree_Cal(p1,p2,dic_nodes):
    # this function returns the degree of of ray with starting point of p1 and end point of p2; p1 and p2 are node ids
    # p1 is taken as the origin at a Cartesian system. The degree of ray is the angle between ray and the positive horizon axis
    # this function works by checking the position of the ray and then adjusting the result calculated by arctan value of delta_y/delta_x
    
    # the output of this function is from 0 to 2pi
    # input point has the same features as node in the road network. 
    
    x1=float(dic_nodes[p1]['coor-X'])
    y1=float(dic_nodes[p1]['coor-Y'])
    x2=float(dic_nodes[p2]['coor-X'])
    y2=float(dic_nodes[p2]['coor-Y'])
    degree=0# initialize the output value
    delta_y=y2-y1
    delta_x=x2-x1
    if delta_y>=0 and delta_x>0: # Quadrant I
        degree=atan(delta_y/delta_x)
    elif delta_y>=0 and delta_x<0: # Quadrant II
        degree=pi+atan(delta_y/delta_x)
    elif delta_y<=0 and delta_x<0: # Quadrant III
        degree=pi+atan(delta_y/delta_x)
    elif delta_y<=0 and delta_x>0: # Quadrant Iv
        degree=2*pi+atan(delta_y/delta_x)
    elif delta_y>0 and delta_x==0:
        degree=pi/2.0
    elif delta_y<0 and delta_x==0:
        degree=3.0*pi/2.0
    else:
        print ("error when calculating the degree")  
    return degree

def turned_Degree_with_Dir(lane12,lane23,rule_of_road,dic_nodes):
    # this function returns the direction changed when a vehicle moving from lane12 to lane23; these two lanes are connected by intersection of node2
    
    # if the output degree difference is more than 2pi or less than -2pi, we need to minus 2pi or plus 2pi to make sure the result is from -pi to pi.
    # the output is from -pi to pi, inside which the positive means left turn while the negative means right turn.
    node11=lane12['node-id1']# id of node1 of lane1
    node12=lane12['node-id2']# id of node2 of lane1
    node21=lane23['node-id1']# id of node1 of lane2
    node22=lane23['node-id2']# id of node2 of lane2
    turned_degree=0 # initialize the turned_degree
    if node12<>node21:
        print ("error, two lanes is not connected by the same node")
    else:
        degree12=degree_Cal(node11, node12,dic_nodes)
        degree23=degree_Cal(node21, node22,dic_nodes)
        turned_degree=degree23-degree12
        if turned_degree>pi:
            turned_degree=turned_degree-2*pi
        elif turned_degree<-pi:
            turned_degree=turned_degree+2*pi
        if turned_degree==pi or turned_degree==-pi:
            if rule_of_road=="Left":
                turned_degree=-pi
            elif rule_of_road=="Right":
                turned_degree=pi
    return turned_degree # positive means left-turn while negative means right-turn; zero means strict straight and slightly moving left or right is not treated as through traffic or straight.
        
def turn_Gen(dic_nodes,dic_lanes,rule_of_road):
    # this function outputs a dictionary of turn and degree of each pair of lanes(one inbound and one outbound of an intersection) at each intersection( or node).
    # the inbound lane and outbound lane can be different types of lanes(slow lane and fast lane)
    # roundabout is not applicable.
    dic_turns={}# output; in the format as follows: dic_turns[laneid1][laneid2]={value:value, direction:left/right/straight,degree:degree},laneid1 and laneid2 must be two of the legs joint at intersection of this node
    for nodeid in dic_nodes:# at intersection of nodeid
        list_lane_start=[]# store the lane id which starts from nodeid
        list_lane_end=[]# store the lane id which ends at nodeid

        # check and assign the lane-ids to the above two lists 
        for laneid in dic_lanes:
            if dic_lanes[laneid]['node-id1']==nodeid:# laneid starts from nodeid
                if not laneid in list_lane_start:
                    list_lane_start.append(laneid)
                else:
                    print("ERROR, same lanes occur")
            if dic_lanes[laneid]['node-id2']==nodeid:# laneid ends at nodeid
                if not laneid in list_lane_end:
                    list_lane_end.append(laneid)
                else:
                    print("ERROR, same lanes occur")
        # calculate the values of dic_turns
        for laneid1 in list_lane_end:# laneid1 ends at nodeid
            lane1=dic_lanes[laneid1]
            dic_turns[laneid1]={}
            for laneid2 in list_lane_start:# laneid2 starts from nodeid
                
                lane2=dic_lanes[laneid2]
                turned_degree=turned_Degree_with_Dir(lane1, lane2,rule_of_road,dic_nodes)
                dire="NA"
                if turned_degree>0:
                    dire="left"
                    if turned_degree==pi:
                        dire="U-turn"
                elif turned_degree<0:
                    dire="right"
                    if turned_degree==-pi:
                        dire="U-turn"
                else:
                    dire="through"
                dic_turns[laneid1][laneid2]={'turned-degree':turned_degree,'direction':dire,'abs degree':abs(turned_degree)}# turned-degree is a value with positive or negative sign
    return dic_turns
def cal_Stress_from_Turn(dire,turned_degree,rule_of_road):
    # this function quantifying the stress from the turning direction and degree turned of a vehicle at an intersection (or node)
    # input is direction turned, degree turned, rule of the road
    # the output is value/number used to describe stress in a quantitative way
    # U-turn is assumed to have the largest stress, while right-turn has a larger stress compared to left-turn at a same degree in case of left-hand traffic
    # a linear function between stress and degree turned is used currently, as no existing research results regarding this relationship has been found.
    # absolute value of degree turned is used in the linear function, and additional parameters for U-turn is 3, right-turn is 2, and left-turn is 1, in case of left-hand traffic.
    
    # decide the parameter in the linear function
    if dire=="U-turn":
        para=3
    elif dire=="through":
        para=0
    elif dire=="left" and rule_of_road=="Left":
        para=1
    elif dire=="left" and rule_of_road=="Right":
        para=2
    elif dire=="right" and rule_of_road=="Left":
        para=2
    elif dire=="right" and rule_of_road=="Right":
        para=1
    else:
        print "Error when quantifying stress"
    stress=para*float(turned_degree)
    #return {'stress':stress,'direction':dire,'turned-degree':turned_degree}
    return stress

def graph_Stress(dic_turns,dic_lanes,rule_of_road):
    # this function converts the dictionary of turned degree and turned direction between each pair of lanes to quantified stress
    # the output graph of stress is used in the stress minimization based route search.
    
    #########################################################################
    ## so the formats of the output graphs are consistent in the Dijkstra4 ##
    # two structures, corresponding to G_stress1 and G_stress2 in Dijkstra4 #
    #########################################################################
    
    # two steps.
    # 1st step, convert the dictionary of turns whose key is (lane id and lane id) to (node id and node id and node id).
    # 2nd step, calculate the stress by the function of Cal_Stress_from_turn
    
    # input of dic_turns is the dictionary of turns from the function of TurnGen, which also shows the format of dic_turns
    # input of dic_lanes is the dictionary of lanes whose key is the lane-id
    # input of rule_of_road is the whether left-hand or right-hand traffic
    
    # e.g., two lanes connected by a node whose id is 2. lane1 is from node1 to node2, while lane2 is from node2 to node3.
    # it means that the stress of lane2 is not a fixed value, depending on the previous connected lane which decides the direction turned and degree turned.
    
    # format of output: dic_graph_stress[node2][node3]={node1:stress-value1,node11:stress-value11,node12:stress-value12...}, where 
    # node1, node11,node12 are the node ids which are upstream node ids of lanes connected to lane from node2 to node3 at node2  
    dic_graph_stress1={}# initialize the graph based on stress, structure 1
    dic_graph_stress2={}# initialize the graph based on stress, structure 2
    for from_laneid in dic_turns:
        node1=dic_lanes[from_laneid]['node-id1']
        dic_graph_stress2[from_laneid]={}
        for to_laneid in dic_turns[from_laneid]:
            dire=dic_turns[from_laneid][to_laneid]['direction']
            turned_degree=dic_turns[from_laneid][to_laneid]['abs degree']           
            node2=dic_lanes[to_laneid]['node-id1']
            node3=dic_lanes[to_laneid]['node-id2']
            stress=cal_Stress_from_Turn(dire, turned_degree, rule_of_road)            
            
            # write dic_graph_stress2
            if dic_graph_stress2[from_laneid].has_key(to_laneid):
                if dic_graph_stress2[from_laneid][to_laneid]<>stress:
                    print "stress is not the same for the same turnings"
            else:
                dic_graph_stress2[from_laneid][to_laneid]=stress
            
            # write dic_graph_stress1
            if dic_graph_stress1.has_key(node2):
                if dic_graph_stress1[node2].has_key(node3):
                    if dic_graph_stress1[node2][node3].has_key(node1):# check if the quantified stress are the same                        
                        if stress<>dic_graph_stress1[node2][node3][node1]:
                            print "error: same direction at same node but different quantified stress"
                    else:
                        dic_graph_stress1[node2][node3][node1]=stress
                else:
                    dic_graph_stress1[node2][node3]={}
                    dic_graph_stress1[node2][node3][node1]=stress
            else:
                dic_graph_stress1[node2]={}
                dic_graph_stress1[node2][node3]={}
                dic_graph_stress1[node2][node3][node1]=stress
                
    return dic_graph_stress1,dic_graph_stress2                
def fixed_Charge_Update(dic_graph,charge_rate):
    # this function assign the charge_rate to each lane as a fixed charge rate
    for key1 in dic_graph:
        for key2 in dic_graph[key1]:
            dic_graph[key1][key2]['fixed-charge']=copy.deepcopy(charge_rate)
    return dic_graph
def fixed_Charge_Update2(dic_graph,charge_rate):
    # this function assign the charge_rate to each lane as a fixed charge rate
    for lane_id in dic_graph: 
        if dic_graph[lane_id]['lane-type']==1:
            dic_graph[lane_id]['fixed-charge']=charge_rate
        else:
            dic_graph[lane_id]['fixed-charge']=0
    return dic_graph      
    
'''
# follows is the test of this module only  
#######################################################################################################################
###     ######   ######      #####    ######
###       ##     ##        ###          ##
###       ##     #####       ####       ##
###       ##     ##             ###     ##
###       ##     ######     #####       ##
#######################################################################################################################

# dictionary to store the information of all nodes
# key is the node id, value is another dictionary which contains features of the node
# an example in the dictionary is as follows: 
# node-id:{node-type:value0,coordinate-type:value1, coor-X:value2, coor-Y:value3]
dic_nodes={}

# dictionary to store the information of all lanes
# one dictionary is a network composed by a certain type of lanes
# key is the lane id, value is another dictionary whose key is the starting node id and value is the third dictionary containing features of lane
# an example in the dictionary is as follows:
# node1-id:{node2-id:{lane-type:value0,lane-id:value1,link-id:value2,link-type:value3,length:value4,free-speed:value5, \
#           free-time:value6,speed:value7,density:value8,travel-time:value9,fixed-charge:value10, \
#           dynamic-charge:value11},node2-id2:{...},...}
# this structure of dictionary only consider only one lane for one lane-type inside a link;otherwise, one more dictionary is needed.
dic_graph_low={}# all lanes belonging to the low fee type
dic_graph_high={}# all lanes belonging to the high fee type

# another dictionary of lane is defined as follows: key is lane id, the value is another dictionary which include other features of the lane
dic_lanes={}
# two types of dictionaries of lane should be updated simultaneously.

# set the rule of road
rule_of_road="Left"

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

# check whether the length is correctly 
for key1 in dic_graph_low:# for any node-id1
    for key2 in dic_graph_low[key1]:# for any node-id2
        print dic_graph_low[key1][key2]
        print dic_graph_high[key1][key2]
        print dic_lanes[dic_graph_low[key1][key2]['lane-id']]


# check the dictionary of turns
outfile_turns=open(r"D:\03Work during PD\31ERP2_in_COI\Sioux Falls network\output_turns-SiouxFalls_gong.csv","ab")
writer_turns=csv.writer(outfile_turns)
# write the current time for distinguishing the implementing date/time
nowTime=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
writer_turns.writerow([nowTime])
# write the head of each column
list_heads_turn=['from-lane-id','to-lane-id','from-lane-nodeid1','from-lane-nodeid2','to-lane-nodeid1','to-lane-nodeid2','direction','abs-degree-turned','original-turned-degree']
writer_turns.writerow(list_heads_turn)

dic_turns=turn_Gen(dic_nodes,dic_lanes,rule_of_road)

for lane1 in dic_turns:
    for lane2 in dic_turns[lane1]:
        turns=dic_turns[lane1][lane2]
        record=[lane1,lane2,dic_lanes[lane1]['node-id1'],dic_lanes[lane1]['node-id2'],dic_lanes[lane2]['node-id1'],dic_lanes[lane2]['node-id2'],turns['direction'],turns['abs degree'],turns['turned-degree']]
        writer_turns.writerow(record)

# check the dictionary of graph of stress
dic_graph_stress1,dic_graph_stress2=graph_Stress(dic_turns, dic_lanes, rule_of_road)
# location and file for outputting the graph of stress in structure 1 
outfile_graph_stress1=open(r"D:\03Work during PD\31ERP2_in_COI\Sioux Falls network\output_graph_stress1-SiouxFalls_gong.csv","ab")
writer_graph_stress1=csv.writer(outfile_graph_stress1)
writer_graph_stress1.writerow([nowTime])
# location and file for outputting the graph of stress in structure 2 
outfile_graph_stress2=open(r"D:\03Work during PD\31ERP2_in_COI\Sioux Falls network\output_graph_stress2-SiouxFalls_gong.csv","ab")
writer_graph_stress2=csv.writer(outfile_graph_stress2)
writer_graph_stress2.writerow([nowTime])


list_heads_graph_stress1=["at-node-id","to-node-id","from-node-id","quantified-stress","direction","turned-degree"]
writer_graph_stress1.writerow(list_heads_graph_stress1)
list_heads_graph_stress2=["from-lane-id","to-lane-id","quantified-stress","direction","turned-degree"]
writer_graph_stress2.writerow(list_heads_graph_stress2)
for node2 in dic_graph_stress1:
    for node3 in dic_graph_stress1[node2]:
        for node1 in dic_graph_stress1[node2][node3]:
            stress=dic_graph_stress1[node2][node3][node1]
            lane12id=dic_graph_low[node1][node2]['lane-id']
            lane23id=dic_graph_low[node2][node3]['lane-id']
            dire=dic_turns[lane12id][lane23id]['direction']
            degree_turned=dic_turns[lane12id][lane23id]['turned-degree']
            record=[node2,node3,node1,stress,dire,degree_turned]
            writer_graph_stress1.writerow(record)
for lane_from in dic_graph_stress2:
    for lane_to in dic_graph_stress2[lane_from]:
        stress=dic_graph_stress2[lane_from][lane_to]
        direction=dic_turns[lane_from][lane_to]['direction']
        degree_turned=dic_turns[lane_from][lane_to]['turned-degree']
        record=[lane_from,lane_to,stress,direction,degree_turned]        
        writer_graph_stress2.writerow(record)
'''