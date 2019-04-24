'''
Created on Oct 31, 2015

@author: gong

The link file has already contained the direction information of link. The link file actually should be called as link direction file

This module is used to calculate the trip length using the map-matched GPS points. The method used is very similar to the one developed by CAOPeng
Finally, the shortest route/path will be shown as a series of node ids

In this revision, the dictionary rather than list is used to contain the links and nodes

applied on 2017/1/4, 16 persons' data this time

revised on 2017/1/30
inter-city trips are also included but calculated separately from intra-city trips.
trip length is calculated according to the travel mode:
length of trips by plane or ship is calculated as direct distance from O to D.
length of trips by auto, bike, walk, rail, tram, and bus are calculated after map-matching.
'''
import csv
import copy
from Dijkstra import * 
################ locate output file ################
outputfile=open(r"D:\03Work during PD\11variability of travel behaviour\trip_mapmatching_only_intra.csv","ab")
writer=csv.writer(outputfile)
outputfile2=open(r"D:\03Work during PD\11variability of travel behaviour\triplength_mapmatched.csv","ab")
writer2=csv.writer(outputfile2)
heads2=["trip id","trip length","1 is for mapmatched trip"]

############### define functions ##################
def Dist(p1,p2):
    return(((p1[1]-p2[1])**2+ (p1[2]-p2[2])**2)**(0.5))
def Dist22(p1,p2):
    return(((p1[0]-p2[0])**2+ (p1[1]-p2[1])**2)**(0.5)) 
def P_on_link(point,link):
#this function returns 1 when the perpendicular foot of point is on the link, otherwise, on the extension of this link, return 0   
    x=point[1]
    y=point[2]
    x1=nodes_dic[link[1]][0]
    y1=nodes_dic[link[1]][1]
    x2=nodes_dic[link[2]][0]
    y2=nodes_dic[link[2]][1]
    A=y2-y1#A,B,C is the coefficients of line function: Ax+By+C=0
    B=x1-x2
    C=x2*y1-x1*y2
    #coordinate of perpendicular foot (from point to link) is x0,y0
    x0=x-A*(A*x+B*y+C)/(A*A+B*B)
    y0=y-B*(A*x+B*y+C)/(A*A+B*B)
    p0=[x0,y0]
    p1=[x1,y1]
    p2=[x2,y2]
    if Dist22(p0,p1)+Dist22(p0,p2)==Dist22(p1,p2):#perpendicular foot is on the link
        return 1
    else:#perpendicular foot is on the extension of the link
        return 0
#define the distance calculation function of GPS point to a link
def Dist_p_l(point,link):
#this function returns the perpendicular distance from GPS point to road link
    x0=point[1]
    y0=point[2]
    x1=nodes_dic[link[1]][0]
    y1=nodes_dic[link[1]][1]
    x2=nodes_dic[link[2]][0]
    y2=nodes_dic[link[2]][1]
    p0=[x0,y0]
    p1=[x1,y1]
    p2=[x2,y2]    
    if P_on_link(point, link):
        return(abs((x1-x0)*(y2-y0)-(x2-x0)*(y1-y0))/(((x1-x2)**2+(y1-y2)**2)**0.5))
    else:
        return(min([Dist22(p0,p1),Dist22(p0,p2)]))
########### read the road node and link into memory (for intra-city trips only)
nodefile=open(r"D:\03Work during PD\11variability of travel behaviour\GIS data\GIS selection\roadnode2.csv","r")
linkfile=open(r"D:\03Work during PD\11variability of travel behaviour\GIS data\GIS selection\roadlink2.csv","r")
nodefile.readline()#the first row is column heads
linkfile.readline()
nodereader=csv.reader(nodefile)
linkreader=csv.reader(linkfile)
nodes=[]#store all nodes
links=[]#store all links
nodes_dic={}#dictionary of nodes
all_graph={}#dictionary of links
for row1 in nodereader:
    node=[]
    for i in range(0,3):        
        node.append(row1[i])#node id, X, Y
    node[1]=float(node[1])
    node[2]=float(node[2])
    nodes.append(node)
    nodes_dic[node[0]]=[node[1],node[2]]
for row2 in linkreader:
    link=[]    
    for i in range(0,4):        
        link.append(row2[i])#link id, start, end, length in meter    
    links.append(link)
    
    nodeid1=link[1]#id of node1
    nodeid2=link[2]#id of node2
    length=float(link[3])    
    if all_graph.has_key(nodeid1):
        all_graph[nodeid1][nodeid2]=length
    else:
        all_graph[nodeid1]={nodeid2:length}
print len(all_graph)    

###### read trip list of intra- and inter-city trips
tripinfofile=open(r"D:\03Work during PD\11variability of travel behaviour\trip_information.csv","r")
tripinfo_reader=csv.reader(tripinfofile)
inside_trip_list=[]#store trip id of inside city trip
outside_trip_list=[]#store trip id of outside city trip
list_tripid_length=[]#store lists,any of which is a list consisting of trip id and its length calculated as cumulative distance between two consecutive GPS points
for row in tripinfo_reader:
    trip_id=row[0]
    trip_length=row[20]
    list_tripid_length.append([trip_id,trip_length,0])#0 is an indicator which is initialized as 0 showing not map-matched
    if row[25]=="NA":#trips outside the city
        outside_trip_list.append(trip_id)
    else:
        inside_trip_list.append(trip_id)
print "there are %d intra-city trips"%(len(inside_trip_list))
print "there are %d inter-city trips"%(len(outside_trip_list))
#import trip points of each intra-city trip, only use trip id for intra-city trip    
for id_trip_len in list_tripid_length:
    tripid=id_trip_len[0]
    if tripid in inside_trip_list:        
        print tripid
        trippoints=[]#all points(timestamp,X,Y) in a trip
        tripfile='{0}.csv'.format(tripid)#trip file need to be updated
        filepath=r"D:\03Work during PD\11variability of travel behaviour\GPS data for each trip(without its activity)_inter_intra"+"\\"+tripfile
        reader=csv.reader(open(filepath))#no column names in the file
        for row in reader:
            point=[]
            point.append(row[3])
            point.append(float(row[18]))
            point.append(float(row[19]))#time, X, Y
            trippoints.append(point)
    #step1, collect all involved links into candidate group for R=0.1; shrink the link length of these links
        R=0.1#ratio used to make the length near the GPS point has a higher probability to be included in the shortest path
        radius=100#set the radius of involving road links
        cand_dic={}#used to collect all candidate links in dictionary either of whose node has a dist less than radius to a point in trip
        nodes_candidate=[]#collect the nodes of candidate links
        for point1 in trippoints:
            for node in nodes:
                if Dist(point1,node)<=radius and not cand_dic.has_key(node[0]):
                    cand_dic[node[0]]=copy.deepcopy(all_graph[node[0]])   
                    #print all_graph[node[0]]
                    #print cand_dic[node[0]]             
                    for key in cand_dic[node[0]].keys():                    
                        cand_dic[node[0]][key]*=R
                    #print all_graph[node[0]]
                    #print cand_dic[node[0]]                  
                    if not node in nodes_candidate:
                        nodes_candidate.append(node)
    
    #step2, search for nodes to which origin and destination should link
    #decide the origin and destination node
    #select the node that has the shortest distance; then select the link containing this node and having the shortest perpendicular distance to the trip point 
        origin_p=trippoints[0]#origin point in the trip
        destin_p=trippoints[-1]#destination point in the trip
        min_dist_o=Dist(origin_p,nodes_candidate[0])#intial value of minimum distance from origin point to all possible nodes, use the dist to the first node
        origin_node=nodes_candidate[0]
        min_dist_d=Dist(destin_p,nodes_candidate[-1])#intial value of minimum distance from destination point to all possible nodes, use the dist to the first node
        destin_node=nodes_candidate[-1]
        for node in nodes:
            if Dist(origin_p,node)<min_dist_o:
                min_dist_o=Dist(origin_p,node)
                origin_node=copy.deepcopy(node)
            if Dist(destin_p,node)<min_dist_d:
                min_dist_d=Dist(destin_p,node)
                destin_node=copy.deepcopy(node)
    
    #step3, search for the shortest route from origin node and destination node
    #two road networks, one has a distance shorter than the radius and the length has been shrunk by R;
    #another road networks is the the whole network excluding the one network
        
    #include the links that is not in cand_dic into cand_dic with original link length
        for key in all_graph.keys():
            if not cand_dic.has_key(key):
                cand_dic[key]=copy.deepcopy(all_graph[key])
    
        
        path=shortestPath(cand_dic, origin_node[0],destin_node[0])#path is in the form of a list containing a series of point ids
        length_of_path=0
        if len(path)>0:
            for i in range(0,len(path)-1):            
                length_of_path=length_of_path+all_graph[path[i]][path[i+1]]    
        else:
            length_of_path=0
        writer.writerow([tripid,length_of_path,path])#origin_p,origin_node[0],min_dist_o,destin_p,destin_node[0],min_dist_d,
        id_trip_len[1]=length_of_path
        id_trip_len[2]=1
for item in list_tripid_length:
    writer2.writerow(item)