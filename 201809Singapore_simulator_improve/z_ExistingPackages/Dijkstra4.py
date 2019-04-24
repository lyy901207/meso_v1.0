#coding=utf-8
# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002
# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228

# modified a bit on 2018/7/14 by Lei GONG
# the structure of dictionary of the graph has been changed a bit.
# the length/travel time of a link is now located in a dictionary,
# which means it needs one more step to get this value.

# extended on 2018 December by Lei GONG
# functions of Dijkstra, shortestPath, and shortestPathNode have been extended respectively to two ways of stress-based route searching.
# the 1st way is calculating the stress by three nodes and two links, where the stress driving to a link is dependent on the previous link and the link's starting node
# the 2nd way is calculating the stress by two links and one node, whose idea is similar to the route search based on travel time (or travel distance) 


# edited and improved on 2019 January by Lei GONG
# revised on the basis of Dijkstra3 by enabling the graph G automatically add virtual origin O, destination D and virtual links between O (or D) and start node (or end node).
# the adding details are different due to different graph G for searching stress-minimization route.
# start and end used as input variable are always node-ids in the following functions. Function of Dijkstra_stress2 uses converted start and end by the function of Graph_stress2_improve \
# which outputs virtual links as start and end.

from PrioDict import priorityDictionary
import datetime

def dijkstra(G,start,end=None):
    """
    Find shortest paths from the start vertex to all
    vertices nearer than or equal to the end.

    The input graph G is assumed to have the following
    representation: A vertex can be any object that can
    be used as an index into a dictionary.  G is a
    dictionary, indexed by vertices.  For any vertex v,
    G[v] is itself a dictionary, indexed by the neighbors
    of v.  For any edge v->w, G[v][w] is the length of
    the edge.  This is related to the representation in
    <http://www.python.org/doc/essays/graphs.html>
    where Guido van Rossum suggests representing graphs
    as dictionaries mapping vertices to lists of neighbors,
    however dictionaries of edges have many advantages
    over lists: they can store extra information (here,
    the lengths), they support fast existence tests,
    and they allow easy modification of the graph by edge
    insertion and removal.  Such modifications are not
    needed here but are important in other graph algorithms.
    Since dictionaries obey iterator protocol, a graph
    represented as described here could be handed without
    modification to an algorithm using Guido's representation.

    Of course, G and G[v] need not be Python dict objects;
    they can be any other object that obeys dict protocol,
    for instance a wrapper in which vertices are URLs
    and a call to G[v] loads the web page and finds its links.
    
    The output is a pair (D,P) where D[v] is the distance
    from start to v and P[v] is the predecessor of v along
    the shortest path from s to v.
    
    Dijkstra's algorithm is only guaranteed to work correctly
    when all edge lengths are positive. This code does not
    verify this property for all edges (only the edges seen
    before the end vertex is reached), but will correctly
    compute shortest paths even for some graphs with negative
    edges, and will raise an exception if it discovers that
    a negative edge has caused it to make a mistake.
    """

    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors
    Q = priorityDictionary()   # est.dist. of non-final vert.
    #print "Q is ",Q
    Q[start] = 0
    #print "Q is ",Q
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        # preventing broken road occur
        if G.has_key(v):
            for w in G[v]:
                vwLength = D[v] + G[v][w]['travel-time']
                if w in D:
                    if vwLength < D[w]:
                        raise ValueError, "Dijkstra: found better path to already-final vertex"
                elif w not in Q or vwLength < Q[w]:
                    Q[w] = vwLength
                    P[w] = v
    
    return (D,P)
def graph_Stress1_Improve(G_time,G_stress,start,end):
    # this function improve the graph of G_stress (based on turnings) as this graph does not include the origin and destination 
    # start is the start node and origin is a virtual point connecting to start node virtually.
    # end is the end node and destination is a virtual point connecting to end node virtually.
    # this function includes the origin and destination into the graph with stress of 0.
    # origin point is noted as O and destination point is noted as D
    # G_time is in the following format:   {node1:{node2:{'travel-time':time1,'speed':speed1,...},node3:{'travel-time':time2,'speed':speed2,...}},node3:{},node4:{}.......}
    # G_stress is in the following format: {node1:{node2:{node0:stress1,node00:stress1,...},node3:{...}...},node3:{}....}, \
    # \ which means a turning from link01(from node0 to node1) to link12(from node1 to node2)
    
    # put O and virtual link from O to start into the graph of G_stress
    if not G_time.has_key(start):#start--node1
        print "ERROR: no link starting from the node of ", start
    else:
        for node2 in G_time[start]:
            if G_stress.has_key(start):
                if G_stress[start].has_key(node2):
                    G_stress[start][node2]['O']=0
                else:
                    G_stress[start][node2]={}
                    G_stress[start][node2]['O']=0
            else:
                G_stress[start]={}
                G_stress[start][node2]={}
                G_stress[start][node2]['O']=0       
    
    # put D and virtual link from end to D into the graph of G_stress
    # actually the following part will not be used in the route searching
    count_end=0# for counting when the end-node of a link is the given end
    for node1 in G_time:
        if G_time[node1].has_key(end):# end--node2
            count_end+=1     
            if G_stress.has_key(end):
                G_stress[end]['D']={}
                G_stress[end]['D'][node1]=0
            else:
                G_stress[end]={}
                G_stress[end]['D']={}
                G_stress[end]['D'][node1]=0
    if count_end>=1:
        return G_stress
    else:
        print "ERROR: no links ending at the node of", end
def graph_Stress2_Improve(G_time,G_stress,start,end,dic_lanes=None):
    # this function improves the Graph (i.e. G_stress used for route searching based on stress-minimization
    # start is the star-node while end is the end-node
    # O and D are virtual nodes of origin and destination; a virtual link connects O and start node and a virtual link connects D and end node
    # ids of virtual links are named as a combination of the nodes, which are different from the the names of other physical links/lanes, as separate lanes needs to be distinguished in some policy evaluation.      
    # G_time is in the following format:   {node1:{node2:{'travel-time':time1,'speed':speed1,...},node3:{'travel-time':time2,'speed':speed2,...}},node3:{},node4:{}.......}
    # G_stress is in the following format: {lane1:{lane2:stress12,lane3:stress13,...},lane2:{lane3:stress23,...}}
        
    # put stress from virtual link to the links starting from start-node into G_stress, start-node is a joint node, virtual link is from O to start-node
    if not G_time.has_key(start):# no links starting from start-node
        print "ERROR: no link starting from the node of ", start
    else:
        virtual_link_id_start="O"+"-"+str(start)# id of virtual link from O to start-node
        G_stress[virtual_link_id_start]={}
        for node2 in G_time[start]:# start is node1
            if dic_lanes==None:
                link_id_start_node2=str(start)+str(node2)# if dic_lanes is given, id of link from start-node to node2
            else:
                link_id_start_node2=dic_lanes[start][node2]['lane-id']# if dic_lane is not given, id of link from start-node to node2; test example in this module belongs to this                
            G_stress[virtual_link_id_start][link_id_start_node2]=0# stress from O via start-node to node2 is set as 0
    
    # put stress from links ending at end-node to virtual link into G_stress, end-node is a joint node, virtual link is from end-node to D
    count_end=0# for counting when the end-node is the given end
    for node1 in G_time:
        virtual_link_id_end=str(end)+"-"+"D"# id of virtual link from end-node to D
        if G_time[node1].has_key(end):# end--node2
            count_end+=1
            if dic_lanes==None:
                link_id_node1_end=str(node1)+str(end)# id of link from node1 to end-node, if dic_lanes is not given; test example in this module belongs to this
            else:
                link_id_node1_end=dic_lanes[node1][end]['lane-id']# id of link from node1 to end-node, if dic_lanes is given
            G_stress[link_id_node1_end]={}
            G_stress[link_id_node1_end][virtual_link_id_end]=0
    if count_end>=1:
        return G_stress,virtual_link_id_start,virtual_link_id_end# start-node and end-node need to be replaced by the virtual links 
    else:
        print "ERROR: no links ending at the node of ", end


def dijkstra_Stress1(G_time,G,start,end=None):
    """
    This function is modified from Dijkstra by including the stress as value for route search.
    Stress-based search has two structures of graph G. This is the structure1 of G.
    The stress value is due to turnings and is obtained from three nodes: a, b, and c. A turning is from link (c to a) to link (a to b). 
    So when calculating the stress of a link, we must know the previous node of this link. 
    Because of this, G_stress needs to be improved by the function of Graph_stress1_improve.
    
    last edited on 2019/1/12 by Lei GONG
    """
    # G_time is the graph for searching best route based on travel time
    # G is the graph for searching best route based on stress, in structure1
    # start and end are the starting and ending nodes for an OD pair
    # virtual O and D need to be included in the graph of G
    # start and end will not be updated/replaced in Graph_stress1_improve
    print "original graph of stress in structure 1 is as follows:"
    print G
    G=graph_Stress1_Improve(G_time, G, start, end)
    print "Improved graph of stress in structure 1 is as follows:"
    print G
    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors
    
    # different from the original Dijkstra, a virtual origin O and a virtual link from O to start is necessary.
    # they are not necessary when building the graph G 
    P[start]='O'# put the link from Origin to the first node, the virtual link, into the predecessor dictionary
     
    Q = priorityDictionary()   # est.dist. of non-final vert.
    Q[start] = 0
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        # preventing broken road occur
        if G.has_key(v):
            for w in G[v]:
                vwLength = D[v] + G[v][w][P[v]]
                if w in D:
                    if vwLength < D[w]:
                        raise ValueError, "Dijkstra: found better path to already-final vertex"
                elif w not in Q or vwLength < Q[w]:
                    Q[w] = vwLength
                    P[w] = v
    
    return (D,P)

def dijkstra_Stress2(G_time,G,start,end=None):
    """
    This function is modified from Dijkstra by including the stress as value for route search.
    The graph of G is used in the same way of Dijkstra but with different representation.
    "start" and "end" and the basic elements in the graph/network is the links; 
    the impedance along the route is the quantified stress from one link to another link which includes the turning information.
    "start" is the virtue link from Origin to the first node in the network; "end" is the virtue link from the last node in the network to the destination. 
    Actually this algorithm just use the original idea of Dijsktra intending to simply the algorithm structure used in Dijkstra_stress1 
    """
    # G_time is the graph for searching best route based on travel time
    # G is the graph for searching best route based on stress, in structure1
    # start and end are the starting and ending nodes for an OD pair
    # virtual O and D need to be included in the graph of G
    # start and end will be updated/replaced (from node-id to lane-id) in Graph_stress1_improve
    
    print "original graph of stress in structure 2 is as follows:"
    print G
    G,start,end=graph_Stress2_Improve(G_time, G, start, end)
    print "Improved graph of stress in structure 2 is as follows:"
    print G
    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors    
    Q = priorityDictionary()   # est.dist. of non-final vert.
    #print "Q is ",Q
    Q[start] = 0
    #print "Q is ",Q
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        # preventing broken road occur
        if G.has_key(v):
            for w in G[v]:
                vwStress = D[v] + G[v][w]
                if w in D:
                    if vwStress < D[w]:
                        raise ValueError, "Dijkstra: found better path to already-final vertex"
                elif w not in Q or vwStress < Q[w]:
                    Q[w] = vwStress
                    P[w] = v
    
    return D,P,start,end
def shortest_Path(G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = dijkstra(G,start,end)
#    flog1 = open('flog1.txt', 'w')
#    flog1.write(prnDict(P))
    Path = []
   
    reach=0
    if P.has_key(end):
        reach=1
    #problem of referencing P
#    elif end not in P.keys():
#        for i_key in P.keys():
#            if end==P[i_key]:
#                reach=1
#                break    
    else:
        reach=0
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        return Path
    else:
        return Path
def shortest_Path_Stress1(G_time,G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra_stress1().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = dijkstra_Stress1(G_time,G,start,end)
#    flog1 = open('flog1.txt', 'w')
#    flog1.write(prnDict(P))
    Path = []
   
    reach=0
    if P.has_key(end):
        reach=1
    #problem of referencing P
#    elif end not in P.keys():
#        for i_key in P.keys():
#            if end==P[i_key]:
#                reach=1
#                break    
    else:
        reach=0
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        return Path
    else:
        return Path
def shortest_Path_Stress2(G_time,G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra_stress2().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P,start,end = dijkstra_Stress2(G_time,G,start,end)
#    flog1 = open('flog1.txt', 'w')
#    flog1.write(prnDict(P))
    Path = []
   
    reach=0
    if P.has_key(end):
        reach=1
    #problem of referencing P
#    elif end not in P.keys():
#        for i_key in P.keys():
#            if end==P[i_key]:
#                reach=1
#                break    
    else:
        reach=0
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        return Path
    else:
        return Path
def shortest_Path_in_Node(G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = dijkstra(G,start,end)
    Path = []
    
    reach=0
    if P.has_key(end):
        reach=1

    else:
        reach=0
    
    # change Path from list to a dictionary (by Lei GONG)
    dic_path={}# {node3:node2,node2:node5,node5:node1....}
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        for i in range(0,len(Path)-1):
            dic_path[Path[i]]=Path[i+1]
        return D,dic_path
    else:
        return D,dic_path

def shortest_Path_in_Node_Stress1(G_time,G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = dijkstra_Stress1(G_time,G,start,end)
    Path = []
    
    reach=0
    if P.has_key(end):
        reach=1

    else:
        reach=0
    
    # change Path from list to a dictionary (by Lei GONG)
    dic_path={}# {node3:node2,node2:node5,node5:node1....}
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        for i in range(0,len(Path)-1):
            dic_path[Path[i]]=Path[i+1]
        return D,dic_path
    else:
        return D,dic_path
def shortest_Path_in_Node_Stress2(G_time,G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    
    # virtual links have been excluded from the output of dictionary of shortest path  
    
    D,P,start,end = dijkstra_Stress2(G_time,G,start,end)
    Path = []
    
    reach=0
    if P.has_key(end):
        reach=1

    else:
        reach=0
    
    # change Path from list to a dictionary (by Lei GONG)
    dic_path={}# {node3:node2,node2:node5,node5:node1....}
    if reach==1:
        while 1:
            Path.append(end)
            if end == start: break
            end = P[end]
        Path.reverse()
        for i in range(1,len(Path)-1-1):# revised a bit to exclude the virtual links included as either key or value in the dictionary of route in lane
            dic_path[Path[i]]=Path[i+1]
        return D,dic_path
    else:
        return D,dic_path



'''
# follows is the test of this module only  
#######################################################################################################################
###     ######   ######      #####    ######
###       ##     ##        ###          ##
###       ##     #####       ####       ##
###       ##     ##             ###     ##
###       ##     ######     #####       ##
#######################################################################################################################


#########################################################################
###### test function shortestPathNode by travel time ####################
print "test function of shortestPathNode"
G = {'s':{'u':{'travel-time':9}, 'x':{'travel-time':5}},'v':{'y':{'travel-time':4}},'x':{'y':{'travel-time':4},'s':{'travel-time':3}},'u':{'y':{'travel-time':1},'z':{'travel-time':3}}}
D1,P1=dijkstra(G,'x','z')
print "D1 and D1[z] is: ", D1,D1['z']
print "P1 is: ",P1
print "result of function--shortestPath is: ", shortest_Path(G,'x','z')
print "result of function--shortestPathNode is: ",shortest_Path_in_Node(G,'x','z')
########################     end    #####################################
#########################################################################



#########################################################################
###### test function shortestPathNode_stress_stru1#######################
# first structure focuses on three nodes and two links
# G_stress1 and G2_stress1 are two sample data sets for test; the latter has larger network
print "----------------------------"
print "test function of shortestPahtNode_stress1"
G = {'a':{'b':1,'d':5},'b':{'c':7,'e':3},'d':{'e':5,'g':5},'e':{'f':1},'f':{'c':1,'i':1},'g':{'h':5},'h':{'e':1},'i':{'h':1}}
G_stress1={'b':{'c':{'a':0},'e':{'a':2}},'d':{'e':{'a':1},'g':{'a':0}},'e':{'f':{'b':1,'d':0,'h':2}},'f':{'c':{'e':1},'i':{'e':2}},'g':{'h':{'d':1}},'h':{'e':{'g':1,'i':2}},'i':{'h':{'f':2}}}#through is 0, right is 2 and left is 1

D1,P1=dijkstra_Stress1(G,G_stress1,'a','i')
print "D1 is: ", D1
print "P1 is: ",P1
print "result of function--shortestPath_stress1 is: ", shortest_Path_Stress1(G,G_stress1,'a','i')
print "result of function--shortestPathNode_stress1 is: ",shortest_Path_in_Node_Stress1(G,G_stress1,'a','i')# O is connected with a without extra stress, and D is connected with i without extra stress 

########################     end    ####################################
########################################################################


#########################################################################
###### test function shortestPathNode_stress_stru2#######################
# second structure focuses on two links and one node
# the structure of G is same as the original but the meaning is different: the key is link id (named as starting node id and ending node id),final value is the quantified stress from turnings
# G_stress2 and G2_stress2 are two sample data sets for test; the latter has larger network
print "-----------------------------"
print "test function of shortestPahtNode_stress2"
G_stress2={'ab':{'bc':0,'be':2},'ad':{'de':1,'dg':0},'de':{'ef':0},'be':{'ef':1},'he':{'ef':2},'ef':{'fc':1,'fi':2},'dg':{'gh':1},'gh':{'he':1},'ih':{'he':2},'fi':{'ih':2}}#through is 0, right is 2 and left is 1

D1,P1,start,end=dijkstra_Stress2(G,G_stress2,'a','i')
print "D1 is: ",D1
print "P1 is: ",P1
print "updated start and end are:", start, end
print "result of function--shortestPath_stress2 is: ", shortest_Path_Stress2(G,G_stress2,'a','i')
print "result of function--shortestPathNode_stress2 is: ",shortest_Path_in_Node_Stress2(G,G_stress2,'a','i')# O is connected with a without extra stress, and D is connected with i without extra stress 
########################     end    ####################################
########################################################################
'''