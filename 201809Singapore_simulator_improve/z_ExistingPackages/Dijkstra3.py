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

# the intention of this revision is to compare the running time of stress-minimization-based route searching algorithms which have different searching method
# the 2nd way is believed to be faster; however, the 1st way is faster after testing. 

from PrioDict import priorityDictionary
import datetime

def Dijkstra(G,start,end=None):
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
def Dijkstra_stress1(G,start,end=None):
    """
    This function is modified from Dijkstra by including the stress as value for route search.
    Stress-based search has two structures of graph G. This is the structure 1 of G.
    The stress value is due to turnings and is obtained from three nodes: a, b, and c. A turning is from link (c to a) to link (a to b). 
    So when calculating the stress of a link, we must know the previous node of this link. 
    
    last edited on 2019/1/12 by Lei GONG
    """

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

def Dijkstra_stress2(G,start,end=None):
    """
    This function is modified from Dijkstra by including the stress as value for route search.
    The graph of G is used in the same way of Dijkstra but with different representation.
    "start" and "end" and the basic elements in the graph/network is the links; 
    the impedance along the route is the quantified stress from one link to another link which includes the turning information.
    "start" is the virtue link from Origin to the first node in the network; "end" is the virtue link from the last node in the network to the destination. 
    Actually this algorithm just use the original idea of Dijsktra intending to simply the algorithm structure used in Dijkstra_stress1 
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
                vwStress = D[v] + G[v][w]
                if w in D:
                    if vwStress < D[w]:
                        raise ValueError, "Dijkstra: found better path to already-final vertex"
                elif w not in Q or vwStress < Q[w]:
                    Q[w] = vwStress
                    P[w] = v
    
    return (D,P)
def shortestPath(G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = Dijkstra(G,start,end)
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
def shortestPath_stress1(G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra_stress1().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = Dijkstra_stress1(G,start,end)
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
def shortestPath_stress2(G,start,end):
    """
    Find a single shortest path from the given start vertex
    to the given end vertex.
    The input has the same conventions as Dijkstra_stress2().
    The output is a list of the vertices in order along
    the shortest path.
    """

    D,P = Dijkstra_stress2(G,start,end)
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
def shortestPathNode(G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = Dijkstra(G,start,end)
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

def shortestPathNode_stress1(G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = Dijkstra_stress1(G,start,end)
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
def shortestPathNode_stress2(G,start,end):
    # revised by Gong
    # this function returns two dictionaries:
    # a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path when given a graph and a pair of origin-destination.
    # these two dictionaries have the same style as Dijkstra
    D,P = Dijkstra_stress2(G,start,end)
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

#G = {'s':{'u':9, 'x':5},'v':{'y':4},'x':{'y':4,'s':3},'u':{'y':1,'z':3}}
#D1,P1=Dijkstra(G,'s','v')
#print D1
#print P1
#print shortestPath(G,'s','v')


#########################################################################
###### test function shortestPathNode by travel time ####################
print "test function of shortestPathNode"
G = {'s':{'u':{'travel-time':9}, 'x':{'travel-time':5}},'v':{'y':{'travel-time':4}},'x':{'y':{'travel-time':4},'s':{'travel-time':3}},'u':{'y':{'travel-time':1},'z':{'travel-time':3}}}
D1,P1=Dijkstra(G,'x','z')
print "D1 and D1[z] is: ", D1,D1['z']
print "P1 is: ",P1
print "result of function--shortestPath is: ", shortestPath(G,'x','z')
print "result of function--shortestPathNode is: ",shortestPathNode(G,'x','z')
########################     end    #####################################
#########################################################################



#########################################################################
###### test function shortestPathNode_stress_stru1#######################
# first structure focuses on three nodes and two links
# G_stress1 and G2_stress1 are two sample data sets for test; the latter has larger network
print "----------------------------"
print "test function of shortestPahtNode_stress1"
G = {'a':{'b':1,'d':5},'b':{'c':7,'e':3},'d':{'e':5,'g':5},'e':{'f':1},'f':{'c':1,'i':1},'g':{'h':5},'h':{'e':1},'i':{'h':1}}
G_stress1={'a':{'b':{'O':0},'d':{'O':0}},'b':{'c':{'a':0},'e':{'a':2}},'d':{'e':{'a':1},'g':{'a':0}},'e':{'f':{'b':1,'d':0,'h':2}},'f':{'c':{'e':1},'i':{'e':2}},'g':{'h':{'d':1}},'h':{'e':{'g':1,'i':2}},'i':{'D':{'f':0},'h':{'f':2}}}#through is 0, right is 2 and left is 1
G2_stress1={'a':{'b':{'O':0},'d':{'O':0}},'b':{'c':{'a':0},'e':{'a':2}},'c':{'j':{'b':0,'f':2}},'d':{'e':{'a':1},'g':{'a':0}},'e':{'f':{'b':1,'d':0,'h':2}},'f':{'c':{'e':1,'k':2},'i':{'e':2,'k':1}},'g':{'h':{'d':1},'p':{'d':0}},'h':{'e':{'g':1,'i':2},'o':{'g':2,'i':1}},'i':{'n':{'f':0},'h':{'f':2},'l':{'f':1}},'j':{'k':{'c':2}},'k':{'f':{'j':1},'l':{'j':0}},'l':{'m':{'k':0,'i':2}},'m':{'D':{'l':0,'n':0}},'n':{'m':{'i':1,'o':0}},'o':{'n':{'h':1},'p':{'h':2}}}#through is 0, right is 2 and left is 1

D1,P1=Dijkstra_stress1(G_stress1,'a','i')
print "D1 is: ", D1
print "P1 is: ",P1
begin1 = datetime.datetime.now()
for i in range(0,10000):#run 10000 times to easily get a time cost of calculation, otherwise the time cost is less than 0.0000 seconds which could not be measured by python 
    Dijkstra_stress1(G2_stress1,'a','i')
    #shortestPath_stress1(G_stress1,'a','i')
    #shortestPathNode_stress1(G_stress1,'a','i')
print "result of function--shortestPath_stress1 is: ", shortestPath_stress1(G_stress1,'a','i')
end1 = datetime.datetime.now()
timeused1=end1-begin1
print "time cost for calculating shortestPath_stress1 is: ", timeused1
print "result of function--shortestPathNode_stress1 is: ",shortestPathNode_stress1(G_stress1,'a','i')# O is connected with a without extra stress, and D is connected with i without extra stress 
print len(G_stress1),len(G2_stress1)
########################     end    ####################################
########################################################################


#########################################################################
###### test function shortestPathNode_stress_stru2#######################
# second structure focuses on two links and one node
# the structure of G is same as the original but the meaning is different: the key is link id (named as starting node id and ending node id),final value is the quantified stress from turnings
# G_stress2 and G2_stress2 are two sample data sets for test; the latter has larger network
print "-----------------------------"
print "test function of shortestPahtNode_stress2"
G_stress2={'Oa':{'ab':0,'ad':0},'ab':{'bc':0,'be':2},'ad':{'de':1,'dg':0},'de':{'ef':0},'be':{'ef':1},'he':{'ef':2},'ef':{'fc':1,'fi':2},'dg':{'gh':1},'gh':{'he':1},'ih':{'he':2},'fi':{'ih':2,'iD':0}}#through is 0, right is 2 and left is 1
G2_stress2={'Oa':{'ab':0,'ad':0},'ab':{'bc':0,'be':2},'ad':{'de':1,'dg':0},'de':{'ef':0},'be':{'ef':1},'he':{'ef':2},'ef':{'fc':1,'fi':2},'dg':{'gh':1,'gp':0},'gh':{'he':1,'ho':2},'ih':{'he':2,'ho':1},'fi':{'ih':2,'iD':0,'in':0,'il':1}, \
            'bc':{'cj':0},'fc':{'cj':2},'cj':{'jk':1},'jk':{'kf':2,'kl':0},'kf':{'fc':2,'fi':1},'kl':{'lm':0},'il':{'lm':2},'lm':{'mD':0},'nm':{'mD':0},'in':{'nm':1},'on':{'nm':0},'ho':{'on':1,'op':2}}#through is 0, right is 2 and left is 1

D1,P1=Dijkstra_stress2(G_stress2,'Oa','iD')
print "D1 is: ",D1
print "P1 is: ",P1
begin2 = datetime.datetime.now()
for i in range(0,10000):
    D1,P1=Dijkstra_stress2(G2_stress2,'Oa','iD')
    #shortestPath_stress2(G_stress2,'Oa','iD')
    #shortestPathNode_stress2(G_stress2,'Oa','iD')
print "result of function--shortestPath_stress2 is: ", shortestPath_stress2(G_stress2,'Oa','iD')
end2 = datetime.datetime.now()
timeused2=end2-begin2
print "time cost for calculating shortestPath_stress2 is: ", timeused2
print "result of function--shortestPathNode_stress2 is: ",shortestPathNode_stress2(G_stress2,'Oa','iD')# O is connected with a without extra stress, and D is connected with i without extra stress 
print len(G_stress2),len(G2_stress2)
########################     end    ####################################
########################################################################




#As an example of the input format, here is the graph from Cormen, Leiserson, 
#    and Rivest (Introduction to Algorithms, 1st edition), page 528:
#G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 
#     'x':{'u':3, 'v':9, 'y':2}, 'y':{'s':7, 'v':6}}
#The shortest path from s to v is ['s', 'x', 'u', 'v'] and has length 9.

#print "next example"
#G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'s':7, 'v':6}}
#Path = shortestPath(G,'s','v')
#print 'The shortest path from s to v: ', Path

# not reachable
#print "the 3rd example" 
#G = {'s':{'u':10, 'x':5}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
#Path = shortestPath(G,'y','s')
#print 'The shortest path from y to s: ', Path

# test for broken roads
#G = {'s':{'u':10, 'x':5, 'p':7}, 'u':{'v':1, 'x':2}, 'v':{'y':4}, 'x':{'u':3, 'v':9, 'y':2}, 'y':{'v':6}}
#Path = shortestPath(G,'s','v')
#print 'The shortest path from s to v: ', Path