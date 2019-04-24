#coding=utf-8
# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002
# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228

# modified a bit on 2018/7/14 by Lei GONG
# the structure of dictionary of the graph has been changed a bit.
# the length/travel time of a link is now located in a dictionary,
# which means it needs one more step to get this value.
from PrioDict import priorityDictionary

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
    print "Q is ",Q
    Q[start] = 0
    print "Q is ",Q
    
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
def Dijkstra_stress(G,start,end=None):
    """
    This function is modified from Dijkstra by including the stress as value for route search.
    The graph of G is used dic_graph_stress which follows the same representation of G in Dijkstra but the value in the most inner dictionary is the quantified stress 
    """

    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors
    Q = priorityDictionary()   # est.dist. of non-final vert.
    Q[start] = 0
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        # preventing broken road occur
        if G.has_key(v):# lane from node v to node w
            for w in G[v]:
                for s in G[v][w]:
                    if v==start:# if v is the start point which means that no stress due to the turn at the node before v
                        vwLength =D[v]+0
                    else:# if v is the start point in the network
                        vwLength = D[v] + G[v][w][s]# lane from node s via node v to node w 
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
    "start" and "end" and the basic elements in the graph/network is the links; the impedance along the route is the quantified stress from one link to another link which includes the turning information.
    "start" is the virtue link from Origin to the first node in the network; "end" is the virtue link from the last node in the network to the destination. 
    Actually this algorithm just use the original idea of Dijsktra intending to simply the algorithm structure used in Dijkstra_stress1 
    """

    D = {}    # dictionary of final distances
    P = {}    # dictionary of predecessors
    Q = priorityDictionary()   # est.dist. of non-final vert.
    print "Q is ",Q
    Q[start] = 0
    print "Q is ",Q
    
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

def shortestPathNode(G,start,end):
    # revised by Gong
    # this function returns a dictionary of cost (time or distance) along the path, \ 
    # and a dictionary of node-pair of links in the shortest path given a graph and a pair of origin-destination.
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

def shortestPathNode_stress(G,start,end):
    # search the route due to the minimum quantified stress.
    # G is the graph of stress which should have the same representation as
    D,P = Dijkstra_stress(G, start, end)
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
###### test function shortestPathNode  ##################################
print "test function of shortestPathNode"
G = {'s':{'u':{'travel-time':9}, 'x':{'travel-time':5}},'v':{'y':{'travel-time':4}},'x':{'y':{'travel-time':4},'s':{'travel-time':3}},'u':{'y':{'travel-time':1},'z':{'travel-time':3}}}
D1,P1=Dijkstra(G,'x','z')
print D1,D1['z']
print P1
print shortestPath(G,'x','z')
print shortestPathNode(G,'x','z')
########################################################################
########################     end    ####################################

#########################################################################
###### test function shortestPathNode_stress_stru1#############################
# first structure focuses on three nodes and two links
print "test function of shortestPahtNode_stress"
G = {'a':{'b':1,'d':5},'b':{'c':7,'e':3},'d':{'e':5,'g':5},'e':{'f':1},'f':{'c':1,'i':1},'g':{'h':5},'h':{'e':1},'i':{'h':1}}
G_stress={'a':{'b':{'O':0},'d':{'O':0}},'b':{'c':{'a':0},'e':{'a':2}},'d':{'e':{'a':1},'g':{'a':0}},'e':{'f':{'b':1,'d':0,'h':2}},'f':{'c':{'e':1},'i':{'e':2}},'g':{'h':{'d':1}},'h':{'e':{'g':1,'i':2}},'i':{'D':{'f':0},'h':{'f':2}}}#through is 0, right is 2 and left is 1
D1,P1=Dijkstra_stress(G_stress,'a','i')
print D1
print P1
print shortestPathNode_stress(G_stress,'O','D')# O is connected with a without extra stress, and D is connected with i without extra stress 
########################################################################
########################     end    ####################################

#########################################################################
###### test function shortestPathNode_stress_stru2#############################
# second structure focuses on two links and one node
# the structure of G is same as the original but the meaning is different: the key is link id (named as starting node id and ending node id),final value is the quantified stress from turnings
print "test function of shortestPahtNode_stress"
G2_stress={'Oa':{'ab':0,'ad':0},'ab':{'bc':0,'be':2},'ad':{'de':1,'dg':0},'de':{'ef':0},'be':{'ef':1},'he':{'ef':2},'ef':{'fc':1,'fi':2},'dg':{'gh':1},'gh':{'he':1},'ih':{'he':2},'fi':{'ih':2,'iD':0}}#through is 0, right is 2 and left is 1
D1,P1=Dijkstra_stress2(G2_stress,'Oa','iD')
print D1
print P1

########################################################################
########################     end    ####################################


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