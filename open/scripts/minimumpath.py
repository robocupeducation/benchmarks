#!/usr/bin/env python
import rospy
from collections import defaultdict
from std_msgs.msg import String
import sys

location2node = {
    'kitchen':'A',
    'living room':'B',
    'studio':'C',
    'entrance':'D',
    'bathroom':'E',
}

node2location = {
    'A':'kitchen',
    'B':'living room',
    'C':'studio',
    'D':'entrance',
    'E':'bathroom',
}

## Graph class allows you to build your graph in python in order to work properly
## with it.
class Graph():
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight


class Arch():
    def __init__(self,startpoint,endpoint,weight = 1):
        self.nodeStart = startpoint
        self.nodeEnd = endpoint
        self.weight = weight
        self.arch = (self.nodeStart,self.nodeEnd,self.weight)

    def changeWeight(self,weight):
        self.weight = weight
        self.arch = (self.nodeStart,self.nodeEnd,self.weight)

    def getArch(self):
        return self.arch

    def getEnd(self):
        return self.nodeEnd

    def forbidpath(self):
        self.changeWeight(999)

class Solver():
    def __init__(self):
        self.solvePublisher = rospy.Publisher('/solve', String, queue_size = 1)
        self.inputSubscriber = rospy.Subscriber("/dijsktra_inp", String, self.inputCallback)
        #Lista que guarda los nodos que necesito saber: El primero es el destino y
        #el resto son excluyentes
        self.nodesList = []

    def inputCallback(self, data):
        if(data.data != "finish"):
            self.nodesList.append(data.data)
        else:
            self.dijstra()
    #CAMBIAR EL CODIGO DE ESTE METODO
    def dijsktra(self):
        # shortest paths is a dict of nodes
        # whose value is a tuple of (previous node, weight)
        shortest_paths = {initial: (None, 0)}
        current_node = initial
        visited = set()

        while current_node != end:
            visited.add(current_node)
            destinations = graph.edges[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinations:
                weight = graph.weights[(current_node, next_node)] + weight_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, weight)
                else:
                    current_shortest_weight = shortest_paths[next_node][1]
                    if current_shortest_weight > weight:
                        shortest_paths[next_node] = (current_node, weight)

            next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not next_destinations:
                return "Route Not Possible"
            # next node is the destination with the lowest weight
            current_node = min(next_destinations, key=lambda k: next_destinations[k][1])

        # Work back through destinations in shortest path
        path = []
        while current_node is not None:
            path.append(node2location[current_node])
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # Reverse path
        path = path[::-1]
        print("[Dijkstra]: " + str(path))
        rate = rospy.Rate(5)

        for location in path:
            rate.sleep()
            self.solvePublisher.publish(location)
            rate.sleep()
        self.solvePublisher.publish("no more points")

graph = Graph()

arch1 = Arch('A', 'B') # xAB
arch2 = Arch('A', 'E', 2) # xAE
arch3 = Arch('A', 'D', 3 ) # xAD
arch4 = Arch('B', 'C') # xBC
arch5 = Arch('B', 'E') # xBE
arch6 = Arch('D', 'C') # xDC
arch7 = Arch('E', 'C') # xEC
arch8 = Arch('E', 'D') # xED

## We create the Arch which belong the graph here ##
archlist = [
    arch1 ,
    arch2 ,
    arch3 ,
    arch4 ,
    arch5 ,
    arch6 ,
    arch7 ,
    arch8 ,
]

## Node exclussion
for j in range (1,len(sys.argv)):
    for i in range(0, len(archlist)-1):
        if archlist[i].getEnd() == sys.argv[j]:
            archlist[i].forbidpath()
## Example

## We put each arch parameter in a list in order to manage them #

edges = [
    arch1.getArch() ,
    arch2.getArch() ,
    arch3.getArch() ,
    arch4.getArch() ,
    arch5.getArch() ,
    arch6.getArch() ,
    arch7.getArch() ,
    arch8.getArch() ,
]

# for i in range(3,len(sys.argv)):
#     archlist[int(sys.argv[i])-1] = ''
#
# for arch in archlist:
#     if arch != '':
#         edges.append(arch.getArch())

for edge in edges:
    graph.add_edge(*edge)

try:
    rospy.init_node('dijstrasolve')
    solver = Solver()
    rospy.spin()
    ## Start point and endpoint in sys.argv
    #solver.dijsktra(graph, sys.argv[1],sys.argv[2])
    # print(dijsktra(graph, sys.argv[1],sys.argv[2]))
except rospy.ROSInterruptException:
    pass
