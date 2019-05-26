#!/usr/bin/env python
import rospy
from collections import defaultdict
from std_msgs.msg import String
import sys
from collections import deque

class Grafo():

    def __init__(self):
        self.relaciones = {}

    def __str__(self):
        return str(self.relaciones)

class Arista():

    def __init__(self, elemento, peso):
        self.elemento = elemento
        self.peso = peso

    def __str__(self):
        return str(self.elemento) + str(self.peso)

    def changeWeight(self,weight):
        self.peso = weight

def agregar(grafo, elemento):
    grafo.relaciones.update({elemento:[]})

def relacionar(grafo, elemento1, elemento2, peso = 1):
    relacionarUnilateral(grafo, elemento1, elemento2, peso)
    relacionarUnilateral(grafo, elemento2, elemento1, peso)

def relacionarUnilateral(grafo, origen, destino, peso):
    arista = Arista(destino, peso)
    grafo.relaciones[origen].append(arista)
    return arista

def caminoMinimo(grafo, origen, destino):
    etiquetas = {origen:(0,None)}
    dijkstra(grafo, destino, etiquetas, [])
    return construirCamino(etiquetas, origen, destino)

def construirCamino(etiquetas, origen, destino):
    if(origen == destino):
        return [origen]
    return construirCamino(etiquetas, origen, anterior(etiquetas[destino])) + [destino]

def dijkstra(grafo, destino, etiquetas, procesados):
    nodoActual = menorValorNoProcesado(etiquetas, procesados)
    if(nodoActual == destino):
        return
    procesados.append(nodoActual)
    for vecino in vecinoNoProcesado(grafo, nodoActual, procesados):
        generarEtiqueta(grafo, vecino, nodoActual, etiquetas)
    dijkstra(grafo, destino, etiquetas, procesados)

def generarEtiqueta(grafo, nodo, anterior, etiquetas):
    etiquetaNodoAnterior = etiquetas[anterior]
    etiquetaPropuesta = peso(grafo, anterior, nodo) + acumulado(etiquetaNodoAnterior),anterior
    if(not(etiquetas.has_key(nodo)) or  acumulado(etiquetaPropuesta) < acumulado(etiquetas[nodo]) ):
        etiquetas.update({nodo:etiquetaPropuesta})

def aristas(grafo, nodo):
    return grafo.relaciones[nodo]

def vecinoNoProcesado(grafo, nodo, procesados):
    aristasDeVecinosNoProcesados = filter(lambda x: not x in procesados, aristas(grafo,nodo))
    return [arista.elemento for arista in aristasDeVecinosNoProcesados]

def peso (grafo, nodoOrigen, nodoDestino):
    return reduce(lambda x,y: x if x.elemento == nodoDestino else y, grafo.relaciones[nodoOrigen]).peso

def acumulado(etiqueta):
    return etiqueta[0]

def anterior(etiqueta):
    return etiqueta[1]

def menorValorNoProcesado(etiquetas, procesados):
    etiquetadosSinProcesar = filter(lambda (nodo,_):not nodo in procesados, etiquetas.iteritems())
    return min(etiquetadosSinProcesar, key=lambda (_, (acum, __)): acum)[0]

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

class Solver():
    def __init__(self):
        self.solvePublisher = rospy.Publisher('/dijsktra_out', String, queue_size = 1)
        self.inputSubscriber = rospy.Subscriber("/dijsktra_inp", String, self.inputCallback)
        #Lista que guarda los nodos que necesito saber: El primero es el destino y
        #el resto son excluyentes
        self.nodesList = []
        self.bweight = 1
        self.dweight = 1
        self.eweight = 1

    def inputCallback(self, data):
        if(data.data != "finish"):
            self.nodesList.append(data.data)
        else:
            self.minimumPath()

    def minimumPath(self):

        a = "A"
        b = "B"
        c = "C"
        d = "D"
        e = "E"

        grafo = Grafo()

        agregar(grafo, a)
        agregar(grafo, b)
        agregar(grafo, c)
        agregar(grafo, d)
        agregar(grafo, e)

        for i in range(2,len(self.nodesList)):
            if self.nodesList[i] == 'B':
                self.bweight = 99
            elif self.nodesList[i] == 'D':
                self.dweight = 99
            elif self.nodesList[i] == 'E':
                self.eweight = 99

        arista1 = relacionar(grafo, a, b, self.bweight)
        arista2 = relacionar(grafo, a, d, self.dweight)
        arista3 = relacionar(grafo, a, e, self.eweight)
        arista4 = relacionar(grafo, b, c, 1)
        arista5 = relacionar(grafo, d, c, 1)
        arista6 = relacionar(grafo, e, c, 1)

        # path = caminoMinimo(grafo,'A', self.nodesList[0])
        path = caminoMinimo(grafo,self.nodesList[0], self.nodesList[1])
        print (path)

        pathway = []
        for way in path:
            pathway.append(node2location[way])

        rate = rospy.Rate(5)
        for location in pathway:
            rate.sleep()
            self.solvePublisher.publish(location)
            rate.sleep()
        self.solvePublisher.publish("end")



try:
    rospy.init_node('dijstrasolve')
    solver = Solver()
    rospy.spin()

except rospy.ROSInterruptException:
    pass
