#!/usr/bin/python

import random
import math

class Tree:

    def __init__(self,vertex):
        self.vertices = []
        self.edges = []
        self.vertices.append(vertex)
    def create(self,goal):
        delta = 10
        stop = False;
        while not stop:
            rand_vertex = self.generateVertex()
            nearest = self.getNearest(rand_vertex)
            new_vertex = self.getNew(nearest,rand_vertex,delta)
            if goal.distance(new_vertex) < delta:
                stop = True;
            self.addVertex(new_vertex)
            self.addEdge(Edge(nearest,new_vertex))

    def addVertex(self, vertex):
        self.vertices.append(vertex)
    def addEdge(self, edge):
        self.edges.append(edge)
    def generateVertex(self):
        return Vertex(random.randrange(0,800,1),random.randrange(0,600,1))
    def getNearest(self,new_vertex):
        nearestVertex = None
        distance = 10000
        for vertex in self.vertices:
            if new_vertex.distance(vertex) < distance:
                distance = new_vertex.distance(vertex)
                nearestVertex = vertex
        return nearestVertex
    def getNew(self,nearest,rand,delta):
        if nearest.distance(rand) < delta:
            return rand
        else:
            slope = (rand.y - nearest.y)/(rand.x - nearest.y)
            new_vertex = Vertex(nearest.x + slope*delta,nearest.y + slope*delta)
            return new_vertex

class Vertex:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance(self,vertex):
        return math.sqrt(math.pow(vertex.x - self.x,2) + math.pow(vertex.y - self.y,2))

class Edge
    def __init__(self,vertex1,vertex2):
        self.vertex1 = vertex1
        self.vertex2 = vertex2