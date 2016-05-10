#!/usr/bin/python

import random
import math

class Tree:

    def __init__(self,vertex):
        self.vertices = []
        self.edges = []
        self.vertices.append(vertex)
    def create(self):
        self
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


class Vertex:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance(self,vertex):
        return math.sqrt(math.pow(vertex.x - self.x,2) + math.pow(vertex.y - self.y,2))
