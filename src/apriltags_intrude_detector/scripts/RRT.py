#!/usr/bin/python

import random
class Tree:

    def __init__(self,vertex):
        self.vertices = []
        self.edges = []
        self.vertices.append(vertex)
    def addVertex(self, vertex):
        self.vertices.append(vertex)
    def addEdge(self, edge):
        self.edges.append(edge)
    def generateVertex(self):
        return Vertex(random.randrange(0,800,1),random.randrange(0,600,1))
    def getNearest(self,new_vertex):
        for vertex in self.vertices:
            vertex


class Vertex:
    def __init__(self,x,y):
        self.x = x
        self.y = y
