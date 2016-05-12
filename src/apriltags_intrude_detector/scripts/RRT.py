#!/usr/bin/python

import random
import math

class Tree:

    def __init__(self,obstacles):
        self.vertices = []
        self.edges = []
        self.obstacles = obstacles
    def create(self,start,goal):
        self.vertices.append(start)
        delta = 150
        path = []
        stop = False;
        final_vertex = Vertex(0,0)
        while not stop:
            rand_vertex = self.generateVertex()
            nearest = self.getNearest(rand_vertex)
            increment = 10
            valid = True
            new_vertex = Vertex(0,0)
            while increment < delta and valid:
                temp_vertex = self.getNew(nearest,rand_vertex,increment)
                for obstacle in self.obstacles:
                    if not obstacle.contains(temp_vertex):
                        new_vertex = temp_vertex
                    else:
                        valid = False
                        break
                increment = increment + 10
            if new_vertex.x ==0 and new_vertex.y == 0:
                continue
            if goal.distance(new_vertex) < delta:
                stop = True;
            self.addVertex(new_vertex)
            self.addEdge(Edge(nearest,new_vertex))
            final_vertex = new_vertex
            # print str(len(self.vertices)) + "X: " + str(new_vertex.x) + " Y: " + str(new_vertex.y)
        next = final_vertex
        path.append(final_vertex)
        while next!=start:
            next = self.backTrack(next);
            path.append(next)
        return path

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
        # if nearest.distance(rand) < delta:
        #     return rand
        # else:
        if rand.x - nearest.y == 0:
            slope = 0
        else:
            slope = (rand.y - nearest.y)/(rand.x - nearest.y)
        x = 0
        y = 0
        if rand.x > nearest.x:
            x = math.floor(nearest.x + delta/(math.sqrt(1 + math.pow(slope,2))))
        else:
            x = math.floor(nearest.x - delta/(math.sqrt(1+math.pow(slope,2))))
        y = math.floor(slope * (x - nearest.x) + nearest.y)
        new_vertex = Vertex(x,y)
        return new_vertex
    def backTrack(self,vertex):
        for edge in self.edges:
            if edge.vertex2 == vertex:
                return edge.vertex1


class Vertex:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance(self,vertex):
        return math.sqrt(math.pow(vertex.x - self.x,2) + math.pow(vertex.y - self.y,2))

class Edge:
    def __init__(self,vertex1,vertex2):
        self.vertex1 = vertex1
        self.vertex2 = vertex2

    def contains(self,vertex):
        return self.vertex1 == vertex or self.vertex2 == vertex

class Obstacle:
    def __init__(self,points):
        self.points = points
    def contains(self, vertex):
        minX, maxX = self.min_maxX()
        minY, maxY = self.min_maxY()
        if vertex.x>minX and vertex.y<maxX and vertex.y>minY and vertex.y < maxY:
            return True
        return False
    def min_maxX(self):
        min = 900
        max = 0
        for point in self.points:
            if point.x < min:
                min = point.x
            if point.x > max:
                max = point.x
        return min, max

    def min_maxY(self):
        min = 900
        max = 0
        for point in self.points:
            if point.y < min:
                min = point.y
            if point.y > max:
                max = point.y
        return min, max

if __name__ == '__main__':
    tree = Tree([])
    start = Vertex(20,20)
    goal = Vertex(500,500)
    path = tree.create(start,goal)
    for p in path:
        print "X: "+str(p.x) + " Y: " + str(p.y)
