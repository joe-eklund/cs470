#!/usr/bin/python

from enum import Enum

from src.apriltags_intrude_detector.scripts.sphero_intrude_gui import Field


class Grid:
    def __init__(self,x,y,polygons):
        self.x = x
        self.y = y
        self.nodes = [[None for i in range(self.x)] for j in range(self.y)]
        for i in range(self.x):
            for j in range(self.y):
                pointX = i*20
                pointY = j*20
                points = [[pointX,pointY],
                          [pointX,pointY+20],
                          [pointX+20,pointY],
                          [pointX+20,pointY+20]]
                self.nodes[i][j] = Node(points)
                for poly in polygons:
                    if self.nodes[i][j].overlaps(poly):
                        self.nodes[i][j].setOverlaps(True)
                if self.nodes[i][j].getOverlaps()==False:
                    if i > 0:
                        if self.nodes[i-1][j].getOverlaps() == False:
                            self.nodes[i][j].addNeighbor(self.nodes[i-1][j])
                            self.nodes[i-1][j].addNeighbor(self.nodes[i][j])
                        if j > 0:
                            if(self.nodes[i][j-1].getOverlaps() == False):
                                self.nodes[i][j-1].addNeighbor(self.nodes[i][j])
                                self.nodes[i][j].addNeighbor(self.nodes[i-1][j])
                            if(self.nodes[i-1][j-1].getOverlaps() == False):
                                self.nodes[i][j].addNeighbor(self.nodes[i-1][j-1])
                                self.nodes[i-1][j-1].addNeighbor(self.nodes[i][j])
                            if j < self.y:
                                if self.nodes[i-1][j+1].getOverlaps() == False:
                                    self.nodes[i][j].addNeighbor(self.nodes[i-1][j+1])
                                    self.nodes[i-1][j+1].addNeighbor(self.nodes[i][j])



class Node:

    def __init__(self,points):
        self.points = points
        for point in points:
            self.centerX += point[0]
            self.centerY += point[1]
            self.centerX = self.centerX/len(points)
        self.centerY = self.centerY/len(points)
        self.neighbors = []
        self.overlaps = False

    def addNeighbor(self,neighbor):
        self.neighbors.append(neighbor)

    def getNeighbors(self):
        return self.neighbors

    def overlaps(self,poly):
        if self.maxX() < self.minX(poly):
            return False
        elif self.minX() > self.maxX(poly):
            return False

        if self.maxY() < self.minY(poly):
            return False
        elif self.minY() > self.maxX(poly):
            return False
        return True

    def setOverlaps(self,overlaps):
        self.overlaps = overlaps

    def getOverlaps(self):
        return self.overlaps
    def maxX(self):
        max = 0
        for point in self.points:
            if point[0] > max:
                max = point[0]
        return max
    def minX(self):
        min = 900
        for point in self.points:
            if point[0] < min:
                min = point[0]
        return min
    def maxY(self):
        max = 0
        for point in self.points:
            if point[1] > max:
                max = point[1]
        return max
    def minY(self):
        min = 700
        for point in self.points:
            if point[1] < min:
                min = point[1]
        return min

    def maxX(self, poly):
        max = 0
        for point in poly.points:
            if point.x > max:
                max = point.x
        return max
    def minX(self,poly):
        min = 900
        for point in poly.points:
            if point.x < min:
                min = point.x
        return min
    def maxY(self,poly):
        max = 0
        for point in poly.points:
            if point.y > max:
                max = point.y
        return max
    def minY(self,poly):
        min = 700
        for point in poly.points:
            if point.y < min:
                min = point.y
        return min
class DIR(Enum):
    N = 90
    NE = 135
    E = 180
    SE = -135
    S = -90
    SW = -45
    W = 0
    NW = 45