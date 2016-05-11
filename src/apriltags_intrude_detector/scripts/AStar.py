#!/usr/bin/python

# from enum import Enum
import heapq
import math

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Grid:
    def __init__(self,x,y,polygons):
        self.debug = True
        #self.start = None
        self.x = x
        self.y = y
        self.nodes = [[0 for a in range(self.x)] for a in range(self.y)]
        for i in range(self.y):
            for j in range(self.x):
                pointY = i*20
                pointX = j*20
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
                                self.nodes[i][j].addNeighbor(self.nodes[i][j-1])
                            if(self.nodes[i-1][j-1].getOverlaps() == False):
                                self.nodes[i][j].addNeighbor(self.nodes[i-1][j-1])
                                self.nodes[i-1][j-1].addNeighbor(self.nodes[i][j])
                            if j < self.y - 1:
                                if self.nodes[i-1][j+1].getOverlaps() == False:
                                    self.nodes[i][j].addNeighbor(self.nodes[i-1][j+1])
                                    self.nodes[i-1][j+1].addNeighbor(self.nodes[i][j])
                    else:
                        if j>0:
                            if(self.nodes[i][j-1].getOverlaps() == False):
                                self.nodes[i][j-1].addNeighbor(self.nodes[i][j])
                                self.nodes[i][j].addNeighbor(self.nodes[i][j-1])

    def toString(self):
        node_line = ""
        for i in range(self.y):
            for j in range(self.x):
                if str(self.nodes[i][j].getOverlaps())[0] == "T":
                    node_line = node_line + "X" + " "
                else:
                    node_line = node_line + "O" + " "
            node_line = node_line + "\n"
        return node_line

    def heuristic(self, a, b):
        (x1, y1) = a.getCenterX(), a.getCenterY()
        (x2, y2) = b.getCenterX(), b.getCenterY()
        return abs(x1 - x2) + abs(y1 - y2)

    def cost(self, current, next):
        (x1, y1) = current.getCenterX(), current.getCenterY()
        (x2, y2) = next.getCenterX(), next.getCenterY()
        return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2));

    def a_star(self, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                print "Found goal"
                break
            # print str(current.getNeighbors())
            for next in current.getNeighbors():
                new_cost = cost_so_far[current] + self.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
                        

        return came_from, cost_so_far

    def find(self,x,y):
        xcoord = math.floor(x/20)
        ycoord = math.floor(y/20)
        return self.nodes[int(ycoord)][int(xcoord)]

class Node:

    def __init__(self,points):
        if len(points) == 0:
            return
        self.centerX = 0
        self.centerY = 0
        self.points = points

        for point in points:
            self.centerX += point[0]
            self.centerY += point[1]
        self.centerX = self.centerX/len(points)
        self.centerY = self.centerY/len(points)
        self.neighbors = []
        self.overlap = False

    def __lt__(self, other):
        return True

    def addNeighbor(self,neighbor):
            self.neighbors.append(neighbor)

    def getNeighbors(self):
        return self.neighbors

    def overlaps(self,poly):
        if self.maxX() < self.minPolyX(poly):
            return False
        if self.minX() > self.maxPolyX(poly):
            return False

        if self.maxY() < self.minPolyY(poly):
            return False
        if self.minY() > self.maxPolyY(poly):
            return False
        return True

    def setOverlaps(self,overlaps):
        self.overlap = overlaps

    def getOverlaps(self):
        return self.overlap

    def getCenterX(self):
        return self.centerX

    def getCenterY(self):
        return self.centerY

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

    def maxPolyX(self, poly):
        max = 0
        for point in poly.points:
            if point.x > max:
                max = point.x
        return max
    def minPolyX(self, poly):
        min = 900
        for point in poly.points:
            if point.x < min:
                min = point.x
        return min
    def maxPolyY(self, poly):
        max = 0
        for point in poly.points:
            if point.y > max:
                max = point.y
        return max
    def minPolyY(self, poly):
        min = 700
        for point in poly.points:
            if point.y < min:
                min = point.y
        return min
# class DIR(Enum):
#     N = 90
#     NE = 135
#     E = 180
#     SE = -135
#     S = -90
#     SW = -45
#     W = 0
#     NW = 45

if __name__ == '__main__':
    grid = Grid(100, 100, [])
    start = grid.nodes[0][50]
    end = grid.nodes[50][60]
    path, cost = grid.a_star(start, end)

    x = 1
    i = end
    print("Node " + str(x) + ": Center X: " + str(path[i].getCenterX()) + " Center Y: " + str(path[i].getCenterY()))
    while path[i] != None:
        x += 1
        print("Length: " + str(cost[i]))
        print("Node " + str(x) + ": Center X: " + str(path[i].getCenterX()) + " Center Y: " + str(path[i].getCenterY()))
        i = path[i]
    print("Total Nodes in Path: " + str(x))
