"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

Original author: AtsushiSakai(@Atsushi_twi)

Modified by: Zachary Serlin (zserlin@bu.edu)

"""

import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np
import time

show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=1.0, goalSampleRate=5, maxIter=500,num_agents=1,past_paths=None):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0],start[1],t=0)
        self.end = Node(goal[0],goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.num_agents = num_agents
        self.past_paths = past_paths
        self.sigma = 10

    def Planning(self, animation=True,safe_size = 1):
        """
        Pathplanning

        animation: flag for animation on or off
        """
        self.nodeList = [self.start]
        saved_start = self
        count = 0
        restarted = 0
        while True:
            #If we get stuck - restart

            if count > 0:
                self.sigma = self.sigma + 20
                if count > 5:
                    self = saved_start
                    count = 0
                    restarted = 1
            else:
                self.sigma = 10
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                if restarted == 0:
                    rnd = [np.random.normal(self.end.x, self.sigma), np.random.normal(self.end.y, self.sigma)]
                if restarted == 1:
                    rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind
            newNode.t += 1

            if not self.__CollisionCheck(newNode, self.obstacleList):
                count += 1
                continue

            if not self.__AgentCollisionCheck(newNode, safe_size, self.past_paths):
                count +=1
                continue

            count = 0

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self,bounds, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis(bounds)
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def __AgentCollisionCheck(self, node, safe_size=1,past_paths=None):

        if past_paths != None:
            for i in range(0,len(past_paths)):
                last_pos = len(past_paths[i])
                if node.t < last_pos:
                    dx = past_paths[i][node.t][0] - node.x
                    dy = past_paths[i][node.t][1] - node.y
                    d = math.sqrt(dx * dx + dy * dy)
                    if d <= safe_size:
                        return False  # collision
                else:
                    dx = past_paths[i][last_pos-1][0] - node.x
                    dy = past_paths[i][last_pos-1][1] - node.y
                    d = math.sqrt(dx * dx + dy * dy)
                    if d <= safe_size:
                        return False  # collision
        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, t=0):
        self.x = x
        self.y = y
        self.parent = None
        self.t = t


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))

    return [x, y, ti]


def LineCollisionCheck(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False

    #  print("OK")

    return True  # OK


def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = GetPathLength(path)
    print('Initial Path: ',len(path))

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        #  print(first)
        second = GetTargetPoint(path, pickPoints[1])
        #  print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)
        print('Final Path: ',len(path))

    return path


def main():
    # ====Search Path with RRT====
    # Parameter

    t = time.time()
    obstacleList = [
        (50, 5, 12),
        (0, 6, 15),
        #(4, 6, 2),
        (-50,-10, 12),
        #(7, 2, 2),
        (0, -50, 12)
    ]  # [x,y,size]
    past_paths = []
    start=[(-90, -90),(90,90),(-90,-70),(90,70),(-90,-50),(90, 50),(-90,-30),(90,30),(-90,-10),(90,10),(-90, 90),(90,-90),(-90,70),(90,-70),(-90,50),(90,-50),(-90,30),(90,-30),(-90,10),(90,-10)]
    goal=[(90, 90),(-90,-90),(90,70),(-90,-70),(90,50),(-90, -50),(90,30),(-90,-30),(90,10),(-90,-10),(90, -90),(-90,90),(90,-70),(-90,70),(90,-50),(-90,50),(90,-30),(-90,30),(90,-10),(-90,10)]
    num_agents = len(start)
    bounds = [-100, 100]
    plot_bounds = [-100, 100, -100, 100]
    agent_radius = 5
    for i in range(0,num_agents):

        rrt = RRT(start=start[i], goal=goal[i],
                  randArea=bounds, obstacleList=obstacleList,past_paths=past_paths)
        #rrt = RRT(start=[(0, 0),(10,0)], goal=[(5, 10),(0,10)],
        #          randArea=[-2, 15], obstacleList=obstacleList)
        path = rrt.Planning(animation=False,safe_size=agent_radius)
        past_paths.append((path[::-1]))
        # Path smoothing
        #maxIter = 1000
        #smoothedPath = PathSmoothing(path, maxIter, obstacleList)
    print('All Found')
    print('Time:',(time.time()-t))
    # Draw final path
    if show_animation:
        #rrt.DrawGraph(plot_bounds)
        for i in range(0,num_agents):
            plt.plot([x for (x, y) in past_paths[i]], [y for (x, y) in past_paths[i]], '-r')
            #plt.plot([x for (x, y) in smoothedPath], [
            #    y for (x, y) in smoothedPath], '-b')
            plt.plot(start[i][0], start[i][1], "or")
            plt.plot(goal[i][0], goal[i][1], "xr")
        plt.axis(plot_bounds)
        for (x, y, size) in obstacleList:
            PlotCircle(x, y, size,plot_bounds)
        plt.grid(True)
        plt.pause(0.01)
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()

    #Play Trajectories as video
    max_time = 0
    for i in range(0,num_agents):
        if len(past_paths[i]) > max_time:
            max_time = len(past_paths[i])
    print(max_time)
    for t in range(0,max_time):
        DrawGraph(plot_bounds,past_paths,num_agents,start,goal,agent_radius,obstacleList,t)
        plt.pause(.01)
        #time.sleep(.01)

def DrawGraph(bounds,past_paths,num_agents,start,goal,agent_radius,obstacleList,t):
    plt.clf()
    for j in range(0,num_agents):
        max_time = len(past_paths[j])
        if max_time > t:
            plt.plot(start[j][0], start[j][1], ".r")
            plt.plot(goal[j][0], goal[j][1], "xr")
            plt.plot(past_paths[j][t][0],past_paths[j][t][1],"sb")
        else:
            plt.plot(start[j][0], start[j][1], ".r")
            plt.plot(goal[j][0], goal[j][1], "xr")
            plt.plot(goal[j][0], goal[j][1],"sb")

    for (x, y, size) in obstacleList:
        PlotCircle(x, y, size,bounds)
    plt.axis(bounds)
    plt.grid(True)

def PlotCircle(x, y, size,bounds):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    plt.axis(bounds)
    plt.grid(True)
    plt.plot(xl, yl, "-k")

if __name__ == '__main__':
    main()
