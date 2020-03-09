#!/usr/bin/python2
from queue import Queue
import math

class BottleQueue:
    class CircleDictionary:
        def __init__(self,radius):
            self.radius = radius
            self.pose_list = list()
        
        def add(self,x,y,yaw):
            valid = True
            for a,b in self.pose_list:
                if math.sqrt(pow(float(a-x),2) + pow(float(b-y),2)) < self.radius:
                    valid = False
                    break
            if valid:
                self.pose_list.append(tuple([x,y,yaw]))
            
            return valid

        def remove(self,x,y,yaw):
            self.pose_list.remove(tuple([x,y,yaw]))
            
    def __init__(self,radius):
        self.circle_dict = self.CircleDictionary(radius)
        self.pose_queue = Queue()
    
    def push(self,x,y,yaw):
        valid = self.circle_dict.add(x,y,yaw)
        if valid:
            self.pose_queue.put(tuple([x,y,yaw]))
        
    def pop(self):
        x,y,yaw = self.pose_queue.get()
        self.circle_dict.remove(x,y,yaw)
        return tuple([x,y,yaw])

    def isEmpty(self):
        return self.pose_queue.empty()

    def print_points(self):
        print(self.circle_dict.pose_list)

    