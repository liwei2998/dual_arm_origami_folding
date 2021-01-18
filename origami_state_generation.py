#!/usr/bin/env python
import numpy_indexed as npi
from shapely.geometry import *
#clockwise?
facets1 = {"1":[[[1,1],[0,0]],[[0,0],[-1,1]]],
          "2":[[[1,1],[0,0]],[[0,0],[1,-1]]],
          "3":[[[0,0],[1,-1]],[[0,0],[-1,-1]]],
          "4":[[[0,0],[-1,-1]],[[0,0],[-1,1]]]}

polygen1 = {"1":[[1,1],[0,0],[-1,1]],
            "2":[[1,1],[1,-1],[0,0]],
            "3":[[1,-1],[0,0],[-1,-1]],
            "4":[[0,0],[-1,-1],[-1,1]]}
state1 = [["1","2","3","4"]]

def ifLineColinear(line1,line2):
    '''
    input two lines, return if they are colinear
    '''
    "line1 = [[0,0],[1,1]]"
    k1 = (line1[0][1] - line1[1][1])/(line1[0][0] - line1[1][0])
    k2 = (line2[0][1] - line2[1][1])/(line2[0][0] - line2[1][0])

    if k1 == k2:
        return 1
    else:
        return 0

def ifLineSame(line1,line2):
    if len(npi.difference(line1,line2)) == 0:
        return 1
    else:
        return 0

def CombineLinearLines(line1,line2):
    '''
    input two colinear lines, return the combined one line
    '''
    "line1 = [[0,0],[1,1]]"
    new_line = npi.exclusive(line1,line2)
    new_line = new_line.tolist()
    return new_line

def findAllCreases(state,facets):
    creases = []
    for i in range(len(state)):
        for j in range(len(state[i])):
            facet = state[i][j]
            creases.append(facets[facet][0])
            creases.append(facets[facet][1])

    return creases

def findMininalSetCrease(crease):
    #find linear creases, and combine them
    min_crease = []
    for i in range(len(crease)-1):
        line1 = crease[i]
        for j in range(i+1,len(crease)):
            line2 = crease[j]
            if ifLineSame(line1,line2)==1:
                continue
            if ifLineColinear(line1,line2)==1:
                min_crease.append(CombineLinearLines(line1,line2))
                break
            elif j == (len(crease)-1):
                min_crease.append(line1)
    return min_crease

crease = findAllCreases(state1,facets1)
print "crease",crease
min_crease = findMininalSetCrease(crease)
print "min_crease",min_crease
#how to set(min_crease)

def findFeasibleCrease(crease,state,polygen):
    feasible_crease=[]
    for i in range(len(state)):
        for k in range(len(crease)):
            point1 = crease[k][0]
            point2 = crease[k][1]
            for j in range(len(state[i])):
                facet = state[i][j]
                poly = polygen[facet]
                if is_inPoly(poly,point1)==1 or is_inPoly(poly,point2)==1:
                    break
                elif j == len(state[i])-1:
                    feasible_crease.append(crease[k])
    return feasible_crease

def is_inPoly(polygen,point):
    line = LineString(polygen)
    pointt = Point(point)
    polygen = Polygon(line)
    return polygen.contains(pointt)

def lineToFunction(line):
    "input line[[x1,y1],[x2,y2]], return k,b (ax+by+c=0)"
    if line[0][0] == line[1][0]:
        b = 0
        if line[0][0] == 0:
            a = 1
            c = 0
            return a,b,c
        c = 1
        a = -1 / line[0][0]
        return a,b,c
    elif line[1][1] == line[0][1]:
        a = 0
        if line[0][1] == 0:
            b = 1
            c = 0
            return a,b,c
        c = 1
        b = -1 / line[0][1]
        return a,b,c
    else:
        k = (line[0][1] -line[1][1]) / (line[0][0] - line[1][0])
        if abs(line[0][1]) == abs(line[0][0]):
            c = 0
            b = 1
            a = -k
            return a,b,c
        b = -1 / (-k*line[0][0] + line[0][1])
        a = -k*b
        c = 1
        return a,b,c

def divideStack(crease,state,polygon,sign):
    base = []
    flap = []
    a,b,c = lineToFunction(crease)
    for i in range(len(state)):
        base_tmp = []
        flap_tmp = []
        for j in range(len(state[i])):
            facet = state[i][j]
            poly = polygon[facet]
            for k in range(len(poly)):
                product = a*poly[k][0]+b*poly[k][1]+c
                if sign == "+":
                    if product > 0:
                        flap_tmp.append(facet)
                        break
                    elif product < 0:
                        base_tmp.append(facet)
                        break
                if sign == "-":
                    if product < 0:
                        flap_tmp.append(facet)
                        break
                    elif product > 0:
                        base_tmp.append(facet)
                        break
        base.append(base_tmp)
        flap.append(flap_tmp)
    return base,flap
feasible_crease = findFeasibleCrease(min_crease,state1,polygen1)
print "feasible_crease",feasible_crease
base,flap = divideStack(feasible_crease[0],state1,polygen1,"+")
print "base_stack",base
print "flap_stack",flap

def reverseStack(base,flap):
    '''
    input base stack and flap stack, return reversed stack
    '''
    new_stack = []
    #base stack will be remained
    for i in range(len(base)):
        new_stack.append(base[i])
    #reverse the flap
    new_flap = flap[::-1]
    for j in range(len(new_flap)):
        new_stack.append(new_flap[j])
    return new_stack

new_stack = reverseStack(base,flap)
print "new_stack",new_stack
