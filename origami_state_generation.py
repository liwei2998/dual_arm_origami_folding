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
stack1 = [["1","2","3","4"]]

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

def findAllCreases(stack,facets):
    creases = []
    for i in range(len(stack)):
        for j in range(len(stack[i])):
            facet = stack[i][j]
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

crease = findAllCreases(stack1,facets1)
print "crease",crease
min_crease = findMininalSetCrease(crease)
print "min_crease",min_crease
#how to set(min_crease)

def findFeasibleCrease(crease,stack,polygen):
    feasible_crease=[]
    for i in range(len(stack)):
        for k in range(len(crease)):
            point1 = crease[k][0]
            point2 = crease[k][1]
            for j in range(len(stack[i])):
                facet = stack[i][j]
                poly = polygen[facet]
                if is_inPoly(poly,point1)==1 or is_inPoly(poly,point2)==1:
                    break
                elif j == len(stack[i])-1:
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

def divideStack(crease,stack,polygon,sign):
    base = []
    flap = []
    a,b,c = lineToFunction(crease)
    for i in range(len(stack)):
        base_tmp = []
        flap_tmp = []
        for j in range(len(stack[i])):
            facet = stack[i][j]
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
feasible_crease = findFeasibleCrease(min_crease,stack1,polygen1)
print "feasible_crease",feasible_crease
base,flap = divideStack(feasible_crease[0],stack1,polygen1,"+")
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

def reversePoint(crease,point):
    a,b,c = lineToFunction(crease)
    x = point[0]
    y = point[1]
    reversed_point = []
    if a == 0:
        x1 = x
        y1 = -2*c/b - y
    elif b == 0:
        y1 = y
        x1 = -2*c/a - x
    elif a !=0 and b!= 0:
        x1 = -1*(2*a*b*y + (a*a-b*b)*x + 2*a*c) / (a*a + b*b)
        y1 = -1*((b*b-a*a)*y + 2*a*b*x + 2*b*c) / (a*a + b*b)
    reversed_point.append(x1)
    reversed_point.append(y1)
    return reversed_point

def reverseLine(crease,line):
    point1 = line[0]
    point2 = line[1]
    reversed_line = []
    new_point1 = reversePoint(crease,point1)
    new_point2 = reversePoint(crease,point2)
    reversed_line.append(new_point1)
    reversed_line.append(new_point2)
    return reversed_line

def reverseCrease(flap,crease,facet_crease):
    new_facet_crease = facet_crease
    # remain creases in base stack
    # reverse creases in flap stack
    for i in range(len(flap)):
        for j in range(len(flap[i])):
            facet = flap[i][j]
            creases = facet_crease[facet]
            new_creases = []
            for k in range(len(creases)):
                new_creases.append(reverseLine(crease,creases[k]))
            new_facet_crease[facet] = new_creases
    return new_facet_crease

def reversePolygen(flap,crease,polygon):
    new_polygen = polygon
    # remain polygens in base stack
    # reverse polygens in flap stack
    for i in range(len(flap)):
        for j in range(len(flap[i])):
            facet = flap[i][j]
            poly = polygon[facet]
            new_poly = []
            for k in range(len(poly)):
                new_poly.append(reversePoint(crease,poly[k]))
            new_polygen[facet] = new_poly
    return new_polygen

reversed_polygen = reversePolygen(flap,feasible_crease[0],polygen1)
print "reversed polygen",reversed_polygen
reversed_creases = reverseCrease(flap,feasible_crease[0],facets1)
print "reversed crease",reversed_creases

def generateNextStateInformation(stack,polygon,facet_crease,crease,sign):
    '''
    Given a feasible crease fold, previous stack and polygen and facet_crease information,
    return new stack and polygen and facet_crease information.
    '''
    #search for base and flap according to this crease
    base,flap = divideStack(feasible_crease[0],stack1,polygen1,sign)

    #generate new stack
    reversed_stack = reverseStack(base,flap)

    #generate new polygen
    reversed_polygen = reversePolygen(flap,feasible_crease[0],polygen1)

    #generate new facet creases
    reversed_creases = reverseCrease(flap,feasible_crease[0],facets1)

    return reversed_stack,reversed_polygen,reversed_creases

state1 = {"stack":stack1,"polygen":polygen1,"facet_crease":facets1}

def generateNextLayerStates(state):
    '''
    input parent node state information, return next layer's children states
    '''
    new_states = []
    #find minimal set of lines taht contain all creases
    crease = findAllCreases(state["stack"],state["facet_crease"])
    min_crease = findMininalSetCrease(crease)
    #how to set(min_crease)
    #find all feasible creases
    feasible_crease = findFeasibleCrease(min_crease,state["stack"],state["polygen"])
    print "feasible crease",feasible_crease
    #generate new states for each feasible crease
    for i in range(len(feasible_crease)):
        state_tmp = {}
        new_stack,new_polygen,new_creases = generateNextStateInformation(state["stack"],state["polygen"],
                                                                         state["facet_crease"],feasible_crease[i],
                                                                         "+")
        state_tmp["stack"] = new_stack
        state_tmp["polygen"] = new_polygen
        state_tmp["facet_crease"] = new_creases
        new_states.append(state_tmp)
    return new_states

new_states = generateNextLayerStates(state1)
print "new state",new_states
print "new_state1",new_states[0]
names = locals()
names["state" + str(2)] = new_states[0]
print "state2",state2
