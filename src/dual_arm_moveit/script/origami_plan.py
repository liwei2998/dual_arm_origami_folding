#!/usr/bin/env python2
import os
import numpy as np
import numpy_indexed as npi

class State(object):

    def __init__(self,path):
        self.path = path
        self.vertice, self.line, self.face, self.line_to_vertice = self.data_process(path)

class State(object):

    def __init__(self,path):
        self.path = path
        self.vertice, self.line, self.face, self.line_to_vertice = self.data_process(path)

    def data_process(self, path):
        '''
        This function is used to process .obj file, will return arrays of vertice, line, and face.
        Vertice: shape(num_v,3), num_v is the number of all vertices. Type: array.
        Line: shape(num_f,num_v_in_f[i],3), num_f is the number of all faces,
            num_v_in_f[i] is the number of vertices in the ith face. Type: array.
        Face: shape(num_f,num_v_in_f[i]), num_f is  the number of all faces,
            num_v_in_face[i] is the number of vertices in the ith face. Type: array.
        Line to vertice: shape(num_v,3,3), num_v is the number of vertices. Type: array.
            restore num_v vertices, and each vertice connects two lines.
            format:[[vertice1,connect line1,connect line2],[vertice2,connect line2,connect line3]...]
        '''
        with open(path) as file:
            vertice = []
            face = []
            while 1:
                line = file.readline()
                if not line:
                    break
                strs = line.split(" ")
                if strs[0] == "v":
                    vertice.append([float(strs[1]), float(strs[2]), float(strs[3])])
                if strs[0] == "f":
                    del strs[0]
                    strs = np.array(strs)
                    strs = strs.astype(int)
                    strs = strs - 1
                    face.append(strs)

        vertice = np.array(vertice)
        vertice = np.around(vertice, decimals=4)

        face = np.array(face)

        line = []
        line_to_vertice = []
        for i in range(len(face)):
            line_temp = []
            for j in range(1, len(face[i])-1):
                line_temp_vector1 = (vertice[face[i][j]] - vertice[face[i][j-1]]).tolist() #clockwise
                line_temp.append(line_temp_vector1)
                line_temp_vector2 = (vertice[face[i][j+1]] - vertice[face[i][j]]).tolist() #clockwise
                line_to_vertice.append([vertice[face[i][j]].tolist(), line_temp_vector1, line_temp_vector2])
            line_temp.append(line_temp_vector2)
            line_temp_vector3 = (vertice[face[i][0]] - vertice[face[i][j+1]]).tolist()
            line_temp.append(line_temp_vector3)
            line_to_vertice.append([vertice[face[i][j+1]].tolist(), line_temp_vector2, line_temp_vector3])
            line_temp_vector4 = (vertice[face[i][1]] - vertice[face[i][0]]).tolist()
            line_to_vertice.append([vertice[face[i][0]].tolist(), line_temp_vector3, line_temp_vector4])

            line.append(line_temp)
        line = np.array(line)
        line_to_vertice = np.array(line_to_vertice)

        return vertice, line, face, line_to_vertice

    def update_path(self, new_path):
        self.path = new_path

class CompareState(object):
    def __init__(self,state1,state2):
        self.state1 = state1
        self.state2 = state2
        self.crease_line, self.crease_point, self.manipulatible_vertices = self.getCreaseLineAndManipulatibleVertice()
        self.grasp_solutions = self.getGraspSolutions()
        self.grasp_point, self.grasp_line, self.grasp_method = self.OptimalGraspTechinique()
        self.GP_target_position = self.getGraspPointTargetPosition()
        self.fix_point, self.fix_line = self.getFixPaperPoints()

    def diff(self,arrayA,arrayB):
        #difference
        diff_a = npi.difference(arrayA,arrayB)
        diff_b = npi.difference(arrayB,arrayA)

        return diff_a, diff_b

    def findbyrow(self,mat,row):
        return np.where((mat == row).all(1))[0]

    def findbyelement(self,mat,element):
        index = []
        for i in range(len(mat)):
            for j in range(len(mat[i])):
                if mat[i][j] == element:
                    index.append([i,j])
        return index

    def symmetricpoint(self,a,b,c,x,y):
        if a == 0:
            x1 = x
            y1 = -2*c/b - y
        elif b == 0:
            y1 = y
            x1 = -2*c/a - x
        elif a !=0 and b!= 0:
            x1 = -1*(2*a*b*y + (a*a-b*b)*x + 2*a*c) / (a*a + b*b)
            y1 = -1*((b*b-a*a)*y + 2*a*b*x + 2*b*c) / (a*a + b*b)
        return x1, y1

    def getCreaseLineAndManipulatibleVertice(self):
        '''
        logic: 1)search starts from the bottom face
               2)compare the difference between vertice[face[i]] of state1 and vertice[face[i]] of state2
                 a)if no difference -> face[i] hasn't been folded during last folding action;
                 b)if 2 new points added in face[i] of state2 -> crease is the line connecting the 2 new points;
                 c)if no new points in face[i] of state2 -> compare the difference between line[i] of state1, 2
                     -> crease is the line that in line[i] of state2 but not in line[i] of state1;
                 d)if 1 new points added in face[i] of state2 -> normalize line[i] in state1 and state2
                     -> crease is the line that in line[i] of state2 but not in line[i] of state1;
               3)search the next face, go back to 2)
        '''
        num_f1 = len(self.state1.face)
        crease_line = []
        v1 = self.state1.vertice
        v2 = self.state2.vertice
        crease_point = []

        for i in range(num_f1):
            f1 = self.state1.face[i]
            f2 = self.state2.face[i]
            l1 = self.state1.line[i]
            l2 = self.state2.line[i]
            diff_1, diff_2 = self.diff(v1[f1], v2[f2])

            if len(diff_1) == 0 and len(diff_2) == 0:
                print "No new added crease line in face ", i
                continue

            elif len(diff_2) == 2:
                #ensure crease line is clockwise
                if (v1[f1[0]] == diff_1[0]).all():
                    crease_line = diff_2[0] - diff_2[1]
                    crease_point.append(diff_2[1])
                    crease_point.append(diff_2[0])
                else:
                    crease_line = diff_2[1] - diff_2[0]
                    crease_point.append(diff_2[0])
                    crease_point.append(diff_2[1])

            elif len(diff_2) == 1:
                line_temp1 = []
                line_temp2 = []
                for j in range(len(l1)):
                    temp = l1[j] / np.linalg.norm(l1[j])
                    line_temp1.append(temp)
                for j in range(len(l2[i])):
                    temp = l2[j] / np.linalg.norm(l2[j])
                    line_temp2.append(temp)
                line_temp1 = np.array(line_temp1)
                line_temp2 = np.array(line_temp2)
                _, crease_line = self.diff(line_temp1, line_temp2)
                crease_line = crease_line.flatten()
                index = self.findbyrow(line_temp2,crease_line)
                index = int(index)
                crease_point.append(v2[f2[index]])
                crease_point.append(v2[f2[index+1]])


            elif len(diff_2) == 0:
                _, crease_line = self.diff(l1, l2)
                crease_line = crease_line.flatten()
                index = self.findbyrow(l2,crease_line)
                index = int(index)
                crease_point.append(v2[f2[index]])
                crease_point.append(v2[f2[index+1]])

            manipulatible_vertices = diff_1
            crease_point = np.array(crease_point)

            if crease_line != []:
                break

        return crease_line, crease_point, manipulatible_vertices

    def getGraspSolutions(self):
        '''
        This function is used to return a series of feasible grasp solutions.
        A solution: [[grasp point],[connect line1, connect line2],"grasp technique"]
        Technique: if the point only appears in one face: flex flip
                   else: scooping
        '''

        v1 = self.state1.vertice
        f1 = self.state1.face
        mv = self.manipulatible_vertices
        lv1 = self.state1.line_to_vertice
        grasp_solutions = []

        for i in range(len(mv)):
            #step1: find connect lines of mv[i]
            connect_lines = []
            index1 = self.findbyrow(lv1[:,0,:],mv[i]) #find mv[i]'s index in lv1
            index1 = int(index1[0])
            connect_lines.append(lv1[index1][1])#find two connect lines of mv[i]
            connect_lines.append(lv1[index1][2])
            #step2: count the appearance time of mv[i]
            index = self.findbyrow(v1,mv[i]) #mv[i]'s index in vertice
            index = int(index)
            count = self.findbyelement(f1,index)
            if len(count) == 1:
                action = "flex flip"
                grasp_solutions.append([mv[i],connect_lines,action])

            elif len(count) > 1:
                action = "scooping"
                grasp_solutions.append([mv[i],connect_lines,action])

        return grasp_solutions

    def rotate(self,theta):
        #clockwise rotate
        return np.array([[np.cos(theta),np.sin(theta),0],[-np.sin(theta),np.cos(theta),0],[0,0,1]])

    def OptimalGraspTechinique(self):
        '''
        This function is used to determine optimal grasp point and grasp method.
        If only one grasp_technique, take the only one.
        If there are several choices, choose the one that minimize the angle between the grasp line and crease line.
        '''
        grasp = self.grasp_solutions
        # transform 45 degree in the plane
#         trans45 = np.array([[0.7071,0.7071,0],[-0.7071,0.7071,0],[0,0,1]])
        #step1: if only one technique, choose the only one
        if len(grasp) == 1:
            grasp_point = grasp[0][0]
            grasp_line = grasp[0][1][0]
            grasp_method = grasp[0][2]
            return grasp_point, grasp_line, grasp_method
        #step2: Else. compute the angle between gripper frame and crease line
        elif len(grasp) > 1:
            norm_crease = self.crease_line / np.linalg.norm(self.crease_line)
            cosangle = []
            for i in range(len(grasp)):
                line = grasp[i][1][0] #grasp line
                # if "flex flip", grasp line rotate 45 degree
                if grasp[i][2] == "flex flip":
                    line = np.dot(self.rotate(np.pi/4),line)
                cosangle1 = line.dot(norm_crease)/np.linalg.norm(line)
                cosangle.append(abs(cosangle1))
            index = cosangle.index(max(cosangle))
            grasp_point = grasp[index][0]
            grasp_line = grasp[index][1][0]
            grasp_method = grasp[index][2]
            return grasp_point, grasp_line, grasp_method

    def vectortoline(self,vector,point):
        '''
        This function is used to tranform a line vector (with a point in this line vector)
            into a line (a*x + b*y + c = 0).
        '''
        vector = vector[0:2] / np.linalg.norm(vector[0:2])
        a = vector[1]
        b = -1*vector[0]
        c = - a*point[0] - b*point[1]
        return a, b, c

    def getGraspPointTargetPosition(self):
        '''
        This fuction is used to compute the target position of grasp point.
        Logic: grasp point and target position are symmetric about the crease line.
        '''
        #step1: get target x, y position of grasp point
        cl = self.crease_line
        cp = self.crease_point[0]
        gp = self.grasp_point[0:2]
        a,b,c = self.vectortoline(cl,cp)
        tar_x, tar_y = self.symmetricpoint(a,b,c,gp[0],gp[1])
        tar_temp = np.array([tar_x, tar_y])
        tar_temp = np.around(tar_temp, decimals=4)
        #step2: find tp's id in diff_v2, GP_target_position(x,y) = (tar_x, tar_y)
        v1 = self.state1.vertice
        v2 = self.state2.vertice
        _, diff_2 = self.diff(v1,v2)
        diff_2_temp = np.delete(diff_2, -1, axis=1)
        for i in range(len(diff_2_temp)):
            if abs(np.linalg.norm(diff_2_temp[i]-tar_temp)) <= 0.5:
                index = i
                break
        GP_target_position = diff_2[index]
        GP_target_position = GP_target_position.flatten()

        return GP_target_position

    def PointsDistance(self,point1,point2):
        dx = point1[0]-point2[0]
        dy = point1[1]-point2[1]
        dis = np.sqrt(pow(dx,2) + pow(dy,2))
        return dis

    def getFixPaperPoints(self):
        #currently we assume that paper must be fixed at the same layer as the new added crease's
        '''
        This function is used to find two points for paper fixing. For the two points, one is the fix point,
            the other is the adjacent point of the fix point, complying with the fix_paper function.
        Logic: a) fix point and grasp point are on the different side of crease line.
               b) distance between fix point and GR_target_position is max.
        '''
        f1 = self.state1.face
        v1 = self.state1.vertice
        cl = self.crease_line
        cp = self.crease_point
        gp = self.grasp_point
        tp = self.GP_target_position[:2]
        #step1: determine fix at which layer
        index = self.findbyrow(v1,gp)[0] # index is gp's id in all vertices
        index = self.findbyelement(f1,index) # search which faces contain gp
        layer = index[0][0] # select the lowest face that contain gp
        f1 = f1[layer]
        #step2: select points on the other side of the crease in the face layer
        a,b,c = self.vectortoline(cl,cp[0]) #get crease line function
        symbol1 = a*gp[0] + b*gp[1] +c #symbol of point * line
        distance = []
        points = []
        for i in range(len(f1)):
            point = v1[f1[i]]
            symbol2 = a*point[0] + b*point[1] + c
            #if the point is on the other side, symbol1*symbol2 < 0
            if symbol1*symbol2 < 0 and abs(symbol2) > 5:
                sym = symbol2
                dis = np.linalg.norm(tp - point[:2]) #distance of tp and the point
                dis1 = np.linalg.norm(gp[:2] - point[:2]) #distance of gp and the point
                distance.append([dis,dis1,f1[i]])
                points.append(point)
        #step3: determine the farthest points
        # a) far from target position, and b) far from the grasp position
        points = np.array(points)
        distance = np.around(np.array(distance), decimals=0)
        dis_tp_temp = np.array(distance)[:,0] #take the dis_tp
        dis_tp_temp = dis_tp_temp.tolist()
        dis_gp_temp = np.array(distance)[:,1] #take the dis_gp
        dis_gp_temp = dis_gp_temp.tolist()
        index_tp = distance[np.where(dis_tp_temp == np.amax(dis_tp_temp))][:,2].astype(int)
        index_gp = distance[np.where(dis_gp_temp == np.amax(dis_gp_temp))][:,2].astype(int)
        fix_point_tp = v1[f1[index_tp]] #points that have max distance with tp
        fix_point_gp = v1[f1[index_gp]] #points that have max distance eith gp
        fix_point = npi.intersection(fix_point_tp,fix_point_gp) #points that have both max distance with tp and gp
        if len(fix_point) == 0:
            fix_point = fix_point_gp #id no intersection, choose points have max dis with gp
        #step4: determine the fix paper line, save several lines in case of collision
        fix_line = []
        #fix point on the left side of cl
        if sym < 0:
            for i in np.arange(np.pi/2,np.pi*7/6,np.pi/6):
                fix_line.append((np.dot(self.rotate(i),cl)).tolist())
        #if fix point on right side of cl
        elif sym > 0:
            for i in np.arange(-np.pi/2,-np.pi*7/6,-np.pi/6):
                fix_line.append((np.dot(self.rotate(i),cl)).tolist())
        fix_line = np.around(np.array(fix_line),decimals = 2)
        return fix_point, fix_line

    def DataFormatinMoveit(self):
        #crease
        self.crease_line = self.crease_line.tolist()
        self.crease_point = self.crease_point / 1000
        self.crease_point[0][2] += 0.71
        self.crease_point[1][2] += 0.71
        self.crease_point = self.crease_point.tolist()
        #grasp
        self.grasp_line = self.grasp_line.tolist()
        self.grasp_point = self.grasp_point / 1000
        self.grasp_point[2] += 0.71
        self.grasp_point = self.grasp_point.tolist()
        #GP target
        self.GP_target_position = self.GP_target_position / 1000
        self.GP_target_position[2] += 0.71
        self.GP_target_position = self.GP_target_position.tolist()
        #fix
        self.fix_line = self.fix_line.tolist()
        for i in range(len(self.fix_point)):
            self.fix_point[i] = self.fix_point[i] / 1000
            self.fix_point[i][2] = self.fix_point[i][2] + 0.71
        self.fix_point = self.fix_point.tolist()
