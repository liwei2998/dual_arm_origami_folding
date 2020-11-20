#!/usr/bin/env python2
import os
import numpy as np
import numpy_indexed as npi

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
        vertice = np.around(vertice, decimals=2)
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
        self.grasp_technique = self.getGraspTechnique()
        self.grasp_point, self.grasp_line, self.grasp_method = self.OptimalGraspTechinique()
        self.GP_target_position = self.getGraspPointTargetPosition()
        self.fix_points_pair = self.getFixPaperPoints()
    def diff(self,arrayA,arrayB):       
        #difference
        diff_a = npi.difference(arrayA,arrayB)
        diff_b = npi.difference(arrayB,arrayA)
 
        return diff_a, diff_b

    def findbyrow(self,mat,row):
        return np.where((mat == row).all(1))[0]
    
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
        num_f2 = len(self.state2.face) #num_f1 <= num_f2
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
                index  = int(index)
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
    
    def getGraspTechnique(self):
        v1 = self.state1.vertice
        f1 = self.state1.face
        mv = self.manipulatible_vertices
        layer = int(mv[0][2] / 0.1)
        num_v = len(f1[layer])
        lv1 = self.state1.line_to_vertice
        grasp_technique = []
        

        for i in range(len(mv)):
            connect_lines = []
            index = self.findbyrow(v1,mv[i]) #mv[i]'s index in vertice
            index = int(index)
            lv1_temp = lv1[:,0,:]
            index1 = self.findbyrow(lv1_temp,mv[i])
            index1 = int(index1[0])
            connect_lines.append(lv1[index1][1])
            connect_lines.append(lv1[index1][2])
            
            count = 0
            for j in range(len(f1)):
                temp = np.where(f1[j]==index)
                temp = temp[0]
                if len(temp)>0:
                    count += 1

            if count == 1:
                action = "flex flip"
                grasp_technique.append([mv[i],connect_lines,action])
                
            elif count > 1:
                action = "scooping"
                v_temp = str(mv[i])
                grasp_technique.append([mv[i],connect_lines,action])                
                
        return grasp_technique

    def OptimalGraspTechinique(self):
        '''
        This function is used to determine optimal grasp point and grasp method.
        If only one grasp_technique, take the only one.
        If there are several choices, choose the one that minimize the angle between the grasp line and crease line.
        '''
        grasp = self.grasp_technique
        # transform 45 degree in the plane
        trans45 = np.array([[0.7071,-0.7071,0],[0.7071,0.7071,0],[0,0,1]])
        if len(grasp) == 1:
            grasp_point = grasp[0][0]
            grasp_line = grasp[0][1][0]
            grasp_method = grasp[0][2]
            return grasp_point, grasp_line, grasp_method
        elif len(grasp) > 1:
            norm_crease = self.crease_line / np.linalg.norm(self.crease_line)
            cosangle = []
            for i in range(len(grasp)):
                line = grasp[i][1][0]
                # grasp axis of flex flip is not align with the connect line, thus need to trans 45
                if grasp[i][2] == "flex flip":
                    line = np.dot(trans45,line)
                cosangle1 = line.dot(norm_crease)/np.linalg.norm(line)
                cosangle.append(cosangle1)
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
        # get target x, y position of grasp point
        cl = self.crease_line
        cp = self.crease_point[0]
        gp = self.grasp_point[0:2]
        a,b,c = self.vectortoline(cl,cp)
        tar_x, tar_y = self.symmetricpoint(a,b,c,gp[0],gp[1])
        tar_temp = np.array([tar_x, tar_y])
        tar_temp = tar_temp.astype(int)
        # find new added points in state2, GP_target_position(x,y) = (tar_x, tar_y) 
        v1 = self.state1.vertice
        v2 = self.state2.vertice
        _, diff_2 = self.diff(v1,v2)
        diff_2_temp = np.delete(diff_2, -1, axis=1)
        index = self.findbyrow(diff_2_temp, tar_temp)
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
        layer = int(self.grasp_point[2] / 0.1)
        f1 = self.state1.face[layer]
        v1 = self.state1.vertice
        cl = self.crease_line
        cp = self.crease_point
        gp = self.grasp_point
        tp = self.GP_target_position[:2]
        a,b,c = self.vectortoline(cl,cp[0])
        symbol1 = a*gp[0] + b*gp[1] +c
        distance = []
        points = []
        #step1: select the opposite points
        for i in range(len(f1)):
            point = v1[f1[i]]
            
            symbol2 = a*point[0] + b*point[1] + c
            #determine if the point is on the opposite side of the crease
            if symbol1*symbol2 < 0:
                #save the distance
                dis = self.PointsDistance(tp,point[:2])
                dis1 = self.PointsDistance(gp[:2],point[:2])
                distance.append([dis,dis1,f1[i]])
                points.append(point)
        #step2: determine the farthest points 
        # a) far from target position, and b) far from the grasp position
        points = np.array(points)
        dis_temp = np.array(distance)[:,0]
        dis_temp = dis_temp.tolist()
        dis_gp_temp = np.array(distance)[:,1]
        dis_gp_temp = dis_gp_temp.tolist()
        index_tp = [] 
        index_gp = []
        #return all temp_max indexs
        for i in range(len(dis_temp)):
            if dis_temp[i] == max(dis_temp):
                index_tp.append(distance[i][2])
            if dis_gp_temp[i] == max(dis_gp_temp):
                index_gp.append(distance[i][2])
        fix_point_tp = v1[f1[index_tp]]
        fix_point_gp = v1[f1[index_gp]]
        fix_point = npi.intersection(fix_point_tp,fix_point_gp)
        #step3: determine the fix paper points pair for each farthest points
        fix_points_pair = []
        for i in range(len(fix_point)):
            index = self.findbyrow(points,np.array(fix_point[i]))
            if index == 0:
                fix_points_pair.append([fix_point[i],cp[1]])
            else:
                fix_points_pair.append([fix_point[i],points[index-1]])
        fix_points_pair = np.array(fix_points_pair)
        
        return fix_points_pair


