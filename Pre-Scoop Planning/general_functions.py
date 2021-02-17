#!/usr/bin/env python
import numpy as np
import re
import time
from math import *
from scipy.optimize import linprog
from jerk import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LinearRing
from shapely.geometry import MultiPoint
from shapely.affinity import rotate
from shapely.affinity import translate
from shapely.affinity import affine_transform
import cvxopt
from cvxopt import matrix
import os
import gurobipy
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

def CtoAB(A,B,C):
    CA=list(map(lambda x: x[0]-x[1], zip(A, C)))
    CB=list(map(lambda x: x[0]-x[1], zip(B, C)))
    ABdistance = np.sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)
    return abs(np.cross(CA,CB))/ABdistance

def generate_vector(A, B):
    return [B[0]-A[0], B[1]-A[1]]

def generate_normal_vector(A, B):
    length = sqrt((B[0]-A[0])**2+(B[1]-A[1])**2)
    return [(B[0]-A[0])/length, (B[1]-A[1])/length]

def inclination_angle(A,B):
    vector_A_to_B = generate_vector(A, B)
    return atan2(vector_A_to_B[1], vector_A_to_B[0])

#print inclination_angle([0,0],[1,0])

def calculate_intersect_angle_AB_CD(A, B, C, D):
    AB=list(map(lambda x: x[0]-x[1], zip(A, B)))
    CD=list(map(lambda x: x[0]-x[1], zip(C, D)))
    ABdistance = np.sqrt((A[0]-B[0])**2+(A[1]-B[1])**2)
    CDdistance = np.sqrt((C[0]-D[0])**2+(C[1]-D[1])**2)
    cos_angle = np.dot(AB,CD)/ABdistance/CDdistance
    sin_angle = np.cross(AB,CD)/ABdistance/CDdistance
    if abs(np.dot(AB,CD))<(ABdistance*CDdistance):
        if cos_angle>=0 and sin_angle>=0:
            tar_angle = acos(cos_angle)
        elif cos_angle>=0 and sin_angle<0:
            tar_angle = -acos(cos_angle)
        elif cos_angle<0 and sin_angle>=0:
            tar_angle = acos(cos_angle)
        else:
            tar_angle = -acos(cos_angle)
        return tar_angle*180/pi
    else:
        return 0

#print calculate_intersect_angle_AB_CD([0,0], [1,0], [0,0], [0,1])

def calculate_intersect_point_AB_CD(A,B,C,D):
    T1 = [[B[1]-A[1], A[0]-B[0]], [D[1] - C[1], C[0] - D[0]]]
    T2 = [[-(B[0]-A[0])*A[1]+(B[1]-A[1])*A[0]], [-(D[0]-C[0])*C[1]+(D[1]-C[1])*C[0]]]
    try:
        return np.squeeze(np.linalg.solve(T1,T2)).tolist()
    except BaseException:
        return False

def calculate_current_CoM(ini_vertex_position, current_vertex_position, ini_CoM_position):
    current_CoM_position_matrix = np.dot(np.dot(np.array([[current_vertex_position[0][0], current_vertex_position[0][1], 1], [current_vertex_position[1][0], current_vertex_position[1][1], 1], [current_vertex_position[2][0], current_vertex_position[2][1], 1]]).T, np.linalg.inv(np.array([[ini_vertex_position[0][0], ini_vertex_position[0][1], 1], [ini_vertex_position[1][0], ini_vertex_position[1][1], 1], [ini_vertex_position[2][0], ini_vertex_position[2][1], 1]]).T)), np.array([[ini_CoM_position[0]], [ini_CoM_position[1]], [1]])).tolist()
    current_CoM_position = [current_CoM_position_matrix[0][0], current_CoM_position_matrix[1][0]]
    return current_CoM_position

def footCtoAB(A,B,C):
    k=float(((C[0]-A[0])*(B[0]-A[0])+(C[1]-A[1])*(B[1]-A[1])))/float(((B[0]-A[0])*(B[0]-A[0])+(B[1]-A[1])*(B[1]-A[1])))
    return [A[0]+k*(B[0]-A[0]), A[1]+k*(B[1]-A[1])]

def calculate_parameter_translate_and_rotate_moveA(ini_A, ini_B, tar_A, tar_B):  
    delta_translate = [tar_A[0]-ini_A[0], tar_A[1]-ini_A[1]]
    A_r = inclination_angle(tar_A, tar_B)-inclination_angle(ini_A, ini_B)
    if A_r > 0:
        A_r = A_r%(2*pi)
        D_r = 'counterclockwise'
    else:
        A_r = A_r%(-2*pi)
        D_r = 'clockwise'
    return delta_translate, D_r, A_r

#print calculate_parameter_translate_and_rotate_moveA([0,0], [1,0], [1,0], [1+0.707,0.707])

def calculate_parameter_translate_and_rotate_moveB(ini_A, ini_B, tar_A, tar_B):  
    delta_translate = [tar_B[0]-ini_B[0], tar_B[1]-ini_B[1]]
    A_r = inclination_angle(tar_A, tar_B)-inclination_angle(ini_A, ini_B)
    if A_r > 0:
        A_r = A_r%(2*pi)
        D_r = 'counterclockwise'
    else:
        A_r = A_r%(-2*pi)
        D_r = 'clockwise'
    return delta_translate, D_r, A_r

#print calculate_parameter_translate_and_rotate_moveB([0,0], [1,0], [1,0], [1+0.707,0.707])

def point_position_after_rotation(current_xy, rotation_pole, desired_angle):
    desired_angle_rad = desired_angle*pi/180
    current_displacement = list(np.array(current_xy)-np.array(rotation_pole))
    rotation_matrix = [[cos(desired_angle_rad), -sin(desired_angle_rad)],[sin(desired_angle_rad), cos(desired_angle_rad)]]
    current_displacement = np.expand_dims(current_displacement, axis=1)
    temp = np.dot(rotation_matrix, current_displacement)
    xy_after_rotate = [list(rotation_pole[0]+temp[0])[0], list(rotation_pole[1]+temp[1])[0]]
    return xy_after_rotate 

#for point in [[0.5,0], [1.2,0], [1.2,1.5], [0.5,1.5], [1.2,0.8]]:
    #print point_position_after_rotation(point, [0.5, 0], 15)

#time2 = time.time()
#a = translate(LineString([[2,5], [3,8]]), xoff=0.3, yoff=0.5)
#a=point_position_after_rotation([2,5], [0,0], 1)
#print time.time()-time2

def point_position_after_rotation_batch(current_xy, rotation_pole, desired_angle):
    desired_angle_rad = np.array(desired_angle)*pi/180
    current_displacement = list(np.array(current_xy)-np.array(rotation_pole))
    rotation_matrix = np.array([[np.cos(desired_angle_rad), -np.sin(desired_angle_rad),np.sin(desired_angle_rad), np.cos(desired_angle_rad)]]).transpose().reshape(len(desired_angle),2,2)
    current_displacement = np.expand_dims(current_displacement, axis=1)
    temp = np.dot(rotation_matrix,current_displacement)
    xy_after_rotate = np.array([list(rotation_pole[0]+temp[:,0,0]), list(rotation_pole[1]+temp[:,1,0])])
    return xy_after_rotate.transpose().tolist()

#print point_position_after_rotation_batch([0,0], [1,0], [0,-30,-45,-60, -90, -120, -180])

def calculate_CoM(current_vertex_point):
    area = 0.0
    x,y = 0.0,0.0
    a = len(current_vertex_point)
    for i in range(a):
        lat = current_vertex_point[i][0]
        lng = current_vertex_point[i][1]
        if i == 0:
            lat1 = current_vertex_point[-1][0]
            lng1 = current_vertex_point[-1][1]
        else:
            lat1 = current_vertex_point[i-1][0]
            lng1 = current_vertex_point[i-1][1]
        fg = (lat*lng1 - lng*lat1)/2.0
        area += fg
        x += fg*(lat+lat1)/3.0
        y += fg*(lng+lng1)/3.0
    x = x/area
    y = y/area
    return x,y

def calculate_axial_symmetric_point_matrix(axis_A, axis_B):
    a=((axis_A[0]-axis_B[0])**2-(axis_A[1]-axis_B[1])**2)/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    b=2*(axis_A[0]-axis_B[0])*(axis_A[1]-axis_B[1])/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    c=(2*axis_A[0]*axis_B[1]**2-2*axis_A[0]*axis_A[1]*axis_B[1]-2*axis_A[1]*axis_B[0]*axis_B[1]+2*axis_A[1]**2*axis_B[0])/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    d=2*(axis_A[0]-axis_B[0])*(axis_A[1]-axis_B[1])/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    e=((axis_A[1]-axis_B[1])**2-(axis_A[0]-axis_B[0])**2)/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    f=(-2*axis_A[0]*axis_B[0]*axis_B[1]-2*axis_A[0]*axis_A[1]*axis_B[0]+2*axis_A[0]**2*axis_B[1]+2*axis_A[1]*axis_B[0]**2)/((axis_B[1]-axis_A[1])**2+(axis_B[0]-axis_A[0])**2)
    return a,b,c,d,e,f

def calculate_axial_symmetric_point(point, axis_A, axis_B):
    a,b,c,d,e,f=calculate_axial_symmetric_point_matrix(axis_A, axis_B)
    point_prime0=a*point[0]+b*point[1]+c
    point_prime1=d*point[0]+e*point[1]+f
    return [point_prime0, point_prime1]

def current_finger_angle(current_vertex_position, ini_vertex_position, finger_index):
    finger_angle = (finger_index%100-1)*5   
    _0, _1, A_r = calculate_parameter_translate_and_rotate(ini_vertex_position[0], ini_vertex_position[1], current_vertex_position[0], current_vertex_position[1])  
    finger_angle += A_r*180/pi  
    return finger_angle

def contact_points_and_normal(current_vertex_position, F1_contact_point, profile_environment, ini_vertex_position=None, F1_index = None, F2_index = None, accuracy = 0.01):
    F1 = F1_contact_point
    contact_index = []
    contact_list = []
    contact_normal_list = []
    contact_local_tangent_list = []
    index = 1
    for i in range(len(current_vertex_position)):
        for j in range(len(profile_environment)):
            if j!=0 and j!=len(profile_environment)-1 and Point(current_vertex_position[i]).distance(Point(profile_environment[j])) <accuracy:
                contact_point = current_vertex_position[i]
                contact_edge_length0 = LineString([profile_environment[j-1], profile_environment[j]]).length
                contact_edge_length1 = LineString([profile_environment[j], profile_environment[j+1]]).length
                contact_normal0 = [(profile_environment[j][1]-profile_environment[j-1][1])/contact_edge_length0, -(profile_environment[j][0]-profile_environment[j-1][0])/contact_edge_length0]
                contact_normal1 = [(profile_environment[j+1][1]-profile_environment[j][1])/contact_edge_length1, -(profile_environment[j+1][0]-profile_environment[j][0])/contact_edge_length1]
                if Point([contact_point[0]+contact_normal0[0]*0.011, contact_point[1]+contact_normal0[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                    contact_normal0 = [-contact_normal0[0], -contact_normal0[1]]
                if Point([contact_point[0]+contact_normal1[0]*0.011, contact_point[1]+contact_normal1[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                    contact_normal1 = [-contact_normal1[0], -contact_normal1[1]]
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        contact_normal_list[contact_list.index(point)] = [contact_normal0,contact_normal1]
                        contact_local_tangent_list[contact_list.index(point)] = [profile_environment[j-1], profile_environment[j], profile_environment[j+1]]
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_normal_list.append([contact_normal0,contact_normal1])
                    contact_local_tangent_list.append([profile_environment[j-1], profile_environment[j], profile_environment[j+1]])
            if j!=len(profile_environment)-1 and Point(current_vertex_position[i]).distance(LineString([profile_environment[j], profile_environment[j+1]]))<accuracy:
                contact_point = current_vertex_position[i]
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_edge_length = LineString([profile_environment[j], profile_environment[j+1]]).length
                    contact_normal = [(profile_environment[j+1][1]-profile_environment[j][1])/contact_edge_length, -(profile_environment[j+1][0]-profile_environment[j][0])/contact_edge_length]
                    if Point([contact_point[0]+contact_normal[0]*0.011, contact_point[1]+contact_normal[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                        contact_normal = [-contact_normal[0], -contact_normal[1]]
                    contact_normal_list.append(contact_normal)
                    contact_local_tangent_list.append([profile_environment[j], profile_environment[j+1]])
            if Point(profile_environment[j]).distance(LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy:
                contact_point = footCtoAB(current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))], profile_environment[j])
                for point in contact_list:
                    if Point(contact_point).distance(Point(point))<accuracy:
                        break
                else:
                    contact_list.append(contact_point)
                    contact_index.append('E'+str(index))
                    index = index+1
                    contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
                    contact_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
                    if Point([contact_point[0]+contact_normal[0]*0.011, contact_point[1]+contact_normal[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                        contact_normal = [-contact_normal[0], -contact_normal[1]]
                    contact_normal_list.append(contact_normal)
                    contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
    if F1 != None:
        if F1_index==None or F1_index<=100 and 'F1' not in contact_index:
            for i in range(len(current_vertex_position)):
                if 'F1' not in contact_index and F1 != None and Point(F1).distance(LineString([current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))]]))<accuracy:
                    contact_list.append(footCtoAB(current_vertex_position[i],current_vertex_position[(i+1)%(len(current_vertex_position))],F1))
                    contact_index.append('F1')
                    contact_edge_length = LineString([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]]).length
                    F1_normal = [(current_vertex_position[(i+1)%(len(current_vertex_position))][1]-current_vertex_position[i][1])/contact_edge_length, -(current_vertex_position[(i+1)%(len(current_vertex_position))][0]-current_vertex_position[i][0])/contact_edge_length]
                    if Point([F1[0]+F1_normal[0]*0.011, F1[1]+F1_normal[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                        F1_normal = [-F1_normal[0], -F1_normal[1]]
                    contact_normal_list.append(F1_normal)
                    contact_local_tangent_list.append([current_vertex_position[i], current_vertex_position[(i+1)%(len(current_vertex_position))]])
                    break
        elif ini_vertex_position!=None and F1_index>100:
            for i in range(len(current_vertex_position)):
                if 'F1' not in contact_index and Point(F1).distance(Point(current_vertex_position[i]))<accuracy:
                    F1_contact_position = current_vertex_position[i]
                    contact_list.append(F1_contact_position)
                    contact_index.append('F1')
                    F1_angle = (F1_index%100-1)*5   
                    _0, _1, A_r = calculate_parameter_translate_and_rotate(ini_vertex_position[0], ini_vertex_position[1], current_vertex_position[0], current_vertex_position[1])  
                    F1_angle += A_r*180/pi  
                    F1_normal = [-sin(F1_angle*pi/180), cos(F1_angle*pi/180)]   
                    if Point([F1_contact_position[0]+F1_normal[0]*0.011, F1_contact_position[1]+F1_normal[1]*0.011]).distance(Polygon(current_vertex_position))!=0:
                        F1_normal = [-F1_normal[0], -F1_normal[1]]
                    contact_normal_list.append(F1_normal)
                    contact_local_tangent_list.append([F1_contact_position, [F1_contact_position[0]+1*cos(F1_angle*pi/180), F1_contact_position[1]+1*sin(F1_angle*pi/180)]])
                    break
    return contact_index, contact_list, contact_normal_list, contact_local_tangent_list

#time5 = time.time()
#print(contact_points_and_normal([[0.5,0], [1.2,0], [1.2,1.5], [0.5,1.5]], [0.5,0.8], [1.2,0.9], [[0,0], [3,0]]))
#print(time.time()-time5)


def circumference(current_vertex_position):
    accumulate_length = LinearRing(current_vertex_position).length
    return accumulate_length

def finger_index2position(current_vertex_position, current_position_index):
    finger_position=list(list(LinearRing(current_vertex_position).interpolate(current_position_index).coords)[0])
    return finger_position
