#!/usr/bin/env python
import numpy as np
from math import *
from scipy.optimize import linprog
import os
from general_functions import *
from shapely.geometry import LineString
from shapely.geometry import Polygon
from shapely.geometry import MultiPoint
from shapely.affinity import rotate
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import trimesh
import copy
from jerk import *

mesh = trimesh.load_mesh('/home/terry/catkin_ws/src/scoop/src/Pre-Scoop Planning/Objects/Domino.STL', process=False, maintain_order=True)
#mesh.show(viewer='gl')
m = copy.deepcopy(mesh)
# FindAntipodalPairs and GetCrossSection
m.apply_transform(trimesh.transformations.projection_matrix((0,0,0),(0,0,1)))
vertices =  np.asanyarray(m.vertices)[:,[0,1]].tolist()
object_projection = xlist(MultiPoint(vertices).convex_hull.boundary.coords[:])[0:-1][::-1]
antipodal_pair_set = []
cross_section_point_set = []
cross_section_tan_vec_set = []
for i in range(len(object_projection)):
    maximum_rotate_angle = (inclination_angle(object_projection[(i+1)%4],object_projection[(i+2)%4])-inclination_angle(object_projection[i],object_projection[(i+1)%4]))*180/pi
    if maximum_rotate_angle<0:
        maximum_rotate_angle += 360
    if 'object_projection_after_rotation' not in dir():
        object_projection_temp = object_projection
    else:
        object_projection_temp = object_projection_after_rotation
    for angle in np.arange(0, maximum_rotate_angle+maximum_rotate_angle/100, maximum_rotate_angle/10):
        object_projection_after_rotation = [point_position_after_rotation(point0, [0,0], -angle) for point0 in object_projection_temp]

        lowest_point_y = Polygon(object_projection_after_rotation).bounds[1]
        object_projection_after_rotation = [[point[0], point[1]-lowest_point_y] for point in object_projection_after_rotation]
        bottom_point = []
        for j in range(len(object_projection)):
            if object_projection_after_rotation[j][1]<0.001:
                bottom_point.append(object_projection[j])
        vertical_distance = Polygon(object_projection_after_rotation).bounds[3]
        upper_point = []
        for j in range(len(object_projection)):
            if abs(object_projection_after_rotation[j][1]-vertical_distance)<0.001:
                upper_point.append(object_projection[j])
        if [bottom_point, upper_point] not in antipodal_pair_set:
            antipodal_pair_set.append([bottom_point, upper_point])
        if len(bottom_point)==1 and len(upper_point)==1:
            cross_section_point_set.append(bottom_point[0])
            cross_section_tan_vec_set.append(generate_normal_vector(bottom_point[0], upper_point[0]))
        elif len(bottom_point)==1:
            cross_section_point_set.append(bottom_point[0])
            cross_section_tan_vec_set.append(generate_normal_vector(bottom_point[0], footCtoAB(upper_point[0],upper_point[1],bottom_point[0])))
        elif len(upper_point)==1:
            cross_section_point_set.append(upper_point[0])
            cross_section_tan_vec_set.append(generate_normal_vector(upper_point[0], footCtoAB(bottom_point[0],bottom_point[1],upper_point[0])))
        else:
            upper_midpoint = [(upper_point[0][0]+upper_point[1][0])/2, (upper_point[0][1]+upper_point[1][1])/2]
            cross_section_point_set.append(upper_midpoint)
            cross_section_tan_vec_set.append(generate_normal_vector(upper_midpoint, footCtoAB(bottom_point[0],bottom_point[1],upper_midpoint)))
slice_set = []
for l in range(len(cross_section_point_set)):
    slice0 = mesh.section(plane_origin=[cross_section_point_set[l][0],cross_section_point_set[l][1],0], plane_normal=[-cross_section_tan_vec_set[l][1],cross_section_tan_vec_set[l][0],0])
    slice_2D, to_3D = slice0.to_planar()
    slice_2D_polygon = slice_2D.polygons_closed[0]
    slice_2D_polygon_bounds = slice_2D_polygon.bounds
    if slice_2D_polygon_bounds[3]-slice_2D_polygon_bounds[1]>slice_2D_polygon_bounds[2]-slice_2D_polygon_bounds[0]:
        slice_2D_polygon = rotate(slice_2D_polygon, -90, origin='center', use_radians=False)
    slides_coords = xlist(slice_2D_polygon.boundary.coords[:])[0:-1]
    slides_coords = [[point[0]-Polygon(slides_coords).bounds[0], point[1]-Polygon(slides_coords).bounds[1]] for point in slides_coords]
    slides_coords = [[round(x0*100)/1500 for x0 in y0] for y0 in slides_coords]
    slides_coords_no_repeated = []
    for point in slides_coords:
        if point not in slides_coords_no_repeated:
            slides_coords_no_repeated.append(point)
    slides_coords = slides_coords_no_repeated
    if slides_coords not in slice_set:
        slice_set.append(slides_coords)
print slice_set

# AnalyzeMobility
for object_vertex_position_on_ground in slice_set:
    env_profile = [[0,sqrt(3)], [1,0], [3,0]]  # thumb and ground
    phi = 10
    object_vertex_position_after_rot = [point_position_after_rotation(point, [0,0], -phi) for point in object_vertex_position_on_ground]
    lowest_point = object_vertex_position_after_rot[0]
    for point in object_vertex_position_after_rot:
        if point[1]<=lowest_point[1]:
            lowest_point = point
    object_vertex_position_after_rot = [[point[0], point[1]-lowest_point[1]] for point in object_vertex_position_after_rot]
    leftmost_distance_x_set = []
    for point in object_vertex_position_after_rot:
        if LineString([[-100, point[1]], [100, point[1]]]).intersects(LineString([[0,sqrt(3)], [1,0]]))==True:
            intersection_point = list(LineString([[-100, point[1]], [100, point[1]]]).intersection(LineString([[0,sqrt(3)], [1,0]])).coords[0])
            leftmost_distance_x_set.append(point[0]-intersection_point[0])
    object_vertex_position = [[point[0]-min(leftmost_distance_x_set), point[1]] for point in object_vertex_position_after_rot]

    feasible_finger_position_set = []
    for i in range(1,101):
        point_index_temp = (i-0.5)*(circumference(object_vertex_position)/100.0)
        finger_position = finger_index2position(object_vertex_position, point_index_temp)
        contact_index, contact_list, contact_normal_list, contact_local_tangent_list = contact_points_and_normal(object_vertex_position, finger_position, env_profile)
        A_ub = np.array(contact_normal_list)
        A_ub = A_ub[:, [1, 0]]
        A_ub[:, 0] = -A_ub[:, 0]
        A_ub = A_ub.tolist()
        b_ub = []
        for k in range(len(contact_list)):
            b_ub.append(-contact_list[k][0]*contact_normal_list[k][1]+contact_list[k][1]*contact_normal_list[k][0])
        f = [1,1]
        res1 = linprog(f, A_ub, b_ub)
        if res1.status == 2:
            Rotate_CW = False
        else:
            A_ub = (-np.array(A_ub)).tolist()
            b_ub = (-np.array(b_ub)).tolist()
            res2 = linprog(f, A_ub, b_ub)
            if res2.status !=2:
                Rotate_CW = False
            else:
                Rotate_CW = True
        if Rotate_CW == True:
            feasible_finger_position_set.append(finger_position)

    if feasible_finger_position_set != []:
        break

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-0.1, 3.1), ylim=(-0.1, 2.1))
ax.set_aspect('equal')
env_ground = [[0,0], [3,0]]
env_thumb = [[0,sqrt(3)], [1,0]]
env_plot0, = ax.plot([i[0] for i in env_ground], [i[1] for i in env_ground], 'grey', lw=2, zorder=-1) 
env_plot1, = ax.plot([i[0] for i in env_thumb], [i[1] for i in env_thumb], 'grey', lw=1.5, zorder=-1) 
verts=[tuple(k) for k in object_vertex_position]
verts.append(tuple(object_vertex_position[0]))
codes = [Path.MOVETO]
for j in range(len(verts)-2):
    codes.append(Path.LINETO)
codes.append(Path.CLOSEPOLY)
path=Path(verts,codes)
patch = patches.PathPatch(path, facecolor='darkgrey', edgecolor='black', lw=2, zorder=0)
ax.add_patch(patch)
for point in feasible_finger_position_set:
    feasible_finger_position, = ax.plot([point[0]], [point[1]], '-o', color='blue', zorder=5, lw=2, markersize=5)
plt.show()



