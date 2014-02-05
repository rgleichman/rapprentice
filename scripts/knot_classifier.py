#!/usr/bin/env python

import numpy as np

#
# line segment intersection using vectors
# see Computer Graphics by F.S. Hill
#
def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(p1,p2,p3,p4) :
    p1=np.float32(p1)
    p2=np.float32(p2)
    p3=np.float32(p3)
    p4=np.float32(p4)
    numa = (p4-p3).dot(perp(p3-p1))
    numb = (p2-p1).dot(perp(p3-p1))
    denom = (p2-p1).dot(perp(p3-p4))
    if denom == 0:
        if numa==0 or numb==0: # coincident lines
            return (0.5,0.5)
        else: # parallel lines
            return None
    ua = (numa / denom)
    ub = (numb / denom)
    if ua >= 0 and ua <= 1 and ub >= 0 and ub <= 1:
        return (ua,ub)
    else:
        return None

def calculateIntersections(rope_nodes):
    """
    Takes in the nodes of a rope with n links.
    Returns the n x n matrix intersections, where intersections[i,j] = u_ij if link i intersects with link j at point pt_i, and intersections[i,j] = -1 otherwise.
    pt_i is the point on the line segment of link i with parameter u_ij.
    """
    intersections = -1*np.ones((rope_nodes.shape[0]-1, rope_nodes.shape[0]-1))
    for i_node in range(rope_nodes.shape[0]-1):
        for j_node in range(i_node+2,rope_nodes.shape[0]-1):
            intersect = seg_intersect(rope_nodes[i_node,:2], rope_nodes[i_node+1,:2], rope_nodes[j_node,:2], rope_nodes[j_node+1,:2])
            if intersect:
                intersections[i_node, j_node] = intersect[0]
                intersections[j_node, i_node] = intersect[1]
    return intersections

def calculateCrossings(rope_nodes):
    """
    Returns a list of crossing patterns by following the rope nodes; +1 for overcrossings and -1 for undercrossings.
    """
    intersections = calculateIntersections(rope_nodes)
    crossings = []
#     links_to_cross_info = {}
#     curr_cross_id = 1
    for i_link in range(intersections.shape[0]):
        j_links = sorted(range(intersections.shape[1]), key=lambda j_link: intersections[i_link,j_link])
        j_links = [j_link for j_link in j_links if intersections[i_link,j_link] != -1]
        for j_link in j_links:
            i_link_z = rope_nodes[i_link,2] + intersections[i_link,j_link] * (rope_nodes[i_link+1,2] - rope_nodes[i_link,2])
            j_link_z = rope_nodes[j_link,2] + intersections[j_link,i_link] * (rope_nodes[j_link+1,2] - rope_nodes[j_link,2])
            i_over_j = 1 if i_link_z > j_link_z else -1
            crossings.append(i_over_j)
#             link_pair_id = (min(i_link,j_link), max(i_link,j_link))
#             if link_pair_id not in links_to_cross_info:
#                 links_to_cross_info[link_pair_id] = []
#             links_to_cross_info[link_pair_id].append((curr_cross_id, i_over_j))
#             curr_cross_id += 1
#     # make sure rope is closed
#     dt_code = [0]*len(links_to_cross_info)
#     for cross_info in links_to_cross_info.values():
#         if cross_info[0][0]%2 == 0:
#             dt_code[cross_info[1][0]/2] = i_over_j * cross_info[0][0]
#         else:
#             dt_code[cross_info[0][0]/2] = i_over_j * cross_info[1][0]
    return crossings

def crossingsToString(crossings):
    s = ''
    for c in crossings:
        if c == 1:
            s += 'o'
        elif c == -1:
            s += 'u'
    return s

def isKnot(rope_nodes):
    crossings = calculateCrossings(rope_nodes)
    s = crossingsToString(crossings)
    knot_topologies = ['uououo', 'uoouuoou']
    for top in knot_topologies:
        if top in s:
            return True
        if top[::-1] in s:
            return True
        flipped_top = top.replace('u','t').replace('o','u').replace('t','o')
        if flipped_top in s:
            return True
        if flipped_top[::-1] in s:
            return True
    return False