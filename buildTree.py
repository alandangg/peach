from audioop import reverse
from copy import copy
import math
import numpy as np
import pandas as pd
import open3d as o3d
import copy
import os

def get_rotation(v1, v2):
    c = np.cross(v1, v2)
    d = np.dot(v1, v2)
    nv1 = np.linalg.norm(v1)
    nv2 = np.linalg.norm(v2)

    zv = [[0.0, -c[2], -c[1]], [c[2], 0.0, -c[0]], [-c[2], c[1], 0.0]]
    zv = np.array(zv)
    R = np.eye(3) + zv + zv**2 * (1 - d) / np.linalg.norm(c) ** 2 / nv1 ** 2

    return R

def distBetwn3DPoints(arr1, arr2):
    deltaX = arr1[0] - arr2[0]
    deltaY = arr1[1] - arr2[1]
    deltaZ = arr1[2] - arr2[2]

    dist = math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2) + math.pow(deltaZ, 2))

    return dist

# def findCylinders(index):
#     cylinders = []
#     key = df['branch'].values[index]
#     cylinders = treeDic[key]
#     return cylinders


def get_rotation_mat(direction, up=[0, 0, 1]):
    R = np.zeros((3, 3))
    up = np.array(up)
    direction = np.array(direction)

    xaxis = np.cross(direction, up)
    xaxis /= np.linalg.norm(xaxis)

    yaxis = np.cross(xaxis, direction)
    yaxis /= np.linalg.norm(yaxis)

    R[0][0] = xaxis[0];
    R[0][1] = yaxis[0];
    R[0][2] = direction[0];

    R[1][0] = xaxis[1];
    R[1][1] = yaxis[1];
    R[1][2] = direction[1];

    R[2][0] = xaxis[2];
    R[2][1] = yaxis[2];
    R[2][2] = direction[2];

    return R

if __name__ == '__main__':

    df = pd.read_csv("Data/germ3a/germ3a.csv") #change to appropriate data for tree
    pcdFile = 'Data/germ3a/germ3a_ds_0025m_2021.txt'

    orderBranchCylDic = {} # { Order : {BranchIdx: {Total cylinders} } }
    orderBranchMidDic = {} # { Order : {BranchIdx: {Midpoint of first cylinder base} } }
    toRemoveBranch = [] # {Order : {BranchIdx} }

    tree = []
    bases = [] # Store the base of the cylinder
    max_branch = df['branch'].max()

    centroid = np.array([0., 0., 0.])
    num_cylinders = len(df)
    resolution = 20 # Resolution of the cylinder segments
    prevBranch = 0
    

    for row_num, (radius, length, x, y, z, a, b, c, branch, order) in df.iterrows():
        branch = int(branch)
        order = int(order)
        row_num = int(row_num)

        if order in orderBranchCylDic:
            if branch in orderBranchCylDic[order]:
                orderBranchCylDic[order][branch].append(row_num)
            else:
                orderBranchCylDic[order][branch] = {}
                orderBranchCylDic[order][branch] = [row_num]
        else:
            orderBranchCylDic[order] = {}
            orderBranchCylDic[order][branch] = {}
            orderBranchCylDic[order][branch] = [row_num]

        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length)
        v2r = np.array([a, b, c])
        v2r /= np.linalg.norm(v2r)
        zv = np.array([0, 0, 1])
        ax = np.cross(zv, v2r)
        angle = np.real(np.arccos(v2r[-1]))

        ax /= np.linalg.norm(ax)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(ax * angle)

        cylinder = cylinder.rotate(R)
        cylinder.compute_vertex_normals()
        l = 0.5 * length
        centroid += [x, y, z]
        cyl_mid = [x + l*a, y + l*b, z + l*c]

        cylinder = cylinder.translate(cyl_mid)

        # The vertices for the cylinder are arranged in resolution * (split + 1) segments
        # The first two verts are the centers of the top and bottom base
        dpoints = np.asarray(cylinder.vertices)
        bpoints = dpoints[len(dpoints) - (resolution):] # Base points, vertices for the lower surface of cylinder
        dpoints = dpoints[2:2+resolution,:]
        
        ## Store these dpoints in a list that has a one-one correspondance 
        ## with our tree list
        bases.append(dpoints)

        if (branch != prevBranch and order == 2):
            prevBranch = branch

            baseMid = [sum(x[0] for x in bpoints) / len(bpoints), # midpoint of X
                sum(x[1] for x in bpoints) / len(bpoints),        # midpoint of Y
                sum(x[2] for x in bpoints) / len(bpoints)]        # midpoint of Z

                
            if order in orderBranchMidDic: # check whether BRANCH ORDER is in the dic
                orderBranchMidDic[order][branch] = {} # Since only the first cylinder is considered, each branch is unique
                orderBranchMidDic[order][branch] = baseMid

            else: # steps to include BRANCH ORDER into the dict -> link BRANCH NUMBER -> append midpoint
                orderBranchMidDic[order] = {}
                orderBranchMidDic[order][branch] = {}
                orderBranchMidDic[order][branch] = baseMid

        tree.append(copy.deepcopy(cylinder))

    
    distThreshold = 0.2

    minIdx = list(orderBranchMidDic[2].keys())[0]
    
    branchIdxList = list(orderBranchMidDic[2].keys())
    idxCounter = 0
    invalid = True
    startCounter = branchIdxList[-1]

    orderBranchMidDic[2] = dict(list(reversed(orderBranchMidDic[2].items())))
    #tree[orderBranchCylDic[2][targetIdx][0]].paint_uniform_color((0., 1., 1.))

    while(startCounter > minIdx):
        targetIdx = branchIdxList[-1 - idxCounter]

        if targetIdx in orderBranchCylDic[2]:
            startCounter = targetIdx - 1
            
            while(startCounter not in orderBranchCylDic[2] and startCounter > minIdx):
                startCounter -= 1
  
            startIdx = startCounter

            while(startIdx > minIdx):
                dist = distBetwn3DPoints(orderBranchMidDic[2][targetIdx], orderBranchMidDic[2][startIdx])

                if (dist < distThreshold and startIdx in orderBranchCylDic[2]):
                    toRemoveBranch.append(orderBranchCylDic[2][startIdx])
                    orderBranchCylDic[2].pop(startIdx)
                    
                startIdx -= 1

        idxCounter += 1


    cylNum = 0

    toRemoveBranch = [cylNum for x in toRemoveBranch for cylNum in x]
    toRemoveBranch = list(set(toRemoveBranch))
    toRemoveBranch = list(sorted(toRemoveBranch))
    for idx in toRemoveBranch[::-1]: # remove indices in reverse
        #print(idx)
        #tree.pop(idx)
        tree[idx].paint_uniform_color((1., 0., 0.))
        #print(idx)`


    o3d.visualization.draw(tree)
    
