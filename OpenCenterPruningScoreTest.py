from copy import copy
import numpy as np
import pandas as pd
import open3d as o3d
import copy
import matplotlib.pyplot as plt

def getRadius(df, x):
    currentBranch = df['radius (m)'][x]
    return currentBranch

def getBranchOrder(df, x):
        currentBranch = df['branch_order'][x]
        return currentBranch

def getRayHits(primitive_list, rays, show_im = True):
    scene = o3d.t.geometry.RaycastingScene() # t.geometry refers to Tensor
    for prim in primitive_list:
        prim_id = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(prim))

    ans = scene.cast_rays(rays)
    plt.imshow(ans['t_hit'].numpy())
    plt.show()

treeDic = {}

df = pd.read_csv("data/germ3aCylinderData.csv") ## ADJUST THESE FOR DIFFERENT TREES
treeData = pd.read_csv("data/germ3aData.csv")   ## ADJUST THESE FOR DIFFERENT TREES

def findCylinders(index):
        cylinders = []
        key = df['branch'].values[index]
        cylinders = treeDic[key]
        return cylinders

for num, val in df['branch'].iteritems():
    if val not in treeDic:
        treeDic[val] = [num]
    else:
        treeDic[val].append(num)

if __name__ == '__main__':
    # Create raycasting scene
    # We will need this later for checking occupancy
    scene = o3d.t.geometry.RaycastingScene()

    #largest gap so far is .4ish

    TreeHeight = treeData["Data"][3]
    CrownDiamAve = treeData["Data"][14]
    CrownDiamMax = treeData["Data"][15]
    CrownBaseHeight = treeData["Data"][18]
    CrownLength = treeData["Data"][19]
    CrownRatio = treeData["Data"][20]
    TrunkLength = treeData["Data"][4]

    
    tree = []
    bases = [] # Store the base of the cylinder
    max_branch = df['branch'].max()
    pcdFile = 'data/germ3a_ds_0025m_2021.xyzrgb'
    pcd = o3d.io.read_point_cloud(pcdFile) # load file
    pcd_centroid = pcd.get_center() # get centroid
    # Shift pcd such that centroid is at origin 
    # convenient for calculations
    pcd.translate(-pcd_centroid)
    np.asarray(pcd.colors)[:] = [1, 1, 0.8]
    
    centroid = np.array([0., 0., 0.])
    num_cylinders = len(df)
    resolution = 20 # Resolution of the cylinder segments

    def getRayHits(primitive_list, rays, show_im = True):
        scene = o3d.t.geometry.RaycastingScene() # t.geometry refers to Tensor
        # Make a list of zero and one order branches
        zero_one_primids = []
        other_primids = []
        for idx, prim in enumerate(primitive_list):
            prim_id = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(prim))
            if getBranchOrder(df, idx) < 2:
                zero_one_primids.append(prim_id)
            else:
                other_primids.append(prim_id)

        zero_one_primids = np.array(zero_one_primids)
        ans = scene.cast_rays(rays)
        geom_ids = np.array(ans['geometry_ids'].numpy())

        zero_hit = np.isin(geom_ids, zero_one_primids)
        other_hit = np.isin(geom_ids, other_primids)

        plt.imshow(ans['t_hit'].numpy())
        plt.show()

        primitives_hit = (ans['t_hit'] != np.inf).to(o3d.core.Dtype.Int32)
        return zero_hit.sum(), primitives_hit.sum()

    for row_num, (radius, length, x, y, z, a, b, c, branch, branch_order) in df.iterrows():
        #coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[x, y, z])
        #tree.append(coord)
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length)
        v2r = np.array([a, b, c])
        zv = np.array([0, 0, 1])
        ax = np.cross(zv, v2r)
        angle = np.arccos(np.dot(zv, v2r))
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(ax * angle)
        cylinder = cylinder.rotate(R)
        cylinder.compute_vertex_normals()

        # Shift cylinders for easier calculation
        l = 0.5 * length
        x -= pcd_centroid[0]
        y -= pcd_centroid[1]
        z -= pcd_centroid[2]
        
        centroid += [x, y, z]
        cyl_mid = [x + l*a, y + l*b, z + l*c]
        cylinder = cylinder.translate(cyl_mid)
        #cylinder.paint_uniform_color((branch / (1.03 * max_branch), branch /(1.01 * max_branch), branch/(1.02*max_branch)))
        # The vertices for the cylinder are arranged in resolution * (split + 1) segments
        # The first two verts are the centers of the top and bottom base
        dpoints = np.asarray(cylinder.vertices)
        dpoints = dpoints[2:2+resolution,:]
        ## Store these dpoints in a list that has a one-one correspondance 
        ## with our tree list
        bases.append(dpoints)
        tree.append(copy.deepcopy(cylinder))

    #### SETUP ####
    res = 1200 ## resolution
    center = 3.
    width = 2.5 ## This represents the grid on top of the tree that you are shooting from
    z_ht = 10. # this doesn't matter much so long it is above the tree height
    marks = np.linspace(center-width, center+width, res)
    ray_dir = -np.array([center, center, z_ht])
    ray_dir /= np.linalg.norm(ray_dir)
    
    mesh_x, mesh_y = np.meshgrid(marks, marks)
    rays = np.stack((mesh_x,
                     mesh_y,
                     np.ones((res, res))*10,
                     np.ones((res, res))*ray_dir[0],
                     np.ones((res, res))*ray_dir[1],
                     np.ones((res, res))*ray_dir[2]),
                    axis = 2
                   )


    rays = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)

    #### PART I - BEFORE PRUNING ####
    #### Get scene hits before pruning 
    zero_fore, primhit_fore = getRayHits(tree, rays)
    print("Before pruning: ", primhit_fore.item())

    treeCentroid = []
    xx = df.x[2] - pcd_centroid[0]
    yy = df.y[2] - pcd_centroid[1]
    zz = df.z[2] + (TreeHeight/1.4) - pcd_centroid[2] ## --- turn into parameter tree height adjust
    treeCentroid.append(xx)
    treeCentroid.append(yy)
    treeCentroid.append(zz)

    highestx = df['x'].max()
    lowestx = df['x'].min()
    highesty = df['y'].max()
    lowesty = df['y'].min()
    highestz = df['z'].max()
    lowestz = df['z'].min()
    xgap = highestx - lowestx
    ygap = highesty - lowesty
    zgap = highestz - lowestz

    xadd = 0
    yadd = 0

    if xgap > ygap:
        if (xgap - ygap) >= .3:
            xadd = 0.2
    else:
        if (ygap - xgap) >= .3:
            yadd = 0.2

    xellipsoid = (CrownDiamAve / 2.7) + xadd
    yellipsoid = (CrownDiamAve / 2.7) + yadd
    zellipsoid = TreeHeight / 1.9

    ellipsoid = o3d.geometry.TriangleMesh.create_sphere(1)
    ellipsoid.vertices = o3d.utility.Vector3dVector(np.asarray(ellipsoid.vertices) * np.array([xellipsoid, yellipsoid, zellipsoid]))
    ellipsoid = ellipsoid.translate((treeCentroid))
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(ellipsoid))

    zeroOrderBranches = []
    oneOrderBranches = []
    notZeroOneOrderBranches = []

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) == 0:
            zeroOrderBranches.append(base)

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) == 1:
            oneOrderBranches.append(base)

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) != 1 and getBranchOrder(df, idx) != 0:
            notZeroOneOrderBranches.append(base)

    radiusCount = 0
    averageRadius = 0
    zeroRadiusAverage = 0
    oneRadiusAverage = 0
    zeroRadiusCount = 0
    oneRadiusCount = 0

    for i, base in enumerate(notZeroOneOrderBranches):
        radius = df["radius (m)"][i]
        radiusCount += 1
        averageRadius += radius
    averageRadius = averageRadius / radiusCount

    for i, base in enumerate(zeroOrderBranches):
        radius = df["radius (m)"][i]
        zeroRadiusCount += 1
        zeroRadiusAverage += radius
    zeroRadiusAverage = zeroRadiusAverage / zeroRadiusCount

    for i, base in enumerate(oneOrderBranches):
        radius = df["radius (m)"][i]
        oneRadiusCount += 1
        oneRadiusAverage += radius
    oneRadiusAverage = oneRadiusAverage / oneRadiusCount

    zeroOrderRadiusTune = -0.01
    oneOrderRadiusTune = -0.01
    regularOrderRadiusTune = 0.01

    remove_idx = []
    for idx, base in enumerate(bases):
        qpoints = o3d.core.Tensor(base, dtype=o3d.core.Dtype.Float32)
        occupancy = scene.compute_occupancy(qpoints)
        cylinders = findCylinders(idx)
        if occupancy.prod():
            for c in cylinders:
                if c >= idx:
                        if getBranchOrder(df, idx) == 0:
                            if getRadius(df, idx) <= zeroRadiusAverage + zeroOrderRadiusTune:
                                remove_idx.append(c)
                        elif getBranchOrder(df, idx) == 1:
                            if getRadius(df, idx) <= (oneRadiusAverage + oneOrderRadiusTune):
                                remove_idx.append(c)
                        else:
                            if getRadius(df, idx) <= (averageRadius + regularOrderRadiusTune):
                                remove_idx.append(c)

    cutCount = 0
    # Build a kdtree
    pcd_tree = o3d.geometry.KDTreeFlann(pcd) 
    remove_idx = list(set(remove_idx))
    remove_idx.sort()
    # make a final tree that only has pruned cylinders
    final_tree = []
    for idx in remove_idx[::-1]: # remove indices in reverse
        cutCount += 1
        # extract the top base point for pcd
        prune_point = np.asarray(tree[idx].vertices)[1]
        
        # locate two nearest neightbours for the point in pcd
        [_, pcd_prune, _] = pcd_tree.search_knn_vector_3d(prune_point, 2)
        np.asarray(pcd.colors)[pcd_prune[1:], :] = [1, 0, 0]
        final_tree.append(tree[idx])
        
        #tree[idx].paint_uniform_color((1., 0., 0.))
        # tree.pop(idx)

    
    #### PART II - AFTER PRUNING ####
    # Get hits from rays 
    zero_aft, primhit_aft = getRayHits(tree, rays)
    print("After pruning: ", primhit_aft.item())
    ratio = zero_aft.item() / zero_fore.item()

    # Ratio of the number of zero order branches
    # hit after and before pruning
    print(f"ZeroSunRatio: {ratio}")

    # for idx, base in enumerate(bases):
    #     if getBranchOrder(df, idx) == 1:
    #         tree[idx].paint_uniform_color((0., 0., 0.))

        
    
    ##Useful Tree Info Printout
    #print("Tree Centroid", treeCentroid)
    #print("Tree Height", TreeHeight)
    #print("Average Crown Diameter", CrownDiamAve)
    #print("Crown Diameter Max", CrownDiamMax)
    #print("Crown Base Height", CrownBaseHeight)
    #print("Crown Length", CrownLength)
    #print("Crown Ratio", CrownRatio)
    #print("X Gap", xgap)
    #print("Y Gap", ygap)
    #print("Trunk Length", TrunkLength)
    #print("Total Cuts Performed", cutCount)
    #print("Zero Order Average Radius", zeroRadiusAverage)
    #print("One Order Average radius", oneRadiusAverage)
    #print("Average Branch Radius", averageRadius)
    #print("xoffset", xadd)
    #print("yoffset", yadd)
    #print("x and y diff", xgap - ygap)

    line_ellipsoid = o3d.geometry.LineSet.create_from_triangle_mesh(ellipsoid) # Convert solid sphere to mesh
    #tree.append(line_ellipsoid)

    final_tree.append(pcd)
    
    o3d.visualization.draw_geometries(final_tree)
