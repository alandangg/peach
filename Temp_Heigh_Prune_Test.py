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

df = pd.read_csv("germ11bCylinder.csv") ## ADJUST THESE FOR DIFFERENT TREES
treeData = pd.read_csv("germ11bData.csv")   ## ADJUST THESE FOR DIFFERENT TREES

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
    pcdFile = 'germ11b.xyzrgb'
    pcd = o3d.io.read_point_cloud(pcdFile) # load file
    pcd_centroid = pcd.get_center() # get centroid
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

    
    ## Adjustable Parameters

    meshType = "ellipsoid"

    if meshType == "cone":
        defaultxscale = 2.3
        defaultyscale = 2.3
        defaultzscale = 1.5
    elif meshType == "ellipsoid":
        defaultxscale = 2.7
        defaultyscale = 2.7
        defaultzscale = 1.9

    xcentroidadjust = 0         ##default 0
    ycentroidadjust = 0         ##default 0
    zcentroidadjust = 0         ##default 0
    xscaling = 0                ##Higher will shrink ellipsoid lower will increase
    yscaling = 0                ##Higher will shrink ellipsoid lower will increase
    zscaling = 0                ##Higher will shrink ellipsoid lower will increase
    xaddthreshold = 0.3         ##Default 0.3, determines how large of a gap is necessary to start scaling extra in one direction
    yaddthreshold = 0.3         ##Default 0.3, determines how large of a gap is necessary to start scaling extra in one direction
    
    if xgap > ygap:
        if (xgap - ygap) >= xaddthreshold:
            xadd = (xgap - ygap) / 2.2
    else:
        if (ygap - xgap) >= yaddthreshold:
            yadd = (xgap - ygap) / 2.2

    zeroOrderBranches = []
    oneOrderBranches = []
    notZeroOneOrderBranches = []

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) == 0:
            zeroOrderBranches.append((base, idx))

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) == 1:
            oneOrderBranches.append((base, idx))

    for idx, base in enumerate(bases):
        if getBranchOrder(df, idx) > 1:
            notZeroOneOrderBranches.append((base, idx))

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



    zeroOrderRadiusTune = zeroRadiusAverage - (zeroRadiusAverage/0.8)
    oneOrderRadiusTune = oneRadiusAverage - (oneRadiusAverage/0.8)
    regularOrderRadiusTune = 0.01

    for base, yy in zeroOrderBranches:
        tree[yy].paint_uniform_color((235/255., 86/255., 0/255.))

    for base, yy in oneOrderBranches:
        tree[yy].paint_uniform_color((26/255., 153/255., 136/255.))

    for base, yy in notZeroOneOrderBranches:
        tree[yy].paint_uniform_color((106/255., 172/255., 200/255.))

    treeCentroid = []
    heightAdjust = 0.05
    xx = df.x[2]
    yy = df.y[2]
    zz = df.z[2]
    treeCentroid = [xx, yy, zz]
    ellipsoid = o3d.geometry.TriangleMesh.create_sphere(1)
    ellipsoid.vertices = o3d.utility.Vector3dVector(np.asarray(ellipsoid.vertices) * np.array([10, 10, 0.1]))
    ellipsoid = ellipsoid.translate((treeCentroid))
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(ellipsoid))
    shapeAppend = ellipsoid

    remove_idx = []
    for idx, base in enumerate(bases):
        qpoints = o3d.core.Tensor(base, dtype=o3d.core.Dtype.Float32)
        occupancy = scene.compute_occupancy(qpoints)
        cylinders = findCylinders(idx)
        if occupancy.prod():
            for c in cylinders:
                remove_idx.append(c)

    cutCount = 0
    remove_idx = list(set(remove_idx))
    remove_idx.sort()
    for idx in remove_idx[::-1]: # remove indices in reverse
        cutCount += 1
        # tree[idx].paint_uniform_color((1., 0., 0.))
        tree.pop(idx)

    ##Useful Tree Info Printout
    # print("Tree Centroid", treeCentroid)
    # print("Tree Height", TreeHeight)
    # print("Average Crown Diameter", CrownDiamAve)
    # print("Crown Diameter Max", CrownDiamMax)
    # print("Crown Base Height", CrownBaseHeight)
    # print("Crown Length", CrownLength)
    # print("Crown Ratio", CrownRatio)
    # print("X Gap", xgap)
    # print("Y Gap", ygap)
    # print("Trunk Length", TrunkLength)
    # print("Total Cuts Performed", cutCount)
    # print("Zero Order Average Radius", zeroRadiusAverage)
    # print("One Order Average radius", oneRadiusAverage)
    # print("Average Branch Radius", averageRadius)
    # print("xoffset", xadd)
    # print("yoffset", yadd)
    # print("x and y diff", xgap - ygap)
    
    lineShape = o3d.geometry.LineSet.create_from_triangle_mesh(shapeAppend) # Convert solid sphere to mesh
    tree.append(lineShape)

    o3d.visualization.draw_geometries(tree)
