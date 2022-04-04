# Peach Tree Pruner
Alan Dang, Jun Yi Chuah, Jacob Nguyen, Tenzin Choerab <br> <br>

# Setting up 
1. Set your **absolute path** for cylinderPath.txt, dataPath.txt, and pcdPath.txt to the appropriate files:
   - cylinderPath requires a CylinderData.csv  
   - dataPath requires Data.csv (Optional)
   - pcdPath requires .xyzrgb
2. Make sure you are running on python 3.8 for the most stable experience. 
3. Run the frontend.py and you will be prompted with a gui. Load all the files and select the corresponding files, prune percentage, and pruning method. Then click prune! <br><br>


# General overview
1. Once you click prune, 2 figures will show up.
2. The first figure display how much sunlight each part of the tree is receiving.  
     - The brigher the green, the more sunlight it receives (Pretty cool huh :)
3. The second figure visualize the tree.
   - Branches highlighted in red represent which parts of the tree will be deleted in order for an optimal prune.
   - Colors other than red are used to increase the visibility of each branches. <br>

### Future implentation:
 - be able to see the before and after of the prune in the same window.
 - integrate the remaining algorithm
 - tabulate the removed points into csv <br>



# Credits:
 - UGA Griffin for the provided Lidar datasets of the trees
