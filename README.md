# peach
# Alan Dang, Jun Yi Chuah, Jacob Nguyen, Tenzin Choerab

# In order to use the software:
# 1. Set your absolute path for cylinderPath.txt, dataPath.txt, and pcdPath.txt to the appropriate files.
#    cylinderPath requires a CylinderData.csv
#    dataPath requires Data.csv
#    pcdPath requires .xyzrgb
# 2. Make sure you are running on python 3.8 for the most stable experience. Run the frontend.py
#    and you will be prompted with a gui. Load all the files and select the corresponding files, prune percentage, and
#    pruning method. Then click prune!
# 3. Once you click prune there will be a figure that appears. This is the top-down view of the raytracing, this tells you how
#    much sunglight each part of the tree is getting, pretty cool huh :)
# 4. You can close both figures out and a virtual environment will open up. This is the visualizer for the pruning algorithm! 
#    The red cylinders represent which parts of the tree will be deleted in order for an optimal prune.
#    Future implentation will be added to see the before and after of the prune in the same window.