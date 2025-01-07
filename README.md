# Point Clouds Registration

This tool applies colored ICP to register source pcds to target pcd and saves the transformation matrices in text file.

## Instructions for use:

Clone the Tools depository.

Run the program as follows (without the " "):
```
cd ~
git clone git@github.com:TAI-Automation/Tools.git
cd Tools/PointClouds_Registration
python3 pcd_reg.py "name of folder/target.pcd" "name(s) of folder(s)/source(s).pcd"
```
The program will save the corresponding transformation matrix text file in the same folder as source pcd.

Place the target and source pcds in the folder named cam1, cam2, cam3, cam4... according to its camera number within the catkin_ws. Within the folder cam1, cam2, cam3, cam4... create two folders, each named "merged" and "raw", and place the merged and raw pcds respectively. Example data are provided in this depository as well.

### Packages required

Python packages:
- os
- numpy
- open3d
- argparse

```
sudo pip install –default-timeout=100 open3d
pip install numpy --upgrade
```

Dependencies (optional):
- GCC >= 4.9 for Open3D Extensions, follow the commands below
  - sudo apt-get install software-properties-common
  - sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  - sudo apt-get update
  - sudo apt-get install gcc-4.9
  - sudo apt-get upgrade libstdc++6 
- If 4.9 does not work, replace it with 5.9 or 7.9
