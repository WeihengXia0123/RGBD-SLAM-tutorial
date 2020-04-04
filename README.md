# RGBD-SLAM-tutorial

## Reference： 
- Tutorial Blog [一起来做RGBD-SLAM系列]https://www.cnblogs.com/gaoxiang12/p/4652478.html
- TUM dataset [dataset]https://vision.in.tum.de/data/datasets/rgbd-dataset

## How to run the code:
- Preparation: download openCV 3.4+ and PCL, Eigen3, and other necessary libraries (I might forget some, but can be found in the includes).
- Step 1: download the dataset from TUM website above, then put it into [$repo_folder]/data/rgb_png and [$repo_folder]/data/depth_png, along with the data/rgb.txt, and data/depth.txt for dataset index. (The code about how to extract images and depths data are written in slamBase.h, you can change the path there)
- Step 2: build and compile the code
```bash
mkdir build
cd build
cmake ..
make
```
- Steo3: run the code in /bin folder:
`.visualOdometry`
