# RGBD-SLAM-tutorial

## Reference： 
- [[Tutorial Blog:一起来做RGBD-SLAM系列]. Gao Xiang](https://www.cnblogs.com/gaoxiang12/p/4652478.html)
- [[slambook-en]. Gao Xiang](https://github.com/gaoxiang12/slambook-en)
- [slambook code](https://github.com/gaoxiang12/slambook2)
- [TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)

## How to run the code:
- Preparation: download openCV 3.4+ and PCL, Eigen3, and other necessary libraries (I might forget some, but can be found in the includes).
- Step 1: download the dataset from TUM website above, then put it into `[$repo_folder]/data/rgb_png` and `[$repo_folder]/data/depth_png`, along with the `data/rgb.txt`, and `data/depth.txt` for dataset index. (The code about how to extract images and depths data are written in slamBase.h, you can change the path there)
- Step 2: build and compile the code
```bash
mkdir build
cd build
cmake ..
make
```
- Steo3: run the code in /bin folder:
`.visualOdometry`

## Code structure
`/src` has the main source files.
- `slamBase.cpp` has most self-defined functions to compute keyPoints and Descriptors, estimate Motion, join point clouds and so on.
- `visualOdometry.cpp` implemented the VO.
- `slamEnd.cpp` integrated the G2O library for optimization.
- `slam.cpp` improved based on `slamEnd` and `vo`, by adding keyFrames extraction and check_loop_closure. It is the complete version based on the above steps, yet still has some bugs.

## Bugs for future improvement
- core dump at some frames
- some wierd DTL algorithm bugs: 
```bash
DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. 
(expected: 'count >= 6'), 
where'count' is 5
```
- too many consecutive `not enough inliers`, maybe the distance between the frames are too far.
```bash
solving PnP
inliers: 0
inliers not enough, abandoning this frame
```


## Example of results

<div align=center><img src="https://github.com/WeihengXia0123/RGBD-SLAM-tutorial/blob/master/example/3.png" width="600" height="600"/>
  
<div align=center><img src="https://github.com/WeihengXia0123/RGBD-SLAM-tutorial/blob/master/example/1.png" width="600" height="600"/>
  
<div align=center><img src="https://github.com/WeihengXia0123/RGBD-SLAM-tutorial/blob/master/example/2.png" width="600" height="600"/>

(core dump, to be continued...)
