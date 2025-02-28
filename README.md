# 3D_Scanning_-_Motion_Capture
This is the repository for (IN2354)3D Scanning & Motion Capture
# Environment setup
Please follow [here](https://github.com/marcbenedi/3dsmc-env?tab=readme-ov-file) from Marcbenedi.  
# Files setup
You can find corresponding files under:  
Original files：\3D_Scanning_and_Motion_Capture\Exercises\3dsmc-original_exercise_file  
Submission: : \3D_Scanning_and_Motion_Capture\Exercises\3dsmc-submission
# Exercise included
- Exercise 1: point cloud mesh
- Exercise 2: fine mesh
- Exercise 3: Optimization
- Exercise 4: Procruste alignment
- Exercise 5: ICP registration

# Project: Kinetic Fusion
The goal of this project is to re-implement the KinectFusion algorithm as described in the original paper KinectFusion: Real-Time Dense Surface Mapping and Tracking [Paper KinectFusion](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/kinectfusion-uist-comp.pdf).

## Rrerequisites
The coded can be compiled on Linux 24.04 with OpenCV and CUDA(CUDA Version: 12.2 using GPＵ:Nvidia gtx 1070)
- NVIDA GPU with [CUDA](https://developer.nvidia.com/cuda-downloads) support
- [OpenCV](http://opencv.org/) (tested with OpenCV 2.4.11)
For mesh generation:
- numpy
- skimage
- plyfile

## Build 
bash on Ubuntu:
```bash
chmod -x compile_local.sh
./compile_local.sh
```

## Run
TSDF voxel and weights generation:
```bash
./KineticFusion
```
mesh fusion:
```bash
python tsdf2mesh.py
```
Then you can check the Mesh by using tools like meshlab.


