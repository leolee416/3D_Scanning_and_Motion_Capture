// ---------------------------------------------------------
// This code is inspired by Andy Zeng, Princeton University
// https://github.com/andyzeng/tsdf-fusion
// 
// created by Zhenjiang Li  zhenjiang1.li@tum.de
// created at 2025-01-20
// updated at 2025-01-27
// ---------------------------------------------------------
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include "utils.hpp"
#include "VirtualSensor.h"

// CUDA kernel function to integrate a TSDF voxel volume given depth images
__global__
void Integrate(float * cam_K, float * cam2base, float * depth_im,
              int im_height, int im_width, int voxel_grid_dim_x, int voxel_grid_dim_y, int voxel_grid_dim_z,
              float voxel_grid_origin_x, float voxel_grid_origin_y, float voxel_grid_origin_z, float voxel_size, float trunc_margin,
              float * voxel_grid_TSDF, float * voxel_grid_weight) {

  int pt_grid_z = blockIdx.x;
  int pt_grid_y = threadIdx.x;

  for (int pt_grid_x = 0; pt_grid_x < voxel_grid_dim_x; ++pt_grid_x) {

    // Convert voxel center from grid coordinates to base frame camera coordinates
    float pt_base_x = voxel_grid_origin_x + pt_grid_x * voxel_size;
    float pt_base_y = voxel_grid_origin_y + pt_grid_y * voxel_size;
    float pt_base_z = voxel_grid_origin_z + pt_grid_z * voxel_size;

    // Convert from base frame camera coordinates to current frame camera coordinates
    float tmp_pt[3] = {0};
    tmp_pt[0] = pt_base_x - cam2base[0 * 4 + 3];
    tmp_pt[1] = pt_base_y - cam2base[1 * 4 + 3];
    tmp_pt[2] = pt_base_z - cam2base[2 * 4 + 3];
    float pt_cam_x = cam2base[0 * 4 + 0] * tmp_pt[0] + cam2base[1 * 4 + 0] * tmp_pt[1] + cam2base[2 * 4 + 0] * tmp_pt[2];
    float pt_cam_y = cam2base[0 * 4 + 1] * tmp_pt[0] + cam2base[1 * 4 + 1] * tmp_pt[1] + cam2base[2 * 4 + 1] * tmp_pt[2];
    float pt_cam_z = cam2base[0 * 4 + 2] * tmp_pt[0] + cam2base[1 * 4 + 2] * tmp_pt[1] + cam2base[2 * 4 + 2] * tmp_pt[2];

    if (pt_cam_z <= 0)
      continue;

    int pt_pix_x = roundf(cam_K[0 * 3 + 0] * (pt_cam_x / pt_cam_z) + cam_K[0 * 3 + 2]);
    int pt_pix_y = roundf(cam_K[1 * 3 + 1] * (pt_cam_y / pt_cam_z) + cam_K[1 * 3 + 2]);
    if (pt_pix_x < 0 || pt_pix_x >= im_width || pt_pix_y < 0 || pt_pix_y >= im_height)
      continue;

    float depth_val = depth_im[pt_pix_y * im_width + pt_pix_x];

    if (depth_val <= 0 || depth_val > 6)
      continue;

    float diff = depth_val - pt_cam_z;

    if (diff <= -trunc_margin)
      continue;

    // Integrate
    int volume_idx = pt_grid_z * voxel_grid_dim_y * voxel_grid_dim_x + pt_grid_y * voxel_grid_dim_x + pt_grid_x;
    float dist = fmin(1.0f, diff / trunc_margin);
    float weight_old = voxel_grid_weight[volume_idx];
    float weight_new = weight_old + 1.0f;
    voxel_grid_weight[volume_idx] = weight_new;
    voxel_grid_TSDF[volume_idx] = (voxel_grid_TSDF[volume_idx] * weight_old + dist) / weight_new;
  }
}

// Loads a binary file with depth data and generates a TSDF voxel volume (5m x 5m x 5m at 1cm resolution)
// Volume is aligned with respect to the camera coordinates of the first frame (a.k.a. base frame)
int main(int argc, char * argv[]) {
  // Initalize VirtualSensor
  VirtualSensor sensor;
  if (!sensor.Init("../Data/rgbd_dataset_freiburg1_xyz/")) {
      std::cerr << "Failed to initialize VirtualSensor!" << std::endl;
      return -1;
  }

  int im_width = sensor.GetColorImageWidth();  // width of depth image
  int im_height = sensor.GetColorImageHeight(); // height of depth image

  // tsdf_voxel_parameters
  int voxel_grid_dim_x = 1000; // x_dimension of voxel grid
  int voxel_grid_dim_y = 1000; // y_dimension of voxel grid
  int voxel_grid_dim_z = 1000; // z_dimension of voxel grid

  // voxel_origin
  float voxel_grid_origin_x = -2.5f; // voxel grid origin x coordinate
  float voxel_grid_origin_y = -2.5f; // voxel grid origin y coordinate
  float voxel_grid_origin_z =  0.0f;  // voxel grid origin z coordinate

  // voxel_size and truncation distance
  float voxel_size = 0.005f;      // voxel size (edge length)(m)
  float trunc_margin = voxel_size * 5;     // truncation distance(m)

  // get intrinsics of depth camera
  Eigen::Matrix3f depth_intrinsics = sensor.GetDepthIntrinsics();
  std::cout << "depth_intrinsics:" << std::endl;
  std::cout << depth_intrinsics << std::endl;
  float cam_K[3 * 3];
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          cam_K[i * 3 + j] = depth_intrinsics(i, j);
      }
  }

  // get base frame
  Eigen::Matrix4f base2world = sensor.GetBaseFrame().inverse();
  std::cout << "base2world:" << std::endl;
  std::cout << base2world << std::endl;
  float base2world_mat[16];
  for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
          base2world_mat[i * 4 + j] = base2world(i, j);
      }
  }

  // camputer inverse of base2world
  float base2world_inv[16];
  invert_matrix(base2world_mat, base2world_inv);

  // camputer inverse of base2world
  float * voxel_grid_TSDF = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  float * voxel_grid_weight = new float[voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z];
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
      voxel_grid_TSDF[i] = 1.0f;
  memset(voxel_grid_weight, 0, sizeof(float) * voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z);

  // camputer inverse of base2world
  float * gpu_voxel_grid_TSDF;
  float * gpu_voxel_grid_weight;
  cudaMalloc(&gpu_voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float));
  cudaMalloc(&gpu_voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());
  cudaMemcpy(gpu_voxel_grid_TSDF, voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_voxel_grid_weight, voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyHostToDevice);
  checkCUDA(__LINE__, cudaGetLastError());
  float * gpu_cam_K;
  float * gpu_cam2base;
  float * gpu_depth_im;
  cudaMalloc(&gpu_cam_K, 3 * 3 * sizeof(float));
  cudaMemcpy(gpu_cam_K, cam_K, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMalloc(&gpu_cam2base, 4 * 4 * sizeof(float));
  cudaMalloc(&gpu_depth_im, im_height * im_width * sizeof(float));
  checkCUDA(__LINE__, cudaGetLastError());

  // camputer inverse of base2world
  int frame_cnt = 0;
  while (sensor.ProcessNextFrame()) {
    frame_cnt++;
    // Get depth image and camera pose
    float* depth_im = sensor.GetDepth();    

    // Get camera pose
    Eigen::Matrix4f cam2world = sensor.GetTrajectory().inverse();
    float cam2world_mat[16];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cam2world_mat[i * 4 + j] = cam2world(i, j);
        }
    }

    // Compute transformation matrices
    float cam2base[16];
    multiply_matrix(base2world_inv,cam2world_mat,  cam2base);
    // print_matrix(cam2base, 4, 4);
    std::cout << "cam2base (4x4):" << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << cam2base[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }

    // Eigen::Matrix4f cam2base_compare = sensor.GetCam2Base();
    // std::cout << "cam2base_compare:" << std::endl;
    // std::cout << cam2base_compare << std::endl;

    // // Inverse of cam2base
    // float cam2base_inv[16] = {0};
    // invert_matrix(cam2base, cam2base_inv);

    // Copy data to GPU
    // cudaMemcpy(gpu_cam2base, cam2base_inv, 4 * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_cam2base, cam2base, 4 * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_depth_im, depth_im, im_height * im_width * sizeof(float), cudaMemcpyHostToDevice);
    checkCUDA(__LINE__, cudaGetLastError());


    std::cout << "Fusing: " << std::endl;
    std::cout << "voxel_grid_dim_z: " << voxel_grid_dim_z << ", voxel_grid_dim_y: " << voxel_grid_dim_y << std::endl;

    // Launch CUDA kernel
    Integrate <<< voxel_grid_dim_z, voxel_grid_dim_y >>>(gpu_cam_K, gpu_cam2base, gpu_depth_im,
                                                        im_height, im_width, voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z,
                                                        voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z, voxel_size, trunc_margin,
                                                        gpu_voxel_grid_TSDF, gpu_voxel_grid_weight);
    // if(frame_cnt == 10) {
    //   break;      
    // }                              
  }
    // Load TSDF voxel grid from GPU to CPU memory
  cudaMemcpy(voxel_grid_TSDF, gpu_voxel_grid_TSDF, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(voxel_grid_weight, gpu_voxel_grid_weight, voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z * sizeof(float), cudaMemcpyDeviceToHost);
  checkCUDA(__LINE__, cudaGetLastError());

  // save TSDF voxel grid to disk as binary file
  SaveVoxelGrid2SurfacePointCloud("tsdf.ply", voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z,
                                  voxel_size, voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z,
                                  voxel_grid_TSDF, voxel_grid_weight, 0.2f, 0.0f);
  // Compute surface points from TSDF voxel grid and save to point cloud .ply file(ASCII)
  std::cout << "Saving surface point cloud (tsdf_ASCII.ply)..." << std::endl;
  SaveVoxelGrid2SurfacePointCloud_ASCII("tsdf_ASCII.ply", voxel_grid_dim_x, voxel_grid_dim_y, voxel_grid_dim_z, 
                                  voxel_size, voxel_grid_origin_x, voxel_grid_origin_y, voxel_grid_origin_z,
                                  voxel_grid_TSDF, voxel_grid_weight, 0.2f, 0.0f);


  // Save TSDF voxel grid and its parameters to disk as binary file (float array)
  std::cout << "Saving TSDF voxel grid values to disk (tsdf.bin)..." << std::endl;
  std::string voxel_grid_saveto_path = "tsdf.bin";
  std::ofstream outFile(voxel_grid_saveto_path, std::ios::binary | std::ios::out);
  float voxel_grid_dim_xf = (float) voxel_grid_dim_x;
  float voxel_grid_dim_yf = (float) voxel_grid_dim_y;
  float voxel_grid_dim_zf = (float) voxel_grid_dim_z;
  outFile.write((char*)&voxel_grid_dim_xf, sizeof(float));
  outFile.write((char*)&voxel_grid_dim_yf, sizeof(float));
  outFile.write((char*)&voxel_grid_dim_zf, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_x, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_y, sizeof(float));
  outFile.write((char*)&voxel_grid_origin_z, sizeof(float));
  outFile.write((char*)&voxel_size, sizeof(float));
  outFile.write((char*)&trunc_margin, sizeof(float));
  for (int i = 0; i < voxel_grid_dim_x * voxel_grid_dim_y * voxel_grid_dim_z; ++i)
    outFile.write((char*)&voxel_grid_TSDF[i], sizeof(float));
  outFile.close();

    return 0;
}
