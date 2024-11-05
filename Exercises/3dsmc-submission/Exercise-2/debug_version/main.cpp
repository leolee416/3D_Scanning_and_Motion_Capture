#include <iostream>

#include "Eigen.h"
#include "ImplicitSurface.h"
#include "MarchingCubes.h"
#include "Volume.h"

int main() {
  std::string filenameIn = "../../Data/normalized.pcb";
  std::string filenameOut = "result.off";

  // implicit surface
  ImplicitSurface *surface;
  // TODO: you have to switch between these surface types
  int choice;
  std::cout << "Choose an implicit surface type:\n";
  std::cout << "1. Sphere\n";
  std::cout << "2. Torus\n";
  std::cout << "3. Hoppe\n";
  std::cout << "4. RBF\n";
  std::cout << "Enter choice (1-4): ";
  std::cin >> choice;

  switch (choice) {
  case 1:
    surface = new Sphere(Eigen::Vector3d(0.5, 0.5, 0.5), 0.4);
    break;
  case 2:
    surface = new Torus(Eigen::Vector3d(0.5, 0.5, 0.5), 0.4, 0.1);
    break;
  case 3:
    surface = new Hoppe(filenameIn);
    break;
  case 4:
    surface = new RBF(filenameIn);
    break;
  default:
    std::cout << "Invalid choice. Defaulting to Sphere.\n";
    surface = new Sphere(Eigen::Vector3d(0.5, 0.5, 0.5), 0.4);
    break;
  }

  // Ensure the surface was created
  if (!surface) {
    std::cout << "ERROR: Failed to create implicit surface!" << std::endl;
    return -1;
  }

  // surface = new Sphere(Eigen::Vector3d(0.5, 0.5, 0.5), 0.4);
  // surface = new Torus(Eigen::Vector3d(0.5, 0.5, 0.5), 0.4, 0.1);
  // surface = new Hoppe(filenameIn);
  // surface = new RBF(filenameIn);

  // fill volume with signed distance values
  unsigned int mc_res = 50; // resolution of the grid, for debugging you can
                            // reduce the resolution (-> faster)
  Volume vol(Vector3d(-0.1, -0.1, -0.1), Vector3d(1.1, 1.1, 1.1), mc_res,
             mc_res, mc_res, 1);
  for (unsigned int x = 0; x < vol.getDimX(); x++) {
    for (unsigned int y = 0; y < vol.getDimY(); y++) {
      for (unsigned int z = 0; z < vol.getDimZ(); z++) {
        Eigen::Vector3d p = vol.pos(x, y, z);
        double val = surface->Eval(p);
        vol.set(x, y, z, val);
      }
    }
  }

  // extract the zero iso-surface using marching cubes
  SimpleMesh mesh;
  for (unsigned int x = 0; x < vol.getDimX() - 1; x++) {
    std::cerr << "Marching Cubes on slice " << x << " of " << vol.getDimX()
              << std::endl;

    for (unsigned int y = 0; y < vol.getDimY() - 1; y++) {
      for (unsigned int z = 0; z < vol.getDimZ() - 1; z++) {
        ProcessVolumeCell(&vol, x, y, z, 0.00f, &mesh);
      }
    }
  }

  // write mesh to file
  if (!mesh.WriteMesh(filenameOut)) {
    std::cout << "ERROR: unable to write output file!" << std::endl;
    return -1;
  }

  delete surface;

  return 0;
}
