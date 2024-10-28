# Mesh Generator

This project is a mesh generator that reads RGBA and depth images, processes them into a set of 3D vertices, and generates a mesh in the OFF format. The mesh is saved as an OFF file, and vertex data can also be exported as a text file. The project uses the FreeImage library to handle image processing.

## Features
- Loads RGBA and depth images.
- Processes images to generate 3D vertices with color information.
- Generates triangle faces for mesh creation based on a specified edge threshold.
- Saves the resulting mesh in the OFF format.


## Dependencies
- **FreeImage**: A library used to load and process images in different formats (RGB and depth images in PNG format are used in this project).
- **C++ Standard Library**: For basic functionality like file I/O, vectors, and mathematical computations.

## Prerequisites

1. **FreeImage**: You must install the FreeImage library to handle image loading and processing.


## Build Instructions
To compile the project, make sure you have FreeImage installed and available on your system. Use the following command to build the program:

```sh
g++ main.cpp -o mesh_generator -lfreeimage
```

## Usage
1. Make sure you have the required RGBA and depth images (e.g., `rgba_image_3.png` and `depth_image_3.png`) in the same directory as the executable.
2. Run the executable:

   ```sh
   ./mesh_generator
   ```
3. The program will output:
   - `output.txt`: A text file containing vertex data (position and color).
   - `Mesh_3.off`: An OFF file representing the generated mesh.

## File Formats
- **OFF Format**: The mesh is saved in the COFF format, which includes color information for each vertex.
- **Text File**: The vertex data includes the 3D coordinates (X, Y, Z) and color components (R, G, B, A).

## Camera Parameters
The program uses the following default camera parameters to project depth into 3D space:
- Focal lengths: `fx = 525.0`, `fy = 525.0`
- Principal point: `cx = 319.5`, `cy = 239.5`
- Depth scaling factor: `factor = 5000.0`

These parameters can be modified in the `main.cpp` file as needed.

## Notes
- The mesh generator uses a simple approach to validate the vertices and ensure that the generated triangles meet a specified edge threshold to avoid overly large triangles.
- Make sure that the RGBA and depth images have the same resolution.

## License
This project is distributed under the MIT License.

