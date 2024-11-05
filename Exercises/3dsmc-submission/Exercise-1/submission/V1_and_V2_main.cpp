// ╔══════════════════════════════════════════╗
// ║                V1 Code                   ║
// ╚══════════════════════════════════════════╝

#include <FreeImage.h>
#include <cmath>   // for sqrt
#include <cstdint> // for uint16_t
#include <fstream>
#include <iostream>
#include <limits> // for std::numeric_limits
#include <vector>

// Constants
const float MINF = -std::numeric_limits<float>::infinity();
const float EDGE_THRESHOLD = 0.1f; // 1cm

// Vertex structure
struct Vertex {
  double x, y, z;    // 3D coordinates
  double r, g, b, a; // color information
};

// Face structure
struct Face {
  int v1, v2, v3; // Indices of the three vertices forming the triangle
};

// Load image helper function
FIBITMAP *loadImage(const std::string &filename) {
  FREE_IMAGE_FORMAT format = FreeImage_GetFileType(filename.c_str(), 0);
  if (format == FIF_UNKNOWN) {
    format = FreeImage_GetFIFFromFilename(filename.c_str());
  }
  if (format == FIF_UNKNOWN) {
    std::cerr << "Unknown file type: " << filename << std::endl;
    return nullptr;
  }
  FIBITMAP *image = FreeImage_Load(format, filename.c_str());
  if (!image) {
    std::cerr << "Unable to load image: " << filename << std::endl;
  }
  return image;
}

// Check if vertex is valid
bool isValidVertex(const Vertex &v) {
  return !(v.x == MINF || v.y == MINF || v.z == MINF);
}

// Check if the edges of the triangle are all below the edge threshold
bool isValidTriangle(const Vertex &v1, const Vertex &v2, const Vertex &v3,
                     float edge_threshold) {
  auto edgeLength = [](const Vertex &v1, const Vertex &v2) -> float {
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) +
                pow(v1.z - v2.z, 2));
  };

  return (edgeLength(v1, v2) < edge_threshold &&
          edgeLength(v1, v3) < edge_threshold &&
          edgeLength(v2, v3) < edge_threshold);
}

// Process images to merge RGB and Depth images into (X, Y, Z, R, G, B, A) format
std::vector<Vertex> processImages(FIBITMAP *rgbaImage, FIBITMAP *depthImage,
                                  double fx, double fy, double cx, double cy,
                                  double factor) {
  unsigned int width = FreeImage_GetWidth(rgbaImage);
  unsigned int height = FreeImage_GetHeight(rgbaImage);

  if (width != FreeImage_GetWidth(depthImage) ||
      height != FreeImage_GetHeight(depthImage)) {
    std::cerr << "Resolution of RGBA image is different from depth image."
              << std::endl;
    return {};
  }

  std::vector<Vertex> vertices;
  RGBQUAD color;

  for (unsigned int v = 0; v < height; ++v) {
    uint16_t *depthRow = (uint16_t *)FreeImage_GetScanLine(depthImage, v);
    for (unsigned int u = 0; u < width; ++u) {
      if (FreeImage_GetPixelColor(rgbaImage, u, v, &color)) {
        uint16_t depth_value = depthRow[u];
        double Z = static_cast<double>(depth_value) / factor; // Compute depth

        // Compute (X, Y, Z) coordinates in space
        double X = (static_cast<double>(u) - cx) * Z / fx;
        double Y = (static_cast<double>(v) - cy) * Z / fy;

        // Create and store vertex
        Vertex vertex = {-X,
                         -Y,
                         -Z,
                         static_cast<double>(color.rgbRed),
                         static_cast<double>(color.rgbGreen),
                         static_cast<double>(color.rgbBlue),
                        //  static_cast<double>(color.rgbReserved)
                        255};

        // Mark as invalid if depth is 0
        if (depth_value == 0) {
          vertex.x = MINF;
          vertex.y = MINF;
          vertex.z = MINF;
        }
        vertices.push_back(vertex);
      }
    }
  }

  return vertices;
}

// Generate triangle faces
std::vector<Face> generateFaces(const std::vector<Vertex> &vertices,
                                unsigned int width, unsigned int height) {
  std::vector<Face> faces;

  for (unsigned int v = 0; v < height - 1; ++v) {
    for (unsigned int u = 0; u < width - 1; ++u) {
      int idx = v * width + u;

      // Get four neighboring vertices
      const Vertex &v1 = vertices[idx];
      const Vertex &v2 = vertices[idx + 1];
      const Vertex &v3 = vertices[idx + width];
      const Vertex &v4 = vertices[idx + width + 1];

      // Check if triangle is valid and if edge lengths are below threshold
      if (isValidVertex(v1) && isValidVertex(v2) && isValidVertex(v3) &&
          isValidTriangle(v1, v2, v3, EDGE_THRESHOLD)) {
        faces.push_back({idx, idx + 1, idx + width});
      }

      if (isValidVertex(v2) && isValidVertex(v3) && isValidVertex(v4) &&
          isValidTriangle(v2, v3, v4, EDGE_THRESHOLD)) {
        faces.push_back({idx + 1, idx + width + 1, idx + width});
      }
    }
  }

  return faces;
}

// Save OFF file
void saveOFF(const std::string &filename, const std::vector<Vertex> &vertices,
             const std::vector<Face> &faces) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return;
  }

  // Write OFF file header
  file << "COFF\n";
  file << vertices.size() << " " << faces.size() << " 0\n";

  // Save vertices
  for (const auto &vertex : vertices) {
    if (isValidVertex(vertex)) {
      file << vertex.x << " " << vertex.y << " " << vertex.z << " "
           << vertex.r / 255.0 << " " << vertex.g / 255.0 << " "
           << vertex.b / 255.0 << " " << vertex.a / 255.0 << "\n";
    } else {
      file << "0.0 0.0 0.0 0.0 0.0 0.0 0.0\n"; // Invalid vertex
    }
  }

  // Save triangle faces
  for (const auto &face : faces) {
    file << "3 " << face.v1 << " " << face.v2 << " " << face.v3 << "\n";
  }

  file.close();
}

// Save vertex data as a text file
void savePixelData(const std::string &filename,
                   const std::vector<Vertex> &vertices) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  for (const auto &vertex : vertices) {
    file << vertex.x << " " << vertex.y << " " << vertex.z << " " << vertex.r
         << " " << vertex.g << " " << vertex.b << " " << vertex.a << "\n";
  }

  file.close();
}

int main() {
  // Camera parameters
  double fx = 525.0;
  double fy = 525.0;
  double cx = 319.5;
  double cy = 239.5;
  double factor = 5000.0;

  FreeImage_Initialise();

  // Load RGBA and depth images
  FIBITMAP *rgbaImage = loadImage("rgba_image_3.png");
  FIBITMAP *depthImage = loadImage("depth_image_3.png");

  if (rgbaImage && depthImage) {
    // Process images and generate vertices
    std::vector<Vertex> vertices =
        processImages(rgbaImage, depthImage, fx, fy, cx, cy, factor);

    // Save vertex data
    savePixelData("output.txt", vertices);

    // Generate valid triangle faces
    unsigned int width = FreeImage_GetWidth(rgbaImage);
    unsigned int height = FreeImage_GetHeight(rgbaImage);
    std::vector<Face> faces = generateFaces(vertices, width, height);

    // Save as OFF file
    saveOFF("Mesh_3.off", vertices, faces);

    FreeImage_Unload(rgbaImage);
    FreeImage_Unload(depthImage);

    std::cout << "OFF Mesh3 saved to Mesh3.off" << std::endl;
  }

  FreeImage_DeInitialise();

  return 0;
}


// ╔══════════════════════════════════════════╗
// ║                V2 Code                   ║
// ╚══════════════════════════════════════════╝
#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

// Data structures for 3D mesh representation
struct Vertex {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector4f position;    // Position (x, y, z, w) where w should be 1.0
    Vector4uc color;      // Color (r, g, b, a) as unsigned chars

    // Default constructor and other constructors can be added here if needed
};

struct Face {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Vertices and their corresponding indices in the vertex array
    Vertex v1, v2, v3;    // Actual vertex data
    int v1_id, v2_id, v3_id;  // Vertex indices

    Face(Vertex vertex1, int id1, Vertex vertex2, int id2, Vertex vertex3, int id3)
        : v1(vertex1), v1_id(id1)
        , v2(vertex2), v2_id(id2)
        , v3(vertex3), v3_id(id3) {}
};

/**
 * Checks if a vertex has valid position coordinates
 * @param v The vertex to check
 * @return true if the vertex position is valid (no MINF values)
 */
bool isValidVertex(const Vertex& v) {
    return !(v.position[0] == MINF ||
             v.position[1] == MINF ||
             v.position[2] == MINF ||
             v.position[3] == MINF);
}

/**
 * Validates if three vertices form a valid triangle based on edge length
 * @param v1, v2, v3 The positions of the three vertices
 * @param edge_threshold Maximum allowed edge length
 * @return true if all edges are shorter than the threshold
 */
bool isValidTriangle(const Vector4f& v1, const Vector4f& v2, const Vector4f& v3, float edge_threshold) {
    return ((v1 - v2).norm() < edge_threshold) &&
           ((v1 - v3).norm() < edge_threshold) &&
           ((v2 - v3).norm() < edge_threshold);
}

/**
 * Writes a 3D mesh to a COFF file
 * @param vertices Pointer to vertex array
 * @param width Width of the vertex grid
 * @param height Height of the vertex grid
 * @param filename Output file path
 * @return true if writing was successful
 */
bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename) {
    const float edgeThreshold = 0.01f;  // Maximum edge length (1cm)
    const unsigned int nVertices = width * height;
    std::vector<Face*> faces;

    // Generate faces by triangulating the vertex grid
    for (unsigned int i = 0; i < height - 1; i++) {
        for (unsigned int j = 0; j < width - 1; j++) {
            // Calculate vertex indices for the current grid cell
            int v1_id = width * i + j;
            int v2_id = width * i + (j + 1);
            int v3_id = width * (i + 1) + j;
            int v4_id = width * (i + 1) + (j + 1);

            // Get corresponding vertices
            const Vertex& v1 = vertices[v1_id];
            const Vertex& v2 = vertices[v2_id];
            const Vertex& v3 = vertices[v3_id];
            const Vertex& v4 = vertices[v4_id];

            // Create upper triangle if valid
            if (isValidTriangle(v1.position, v2.position, v3.position, edgeThreshold)) {
                faces.push_back(new Face(v1, v1_id, v2, v2_id, v3, v3_id));
            }

            // Create lower triangle if valid
            if (isValidTriangle(v2.position, v3.position, v4.position, edgeThreshold)) {
                faces.push_back(new Face(v2, v2_id, v3, v3_id, v4, v4_id));
            }
        }
    }

    // Write to COFF file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        // Clean up allocated faces before returning
        for (Face* face : faces) {
            delete face;
        }
        return false;
    }

    // Write header
    outFile << "COFF\n";
    outFile << nVertices << " " << faces.size() << " 0\n";

    // Write vertices
    for (unsigned int i = 0; i < nVertices; i++) {
        const Vertex& vertex = vertices[i];
        // Write position (use (0,0,0) for invalid vertices)
        if (isValidVertex(vertex)) {
            outFile << vertex.position[0] << " "
                   << vertex.position[1] << " "
                   << vertex.position[2];
        } else {
            outFile << "0.0 0.0 0.0";
        }
        // Write color
        outFile << " " << static_cast<int>(vertex.color[0])
                << " " << static_cast<int>(vertex.color[1])
                << " " << static_cast<int>(vertex.color[2])
                << " " << static_cast<int>(vertex.color[3])
                << "\n";
    }

    // Write faces
    for (const Face* face : faces) {
        outFile << "3 " << face->v1_id << " "
                       << face->v2_id << " "
                       << face->v3_id << "\n";
    }

    // Clean up and close file
    for (Face* face : faces) {
        delete face;
    }
    outFile.close();

    return true;
}

int main()
{
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		// Back-project the image pixels to the world space
		for (int h = 0; h < sensor.GetDepthImageHeight(); ++h) {
			for (int w = 0; w < sensor.GetDepthImageWidth(); ++w) {
				int index = sensor.GetDepthImageWidth() * h + w;  // Calculate index
				float depth = depthMap[index];

				if (depth != MINF) {
					// Depth information exists for the vertex; apply back projection

					// Compute pinhole coordinates
					float x = depth * w; // Scale screen space x
					float y = depth * h; // Scale screen space y
					Vector4f pinhole_coord(x, y, depth, 1.0f); // Use constructor directly set pin_coord

					// Create inverse intrinsic matrix only once per pixel
					Matrix4f depthIntrinsicsInv = MatrixXf::Identity(4, 4);
					depthIntrinsicsInv(0, 0) = 1.0f / fX;
					depthIntrinsicsInv(1, 1) = 1.0f / fY;
					depthIntrinsicsInv(0, 2) = -cX / fX;
					depthIntrinsicsInv(1, 2) = -cY / fY;

					// Transform to world coordinates
					Vector4f real_coord = trajectoryInv * depthExtrinsicsInv * depthIntrinsicsInv * pinhole_coord;

					// Set homogeneous coordinate
					real_coord[3] = 1.0f;

					// Assign vertex position
					vertices[index].position = real_coord;

					// Assign color from color map
					vertices[index].color = Vector4uc(
						colorMap[4 * index],
						colorMap[4 * index + 1],
						colorMap[4 * index + 2],
						colorMap[4 * index + 3]
					);
				} else {
					// Depth value is invalid; set default vertex position and color
					vertices[index].color = Vector4uc(0, 0, 0, 0);
					vertices[index].position = Vector4f(MINF, MINF, MINF, MINF);
				}
			}
		}
		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}