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
        Vertex vertex = {X,
                         Y,
                         Z,
                         static_cast<double>(color.rgbRed),
                         static_cast<double>(color.rgbGreen),
                         static_cast<double>(color.rgbBlue),
                         static_cast<double>(color.rgbReserved)};

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
