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