import numpy as np
from skimage import measure
from plyfile import PlyData, PlyElement

def tsdf_to_mesh_ply(tsdf_bin='tsdf.bin', ply_out='mesh.ply'):
    """
    读取 tsdf.bin 文件，重建等值面并输出到 mesh.ply 文件。
    """
    
    # 1) 读取二进制文件，并解析头信息
    with open(tsdf_bin, 'rb') as fid:
        tsdf_header = np.fromfile(fid, dtype=np.float32, count=8)
        # 前 3 个为体素网格维度 (xDim, yDim, zDim)
        voxel_grid_dim = tsdf_header[0:3].astype(np.int32)
        # 中 3 个为体素网格在世界坐标系下的原点 (originX, originY, originZ)
        voxel_grid_origin = tsdf_header[3:6]
        # 接下来是体素大小
        voxel_size = tsdf_header[6]
        # 最后是截断距离（可按需使用）
        trunc_margin = tsdf_header[7]
        
        # 读取 TSDF 数据
        num_voxels = voxel_grid_dim[0] * voxel_grid_dim[1] * voxel_grid_dim[2]
        tsdf_vals = np.fromfile(fid, dtype=np.float32, count=num_voxels)
    
    # 2) 将 TSDF 数组变形为 3D
    #   注意这里对应的是 (dimX, dimY, dimZ)，
    #   也就是形状 [xDim, yDim, zDim]
    tsdf_vals = tsdf_vals.reshape(voxel_grid_dim[0],
                                  voxel_grid_dim[1],
                                  voxel_grid_dim[2])
    
    # 3) 提取等值面（0 等值面）
    #   skimage.measure.marching_cubes(…) 返回 (verts, faces, normals, values)
    #   - verts.shape = (N, 3)，每一行是 (z, y, x) 顺序
    #   - faces.shape = (M, 3)，为顶点的索引（0-based）
    verts, faces, normals, values = measure.marching_cubes(tsdf_vals, level=0)
    
    # 4) 生成颜色（与 MATLAB 中的 [175,198,233] 相同的淡蓝色）
    #   这里 color 数组大小为 (N, 3)
    color = np.array([175, 198, 233], dtype=np.uint8)
    color = np.tile(color, (verts.shape[0], 1))
    
    # 5) 从体素坐标系转换到“世界”或相机坐标系（与 MATLAB 对应）
    #
    #   MATLAB 中的做法:
    #       meshPoints(1,:) = origin(1) + points(2,:)*voxelSize;
    #       meshPoints(2,:) = origin(2) + points(1,:)*voxelSize;
    #       meshPoints(3,:) = origin(3) + points(3,:)*voxelSize;
    #
    #   而 skimage 的 marching_cubes 对 [xDim, yDim, zDim] 的数据返回的
    #   verts 通常是 [z, y, x] 顺序。
    #
    #   因此，如果想和 MATLAB 结果对应：
    #       newX = originX + oldY * voxel_size
    #       newY = originY + oldX * voxel_size
    #       newZ = originZ + oldZ * voxel_size
    #
    #   如果您确认 marching_cubes 返回的 verts[:, 0] 就是 z，
    #   verts[:, 1] 就是 y， verts[:, 2] 就是 x。
    #   则可以这样写:
    
    mesh_x = voxel_grid_origin[0] + verts[:, 1] * voxel_size
    mesh_y = voxel_grid_origin[1] + verts[:, 0] * voxel_size
    mesh_z = voxel_grid_origin[2] + verts[:, 2] * voxel_size
    mesh_points = np.column_stack((mesh_x, mesh_y, mesh_z))
    
    # 6) 由于 MATLAB 中 faces 做了一个顺序反转 (faces = faces([3 2 1],:)),
    #    相当于把每个三角形顶点索引的顺序从 (v1, v2, v3) 翻转为 (v3, v2, v1)。
    #    在 Python 中可以用 faces[:, ::-1] 或者指定列的方式：
    faces = faces[:, [2, 1, 0]]
    
    # (注：scikit-image 输出的 faces 索引本身就是 0-based，MATLAB 需要减 1)
    # 这里不需要 faces - 1。
    
    # 7) 写出二进制 PLY 文件
    #    下面使用 plyfile 库；也可以手动按字节格式写，或者使用其它库如 open3d、trimesh 等。
    
    # 首先构造顶点的 structured array，包含 x, y, z, red, green, blue
    vertex_data = np.empty(len(mesh_points),
                           dtype=[
                               ("x", "f4"), 
                               ("y", "f4"), 
                               ("z", "f4"),
                               ("red", "u1"),
                               ("green", "u1"),
                               ("blue", "u1")
                           ])
    vertex_data["x"] = mesh_points[:, 0]
    vertex_data["y"] = mesh_points[:, 1]
    vertex_data["z"] = mesh_points[:, 2]
    vertex_data["red"]   = color[:, 0]
    vertex_data["green"] = color[:, 1]
    vertex_data["blue"]  = color[:, 2]
    
    # 然后构造面信息
    face_data = np.empty(len(faces), dtype=[("vertex_indices", "i4", (3,))])
    face_data["vertex_indices"] = faces
    
    # 打包为 PlyElement
    ply_el_verts = PlyElement.describe(vertex_data, 'vertex')
    ply_el_faces = PlyElement.describe(face_data, 'face')
    
    ply_data = PlyData([ply_el_verts, ply_el_faces], text=False)
    
    # 指定以 binary_little_endian 格式写文件
    # plyfile 可以使用 encoding='binary_little_endian' 或通过 text=False, byte_order='<' 来指定。
    ply_data.write(ply_out)    
    print(f"Done! Mesh written to: {ply_out}")

if __name__ == "__main__":
    tsdf_to_mesh_ply('tsdf.bin', 'mesh.ply')
