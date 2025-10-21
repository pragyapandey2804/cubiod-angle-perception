import os
import cv2
import numpy as np
import open3d as o3d
import math
import pandas as pd
from sklearn.decomposition import PCA

# === Paths ===
depth_folder = os.path.expanduser("~/10x/depth_images")
output_axis_file = os.path.expanduser("~/10x/rotation_axis.txt")

# Camera intrinsics (same as before)
fx, fy = 525.0, 525.0
cx, cy = 319.5, 239.5

# === Helper functions ===
def depth_to_pointcloud(depth_image):
    """Convert depth image to 3D point cloud"""
    h, w = depth_image.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_image.astype(np.float32)
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    return points

def fit_plane(points):
    """Fit plane using RANSAC and return normal"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.01, ransac_n=3, num_iterations=1000
    )
    a, b, c, d = plane_model
    return np.array([a, b, c])

# === Collect all normals ===
normals = []
for filename in sorted(os.listdir(depth_folder)):
    if not filename.endswith(".png"):
        continue

    filepath = os.path.join(depth_folder, filename)
    depth = cv2.imread(filepath, cv2.IMREAD_UNCHANGED).astype(np.float32)
    depth[depth <= 0] = np.nan
    points = depth_to_pointcloud(depth)
    points = points[np.isfinite(points).all(axis=1)]

    normal = fit_plane(points)
    normals.append(normal / np.linalg.norm(normal))  # normalize
    print(f"Extracted normal from {filename}: {normal}")

normals = np.array(normals)

# === PCA to find rotation axis ===
pca = PCA(n_components=3)
pca.fit(normals)

# The axis of rotation is the direction with least variance (last component)
rotation_axis = pca.components_[-1]
rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

# === Save to file ===
np.savetxt(output_axis_file, rotation_axis, fmt="%.6f")
print(f"\nAxis of rotation vector (camera frame): {rotation_axis}")
print(f"Saved to {output_axis_file}")
