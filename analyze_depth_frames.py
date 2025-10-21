import os
import cv2
import numpy as np
import open3d as o3d
import math
import csv

# === Configuration ===
depth_folder = os.path.expanduser("~/10x/depth_images")
output_table = os.path.expanduser("~/10x/depth_analysis.csv")

# Camera intrinsics (approximate)
fx, fy = 525.0, 525.0
cx, cy = 319.5, 239.5

# === Helper functions ===
def depth_to_pointcloud(depth_image):
    """Convert depth image to 3D point cloud (Nx3)"""
    h, w = depth_image.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_image.astype(np.float32)
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    return points

def fit_plane_open3d(points):
    """Use RANSAC plane fitting on Open3D PointCloud"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.01, ransac_n=3, num_iterations=1000
    )
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    return normal, inliers

def angle_between_normals(n1, n2):
    """Compute angle (in degrees) between two vectors"""
    n1, n2 = n1 / np.linalg.norm(n1), n2 / np.linalg.norm(n2)
    return np.degrees(np.arccos(np.clip(np.dot(n1, n2), -1.0, 1.0)))

# === Main Loop ===
results = []
camera_normal = np.array([0, 0, 1])  # Camera looks along +Z

for filename in sorted(os.listdir(depth_folder)):
    if not filename.endswith(".png"):
        continue

    filepath = os.path.join(depth_folder, filename)
    depth = cv2.imread(filepath, cv2.IMREAD_UNCHANGED).astype(np.float32)

    # Remove invalid depths
    depth[depth <= 0] = np.nan
    valid_mask = ~np.isnan(depth)
    if np.count_nonzero(valid_mask) < 1000:
        continue

    # Convert to point cloud
    points = depth_to_pointcloud(depth)
    points = points[np.isfinite(points).all(axis=1)]

    # Fit plane to point cloud
    normal, inliers = fit_plane_open3d(points)

    # Compute normal angle
    angle = angle_between_normals(normal, camera_normal)

    # Estimate visible area (approx. from inlier count)
    inlier_points = points[inliers]
    pixel_size = np.mean(np.diff(np.unique(points[:, 0]))) * np.mean(np.diff(np.unique(points[:, 1])))
    area_est = len(inliers) * (pixel_size ** 2)

    results.append([filename, round(angle, 2), round(area_est, 4)])

    print(f"✅ {filename}: Angle = {angle:.2f}°, Area ≈ {area_est:.4f} m²")

# === Save results to CSV ===
with open(output_table, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Image", "Normal Angle (deg)", "Visible Area (m^2)"])
    writer.writerows(results)

print(f"\nSaved results to {output_table}")
