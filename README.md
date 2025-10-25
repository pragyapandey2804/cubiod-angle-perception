# Cuboid Angle perception

**Author:** Pragya Pandey  
**Operating Environment:** Ubuntu 24.04 (WSL) with ROS 2 Jazzy  
**Topic:** *Estimation of Normal Angle, Visible Area, and Axis of Rotation of a Rotating Cuboid using Depth Data*

---

## Abstract

This project aims to estimate the **normal angle**, **visible area**, and **axis of rotation** of a cuboidal object rotating around its central axis using depth sensor data. Depth frames recorded in a ROS 2 bag were processed to extract geometric features of the visible face across multiple timestamps.  

The approach involves:

- Extracting depth images
- Fitting planes using RANSAC to determine orientation and visible area of each face
- Applying PCA on normal vectors to determine the rotation axis  

This method demonstrates the integration of depth perception, 3D point cloud processing, and geometric reasoning in robotics.

---

## 1. Introduction

This report presents the approach and implementation used to estimate the **normal angle**, **visible area**, and **axis of rotation** of a 3D cuboidal box rotating about its central axis.  

Data was obtained from a **depth imaging sensor** that captured depth frames of the cuboid at multiple timestamps. Using these depth images, a sequence of computational steps was implemented to extract geometric information from each frame and determine the rotation behavior of the cuboid.

---

## 2. Problem Understanding

A depth sensor provides distance information in meters for every pixel in an image. As the cuboid rotates, different faces become visible to the camera at different angles.  

The task required estimating the following from depth data:

1. **Normal Angle:** The angle between the camera’s viewing direction and the normal vector of the cuboid’s visible face in each frame.  
2. **Visible Area:** The surface area of the largest visible face of the cuboid as seen by the camera (in square meters).  
3. **Axis of Rotation:** The fixed direction in space about which the cuboid rotates — represented as a 3D vector in the camera’s coordinate frame.

---

## 3. Methodology and Algorithm

### Step 1: Extraction of Depth Images from ROS 2 Bag

The provided dataset was in **ROS 2 bag format** (`depth.db3` and `metadata.yaml`) storing recorded sensor messages.  

Using the ROS 2 Python API (`rosbag2_py`) and `cv_bridge`, a script **`extract_depth_frames.py`** was created to:

- Read depth images from the ROS 2 bag topic `/depth`
- Convert ROS image messages to OpenCV images
- Save each frame as `.png` files in the `depth_images/` folder

This conversion is necessary because libraries like Open3D and OpenCV cannot directly read ROS 2 message formats.

---

### Step 2: Computation of Normal Angle and Visible Area

Each depth image is converted into a **3D point cloud** using the pinhole camera model:

$$
x = (u - c_x)\frac{z}{f_x}, \quad
y = (v - c_y)\frac{z}{f_y}, \quad
z = z
$$

Where:  

- \( (u, v) \) are pixel coordinates  
- \( (f_x, f_y) \) are focal lengths  
- \( (c_x, c_y) \) are camera principal point offsets  

A **RANSAC plane fitting** algorithm (Open3D) detects the planar surface corresponding to the visible cuboid face.  

From the fitted plane equation:

$$
ax + by + cz + d = 0
$$

The **plane normal vector** is:

$$
\vec{n} = [a, b, c]
$$

#### Normal Angle Calculation

The normal angle between the plane and the camera’s Z-axis is:

$$
\theta = \cos^{-1}\left( \frac{\vec{n} \cdot [0, 0, 1]}{|\vec{n}|} \right)
$$

This gives the **tilt** of the visible face relative to the camera.

#### Visible Area Estimation

The visible area is estimated by counting inlier points belonging to the fitted plane and multiplying by the pixel area:

$$
A_{visible} = N_{inliers} \times (\text{pixel size})^2
$$

Results for each frame — image name, normal angle, and visible area — are saved in **`depth_analysis.csv`**.

---

### Step 3: Axis of Rotation Estimation

As the cuboid rotates, the normal vectors of its visible face change. To find the **rotation axis**, the script **`rotation_axis.py`**:

1. Collects all normal vectors across frames  
2. Applies **Principal Component Analysis (PCA)**  
3. The PCA component with **least variance** corresponds to the **axis of rotation**  

The resulting axis vector is normalized and saved in **`rotation_axis.txt`**, representing the rotation direction in the camera’s coordinate frame.

---

## 4. Results

| Image Name    | Normal Angle (°) | Visible Area (m²) |
| ------------- | ---------------- | ---------------- |
| depth_000.png | 65.49            | 0.0284           |
| depth_001.png | 15.45            | 0.0504           |
| depth_002.png | 34.53            | 0.0142           |
| depth_003.png | 50.75            | 0.0112           |
| depth_004.png | 30.25            | 0.0287           |
| depth_005.png | 0.00             | 0.0569           |
| depth_006.png | 50.22            | 0.0338           |

**Axis of rotation vector** example from `rotation_axis.txt`: [0.005, 0.999, -0.010]


This represents the rotation direction in the 3D camera coordinate system.

---

## 5. Conclusion

The implemented pipeline successfully computes the geometric and motion parameters of a rotating cuboid from depth sensor data.  

- **Plane fitting** identifies the visible surface in each frame  
- Orientation and area are calculated  
- PCA across frames estimates the **axis of rotation**  

This demonstrates how **depth perception** and **3D geometric reasoning** can infer real-world motion, showcasing practical **computer vision** and **spatial analysis** in robotics.

---

## 6. File and Folder Structure
```
Perception_Assignment_PragyaPandey/
│
├── extract_depth_frames.py        # Script to extract depth images from ROS 2 bag
├── analyze_depth_frames.py        # Script to estimate normal angle & visible area
├── rotation_axis.py               # Script to estimate axis of rotation
│
├── depth_analysis.csv             # Table of image number, normal angle, visible area
├── rotation_axis.txt              # Axis of rotation vector (camera frame)
├── Approach_and_Algorithm.pdf     # Detailed algorithm explanation
│
├── depth_images/                  # Folder containing extracted depth image frames (.png)
└── rosbags/depth/                 # Folder containing original ROS 2 bag files (depth.db3 + metadata.yaml)
```


Each script is commented and can be executed sequentially to reproduce results. All numerical results and explanations are included for verification.

---

### End of README

