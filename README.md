# Point Cloud Detection Application

This is an application that detects objects from point cloud data by applying ground segmentation techniques and Euclidean clustering. 
<br /> It also visualizes the original point cloud data along with bounding boxes for detected objects.

## Building

This application requires [CMake](https://cmake.org/) 3.2.1 (or higher) and [PCL](https://pointclouds.org/) 1.11.1 (or higher).  
  
Building on Debian/Ubuntu:

```bash
git clone https://github.com/joshuakurien/point-cloud-bounding-boxes
cd point-cloud-bounding-boxes
mkdir build && cd build
cmake ..
make
```

## Usage

To run the application run the following command in the build directory:
```bash
./main
```

## Output
### Original Data Visualization
![Original Data](sample/original.png?raw=true "Original Data")

### Post-Segmentation Visualization
![Post-Segmentation](sample/post_segmentation.png?raw=true "Post-Segmentation")

## References
Ground segmentation algorithm based on the following paper:
```bash
@article{article,
  author = {Chu, Phuong and Cho, Seoungjae and Sim, S. and 
  Kwak, K. and Cho, Kyungeun},
  year = {2017},
  month = {01},
  pages = {491-499},
  title = {A Fast Ground Segmentation Method for 3D Point Cloud},
  volume = {13},
  journal = {Journal of Information Processing Systems},
  doi = {10.3745/JIPS.02.0061}
}
```
