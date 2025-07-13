# BenchRNR: Dataset for Benchmarking Repetitive and Non-repetitive Scanning LiDAR for Infrastructure-based Vehicle Localization



## 1. Introduction
<div align="center" style="display: flex; justify-content: center; gap: 20px;">
<img src="img/scanning.jpg" alt="scanning pattern" width="45%" />
<img src="img/RNR.jpg" alt="RNR" width="45%" />
</div>

BenchRNR is a point cloud dataset for Infrastructure-based Vehicle Localization with both repetitive  and non-repetitive scanning LiDAR, in an effort to benchmark the performance of different LiDAR scanning patterns. It also provides the code for four typical vehicle pose estimation methods: 

+ point cloud segmentation + oriented bounding box (Seg+obb)
+ point cloud segmentation + convex-hull-based vehicle pose estimation method (Seg+obb)
+ point cloud segmentation + vehicle point cloud template registration (Register-Loc)
+ point cloud object detector PV-RCNN (PV-RCNN)


Our code has been released.




### Related Video
Our accompanying video is now available on [[YouTube](https://www.youtube.com/watch?v=5vLmBrVXfXU)]


## 2. Dataset Download
<div align="center">
<img src="img/datasample.jpg" alt="datasample" width="75%" />
</div>
Our data is released in the form of rosbags. There are eight rosbags in total, containing 5,428 frames of LiDAR point clouds. The sensor configuration including:

+ one repetitive scanning LiDAR (OT-128, 10Hz)
+ two non-reptitive scanning LiDAT (Livox Avia, 10Hz)
+ two camera (USB camera, 10Hz)

 Download the data from [[BenchRNR Datasets](https://pan.sjtu.edu.cn/web/share/b48de37f470fb8a3d1f886c41495ee78)]

### Quick Start

See [quick_start](quick_start.md) for detailed installation and usage guide in this document.

## News
+ 2025-07-11: First commit on codes, allowing for both multi-line LiDAR & non-repetitive scanning LiDAR with auto-run and testing scripts. 👩‍💻
+ 2025-07-01: Our paper has been accepted by ITSC2025 🎉
+ 2025-05-08: Release Dataset at [[BenchRNR Datasets](https://pan.sjtu.edu.cn/web/share/b48de37f470fb8a3d1f886c41495ee78)]. 📂


## Acknowledgements
The realization of convex-hull-based vehicle pose estimation method is built on [CH-MOA-ROS](https://github.com/HMX2013/CH-MOA-ROS)


PV-RCNN is trained by [OpenPCDet](https://github.com/open-mmlab/OpenPCDet.git) using roadside point cloud data from [DAIR-V2X-I](https://github.com/AIR-THU/DAIR-V2X). 



