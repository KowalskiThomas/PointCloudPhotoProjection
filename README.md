# PointCloudPhotoProjection

**An implementation of a point cloud onto image projection using C++ / OpenCV**

This was created in the context of a university group project dealing with LiDAR-tampering related security issues in autonomous cars. As a first step, we implemented different components of the car using accessible technology (Veloydne LiDAR, four Android phones streaming images, [Robot Operating System](https://www.ros.org/) as the system centre.

If you're not interested in the ROS version, simply checkout branch `raw` and build the project:

```
git checkout raw
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
./render /home/my_name/data
```

Have fun!
