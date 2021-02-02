# Lidar_Obstacle_Detection
<img src="https://github.com/CuteJui/Lidar_Obstacle_Detection/blob/master/readme_resource/demo.gif"/>
This is the first project of Sensor Fusion Nanodegree of Udacity. First of all, I segmented the input point cloud data into road plane cloud and obstacle cloud by using 3D RANSAC algorithm. Next, I built a KD-Tree for obstacle cloud and then clustered them by Euclidean Clustering. Finally, I placed bounding box to enclose these vehicles. The structure of the environment is built by the main instructor, Aaron Brown.

## Usage
Clone the Lidar Obstacle Detection package.
```
git clone https://github.com/CuteJui/Lidar_Obstacle_Detection.git
```
Go to the Lidar Obstacle Detection directory
```
cd /home/user/Lidar_Obstacle_Detection
```
Create a new directory
```
mkdir build
```
Go into the build directory
```
cd build
```
Run cmake pointing to the CMakeList.txt in the root
```
cmake ..
```
Run make
```
make
```
Run the executable
```
./enviroment
```
