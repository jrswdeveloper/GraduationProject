# GraduationProject -- Autonomous Car
 The car advancing on rough terrain.
 
**Description:** An image of the region will be taken with the depth camera in the environment and obstacle recognition and detection will be made by 3d image processing. Then, it will be sent to the Arduino's MCU via Bluetooth and the car will advance to the target without hitting the obstacles.

**Hardware:** 
              <pre>Depth Camera Intel Realsense R200 
                     Arduino Mega 
                     The Car Platform
                     Ground mat
                     Obstacles such as some 3d geometric shapes
                    Camera Stabilizer Support </pre>
             
             
**Software:**  <pre>PCL (Point Cloud Library) for cloud processing
               C++
               Visual Studio 2015
               Intel RealSense SDK 1.0
               Intel RealSense R200 Depth Camera Manager 2.1.27.2853
               Cloud Compare </pre>
               

This project consists of 3 stages which are 3D Image Processing, Path Planning and advancing the car with Arduino. But the last stage could not be realized due to the covid-19 pandemic, because my setup equipments are in the university. 

## Stage 1 : 3D Image Processing

The first thing we needed to do was create the dataset. For this reason, we bought the equipment necessary for the scene setup with my project friend Büşra Göl and realized the scene setup. With the Intel Real Sense R200 depth camera, we took the image of the scene and objects at a height of 2 meters. Then we saved it as a file with the .pcd(point cloud data) extension and created our data set.

![ds1](https://user-images.githubusercontent.com/62018540/85332391-8c6f4d80-b4e0-11ea-8618-b61725c66b2a.jpg)
Figure 1. Scene view from side

![ds3](https://user-images.githubusercontent.com/62018540/85332447-a577fe80-b4e0-11ea-84a1-59cf652b3187.JPG)
Figure 2. Scene view from the top-side

![ds4](https://user-images.githubusercontent.com/62018540/85332471-af016680-b4e0-11ea-8dae-bafc6406b0ed.JPG)
Figure 3. Scene view from top


After creating the dataset, we had to segment the plane and objects. We decided to start with implementation of Ground Plane Segmentation, and we used RANSAC(Random Sample Consensus) algorithm for that. 

![groundplane](https://user-images.githubusercontent.com/62018540/85332510-bf194600-b4e0-11ea-8e70-900594efe772.JPG)
Figure 4. Ground Plane without Objects  


Then we continued with object segmentation. And we used Euclidean Clustering algorithm for this.

![objs](https://user-images.githubusercontent.com/62018540/85332543-cd676200-b4e0-11ea-8fc9-733d4e4858b7.JPG)
Figure 5. Objects without Ground Plane  


Now that we have finished our segmentation processes, we can proceed the detection process. For object detection, we first thought of using bounding boxes. We did not need to find a 3-dimensional bounding box because the car would not be displaced in the 3rd dimension. For us, only the width and height measurements were sufficient. So we decided to find a 2-dimensional bounding box. First we found the projections of the objects to the ground plane, and then we thought of finding the bounding boxes of the projections of the objects. However, it was very difficult for us to draw a bounding box polygon with the PCL library and save it to the file, so after doing some research, we deduced that finding the convex hulls of the projections is also for the same purpose.
