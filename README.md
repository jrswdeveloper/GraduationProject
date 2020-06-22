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
               Intel RealSense R200 Depth Camera Manager 2.1.27.2853 </pre>
               

This project consists of 3 stages which are 3D Image Processing, Path Planning and advancing the car with Arduino. But the last stage could not be realized due to the covid-19 pandemic, because my setup equipments are in the university. 

# Stage 1 : 3D Image Processing

The first thing we needed to do was create the dataset. For this reason, we bought the equipment necessary for the scene setup with my project friend Büşra Göl and realized the scene setup. With the Intel Real Sense R200 depth camera, we took the image of the scene and objects at a height of 2 meters. Then we saved it as a file with the pcd extension and created our data set.




After creating the dataset, we had to segment the plane and objects. I and Büşra(my project friend) started with Ground Plane Segmentation, and we used RANSAC(Random Sample Consensus) algorithm for this.
