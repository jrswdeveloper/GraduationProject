# GraduationProject
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
               

**Note:**  *The project is still continue, not completed yet. The uploaded codes include the getting real world coordinates and saving 3d data(Source.cpp), Ground plane segmentation with RANSAC algorithm, object segmentation with Euclidean Clustering algorithm and Object detection with Bounding Box(Source2.cpp).*

