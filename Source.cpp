#include <windows.h>
#include <wchar.h>
#include "pxcsensemanager.h"
#include <opencv2/opencv.hpp>
#include <iostream> 
#include "util_render.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
#define MAX_FRAMES 150
int wmain(int argc, WCHAR* argv[]) {
	// initialize the util render
	UtilRender *renderDepth = new UtilRender(L"DEPTH STREAM");
	// create the PXCSenseManager
	PXCSenseManager *psm = 0;
	psm = PXCSenseManager::CreateInstance();
	if (!psm) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		return 1;
	}
	// select the color stream of size 640x480 and depth stream of size 320x240
	//psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480);
	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 480, 360);
	// initialize the PXCSenseManager
	if (psm->Init() != PXC_STATUS_NO_ERROR) return 2;
	PXCPoint3DF32 *pos3D = new PXCPoint3DF32[480 * 360];
	for (int i = 0; i < MAX_FRAMES; i++) {
		// This function blocks until all streams are ready (depth and color)
		// if false streams will be unaligned
		if (psm->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;
		// retrieve all available image samples
		PXCCapture::Sample *sample = psm->QuerySample();
		PXCImage  *depthMap = sample->depth;
		renderDepth->RenderFrame(sample->depth);
		PXCImage::ImageData depthImage;
		depthMap->AcquireAccess(PXCImage::ACCESS_READ, &depthImage);
		PXCImage::ImageInfo imgInfo = depthMap->QueryInfo();
		int depth_stride = depthImage.pitches[0] / sizeof(pxcU16);
		PXCPoint3DF32 * pos3d = new PXCPoint3DF32[480 * 360];
		PXCCaptureManager* captureManager = psm->QueryCaptureManager();
		PXCCapture::Device* device = captureManager->QueryDevice();
		PXCProjection* projection = device->CreateProjection();
		pxcU16 *dpixels = (pxcU16*)depthImage.planes[0];
		unsigned int dpitch = depthImage.pitches[0] / sizeof(pxcU16);
		pxcStatus sts = psm->AcquireFrame(true);
		sts = projection->QueryVertices(depthMap, &pos3D[0]);
		if (sts < pxcStatus::PXC_STATUS_NO_ERROR) {
			wprintf_s(L"Projection was unsuccessful! \n");
			psm->Close();
		}
		
		if (!renderDepth->RenderFrame(depthMap)) break;
		// release or unlock the current frame to fetch the next frame
		psm->ReleaseFrame();
	}

	for (int k = 0; k < 480 * 360; k++) {

		if (pos3D[k].x != 0) {
			cout << " xx is: " << pos3D[k].x << endl;
		}

		if (pos3D[k].y != 0) {

			cout << " yy is: " << pos3D[k].y << endl;
		}
		if (pos3D[k].z != 0) {
			cout << " zz is: " << pos3D[k].z << endl;
		}
		//Sleep(100);
	}
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
	// Fill in the cloud data
	cloud.width = 480;
	cloud.height = 360;
	cloud.is_dense = true;
	cloud.points.resize(cloud.width * cloud.height);
	cloud.sensor_orientation_ = Eigen::Quaternionf(0, 1, 0, 0);

	int writeCount = 0;
	for (std::size_t i = 0; i < cloud.points.size(); ++i)
	{
	cloud.points[i].x = pos3D[i].x;
	cloud.points[i].y = pos3D[i].y;
	cloud.points[i].z = pos3D[i].z;
	cloud.points[i].normal[0] = 1.f;
	cloud.points[i].normal[1] = 0.f;
	cloud.points[i].normal[2] = 0.0f;
	cloud.points[i].r = 255;
	cloud.points[i].g = 125;
	cloud.points[i].b = 0;
	}
	writeCount++;
	//char filename[100];
	//sprintf(filename, "C:/Users/Hp/Desktop/ornek.xyz", writeCount);
	//pcl::io::savePCDFileASCII(filename, cloud);
	pcl::io::savePCDFile("D:/Hp/Desktop/newsahne_pcd.pcd",cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to newsahne.pcd." << std::endl;
	for (std::size_t i = 0; i < cloud.points.size(); ++i)
	std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	
	//delete renderColor;
	delete renderDepth;
	// close the last opened streams and release any session and processing module instances
	psm->Release();
	return 0;
}
	
