#include "KinectPlaneCalibration.h"
#include <vector>

//unsigned short depth[MAX_DEPTH];
//char *depth_data;
bool roiSelected = false;
const char* PLANE_PARAMETERS = "Parameters_";
const char* PLANE_NORMALS = "Normals_";

bool pointSelected = false;

ofstream KinectPlaneCalibration::outDebug(filePaths::DEBUG_FILEPATH, ios::out);

KinectPlaneCalibration::KinectPlaneCalibration(void)
{
}

KinectPlaneCalibration::~KinectPlaneCalibration(void)
{
}

//Listeners

void getPixelColor_callBack(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		XnRGB24Pixel* rgbImg = (XnRGB24Pixel*)param;
		int r,g,b;			
		r = rgbImg[y*XN_VGA_X_RES+x].nRed;
		g = rgbImg[y*XN_VGA_X_RES+x].nGreen;
		b= rgbImg[y*XN_VGA_X_RES+x].nBlue;

		cout << "RGB: " << r << ", " << g << ", " << b << endl;

	}
}


void selectROI_callBack(int event, int x, int y, int flags, void* param)
{
	Plane* plane = (Plane*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		XnPoint3D p;
		p.X = x;
		p.Y = y;
		if (plane->getInitPoint().X == -1)
		{
			plane->setInitPoint(p);
		}
		else
		{
			plane->setEndPoint(p);
			bool t = plane->isROISelected();
			roiSelected = true;
		}
	}
}

void selectPoint_callBack(int event, int x, int y, int flags, void* param)
{
	XnPoint3D* p = (XnPoint3D*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		p->X = x;
		p->Y = y;
		pointSelected = true;
	}
}


void selectPoints_callBack(int event, int x, int y, int flags, void* param)
{
	XnPoint3D* roi = (XnPoint3D*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
		if (roi[0].X == -1)
		{
			roi[0].X = x;
			roi[0].Y = y;
		}
		else if (roi[1].X == -1)
		{
			roi[1].X = x;
			roi[1].Y = y;
		}
		else
		{
			roi[2].X = x;
			roi[2].Y = y;
			roiSelected = true;
		}
}

//Publics

/*
Make a color filter over rgbMap and store all the pixels of the filter in the list 'points'
*/
void KinectPlaneCalibration::colorFilter(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel* depthMap)
{
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
			if ((rgbMap->nRed < 140 && rgbMap->nRed > 50) &&
				(rgbMap->nGreen < 55 && rgbMap->nGreen > 5 && rgbMap->nGreen < rgbMap->nBlue && (rgbMap->nGreen+rgbMap->nGreen/2)<rgbMap->nRed) &&
				(rgbMap->nBlue < 60 && rgbMap->nBlue > 5) && (rgbMap->nBlue+rgbMap->nBlue/2)<rgbMap->nRed && (rgbMap->nBlue > rgbMap->nRed/3) &&
				depthMap[y * XN_VGA_X_RES + x] != 0)

			{
				XnPoint3D p;
				p.X = x;
				p.Y = y;
				p.Z = depthMap[y * XN_VGA_X_RES + x];
				points->push_back(p);
			}
			rgbMap++;
		}
	}
}

void KinectPlaneCalibration::colorFilter_RGB(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap,const vector<vector<vector<double>>>* hist)
{
	int step = 256/8;
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
			if (depthMap[y * XN_VGA_X_RES + x] != 0)
			{
				int r = rgbMap->nRed;
				int g = rgbMap->nGreen;
				int b = rgbMap->nBlue;
				int rBin = r/step;
				int gBin = g/step;
				int bBin = b/step;
				double prob = (*hist)[rBin][gBin][bBin];
				if (prob > .01)
				{
					XnPoint3D p;
					p.X = x;
					p.Y = y;
					p.Z = depthMap[y * XN_VGA_X_RES + x];
					points->push_back(p);
				}
			}
			rgbMap++;
		}
	}
}


void KinectPlaneCalibration::colorFilter_HSV(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap, const vector<vector<double>>* hist)
{

	IplImage *rgbImg = cvCreateImage(cvSize(640,480), 8, 3);
	IplImage * hsvImg = cvCreateImage(cvGetSize(rgbImg), rgbImg->depth, 3);
	Utils::fillImageData(rgbImg, rgbMap, depthMap);
	cvCvtColor(rgbImg, hsvImg, CV_BGR2HSV);

	int step = 256/8;
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		const uchar* ptr = (const uchar*)(hsvImg->imageData + y*hsvImg->widthStep);
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
				int h = *ptr++;
				int s = *ptr++;
				int v = *ptr++;
				int hBin = h/step;
				int sBin = s/step;
				double prob = (*hist)[hBin][sBin];
				if (prob > COLORFILTER_THRESHOLD)
				{
					XnPoint3D p;
					p.X = x;
					p.Y = y;
					p.Z = depthMap[y * XN_VGA_X_RES + x];
					points->push_back(p);
				}
		}
	}
}



/*
Take depth and RGB images of one camera, shows them and stores in memory the maps (RGB and depth).
*/
void KinectPlaneCalibration::getCameraMaps(CameraProperties* cam, vector<XnDepthPixel*>* depthMap_lst, vector <XnRGB24Pixel*>* rgbMap_lst, int id)
{
	IplImage *depthImg;//, *rgbImg;

	depthImg = cvCreateImageHeader(cvSize(640,480), 8, 3);
//	rgbImg = cvCreateImage(cvSize(640,480), 8, 3);

	cam->getContext()->StartGeneratingAll();
	cam->getContext()->WaitAndUpdateAll();	
	(*depthMap_lst)[id] = (XnDepthPixel*) cam->getDepthNode()->GetDepthMap();		
	(*rgbMap_lst)[id] =  (XnRGB24Pixel*) cam->getImageNode()->GetRGB24ImageMap();	
	cam->getContext()->StopGeneratingAll();


	//create depth image
	unsigned short depth[MAX_DEPTH];
	char *depth_data = (char*) malloc(640*480*3);
	raw2depth(depth);
	depth2rgb((*depthMap_lst)[id], depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	//create rgb image
//	Utils::fillImageData(rgbImg, (*rgbMap_lst)[id], (*depthMap_lst)[id]);

	//Create window name
	char idCam[10];
	char nameWind_D[20];
//	char nameWind_RGB[20];
	itoa(cam->getCamId(), idCam, 10);
//	strcpy(nameWind_RGB, "RGB ");
//	strcat(nameWind_RGB, idCam);
	strcpy(nameWind_D, "Depth ");
	strcat(nameWind_D, idCam);

//	cvShowImage(nameWind_RGB, rgbImg);
	cvShowImage(nameWind_D, depthImg);
	cvWaitKey(2000);
	//free memory
//	cvDestroyWindow(nameWind_RGB);
	cvDestroyWindow(nameWind_D);
//	cvReleaseImage(&rgbImg);
	cvReleaseImageHeader(&depthImg);
	free(depth_data);

}

/*
Take depth and RGB images of one camera, shows them and stores in memory the maps (RGB and depth).
*/
void KinectPlaneCalibration::getCameraMaps_Manual(CameraProperties* cam, vector<XnDepthPixel*>* depthMap_lst, vector <XnRGB24Pixel*>* rgbMap_lst, int nPlanes)
{
	IplImage* depthImg = cvCreateImageHeader(cvSize(640,480), 8, 3);
	IplImage* rgbImg = cvCreateImage(cvSize(640,480), 8,3);
	int total = XN_VGA_X_RES*XN_VGA_Y_RES;
	cam->getContext()->StartGeneratingAll();
	
	unsigned short depth[MAX_DEPTH];
	char *depth_data;

	//Create window name
	char idCam[10];
	char nameWind_D[50];
	char nameFileDepth[130];
	char nameFileRGB[130];
	itoa(cam->getCamId(), idCam, 10);
	strcpy(nameWind_D, "Depth ");
	strcat(nameWind_D, idCam);

	char strIdPlane[10];
	if (cam->getCamId() == 1)
	{
		strcpy(nameFileDepth, filePaths::CAM1_CALIBRATION_DATA);
		strcpy(nameFileRGB, filePaths::CAM1_CALIBRATION_DATA);
	}
	else
	{
		strcpy(nameFileDepth, filePaths::CAM2_CALIBRATION_DATA);
		strcpy(nameFileRGB, filePaths::CAM2_CALIBRATION_DATA);
	}


	strcat(nameFileDepth, "Depth ");
	strcat(nameFileDepth, idCam);
	strcat(nameFileRGB, "RGB ");
	strcat(nameFileRGB, idCam);
	ofstream depthStream, rgbStream;

	char depthTmp[130];
	char rgbTmp[130];

	for (int id = 0; id < nPlanes; id++)
	{
		(*depthMap_lst)[id] = new XnDepthPixel[XN_VGA_X_RES*XN_VGA_Y_RES];
		(*rgbMap_lst)[id] = new XnRGB24Pixel[XN_VGA_X_RES*XN_VGA_Y_RES];
		cam->getContext()->WaitAndUpdateAll();	
		const XnDepthPixel* dM = (XnDepthPixel*) cam->getDepthNode()->GetDepthMap();		
		const XnRGB24Pixel* rgbM =  (XnRGB24Pixel*) cam->getImageNode()->GetRGB24ImageMap();	

		//Create the name of the file
		strcpy(depthTmp, nameFileDepth);
		strcpy(rgbTmp, nameFileRGB);
		itoa(id, strIdPlane, 10);
		strcat(depthTmp,strIdPlane);
		strcat(rgbTmp, strIdPlane);
		strcat(depthTmp, ".txt");
		strcat(rgbTmp, ".txt");
		depthStream.open(depthTmp);
		rgbStream.open(rgbTmp);

		
		//Copy depth and rgb map
		for (int i = 0; i < total; i++)
		{
			(*depthMap_lst)[id][i] = dM[i];
			depthStream << dM[i] << " ";
			(*rgbMap_lst)[id][i] = rgbM[i];
			rgbStream << (int)rgbM[i].nRed << " " << (int)rgbM[i].nGreen << " " << (int)rgbM[i].nBlue << " ";
		}
		depthStream.close();
		rgbStream.close();

		//create rgb imaage
//		Utils::fillImageData(rgbImg, rgbM, dM);

//		cvSaveImage(rgbTmp, rgbImg);


		//create depth image
		depth_data = (char*) malloc(640*480*3);
		raw2depth(depth);
		depth2rgb((*depthMap_lst)[id], depth, depth_data);
		cvSetData(depthImg, depth_data, 640*3);

	
		cvShowImage(nameWind_D, depthImg);
		cvWaitKey(0);
		cvDestroyWindow(nameWind_D);
		free(depth_data);
		Sleep(2000);

//		Beep(500, 650);
	}

	cam->getContext()->StopGeneratingAll();


	//free memory
//	cvDestroyWindow(nameWind_RGB);
	cvDestroyWindow(nameWind_D);
//	cvReleaseImage(&rgbImg);
	cvReleaseImageHeader(&depthImg);
//	free(depth_data);

}

/*
Create a labelled image with connected components. The component with biggest area is the point list result
*/
void KinectPlaneCalibration::labellingNoiseFilter(list<XnPoint3D>* pointsSrc, Plane* plane, const XnDepthPixel* depthMap)
{
	//initializes the mask and binary image
	IplImage* img = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 1);
	IplImage* tmp = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 1);
	IplImage* labelImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_LABEL, 1);
	CvBlobs blobs;
	
	Utils::initImage(img , 0);
	//updates the binaryImage with the points in 'points2Check'
	list<XnPoint3D>::iterator it;
	for (it=pointsSrc->begin(); it!=pointsSrc->end(); ++it)
	{
		XnPoint3D p = *it;
		((uchar*)(img->imageData + (int)p.Y*img->widthStep))[(int)p.X] = 255;
	}
	
	cvThreshold(img, img, 100,255, CV_THRESH_BINARY);
	
	unsigned int result = cvLabel(img, labelImg, blobs);

	cvFilterByLabel(blobs,cvGreaterBlob(blobs));

	unsigned int area =  blobs.begin()->second->area;
	cout << "Area: " << area << endl;

	if (area > AREA_THRESHOLD)
	{
		XnPoint3D initPoint, endPoint; 

		int minX = blobs.begin()->second->minx;
		int maxX = blobs.begin()->second->maxx;
		int minY = blobs.begin()->second->miny;
		int maxY = blobs.begin()->second->maxy;
		int offX = abs(maxX-minX)*20/100;
		int offY = abs(maxY-minY)*20/100;
			
		initPoint.X = minX + offX;
		initPoint.Y = minY + offY;
		endPoint.X = maxX - offX;
		endPoint.Y = maxY - offY;

		plane->setInitPoint(initPoint);
		plane->setEndPoint(endPoint);
	}



}


/*
Perform a color filter in the rgb image and use it as a seed for the growing plane detection method.
*/
void KinectPlaneCalibration::capturePlane(Plane* plane, const XnRGB24Pixel* rgbMap, const XnDepthPixel* depthMap, void* param, const int maskSz, CameraProperties* cam, int idPlane, int filterType)
{
	list<XnPoint3D> planePointLst; // seed
	int rgbPlanes[] = {100,100,100};  //fill color
	IplImage *depthImg = cvCreateImageHeader(cvSize(640,480), 8, 3);

	ofstream planeNormal;
	Utils::createGeneralOutStream(&planeNormal, PLANE_NORMALS, idPlane, cam->getCamId());

	//Create window name
	char idCam[10];
	char nameWind_D[20];
	itoa(cam->getCamId(), idCam, 10);
	strcpy(nameWind_D, "Depth cam");
	strcat(nameWind_D, idCam);
	cvNamedWindow(nameWind_D, 1);

	//create depth image
	unsigned short depth[MAX_DEPTH];
	char *depth_data = (char*) malloc(640*480*3);
	raw2depth(depth);
	depth2rgb(depthMap, depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	if (filterType == RGB_FILTER) // rgb filter
	{
		const vector<vector<vector<double>>>* histRGB = (const vector<vector<vector<double>>>*)param;
		colorFilter_RGB(&planePointLst, rgbMap, depthMap, histRGB); //histogram technique
	}
	else
	{
		const vector<vector<double>>* histHSV = (const vector<vector<double>>*)param;
		colorFilter_HSV(&planePointLst, rgbMap, depthMap, histHSV); //hsv filter
	}


	labellingNoiseFilter(&planePointLst, plane, depthMap);
	planePointLst.clear();

	if (!plane->isROISelected())
	{
		cout << "Not found proper seed for plane " << idPlane << " (camera " << cam->getCamId() << "). Select it manually." << endl;
		getROISeed(nameWind_D, plane, depthImg);
		cvDestroyWindow(nameWind_D);
	}
	
	
	generateListPoint(plane, &planePointLst, depthMap, maskSz);

	DynamicPlane planeCreator(&planePointLst, depthImg->width, depthImg->height, maskSz, rgbPlanes, idPlane, cam);
	planeCreator.makePlaneGrow(nameWind_D, depthImg, depthMap);
	plane->setParameters(planeCreator.getPlaneParameters());
	CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
	unitNormal(unitNorm, plane->getParameters());
	plane->setNormal(unitNorm);

//DEBUG BEGIN
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 0, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 1, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 2, 0) << endl;
//DEBUG END	

	plane->setDistance(Utils::calculatePlaneDistance(plane->getParameters(), plane->getNormal()));
	cout << "Plane " << idPlane << " captured in camera " << cam->getCamId() << endl;
	cvWaitKey(2000);
	//free memory
	cvDestroyWindow(nameWind_D);
	cvReleaseImageHeader(&depthImg);
	free(depth_data);	
}

/*
Perform a color filter in the rgb image and use it as a seed for the growing plane detection method.
*/
void KinectPlaneCalibration::capturePlaneManual(Plane* plane, const XnDepthPixel* depthMap,  const XnRGB24Pixel* rgbMap, const int maskSz, CameraProperties* cam, int idPlane)
{
	list<XnPoint3D> planePointLst; // seed
	int rgbPlanes[] = {100,100,100};  //fill color
	IplImage *depthImg = cvCreateImageHeader(cvSize(640,480), 8, 3);
	IplImage *rgbImg = cvCreateImage(cvSize(640,480), 8, 3);

	ofstream planeNormal;
	Utils::createGeneralOutStream(&planeNormal, PLANE_NORMALS, idPlane, cam->getCamId());

	//Create window name
	char idCam[10];
	char nameWind_D[20];
	char nameWind_RGB[20];
	itoa(cam->getCamId(), idCam, 10);
	strcpy(nameWind_RGB, "RGB ");
	strcat(nameWind_RGB, idCam);
	strcpy(nameWind_D, "Depth ");
	strcat(nameWind_D, idCam);

	//create rgb image
	Utils::fillImageData(rgbImg, rgbMap, depthMap);
	//create depth image
	unsigned short depth[MAX_DEPTH];
	char *depth_data = (char*) malloc(640*480*3);
	raw2depth(depth);
	depth2rgb(depthMap, depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	cvNamedWindow(nameWind_RGB,1);
	getROISeed(nameWind_RGB, plane, rgbImg);

	cvRectangle(rgbImg, cvPoint(plane->getInitPoint().X, plane->getInitPoint().Y), cvPoint(plane->getEndPoint().X, plane->getEndPoint().Y),cvScalar(0,0,255)); 
	cvShowImage(nameWind_RGB, rgbImg);
	cvWaitKey(0);

	//////////////////
	generateListPoint(plane, &planePointLst, depthMap, maskSz);
	DynamicPlane planeCreator(&planePointLst, depthImg->width, depthImg->height, maskSz, rgbPlanes, idPlane, cam);
	planeCreator.makePlaneGrow(nameWind_RGB, rgbImg, depthMap);
	plane->setParameters(planeCreator.getPlaneParameters());
	CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
	unitNormal(unitNorm, plane->getParameters());
	plane->setNormal(unitNorm);

//DEBUG BEGIN
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 0, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 1, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane->getNormal(), float, 2, 0) << endl;
//DEBUG END	
	
	plane->setDistance(Utils::calculatePlaneDistance(plane->getParameters(), plane->getNormal()));
	cout << "Plane " << idPlane << " captured in camera " << cam->getCamId() << endl;
	cvWaitKey(5000);
/////////////////


	//free memory
	cvDestroyWindow(nameWind_D);
	cvReleaseImageHeader(&depthImg);
	cvDestroyWindow(nameWind_RGB);
	cvReleaseImageHeader(&rgbImg);
}

/*
Make a color filter to set the seed.
*/
void KinectPlaneCalibration::getPlanesCameras(vector<Plane*> planes1, vector<Plane*> planes2, CameraProperties* cam1, CameraProperties* cam2, const int nPlanes, const int maskSz , void* hist, int typeFilter)
{

	//First take the imaes of all the planes
	vector<XnDepthPixel*> depthMap1_lst(nPlanes);
	vector<XnDepthPixel*> depthMap2_lst(nPlanes);
	vector <XnRGB24Pixel*> rgbMap1_lst(nPlanes);
	vector <XnRGB24Pixel*> rgbMap2_lst(nPlanes);

	Beep(500, 550);
	Sleep(5000);
	//use threads for taking the images
	//for (int i = 0; i < nPlanes; i++)
	//{
		boost::thread thr(&KinectPlaneCalibration::getCameraMaps_Manual, this, cam2, &depthMap2_lst, &rgbMap2_lst, nPlanes);
		getCameraMaps_Manual(cam1, &depthMap1_lst, &rgbMap1_lst, nPlanes);
		thr.join();
		Beep(300, 250);
//		Sleep(2000);
//	}

	Beep(500, 650);
	for (int i = 0; i < nPlanes; i++)
	{
		boost::thread thr2(&KinectPlaneCalibration::capturePlane, this, planes2[i], rgbMap2_lst[i], depthMap2_lst[i], (void*)hist, maskSz, cam2, i, typeFilter);
		capturePlane(planes1[i], rgbMap1_lst[i], depthMap1_lst[i], (void*)hist, maskSz, cam1, i, typeFilter);
		thr2.join();
		cout << "Plane " << i << " captured in both cameras" << endl;
	}


	cout << "Plane captured finished" << endl;

}

/*
Uses a manual process for setting the seed.
*/
void KinectPlaneCalibration::getPlanesCameras_Manual(vector<Plane*> planes1, vector<Plane*> planes2, CameraProperties* cam1, CameraProperties* cam2, const int nPlanes, const int maskSz)
{
	//First take the imaes of all the planes
	vector<XnDepthPixel*> depthMap1_lst(nPlanes);
	vector<XnDepthPixel*> depthMap2_lst(nPlanes);
	vector <XnRGB24Pixel*> rgbMap1_lst(nPlanes);
	vector <XnRGB24Pixel*> rgbMap2_lst(nPlanes);

	Beep(500, 550);
	Sleep(5000);
	//use threads for taking the images
//	for (int i = 0; i < nPlanes; i++)
//	{
		boost::thread thr(&KinectPlaneCalibration::getCameraMaps_Manual, this, cam2, &depthMap2_lst, &rgbMap2_lst, nPlanes);
		getCameraMaps_Manual(cam1, &depthMap1_lst, &rgbMap1_lst, nPlanes);
		thr.join();
		Beep(300, 250);
//		cout << "Image " << i << " stored in memory." << endl;
		//Sleep(2000);
//	}

	Beep(500, 650);
	for (int i = 0; i < nPlanes; i++)
	{
		boost::thread thr2(&KinectPlaneCalibration::capturePlaneManual, this, planes2[i], depthMap2_lst[i], rgbMap2_lst[i], maskSz, cam2, i);
		capturePlaneManual(planes1[i], depthMap1_lst[i], rgbMap1_lst[i], maskSz, cam1, i);
		thr2.join();
		cout << "Plane " << i << " captured in both cameras" << endl;
	}

	cout << "Plane captured finished" << endl;

}

/*
Calculates the parameters of the plane defined by a set of points. Fits a set of 3D points into a plane 
using the least square technique.
*/
void KinectPlaneCalibration::getNormalVectPlane_LS(vector<Plane*> planes, CameraProperties* cam, const int nPlanes, const int maskSz)
{
	//Depth image representation and listener for choosing a set of points
	IplImage* depthImage = cvCreateImageHeader(cvSize(640,480),8,3);
	unsigned short depth[MAX_DEPTH];
	char *depth_data = (char*) malloc(640*480*3);
	raw2depth(depth);
	
	cam->getContext()->StartGeneratingAll();
	cam->getContext()->WaitAndUpdateAll();	
	const XnDepthPixel* pDepthMap = cam->getDepthNode()->GetDepthMap();
	cam->getContext()->StopGeneratingAll();
	depth2rgb(pDepthMap, depth, depth_data);
	cvSetData(depthImage,depth_data, 640*3);	


	//Calcualte normal vector using a roi
//	getNormalwithROI(&planes, depthImage, cam, normalVec, pDepthMap, nPlanes);
	getNormalGrowingRegion(depthImage, cam, planes, pDepthMap, nPlanes, maskSz);
	

	// free memory
	cvReleaseImageHeader(&depthImage);
	free(depth_data);

}

/*
Calculates the translation matrix using as a point the mean of all the special (closest) points
*/
void KinectPlaneCalibration::calculateTranslation_mean(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation, const int nPlane, CvMat* translation)
{
	//using the best correspondences
//	vector<int> positions = getPlanePositions(cam1Planes, cam2Planes, rotation);
//	XnPoint3D* cam1Point = getMeanPoint(cam1Planes, positions);
//	XnPoint3D* cam2Point = getMeanPoint(cam2Planes, positions);

	//using all the correspondeces
	XnPoint3D* cam1Point = getMeanPoint(cam1Planes);
	XnPoint3D* cam2Point = getMeanPoint(cam2Planes);

	KinectPlaneCalibration::outDebug << "Cam1 point (translation): " << cam1Point->X << ", " << cam1Point->Y << ", " << cam1Point->Z <<endl;
	KinectPlaneCalibration::outDebug << "Cam2 point (translation): " << cam2Point->X << ", " << cam2Point->Y << ", " << cam2Point->Z <<endl;
	getTranslationFrom2Points(cam1Point, cam2Point, rotation, translation);
}

/*
Calculates the translation matrix using as a point correspondece the closest points to all planes.
*/
void KinectPlaneCalibration::calculateTranslation3(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation, const int nPlane, CvMat* translation)
{
	//Get the 3D points
	XnPoint3D* cam1Point = getClosestPoint(cam1Planes); //Nx = D
	XnPoint3D* cam2Point = getClosestPoint(cam2Planes); //Nx = D

//	XnPoint3D cam1Point, cam2Point;
	//mean of all closest points
//	cam1Point.X = -744.7;  cam1Point.Y = -137.5; cam1Point.Z = 3155.2;
//	cam2Point.X = 709.1;  cam2Point.Y = -278.6; cam2Point.Z = 3078.8;
	
	//Same points as point calibration method uses
//	cam1Point.X = -517.833; cam1Point.Y = 106.333; cam1Point.Z = 2240.67;
//	cam2Point.X = 896.667; cam2Point.Y = 166.5; cam2Point.Z = 2178.33;

	KinectPlaneCalibration::outDebug << "Cam1 point (translation): " << cam1Point->X << ", " << cam1Point->Y << ", " << cam1Point->Z <<endl;
	KinectPlaneCalibration::outDebug << "Cam2 point (translation): " << cam2Point->X << ", " << cam2Point->Y << ", " << cam2Point->Z <<endl;
	getTranslationFrom2Points(cam1Point, cam2Point, rotation, translation);

}

/*
Calculates the translation matrix using one correspondence point selected by the user
*/
void KinectPlaneCalibration::calculateTranslation2(CameraProperties*cam1, CameraProperties* cam2, const CvMat* rotation, CvMat* translation)
{
	//Get a point correspondence from both images
	XnPoint3D* cam1Point = getPointFromImage(cam1);
	XnPoint3D* cam2Point = getPointFromImage(cam2);

	//Same point used in point calibration method
//	XnPoint3D cam1Point, cam2Point;
//	cam1Point.X = -516; cam1Point.Y = 114.667; cam1Point.Z = 2036.83;
//	cam2Point.X = 532.667; cam2Point.Y = 162.833; cam2Point.Z = 2006;

	
	KinectPlaneCalibration::outDebug << "Cam1 point (translation): " << cam1Point->X << ", " << cam1Point->Y << ", " << cam1Point->Z <<endl;
	KinectPlaneCalibration::outDebug << "Cam2 point (translation): " << cam2Point->X << ", " << cam2Point->Y << ", " << cam2Point->Z <<endl;


	getTranslationFrom2Points(cam1Point, cam2Point, rotation, translation);
}

/*
Calculate translation using a pair known point corresopndences
*/
void KinectPlaneCalibration::calculateTranslation_hardCoded(XnPoint3D* p1, XnPoint3D* p2, CvMat* rotation, CvMat* translation)
{
	getTranslationFrom2Points(p1, p2, rotation, translation);
}

/*
Calculate the translation from cam1 to cam2
*/
void KinectPlaneCalibration::calculateTranslation(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation1_2, const int nPlane, CvMat* translation)
{
	//rotate all the normals of camera 1

	//Rotate normals.
	vector<CvMat*> cam1NormalRotated(nPlane);
	for (int i = 0; i < nPlane; i++)
	{
		const CvMat* normalMat = cam1Planes[i]->getNormal();
		cam1NormalRotated[i] = cvCreateMat(3,1,CV_32FC1);
		cvMatMul(rotation1_2, normalMat, cam1NormalRotated[i]);
	}

	//Create matrices and equation system. Nt = D. Where
	//N(nx3) = with the normals of camera 2.
	CvMat* N = cvCreateMat(nPlane, 3, CV_32FC1);
	float *ptrN, *ptrCam2;
	for (int r = 0; r < nPlane; r++)
	{
		const CvMat* norm = cam2Planes[r]->getNormal(); // 3x1 matrix

		ptrN = (float*)(N->data.fl + (r*N->step/sizeof(float)));	
		for (int c = 0; c < 3; c++)
		{
			ptrCam2 = (float*)(norm->data.fl + (c*norm->step/sizeof(float)));
			*ptrN++ = *ptrCam2;
		}
	}

	//t(3x1) = (tx,ty,tz)^T
	CvMat* t = cvCreateMat(3,1, CV_32FC1);

	//D(nx1) = distance(cam2) - distance(cam1)*normal(cam2)*normal_rotated(cam1). Each row is one plane.
	CvMat* D = cvCreateMat(nPlane, 1, CV_32FC1);

	for (int i = 0; i < nPlane; i++)
	{
		float* ptr = (float*)(D->data.fl + (i*D->step/sizeof(float)));
		CvMat* n2_T = cvCreateMat(1,3,CV_32FC1);
		cvTranspose(cam2Planes[i]->getNormal(), n2_T);
		CvMat* tmp = cvCreateMat(1,1, CV_32FC1);

		cvMatMul(n2_T, cam1NormalRotated[i], tmp);
		float tmpValue = *(float *)(tmp->data.fl);
		*ptr = cam2Planes[i]->getDistance() - (cam1Planes[i]->getDistance()*tmpValue);
	}

	CvMat* N_Pseudo = cvCreateMat(3, nPlane, CV_32FC1);
	if (nPlane == 3 && cvDet(N) != 0)
		cvInv(N, N_Pseudo, CV_LU);
	else
		cvInv(N, N_Pseudo, CV_SVD);

	cvMatMul(N_Pseudo, D, t);

	cvTranspose(t, translation);
}

/*
Calculates a 3d rotation matrix between two vectors.
*/
void KinectPlaneCalibration::calculateRotation(vector<Plane*> planes1, vector<Plane*> planes2, CvMat* rotation, int nPlanes)
{
	CvMat* c1Normals = cvCreateMat(3, nPlanes, CV_32FC1);
	CvMat* c2Normals = cvCreateMat(3, nPlanes, CV_32FC1);

	jointVectors(c1Normals, planes1, nPlanes);
	jointVectors(c2Normals, planes2, nPlanes);

	CvMat* c2NormalsTra = cvCreateMat(nPlanes,3, CV_32FC1);
	cvTranspose(c2Normals, c2NormalsTra);
	CvMat* S = cvCreateMat(3, 3, CV_32FC1);
	cvMatMul(c1Normals, c2NormalsTra, S);

	CvMat* UT = cvCreateMat(3,3, CV_32FC1);
	CvMat* V = cvCreateMat(3,3, CV_32FC1);
	CvMat* W = cvCreateMat(3,3, CV_32FC1);
	cvSVD(S, W, UT, V, CV_SVD_U_T);

	CvMat* I = cvCreateMat(3,3, CV_32FC1);
	CvMat* temp = cvCreateMat(3,3,CV_32FC1);
	cvMatMul(V, UT, temp);
	double det = cvDet(temp);
	cvSetIdentity(I);
	CV_MAT_ELEM(*I, float, 2,2) = det;
	cvMatMul(V, I, temp);
	cvMatMul(temp, UT, rotation);

}



//Private
void KinectPlaneCalibration::initROIPoints(XnPoint3D* roi, int size)
{
	for (int i = 0; i < size; i++)
	{
		roi[i].X = -1;
		roi[i].Y = -1;
		roi[i].Z = -1;
	}
	roiSelected = false;
}

void KinectPlaneCalibration::raw2depth(unsigned short* depth)
{
	int i;
	for ( i=0; i<MAX_DEPTH; i++) {
		float v = (float)i/MAX_DEPTH;//for visualization purposes only
		v = powf(v, 2);
		v = v*36*256;
		depth[i] = v;
	}
}

void KinectPlaneCalibration::depth2rgb(const XnDepthPixel* Xn_disparity, unsigned short* depth, char *depth_data){
	int i;

	for (i=0; i<307200; i++) {
		int pval = depth[Xn_disparity[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
		case 0:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255-lb;
			break;
		case 1:
			depth_data[3*i+0] = 255;
			depth_data[3*i+1] = lb;
			depth_data[3*i+2] = 0;
			break;
		case 2:
			depth_data[3*i+0] = 255-lb;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = 0;
			break;
		case 3:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255;
			depth_data[3*i+2] = lb;
			break;
		case 4:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 255-lb;
			depth_data[3*i+2] = 255;
			break;
		case 5:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 255-lb;
			break;
		default:
			depth_data[3*i+0] = 0;
			depth_data[3*i+1] = 0;
			depth_data[3*i+2] = 0;
			break;
		}
	}
}

/*
Sustracts two vectors
*/
void KinectPlaneCalibration::calculateVector(XnPoint3D* v, const XnPoint3D* p1, const XnPoint3D* p2)
{
	v->X = p1->X - p2->X;
	v->Y = p1->Y - p2->Y;
	v->Z = p1->Z - p2->Z;
}


/*
Fit a set of points into a plane and calculates its unit normal vector
*/
void KinectPlaneCalibration::calculateNormalVectorsDynamicRegions(list<XnPoint3D>* planePoints, CvMat* normal, CameraProperties* cam, int planeNum)
{
	//create coordinates and depth matrices
	int totalPoints = planePoints->size();
	CvMat* depthMat = cvCreateMat(totalPoints,1,CV_32FC1);
	CvMat* coordinatesMat = cvCreateMat(totalPoints,3,CV_32FC1);
	float* depthPtr;
	float* coorPtr;
	list<XnPoint3D>::iterator it;
	int row = 0;
	for (it=planePoints->begin(); it!=planePoints->end(); ++it)
	{
		depthPtr = (float*)(depthMat->data.fl  + row*depthMat->step/sizeof(float));
		XnPoint3D p = *it;
		*depthPtr = p.Z;
		//Back-project the point
		float xCam = (p.X - cam->getOx())*cam->getPixelSize()*2*p.Z/cam->getFocalLenghtX();
		float yCam = (p.Y - cam->getOy())*cam->getPixelSize()*2*p.Z/cam->getFocalLenghtY();

		coorPtr = (float*)(coordinatesMat->data.fl + row*coordinatesMat->step/sizeof(float));
		*coorPtr = xCam;
		coorPtr++;
		*coorPtr = yCam;
		coorPtr++;
		*coorPtr = 1.;		
		row++;
	}
//DEBUG BEGIN
	KinectPlaneCalibration::outDebug << "Depth Matrix" << endl;
	Utils::writeMatrixValues(depthMat, &KinectPlaneCalibration::outDebug);

	KinectPlaneCalibration::outDebug << "Coordinates Matrix" << endl;
	Utils::writeMatrixValues(coordinatesMat, &KinectPlaneCalibration::outDebug);
//DEBUG END

		//Calculate the pseudo inverse
	CvMat* coordinates_Pseudo = cvCreateMat(3, totalPoints, CV_32FC1);
	cvInvert(coordinatesMat, coordinates_Pseudo, CV_SVD);
	CvMat* parameters = cvCreateMat(3,1,CV_32FC1);
	cvMatMul(coordinates_Pseudo, depthMat, parameters);
//DEBUG BEGIN
	KinectPlaneCalibration::outDebug << "Parameters" << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *parameters, float, 0, 0) << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *parameters, float, 1, 0) << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *parameters, float, 2, 0) << endl;
//DEBUG END

	//Another way of calculating the normal vector
//	normalCrossProduct(normal, parameters);
	
	//Good method of calculating the normal
	unitNormal(normal, parameters);

//DEBUG BEGIN
	KinectPlaneCalibration::outDebug << "Normal" << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *normal, float, 0, 0) << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *normal, float, 1, 0) << endl;
	KinectPlaneCalibration::outDebug << CV_MAT_ELEM( *normal, float, 2, 0) << endl;
//DEBUG END

	//free memory
	cvReleaseMat(&parameters);	
	cvReleaseMat(&depthMat);
	cvReleaseMat(&coordinatesMat);
	cvReleaseMat(&coordinates_Pseudo);
}

/*
Calcualtes the normal vector of a plane using a ROI
*/
void KinectPlaneCalibration::calculateNormalVectorsROI(Plane* plane, CvMat* normal, const XnDepthPixel* pDepthMap, CameraProperties* cam, int planeNum)
{
		
	int initX = plane->getInitPoint().X;
	int endX =  plane->getEndPoint().X;
	int initY = plane->getInitPoint().Y;
	int endY =  plane->getEndPoint().Y;
	int width = endX - initX;
	int height = endY - initY;
	int totalPix = 0;

	ofstream xPlaneCoor;
	ofstream yPlaneCoor;
	ofstream zPlaneCoor;
	ofstream planeParam;
	ofstream planeNormal;
	Utils::createCoordinateOutStream(&xPlaneCoor, &yPlaneCoor, &zPlaneCoor, planeNum, cam->getCamId());
	Utils::createGeneralOutStream(&planeParam, PLANE_PARAMETERS, planeNum, cam->getCamId());
	Utils::createGeneralOutStream(&planeNormal, PLANE_NORMALS, planeNum, cam->getCamId());


	//Checks all the pixel with depth info
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float depth = pDepthMap[(initY+y)*XN_VGA_X_RES+(initX+x)];
			if (depth != 0)
				totalPix++;
		}
	}

	if (totalPix != (height*width))
		cout << "Error: some pixels without depth values" << endl;

	//Create the matrices to solve the system Ax=z. Where A is the point coordinates, x the parameters of the plane
	// and z the depth of the points. Use the plane ecuation: Ax + By + C = Z
	CvMat* depthMat = cvCreateMat(totalPix,1,CV_32FC1);
	CvMat* coordinatesMat = cvCreateMat(totalPix,3,CV_32FC1);
	int cont = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float depth = pDepthMap[(initY+y)*XN_VGA_X_RES+(initX+x)];
			if (depth != 0)
			{
				CV_MAT_ELEM( *depthMat, float, cont, 0) = depth;

				//Back project
				float xCam = ((initX+x) - cam->getOx())*cam->getPixelSize()*2*depth/cam->getFocalLenghtX();
				float yCam = ((initY+y) - cam->getOy())*cam->getPixelSize()*2*depth/cam->getFocalLenghtY();
//DEBUG BEGIN
xPlaneCoor << (int)xCam << " ";
yPlaneCoor << (int)yCam << " ";
zPlaneCoor << depth << " ";
//DEBUG END

				CV_MAT_ELEM( *coordinatesMat, float, cont, 0) = xCam;
				CV_MAT_ELEM( *coordinatesMat, float, cont, 1) = yCam;
				CV_MAT_ELEM( *coordinatesMat, float, cont, 2) = 1;
				cont++;
			}
		}
//DEBUG BEGIN
xPlaneCoor << endl;
yPlaneCoor << endl;
zPlaneCoor << endl;
//DEBUG END
	}
	//Calculate the pseudo inverse
	CvMat* coordinates_Pseudo = cvCreateMat(3, totalPix, CV_32FC1);
	cvInvert(coordinatesMat, coordinates_Pseudo, CV_SVD);
	CvMat* parameters = cvCreateMat(3,1,CV_32FC1);
	cvMatMul(coordinates_Pseudo, depthMat, parameters);
//DEBUG BEGIN
planeParam << CV_MAT_ELEM( *parameters, float, 0, 0) << endl;
planeParam << CV_MAT_ELEM( *parameters, float, 1, 0) << endl;
planeParam << CV_MAT_ELEM( *parameters, float, 2, 0) << endl;
//DEBUG END

	//Another way of calculating the normal vector
//	normalCrossProduct(normal, parameters);
	
	//Good method of calculating the normal
	unitNormal(normal, parameters);

//DEBUG BEGIN
planeNormal << CV_MAT_ELEM( *normal, float, 0, 0) << endl;
planeNormal << CV_MAT_ELEM( *normal, float, 1, 0) << endl;
planeNormal << CV_MAT_ELEM( *normal, float, 2, 0) << endl;
//DEBUG END

	//free memory
	cvReleaseMat(&parameters);	
	cvReleaseMat(&depthMat);
	cvReleaseMat(&coordinatesMat);
	cvReleaseMat(&coordinates_Pseudo);
}

/*
Create a unique matrix with the info from the vectors. Each vector is a row in the output matrix
*/
void KinectPlaneCalibration::jointVectors(CvMat* normals, vector<Plane*> planes, int nPlanes)
{
	const float* ptrNormal; 
	float* ptrNormalJoint;

	for (int i = 0; i < nPlanes; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ptrNormal = (const float*)(planes[i]->getNormal()->data.fl + (j*planes[i]->getNormal()->step/sizeof(float)));
			ptrNormalJoint = (float*)normals->data.fl + (j*normals->step/sizeof(float)) + i;
			*ptrNormalJoint = *ptrNormal;		
		}
	}
}


/*
Given a plane equation, finds out a point in the plane.
*/
void KinectPlaneCalibration::calculatePointInPlane(CvMat* param, XnPoint3D* p)
{
	float A = CV_MAT_ELEM( *param, float, 0, 0);
	float B = CV_MAT_ELEM( *param, float, 1, 0);
	float C = CV_MAT_ELEM( *param, float, 2, 0);
	p->Z = A*p->X + B*p->Y + C;
}


/*
Creates a vector from two points in a plane. (p1-p2)
*/
void KinectPlaneCalibration::createVectorFromPoints(const XnPoint3D* p1, const XnPoint3D* p0, XnPoint3D* v)
{
	v->X = p1->X - p0->X;
	v->Y = p1->Y - p0->Y;
	v->Z = p1->Z - p0->Z;
}


/*
Calculates the unit normal vector of the plane defined by 'param' (Ax + By + C = Z)
*/
void KinectPlaneCalibration::normalCrossProduct(CvMat* normal, CvMat* param)
{
	XnPoint3D points[3];
	points[0].X = 10; points[0].Y = 20;
	points[1].X = 12; points[1].Y = 32;
	points[2].X = 21; points[2].Y = 12;
	for (int i = 0; i < 3; i++)
	{
		calculatePointInPlane(param, &points[i]);
	}

	XnPoint3D vectors[2];
	for (int i = 0; i < 2; i++)
	{
		createVectorFromPoints(&points[i+1], &points[0], &vectors[i]);
	}
	
	CvMat* v1 = cvCreateMat(3,1, CV_32FC1);
	CvMat* v2 = cvCreateMat(3,1, CV_32FC1);
	CvMat* normalVect = cvCreateMat(3,1, CV_32FC1);
	Utils::fillTheMatrix(v1, &vectors[0]);
	Utils::fillTheMatrix(v2, &vectors[1]);
	cvCrossProduct(v1, v2, normalVect);
	float lengthNormal = cvNorm(normalVect);
	CV_MAT_ELEM( *normal, float, 0, 0) = CV_MAT_ELEM( *normalVect, float, 0, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 1, 0) = CV_MAT_ELEM( *normalVect, float, 1, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 2, 0) = CV_MAT_ELEM( *normalVect, float, 2, 0)/lengthNormal;
}

/*
Calculates the normal vector of a plane defined by 'param' (Ax+By+C=Z) adding the restriction
of being a unit normal vector.
*/
void KinectPlaneCalibration::unitNormal(CvMat* normal, const CvMat* param)
{
	CvMat* normalVect = cvCreateMat(3,1,CV_32FC1);
	CV_MAT_ELEM( *normalVect, float, 0, 0) = CV_MAT_ELEM( *param, float, 0, 0);
	CV_MAT_ELEM( *normalVect, float, 1, 0) = CV_MAT_ELEM( *param, float, 1, 0);
	CV_MAT_ELEM( *normalVect, float, 2, 0) = -1;

	float lengthNormal = cvNorm(normalVect);
	CV_MAT_ELEM( *normal, float, 0, 0) = CV_MAT_ELEM( *normalVect, float, 0, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 1, 0) = CV_MAT_ELEM( *normalVect, float, 1, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 2, 0) = CV_MAT_ELEM( *normalVect, float, 2, 0)/lengthNormal;
}


/*
Calculate the normal vector using a selected ROI
*/
void KinectPlaneCalibration::getNormalwithROI(vector<Plane>* planes, const IplImage* depthImage, CameraProperties* cam, CvMat* normalVec[], const XnDepthPixel* pDepthMap, int nPlanes)
{
	cvNamedWindow("Depth Image", 1);
	
	for (int i = 0; i < nPlanes; i++)
	{
		cvSetMouseCallback("Depth Image", selectROI_callBack, (Plane*)&(*planes)[i]);
		cvShowImage("Depth Image", depthImage);
		while (!roiSelected)
			cvWaitKey(1);

		cout << "Camera " << cam->getCamId() << " Plane " << i << " selected. " << endl;
		calculateNormalVectorsROI(&(*planes)[i], normalVec[i], pDepthMap, cam, i);
		roiSelected = false;
	}
	cvDestroyAllWindows();
}

/*
Allows the user to select a point in the image. (It will be used as a seed for the growing plane)
*/
void KinectPlaneCalibration::getPointSeed(char* windowName, XnPoint3D* p, IplImage* depthImage)
{
	cvSetMouseCallback(windowName, selectPoint_callBack, (XnPoint3D*)p);
	cvShowImage(windowName, depthImage);
	while (!pointSelected)
		cvWaitKey(1);

	pointSelected = false;
}

/*
Allows the user to select a ROI in the image. (It will be used as a seed for the growing plane)
*/
void KinectPlaneCalibration::getROISeed(char* windowName, Plane* plane, IplImage* depthImage)
{
	cvSetMouseCallback(windowName, selectROI_callBack, (Plane*)plane);
	cvShowImage(windowName, depthImage);
	while (!plane->isROISelected())
		cvWaitKey(1);

	roiSelected = false;
	cvDestroyWindow(windowName);
}


/*
Generates a list of points with all the points of the ROI defined by 'plane'
*/
void KinectPlaneCalibration::generateListPoint(Plane* plane, list<XnPoint3D>* lst, const XnDepthPixel* pDepthMap, int maskSize)
{
	int maskOffset = (maskSize/2);

	int totalDepth = 0;
	double avgDepth;

	int contX = plane->getInitPoint().X;
	int contY;
	while (contX < plane->getEndPoint().X)
	{
		contY = plane->getInitPoint().Y;
		while (contY < plane->getEndPoint().Y)
		{
			XnPoint3D p;
			p.X = contX;
			p.Y = contY;
			p.Z = pDepthMap[contY*XN_VGA_X_RES+contX];
			totalDepth += p.Z;
			lst->push_back(p);
			contY += (2*maskOffset); 
		}
		contX += (2*maskOffset);
	}
	//Remove noise
	avgDepth = totalDepth/lst->size();
	list<XnPoint3D>::iterator it = lst->begin();
	while(it != lst->end())
	{
		XnPoint3D p = *it;
		if (abs(p.Z-avgDepth) > 300)
			it = lst->erase(it);
		else
			it++;
	}
			
}


/*
Calculate the normal vector using a selected ROI
*/
void KinectPlaneCalibration::getNormalGrowingRegion(IplImage* depthImage, CameraProperties* cam, vector<Plane*> planes, const XnDepthPixel* pDepthMap, int nPlanes, const int maskSz)
{
	char* windowName = "Depth Image";
	int rgbPlanes[3];
	cvNamedWindow(windowName, 1);
//	int maskSize = 9;
	
	bool planeSelected = false;
	for (int i = 0; i < nPlanes; i++)
	{
		ofstream planeNormal;
		Utils::createGeneralOutStream(&planeNormal, PLANE_NORMALS, i, cam->getCamId());
		Utils::generateRandomColor(rgbPlanes);

	/*	XnPoint3D p;
		getPointSeed(windowName, &p, depthImage);	
		p.Z = pDepthMap[(int)p.Y*XN_VGA_X_RES+(int)p.X];
		DynamicPlane planeCreator(&p, depthImage->width, depthImage->height,9, rgbPlanes, i, cam);*/

		list<XnPoint3D> planePointLst;
		getROISeed(windowName, planes[i], depthImage);
		generateListPoint(planes[i], &planePointLst, pDepthMap, maskSz);
		DynamicPlane planeCreator(&planePointLst, depthImage->width, depthImage->height, maskSz, rgbPlanes, i, cam);

		planeCreator.makePlaneGrow(windowName, depthImage, pDepthMap);
		
		planes[i]->setParameters(planeCreator.getPlaneParameters());
		CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
		unitNormal(unitNorm, planes[i]->getParameters());
		planes[i]->setNormal(unitNorm);
		planes[i]->setDistance(Utils::calculatePlaneDistance(planes[i]->getParameters(), planes[i]->getNormal()));
KinectPlaneCalibration::outDebug << "Distance plane: " << i << " Camera: " << cam->getCamId() << ": " << planes[i]->getDistance() << endl;

//DEBUG BEGIN
planeNormal << CV_MAT_ELEM( *planes[i]->getNormal(), float, 0, 0) << endl;
planeNormal << CV_MAT_ELEM( *planes[i]->getNormal(), float, 1, 0) << endl;
planeNormal << CV_MAT_ELEM( *planes[i]->getNormal(), float, 2, 0) << endl;
//DEBUG END	
		//calculateNormalVectorsDynamicRegions(planeCreator.getPlanePoints(), normalVec[i], cam, i);
		cout << "Camera " << cam->getCamId() << " Plane " << i << " selected. " << endl;
//		calculateNormalVectors(&(*planes)[i], normalVec[i], pDepthMap, cam, i);
	}
	cvWaitKey(0);
	cvDestroyAllWindows();
}


/*
Return a 3D point clicked by the user in the depth image
*/
XnPoint3D* KinectPlaneCalibration::getPointFromImage(CameraProperties* cam)
{
	char* windowTransName = "Translation";
	XnPoint3D* p3D = new XnPoint3D;
	XnPoint3D p2D;

	IplImage* depthImage = cvCreateImageHeader(cvSize(640,480),8,3);
	unsigned short depth[MAX_DEPTH];
	char *depth_data = (char*) malloc(640*480*3);
	raw2depth(depth);	
	cam->getContext()->StartGeneratingAll();
	cam->getContext()->WaitAndUpdateAll();	
	const XnDepthPixel* pDepthMap = cam->getDepthNode()->GetDepthMap();
	depth2rgb(pDepthMap, depth, depth_data);
	cvSetData(depthImage,depth_data, 640*3);	
	cam->getContext()->StopGeneratingAll();

	cvNamedWindow(windowTransName, 1);
	getPointSeed(windowTransName, &p2D, depthImage);

	cvWaitKey(0);
	cvDestroyAllWindows();

	p2D.Z = pDepthMap[(int)p2D.Y*XN_VGA_X_RES+(int)p2D.X];

	Utils::backProjectPoint(&p2D, p3D, cam);

	KinectPlaneCalibration::outDebug << "Point 2D: " << p2D.X << ", " << p2D.Y << ", " << p2D.Z << endl;
	KinectPlaneCalibration::outDebug << "Point 3D: " << p3D->X << ", " << p3D->Y << ", " << p3D->Z << endl;
	free(depth_data);
	return p3D;
}

void KinectPlaneCalibration::getTranslationFrom2Points(XnPoint3D* p1, XnPoint3D* p2, const CvMat* rotation, CvMat* t)
{
	CvMat *cam1P_Mat, *cam2P_Mat;
	cam1P_Mat = cvCreateMat(3,1,CV_32FC1);
	cam2P_Mat = cvCreateMat(3,1,CV_32FC1);
	Utils::fillTheMatrix(cam1P_Mat, p1);
	Utils::fillTheMatrix(cam2P_Mat, p2);

	CvMat* trans_T = cvCreateMat(3,1,CV_32FC1);
	CvMat* tmp = cvCreateMat(3,1, CV_32FC1);
	cvMatMul(rotation, cam1P_Mat, tmp);
	cvSub(cam2P_Mat, tmp, trans_T);
	cvTranspose(trans_T, t);
}


/*
Calculates the closest point to all the planes. Nx=D. Where N is the matrix of the normals (nx3). x is the matrix that represets
the point (3x1). And D (nx1) represets the distances
*/
XnPoint3D* KinectPlaneCalibration::getClosestPoint(vector<Plane*> planes)
{
	XnPoint3D* out = new XnPoint3D;
	int nPlanes = planes.size();
	//Create N
	CvMat* N = cvCreateMat(nPlanes, 3, CV_32FC1);
	float *ptrN, *ptrCam2;
	for (int r = 0; r < nPlanes; r++)
	{
		const CvMat* norm = planes[r]->getNormal(); // 3x1 matrix

		ptrN = (float*)(N->data.fl + (r*N->step/sizeof(float)));	
		for (int c = 0; c < 3; c++)
		{
			ptrCam2 = (float*)(norm->data.fl + (c*norm->step/sizeof(float)));
			*ptrN++ = *ptrCam2;
		}
	}

	//Create X
	CvMat* x = cvCreateMat(3,1, CV_32FC1);

	//Create D
	CvMat* D = cvCreateMat(nPlanes, 1, CV_32FC1);
	for (int i = 0; i < nPlanes; i++)
	{
		float* ptr = (float*)(D->data.fl + (i*D->step/sizeof(float)));
		*ptr = planes[i]->getDistance() ;
	}

	
	CvMat* N_Pseudo = cvCreateMat(3, nPlanes, CV_32FC1);
	if (nPlanes == 3 && cvDet(N) != 0)
		cvInv(N, N_Pseudo, CV_LU);
	else
		cvInv(N, N_Pseudo, CV_SVD);

	cvMatMul(N_Pseudo, D, x);

	//Assign values
	out->X = *(float*)(x->data.fl);
	out->Y = *(float*)(x->data.fl + (x->step/sizeof(float)));
	out->Z = *(float*)(x->data.fl + (2*x->step/sizeof(float)));

	return out;
}


/*
Calculate the mean point of all the special points of the planes. First calculate the closest point of 
each plane to the origin and  then calculate the mean of all of them.
*/
XnPoint3D* KinectPlaneCalibration::getMeanPoint(vector<Plane*> planes)
{
	XnPoint3D* out = new XnPoint3D;
	out->X = 0; out->Y = 0; out->Z = 0;
	int size = planes.size();
	float *ptrN;
	for (int i = 0; i < size; i++)
	{
		const CvMat* norm = planes[i]->getNormal(); // 3x1 matrix
		ptrN = (float*)(norm->data.fl);
		out->X += (planes[i]->getDistance()*(*ptrN));

		ptrN = (float*)(norm->data.fl + (norm->step/sizeof(float)));
		out->Y += (planes[i]->getDistance()*(*ptrN));

		ptrN = (float*)(norm->data.fl + (2*norm->step/sizeof(float)));
		out->Z += (planes[i]->getDistance()*(*ptrN));
	}

	out->X /= size;
	out->Y /= size;
	out->Z /= size;


	return out;
}


/*
Get the position of the planes, where the rotation works better
*/
vector<int> KinectPlaneCalibration::getPlanePositions(vector<Plane*> planes1, vector<Plane*> planes2, const CvMat* rotation)
{
	vector<int> planePositions;
	for (int i=0; i < planes1.size(); i++)
	{
		Plane* p1 = planes1[i];
		Plane* p2 = planes2[i];

		const CvMat *n1, *n2; // 3x1 matrices
		n1 = p1->getNormal();
		n2 = p2->getNormal();

	
		CvMat* n1Rotat = cvCreateMat(3,1, CV_32FC1);
		cvMatMul(rotation, n1, n1Rotat);
		
		CvMat* n1R_Trans = cvCreateMat(1,3, CV_32FC1);
		cvTranspose(n1Rotat, n1R_Trans);

		CvMat* tmp = cvCreateMat(1,1, CV_32FC1);
		cvMatMul(n1R_Trans, n2, tmp);

		float tmpVal = *(tmp->data.fl);
		float angle = acos(tmpVal)*180/CV_PI;
		KinectPlaneCalibration::outDebug << "Angle plane(" <<i<< ")= " << angle << endl;

		if (angle < 3)
			planePositions.push_back(i);
	}

	return planePositions;
}

/*
Get the mean point of the best closest points. The best closest points are the points that come from the planes correspondences 
whose normals match well after the rotation.
*/
XnPoint3D* KinectPlaneCalibration::getMeanPoint(vector<Plane*> planes, vector<int> planePositions)
{
	
	XnPoint3D* out = new XnPoint3D;
	out->X = 0; out->Y = 0; out->Z = 0;
	int size = planePositions.size();
	float *ptrN;
	for (int j = 0; j < size; j++)
	{
		int pos = planePositions[j];
		Plane* p = planes[pos];
				
		const CvMat* norm = p->getNormal(); // 3x1 matrix
		ptrN = (float*)(norm->data.fl);
		out->X += (p->getDistance()*(*ptrN));

		ptrN = (float*)(norm->data.fl + (norm->step/sizeof(float)));
		out->Y += (p->getDistance()*(*ptrN));

		ptrN = (float*)(norm->data.fl + (2*norm->step/sizeof(float)));
		out->Z += (p->getDistance()*(*ptrN));
	}

	out->X /= size;
	out->Y /= size;
	out->Z /= size;


	return out;
}