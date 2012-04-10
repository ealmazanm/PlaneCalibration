#pragma once
#include "XnCppWrapper.h"
#include "CameraProperties.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp> 
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "DynamicPlane.h"
#include "stdio.h"
#include "Utils.h"
#include "filePaths.h"
#include "Plane.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <new>
#include <list>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cvblob.h>


//using namespace boost::filesystem; 
using namespace std;
using namespace xn;
using namespace cvb;
using std::string;

const double COLORFILTER_THRESHOLD = 0.1;
const int AREA_THRESHOLD = 600;
const int RGB_FILTER = 0;
const int HSV_FILTER = 1;
const int MAX_DEPTH = 10000;
const int MAX_PLANES = 74;
const int MIN_PLANES = 0;
const int GROUND_TRUTH = 1730;
class KinectPlaneCalibration
{
public:
	KinectPlaneCalibration(void);
	~KinectPlaneCalibration(void);
//	void workerFunc(int i);
	void getPlanesCameras_Manual(vector<Plane*> planes1, vector<Plane*> planes2, CameraProperties* cam1, CameraProperties* cam2, const int nPlanes, const int maskSz);
	void getPlanesCameras(vector<Plane*> planes1, vector<Plane*> planes2, CameraProperties* cam1, CameraProperties* cam2, const int nPlanes, const int maskSz , void* hist, int typeFilter);
	void getNormalVectPlane_LS(vector<Plane*> normalVec, CameraProperties* cam, const int, const int);

	void calculateRotation(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, CvMat* rotation, int nPlanes);
	void calculateTranslation(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation, const int nPlane, CvMat* translation);
	void calculateTranslation2(CameraProperties*cam1, CameraProperties* cam2, const CvMat* rotation, CvMat* translation);
	void calculateTranslation3(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation, const int nPlane, CvMat* translation);
	void calculateTranslation_mean(vector<Plane*> cam1Planes, vector<Plane*> cam2Planes, const CvMat* rotation, const int nPlane, CvMat* translation);
	void calculateTranslation_hardCoded(XnPoint3D* p1, XnPoint3D* p2, CvMat* rotation, CvMat* translation);
	static  ofstream outDebug;
private:
	void labellingNoiseFilter(list<XnPoint3D>* pointsSrc, Plane* plane, const XnDepthPixel* depthMap);
	void capturePlaneManual(Plane* plane, const XnDepthPixel* depthMap,  const XnRGB24Pixel* rgbMap, const int maskSz, CameraProperties* cam, int idPlane);
	void getCameraMaps_Manual(CameraProperties* cam, vector<XnDepthPixel*>* depthMap_lst, vector <XnRGB24Pixel*>* rgbMap_lst, int id);
	void capturePlane(Plane* plane, const XnRGB24Pixel* rgbMap, const XnDepthPixel* depthMap, void* param, const int maskSz, CameraProperties* cam, int idPlane, int filterType);
	void getCameraMaps(CameraProperties* cam, vector<XnDepthPixel*>* depthMap_lst, vector <XnRGB24Pixel*>* rgbMap_lst, int id);
	void colorFilter(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap1);
	void colorFilter_RGB(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap1,const vector<vector<vector<double>>>* hist);
	void colorFilter_HSV(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap, const vector<vector<double>>* hist);
	void initROIPoints(XnPoint3D* roi, int size);
	void raw2depth(unsigned short* depth);
	void depth2rgb(const XnDepthPixel* Xn_disparity, unsigned short* depth, char *depth_data);
	void calculateVector(XnPoint3D* v, const XnPoint3D* p1, const XnPoint3D* p2);
	void calculateNormalVectorsROI(Plane*, CvMat*, const XnDepthPixel*, CameraProperties* cam, int planeNum);
	void calculateNormalVectorsDynamicRegions(list<XnPoint3D>*, CvMat* normal, CameraProperties* cam, int planeNum);
	void jointVectors(CvMat* c1Normals, vector<Plane*> planes, int nPlanes);
	void initOutStreamPath(char* xPlane, char* yPlane, char* zPlane);
	void createFullPath(const int pNum, const int cId, char* plane);
	void calculatePointInPlane(CvMat* param, XnPoint3D* p);
	void createVectorFromPoints(const XnPoint3D* p1, const XnPoint3D* p0, XnPoint3D* v);
	void normalCrossProduct(CvMat* normal, CvMat* param);
	void unitNormal(CvMat* normal, const CvMat* param);
//	void createCoordinateOutStream(ofstream* xPlaneCoor, ofstream* yPlaneCoor, ofstream* zPlaneCoor, const int planeNum, const int camId);
//	void createGeneralOutStream(ofstream* planeOutStream, const char* fileName, const int planeNum, const int camId);
	void getNormalwithROI(vector<Plane>* planes, const IplImage* depthImage, CameraProperties* cam, CvMat* normalVec[], const XnDepthPixel* pDepthMap, int nPlanes);
	void getNormalGrowingRegion(IplImage* depthImage, CameraProperties* cam, vector<Plane*> planes, const XnDepthPixel* pDepthMap, int nPlanes, const int maskSize);
	void getPointSeed(char* windowName, XnPoint3D* p, IplImage* depthImage);
	void getROISeed(char* windowName, Plane* plane, IplImage* depthImage);
	void generateListPoint(Plane* plane, list<XnPoint3D>* lst, const XnDepthPixel* pDepthMap, int maskSize);
	XnPoint3D* getPointFromImage(CameraProperties* cam);
	void getTranslationFrom2Points(XnPoint3D* p1, XnPoint3D* p2, const CvMat* rotation, CvMat* t);
	XnPoint3D* getClosestPoint(vector<Plane*> planes);
	XnPoint3D* getMeanPoint(vector<Plane*> planes);
	XnPoint3D* getMeanPoint(vector<Plane*> planes, vector<int> planePositions);
	vector<int> getPlanePositions(vector<Plane*> planes1, vector<Plane*> planes2, const CvMat* rotation);
};

