#include "PlaneCalibration.h"


void showTranslationMenu()
{
	cout << "Translation method" << endl;
	cout << " '1' : Click a point" << endl;
	cout << " '2' : Use the special point to all planes" << endl;
	cout << " '3' : Use the mean of all special points" << endl;
	cout << " '4' : Use all the special points" << endl;
	cout << " '5' : Use hard coded point" << endl;
}


/*
Allocate 'n' CvMat in the array
*/
void allocatePlanes(vector<Plane*>* planes, int n)
{
	for (int i = 0; i < n; i++)
	{
		Plane* p = new Plane;
		(*planes)[i] = p;
	}
}


/*
Checks if 'val' is in the list 'intList'.
*/
bool isInList(vector<int>* intList, int val, int size)
{
	bool found = false;
	int j = 0;
	while (j < size && !found)
	{	
		found = (*intList)[j] == val;
		j++;
	}	
	return found;
}

/*
genearte random integer numbers between max_planes and min_planes values to fill up the vector
*/
void createIntRandomVect(vector<int>* planePos, int size, vector<int>* auxValues, int max_value)
{	
	int val = 0;
	for (int i = 0; i < size; i++)
	{
		val = Utils::getRandomNumber(max_value, MIN_PLANES);

		if (auxValues == NULL)
		{
			while (isInList(planePos, val, i))
			{	
				val = Utils::getRandomNumber(max_value, MIN_PLANES);
			}
		}
		else
		{
			while (isInList(planePos, val, i) || isInList(auxValues, val, auxValues->size()))
			{	
				val = Utils::getRandomNumber(max_value, MIN_PLANES);
			}
		}
		(*planePos)[i] = val;
	}
}

/*
Initialize all the positions in filePaths
*/
void initFilePath(vector<char*>* filePaths, int nFiles, char* prefix, char* fileName, char* camId, char* suffix, vector<int>* planePositions)
{
	char strNplane[10];
	for (int i = 0; i < nFiles; i++)
	{
		//allocate enough memory
		(*filePaths)[i] = new char[130];

		//creates the first part of the file path
		strcpy((*filePaths)[i], prefix);
		strcat((*filePaths)[i], fileName);
		//creates the second part of the path
		int pos = (*planePositions)[i];
		itoa(pos, strNplane, 10);
		strcat((*filePaths)[i], camId);
		strcat((*filePaths)[i], strNplane);
		strcat((*filePaths)[i], suffix);
	}
}

int getNumberOfFiles(char* folderPath)
{
	int out = 0;
	boost::filesystem::directory_iterator end ;
	//depthMap cam1
	for( boost::filesystem::directory_iterator iter(folderPath) ; iter != end ; ++iter )
      if ( !is_directory( *iter ) )
		  out++;

	return out;
}

int getNumberOfPlanes()
{
	int out, tmp;
	out = 0;

	boost::filesystem::directory_iterator end;
	for( boost::filesystem::directory_iterator iter(filePaths::CAM1_CALIBRATION_DATA) ; iter != end ; ++iter )
	{
      if (is_directory( *iter ))
	  {
		  string strFile = iter->path().string();
		  strFile.replace(strFile.find('\\'),1, "/");
		  char *charFile=new char[strFile.size()+1];
		  charFile[strFile.size()]=0;
		  memcpy(charFile,strFile.c_str(),strFile.size());
	
		  tmp = getNumberOfFiles(charFile);
		  if (out == 0)
			  out = tmp;
		  else if (out != tmp)
		  {
			  cout << "Error in number of calibration planes" << endl;
			  exit(1);
		  }
	  }
	}
	
	for( boost::filesystem::directory_iterator iter(filePaths::CAM2_CALIBRATION_DATA) ; iter != end ; ++iter )
	{
      if (is_directory( *iter ))
	  {
		  string strFile = iter->path().string();
		  strFile.replace(strFile.find('\\'),1, "/");
		  char *charFile=new char[strFile.size()+1];
		  charFile[strFile.size()]=0;
		  memcpy(charFile,strFile.c_str(),strFile.size());
		  tmp = getNumberOfFiles(charFile);
		  if (out != tmp)
		  {
			  cout << "Error in number of calibration planes" << endl;
			  exit(1);
		  }
	  }
	}
	return out;
}

void initMenu(int* nTrain, int* nTest)
{
	/*cout << "How many planes will be used for training? " ;
	cin >> *nTrain;*/
	*nTrain = 45;

	/*cout << "How many planes will be used for testing? ";
	cin >> *nTest;*/
	*nTest = 10;
}

void showImage(char* windowName, IplImage* img)
{
	cvNamedWindow(windowName);
	cvShowImage(windowName, img);
	cvWaitKey(0);
	cvDestroyWindow(windowName);
}

int main()
{
	srand(time(0));

	KinectPlaneCalibration kinectCalib;
	CameraProperties cam1, cam2;
	Utils::rgbdInitAligned(&cam1, &cam2);
	Utils::initIntrinsicParameters(&cam1);
	Utils::initIntrinsicParameters(&cam2);

	int refPlaneNum = getNumberOfPlanes();

	int nTrain = refPlaneNum;
	int nTest = 1;
	initMenu(&nTrain, &nTest);
//	while(nTrain+nTest > refPlaneNum)
//	{
//		cout << "The sum of train and test samples must be less or equal than " << refPlaneNum << endl;
//		initMenu(&nTrain, &nTest);
//	}

	int times = 1;
	//cout << "How many times do you want to evaluate? ";
	//cin >> times;

	int translationType = 1;
	//cout << "Translation method: " << endl;
	//cout << "1.- All special points. " << endl;
	//cout << "2.- Mean of all special points. " << endl;
	//cout << "3.- Spacial point to all planes " << endl;
	//cin >> translationType;
//	float errorAvg_full = 0;

	ofstream errorStream, translationStream;
	char streamName[150];
	char errorStreamName[150];
	strcpy(streamName, filePaths::PLANE_CALIBRATION_ERROR);
	strcpy(errorStreamName, filePaths::PLANE_CALIBRATION_ERROR);
	char typeTrans[50];
	if (translationType == 1)
		strcpy(typeTrans, "/allSpecial.txt");
	else if (translationType == 2)
		strcpy(typeTrans, "meanSpecial.txt");
	else
		strcpy(typeTrans, "specialToAll.txt");
	
	strcat(streamName, typeTrans);
	strcat(errorStreamName, "errorsDistr_hist.txt");
	errorStream.open(errorStreamName);
	translationStream.open(streamName, ios::app);
	float totalVar = 0;
	for (int k = 0; k < times; k++)
	{
		vector<float> errors(nTest);
		float sumError = 0;
		for (int iTest = 0; iTest < nTest; iTest++)
		{
			//training planes
			vector<Plane*> cam1Planes(nTrain);
			vector<Plane*> cam2Planes(nTrain);
			allocatePlanes(&cam1Planes, nTrain);
			allocatePlanes(&cam2Planes, nTrain);
			//test planes
			vector<XnPoint3D*> cam1Centroids(1);
			vector<XnPoint3D*> cam2Centroids(1);

			//Create positions
			vector<int> trainPos(nTrain);
			vector<int> testPos(1);
			//obtain randomly sample positions
			createIntRandomVect(&trainPos, nTrain, NULL, refPlaneNum);
			createIntRandomVect(&testPos, 1, &trainPos, refPlaneNum);

			//create file paths names for loading the planes
			char* suffix_xml = ".xml";
			char* suffix_txt = ".txt";
			char* paramStr = "Param ";
			char* normStr = "Normal ";
			char* centrStr = "Centroid ";

			//training
			vector<char*> cam1PlaneParamsFilePaths(nTrain);
			vector<char*> cam1PlaneNormsFilePaths(nTrain);
			vector<char*> cam2PlaneParamsFilePaths(nTrain);
			vector<char*> cam2PlaneNormsFilePaths(nTrain);
			//test
			vector<char*> cam1CentroidsFilePaths(1);
			vector<char*> cam2CentroidsFilePaths(1);

			initFilePath(&cam1PlaneParamsFilePaths, nTrain, filePaths::CAM1_CALIB_PARAMETERS, paramStr, "1", suffix_xml, &trainPos);
			initFilePath(&cam1PlaneNormsFilePaths, nTrain, filePaths::CAM1_CALIB_NORMALS, normStr, "1", suffix_xml, &trainPos);
			initFilePath(&cam2PlaneParamsFilePaths, nTrain, filePaths::CAM2_CALIB_PARAMETERS, paramStr, "2", suffix_xml, &trainPos);
			initFilePath(&cam2PlaneNormsFilePaths, nTrain, filePaths::CAM2_CALIB_NORMALS, normStr, "2", suffix_xml, &trainPos);

			initFilePath(&cam1CentroidsFilePaths, 1, filePaths::CAM1_CALIB_CENTROIDS, centrStr, "1", suffix_txt, &testPos);
			initFilePath(&cam2CentroidsFilePaths, 1, filePaths::CAM2_CALIB_CENTROIDS, centrStr, "2", suffix_txt, &testPos);
			//end of creating file paths

			//load planes for training and testing
			for (int i = 0; i < nTrain; i++)
			{		
				//training (load planes)
				cam1Planes[i]->setParameters((CvMat*)cvLoad(cam1PlaneParamsFilePaths[i]));
				cam1Planes[i]->setNormal((CvMat*)cvLoad(cam1PlaneNormsFilePaths[i]));
				cam2Planes[i]->setParameters((CvMat*)cvLoad(cam2PlaneParamsFilePaths[i]));
				cam2Planes[i]->setNormal((CvMat*)cvLoad(cam2PlaneNormsFilePaths[i]));
				cam1Planes[i]->setDistance(Utils::calculatePlaneDistance(cam1Planes[i]->getParameters(), cam1Planes[i]->getNormal()));
				cam2Planes[i]->setDistance(Utils::calculatePlaneDistance(cam2Planes[i]->getParameters(), cam2Planes[i]->getNormal()));
			}

			//START CALIBRATION
			//Calculate the rotation from cam1 to cam2
			CvMat* rotation = cvCreateMat(3,3,CV_32FC1);
			kinectCalib.calculateRotation(cam1Planes, cam2Planes, rotation, nTrain);
			//Rotation from cam2 to cam1
			CvMat* rotation21 = cvCreateMat(3,3,CV_32FC1);
			cvTranspose(rotation, rotation21);
			cvSave(filePaths::CAM1_2_ROTATION_FILEPATH, rotation);
			cvSave(filePaths::CAM2_1_ROTATION_FILEPATH, rotation21);
			cam2.setRotationMatrix(rotation21);
			cam1.setRotationMatrix(rotation);
			//Calculate the translation from cam1 to cam2
			CvMat* translation = cvCreateMat(1, 3, CV_32FC1);
			if (translationType == 1)
			{
				//use all the special points
				kinectCalib.calculateTranslation(cam1Planes, cam2Planes, rotation, nTrain, translation);
			}
			else if (translationType == 2)
			{
				//use the mean of all the special points
				kinectCalib.calculateTranslation_mean(cam1Planes, cam2Planes, rotation, refPlaneNum-1, translation);
			}
			else
			{
				//use the closest point to all the special points
				kinectCalib.calculateTranslation3(cam1Planes, cam2Planes, rotation, refPlaneNum-1, translation);
			}

			//translation from cam2 to cam1
			CvMat* translation2_1T = cvCreateMat(3, 1, CV_32FC1);
			CvMat* translation2_1 = cvCreateMat(1, 3, CV_32FC1);
			CvMat*translationT = cvCreateMat(3,1, CV_32FC1);
			cvTranspose(translation, translationT);
			cvMatMul(rotation21, translationT, translation2_1T);
			cvTranspose(translation2_1T, translation2_1);
			CvMat* translation2_1_neg = cvCreateMat(1,3,CV_32FC1);
			Utils::changeSign(translation2_1, translation2_1_neg);

			cvSave(filePaths::CAM1_2_TRANSLATION_FILEPATH, translation);
			cvSave(filePaths::CAM2_1_TRANSLATION_FILEPATH, translation2_1_neg);
			cam1.setTranslationMatrix(translation);
			cam2.setTranslationMatrix(translation2_1_neg);
			//END CALIBRATION

			//START EVALUATION
			ifstream centroid1Stream, centroid2Stream;			
			//Evaluate the calibration Distance(centr1 , (rotation*centr2 + translation))
			//load the centroids
			double x, y, z;
			centroid1Stream.open(cam1CentroidsFilePaths[0]);
			centroid2Stream.open(cam2CentroidsFilePaths[0]);
			cam1Centroids[0] = new XnPoint3D;
			centroid1Stream >> x;
			centroid1Stream >> y;
			centroid1Stream >> z;
			cam1Centroids[0]->X = x; cam1Centroids[0]->Y = y; cam1Centroids[0]->Z = z;
			centroid1Stream.close();
			cam2Centroids[0] = new XnPoint3D;
			centroid2Stream >> x;
			centroid2Stream >> y;
			centroid2Stream >> z;
			cam2Centroids[0]->X = x; cam2Centroids[0]->Y = y; cam2Centroids[0]->Z = z;
			centroid2Stream.close();

			XnPoint3D cntr2_rt;
			CvMat* centr2_mat = cvCreateMat(3,1,CV_32FC1);
			Utils::fillTheMatrix(centr2_mat,cam2Centroids[0]);

			CvMat* tmp = cvCreateMat(3, 1, CV_32FC1);
			cvMatMul(cam2.getRotationMatrix(), centr2_mat, tmp);
			CvMat* out = cvCreateMat(3,1, CV_32FC1);
			CvMat* trans_II_trans = cvCreateMat(3,1, CV_32FC1);
			cvTranspose(cam2.getTranslationMatrix(), trans_II_trans);
			cvAdd(tmp, trans_II_trans, out);

			cntr2_rt.X = *out->data.fl;
			cntr2_rt.Y = *(out->data.fl + (out->step/sizeof(float)));
			cntr2_rt.Z = *(out->data.fl + 2*(out->step/sizeof(float)));

			float d = sqrt(pow(cam1Centroids[0]->X-cntr2_rt.X, 2) + pow(cam1Centroids[0]->Y-cntr2_rt.Y,2) + pow(cam1Centroids[0]->Z-cntr2_rt.Z,2));
			KinectPlaneCalibration::outDebug << testPos[0] << endl;
			KinectPlaneCalibration::outDebug << "Distance: "<< d << endl;
			KinectPlaneCalibration::outDebug << "Error: "<< abs(d - GROUND_TRUTH) << endl;
			KinectPlaneCalibration::outDebug << "***********************************" << endl;

			errors[iTest] = d - GROUND_TRUTH;		
		}

		float mean = 0;
		float sumTmp = 0;
		float totalError = 0;
		for (int j = 0; j < nTest; j++)
		{
			float e = errors[j];
			totalError += abs(e);
			sumTmp += pow(e, 2); 
			mean += e;
			errorStream << nTrain << " " << e << endl;
		}
		mean /= nTest; 
		float variance = sqrt(sumTmp/nTest);
		cout << "Error: " << totalError/nTest << "mm." << endl;
		translationStream << "5" << " " << mean+variance << endl;
	}

	errorStream.close();
	translationStream.close();
}