#include "pattern.h"
#include <iostream>
#include "opencv2\core\core.hpp"      
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"
using namespace cv;
using namespace std;

namespace AugR {

	
	
	Pattern::Pattern(double param1){
		id =-1;
		size = param1;
		orientation = -1;
		confidence = -1;
	
		rotVec = (Mat_<float>(3,1) << 0, 0, 0);
		transVec = (Mat_<float>(3,1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);
		

	}

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);		
	}

	void Pattern::showPattern()
	{
		//cout << "Pattern ID: " << id << endl;
		//cout << "Pattern Size: " << size << endl;
		//cout << "Pattern Confedince Value: " << confidence << endl;
		//cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
	//	cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i<3; i++){
		//cout << rotMat.at<float>(i,0) << "\t" << rotMat.at<float>(i,1) << "\t" << rotMat.at<float>(i,2) << " |\t"<< transVec.at<float>(i,0) << endl;
		}
	}

	void Pattern::getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{
		
		CvMat objectPts;//header for 3D points of pat3Dpts
		CvMat imagePts;//header for 2D image points of pat2Dpts 
		CvMat intrinsics = cameraMatrix;
		CvMat distCoeff = distortions;
		CvMat rot = rotVec;
		CvMat tra = transVec;
//		CvMat rotationMatrix = rotMat; // projectionMatrix = [rotMat tra];

		CvPoint2D32f pat2DPts[4];
		for (int i = 0; i<4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}

		//3D points in pattern coordinate system
		CvPoint3D32f pat3DPts[4];
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;
		cvInitMatHeader(&objectPts, 4, 3,CV_32FC1,pat3DPts);
		cvInitMatHeader(&imagePts, 4, 2,CV_32FC1,pat2DPts);
		
		//find extrinsic parameters
		cvFindExtrinsicCameraParams2(&objectPts, &imagePts, &intrinsics, &distCoeff, &rot, &tra);
	}

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix)
	{

		CvScalar color = cvScalar(0,0,255);
		
	//	switch (id){
	//	case 1:
	//	 color = cvScalar(255,0,255);
	//	break;
	//		case 2:
	//			 color = cvScalar(255,255,0);
	//			break;
	//		case 3:
	//			 color = cvScalar(0,255,255);
	//			break;
	//	}

		//model 3D points: they must be projected to the image plane
		Mat modelPts = (Mat_<float>(8,3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
			0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size );

	//	Mat modelPts = (Mat_<float>(8,3) << 0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size);


		std::vector<cv::Point2f> model2ImagePts;
		vector<Point> points(4);
		/* project model 3D points to the image. Points through the transformation matrix 
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the 
		camera CS, and then, points are projected using camera parameters 
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts); 
	//	cout<<"rotation"<<endl;
	//	cout<<rotVec<<endl;
	//	cout<<"tranlation"<<endl;
	//	cout<<transVec<<endl;
		//draw cube, or whatever
		//int i;
	//	for (i =0; i<4; i++){
	//		cv::line(frame, model2ImagePts.at(i%4), model2ImagePts.at((i+1)%4), color, 3);
	//	}

//		for (i =4; i<7; i++){
//			cv::line(frame, model2ImagePts.at(i%8), model2ImagePts.at((i+1)%8), color, 3);
//		}
//		cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), color, 3);
//		for (i =0; i<4; i++){
//			cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i+4), color, 3);
//		}
		
		//draw the line that reflects the orientation. It indicates the bottom side of the pattern
//		cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), cvScalar(80,255,80), 3);
	//	Mat other_img;
		Mat small_img = imread("1.jpg", CV_32FC1);
		Mat resized;
		resize(small_img,resized,Size(model2ImagePts.at(1).x-model2ImagePts.at(0).x,model2ImagePts.at(3).y-model2ImagePts.at(0).y));
	
	//	if (frame.empty()) 
	//	{
	
	
	//	}
	//	else
	//	{
		//	cvtColor( resized, other_img, CV_RGB2BGR );
		//	cout<<"0";
		//	cout<<model2ImagePts.at(0)<<endl;

	//		cout<<"1";
	//		cout<<model2ImagePts.at(1)<<endl;

	//		cout<<"3";
	//		cout<<model2ImagePts.at(3)<<endl;


	//	float h= model2ImagePts.at(3).y-model2ImagePts.at(0).y;
	//	float w = model2ImagePts.at(1).x-model2ImagePts.at(0).x;
	//	float x1 = (model2ImagePts.at(0).x+model2ImagePts.at(2).x)/2;
	//	float y1 = (model2ImagePts.at(0).y+model2ImagePts.at(2).y)/2;
		
//		CvRect myrect=cvRect(x1-w/2,y1-h/2,w,h);

			//other_img.copyTo(frame(cv::Rect(model2ImagePts.at(0).x,model2ImagePts.at(0).y,resized.cols,resized.rows)));

		//other_img.copyTo(frame(cvRect(x1-w/2,y1-h/2,w,h)));   //<---MAAAIN
		
		std::vector<cv::Point2f> imgCorners ;
		imgCorners.push_back(Point2f(0.0,0.0));
		imgCorners.push_back(Point2f(0.0,resized.rows));
		imgCorners.push_back(Point2f(resized.cols,resized.rows));
		imgCorners.push_back(Point2f(resized.cols,0.0));

		std::vector<cv::Point> warpedCorners ;
		warpedCorners.push_back(model2ImagePts.at(0));
			warpedCorners.push_back(model2ImagePts.at(1));
			warpedCorners.push_back(model2ImagePts.at(2));
			warpedCorners.push_back(model2ImagePts.at(3));
		
			std::vector<cv::Point2f> warpedCorners2 ;
		
			warpedCorners2.push_back(model2ImagePts.at(0));
			warpedCorners2.push_back(model2ImagePts.at(1));
			warpedCorners2.push_back(model2ImagePts.at(2));
			warpedCorners2.push_back(model2ImagePts.at(3));
			

	Mat T = getPerspectiveTransform(imgCorners,warpedCorners2);

		cv::Mat warpedImg;
		cv::warpPerspective(resized, warpedImg, T, frame.size()); 
		cv::fillConvexPoly(frame, warpedCorners, cv::Scalar::all(0), CV_AA);
		cv::bitwise_or(warpedImg, frame, frame);

	
		//	std::vector<cv::Point> contour;
		//	std::vector<std::vector<cv::Point> > contourVec;
			
	//		contour.push_back(model2ImagePts.at(0));
	//		contour.push_back(model2ImagePts.at(1));
	//		contour.push_back(model2ImagePts.at(2));
	//		contour.push_back(model2ImagePts.at(3));
	//		contourVec.push_back(contour);
			
	//	    drawContours(frame,contourVec,0,Scalar(255,0,0),CV_FILLED,8); //Replace i with 0 for index. 


		
			cv::line(frame, model2ImagePts.at(4), model2ImagePts.at((5)), color, 3);
				cv::line(frame, model2ImagePts.at(5), model2ImagePts.at((6)), color, 3);
					cv::line(frame, model2ImagePts.at(6), model2ImagePts.at((7)), color, 3);
						cv::line(frame, model2ImagePts.at(4), model2ImagePts.at(7), color, 3);	
					
			
		//}
		model2ImagePts.clear();
	}

}