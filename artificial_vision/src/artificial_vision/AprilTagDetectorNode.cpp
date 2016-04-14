#include <ros/ros.h>
#include "opencv2/opencv.hpp" 
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag36h11.h"

using namespace ros;
using namespace std;
using namespace cv;
using namespace AprilTags;

int main(int argc, char **argv)
{

	init(argc, argv, "tag_detector");

	VideoCapture video("/home/viki/apriltag-test.mp4");
	if(!video.isOpened())
	{
		cout << "Cannot open file!" << endl;
		return -1;
	}

	TagDetector detector(tagCodes16h5);
	while(true)
	{
		Mat frame, frame_downscaled, frame_dg;
		video >> frame;

		if(frame.empty())
			break;

		resize(frame, frame_downscaled, Size(), 0.25,0.25);
		cvtColor(frame_downscaled, frame_dg, CV_BGR2GRAY);

		std::vector<TagDetection> tags_detected;
		tags_detected = detector.extractTags(frame_dg);
		

		for(int i=0; i<tags_detected.size(); i++)
		{
			cout << tags_detected[i].good << endl;
			cout << tags_detected[i].obsCode << endl;
			tags_detected[i].draw(frame_dg);	
		}

		imshow("AprilTag test", frame_dg);
		waitKey(20);
	}

	return 0;
}
