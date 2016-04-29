#include <ros/ros.h>

#include "opencv2/opencv.hpp"

#include "TagDetector.hpp"
#include "TagFamilyFactory.hpp"

using namespace ros;
using namespace std;
using namespace cv;
using namespace april::tag;

int main(int argc, char **argv)
{
	init(argc, argv, "tag_detector");

	VideoCapture video("/home/ubuntu/apriltag-tracking-test.mp4");
	
	int ex = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));
	Size S = Size((int) (video.get(CV_CAP_PROP_FRAME_WIDTH)/3), (int) (video.get(CV_CAP_PROP_FRAME_HEIGHT)/3)); 

	cout << S << endl;

	VideoWriter outputVideo("/home/ubuntu/apriltag-tracking-test-PROCESED.avi", CV_FOURCC('X','V','I','D'), video.get(CV_CAP_PROP_FPS), S, true); 


	if(!video.isOpened())
	{
		cout << "Cannot open  input file!" << endl;
		return -1;
	}

	if(!outputVideo.isOpened())
	{
		cout << "Cannot open output file!" << endl;
		return -1;
	}

	vector<cv::Ptr<TagFamily> > gTagFamilies;
	TagFamilyFactory::create("0", gTagFamilies);
	TagDetector detector(gTagFamilies);

	while(true)
	{
		Mat frame_ori, frame;
		video >> frame_ori;

		cout << "Frame ori" << frame_ori.size().width << "x" << frame_ori.size().height << endl;

		resize(frame_ori, frame, Size(), 1.0/3.0, 1.0/3.0);

		cout << "Frame" << frame.size().width << "x" << frame.size().height << endl;

		if(frame.empty())
			break;

		vector<TagDetection> tags_detected;
		
		detector.process(frame, tags_detected);
	
		for(int i=0; i<tags_detected.size() && tags_detected[i].id == 2; i++)
			tags_detected[i].draw(frame);	

		outputVideo << frame;

	}

	return 0;
}
