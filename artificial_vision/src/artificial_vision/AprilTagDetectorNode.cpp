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

	VideoCapture video("/home/viki/apriltag-tracking-test.mp4");
	
	int ex = static_cast<int>(video.get(CV_CAP_PROP_FOURCC));
	Size S = Size((int) video.get(CV_CAP_PROP_FRAME_WIDTH), (int) video.get(CV_CAP_PROP_FRAME_HEIGHT)); 

	VideoWriter outputVideo("/home/viki/apriltag-tracking-test-PROCESED.mp4", ex, video.get(CV_CAP_PROP_FPS), S, true); 


	if(!video.isOpened() || !outputVideo.isOpened())
	{
		cout << "Cannot open output or input file!" << endl;
		return -1;
	}

	vector<cv::Ptr<TagFamily> > gTagFamilies;
	TagFamilyFactory::create("0", gTagFamilies);
	TagDetector detector(gTagFamilies);

	while(true)
	{
		Mat frame;
		video >> frame;

		if(frame.empty())
			break;

		vector<TagDetection> tags_detected;
		
		detector.process(frame, tags_detected);
	
		for(int i=0; i<tags_detected.size() && tags_detected[i].id == 2; i++)
			tags_detected[i].draw(frame);	

		outputVideo << frame;

		imshow("AprilTag test", frame);
		waitKey(1);
	}

	return 0;
}
