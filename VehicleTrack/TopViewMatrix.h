#pragma once
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct bbox {
	int x, y, w, h;
};

#ifndef YOLO_V2_CLASS_HPP
struct bbox_t {
	unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;                    // confidence - probability that the object was found correctly
	unsigned int obj_id;           // class of object - from range [0, classes-1]
	unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
	unsigned int frames_counter;   // counter of frames on which the object was detected
	float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
};
#endif
class TopViewMatrix
{
public:

	TopViewMatrix();
	~TopViewMatrix();
	void init(vector<Point2f>, vector<Point2f>);
	vector<Point2f> transform(vector<Point2f> src);
	vector<Point2f> transform(vector<Point> src);
	Point2f transformPoint(Point2f pt);
	Point2f transformCOB(bbox box);
	Point2f transformCOB(bbox_t box);
	bool IsEmpty();
private:
	Mat H;
	int orientation;
};

