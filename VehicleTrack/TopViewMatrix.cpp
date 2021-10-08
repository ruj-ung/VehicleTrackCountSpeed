#include "TopViewMatrix.h"

TopViewMatrix::TopViewMatrix()
{
	H = NULL;
	orientation = 0;
}

TopViewMatrix::~TopViewMatrix()
{
}

void TopViewMatrix::init(vector<Point2f> src, vector<Point2f> dst)
{
	// compute transformation matrix
	H = getPerspectiveTransform(src, dst);
	// set orientation
	orientation = (src[0].x <= src[3].x) ? 0 : 1;
}

vector<Point2f> TopViewMatrix::transform(vector<Point2f> src)
{
	vector<Point2f> dst;
	perspectiveTransform(src, dst, H);
	return dst;
}

vector<Point2f> TopViewMatrix::transform(vector<Point> src)
{
	vector<Point2f> dst, s;
	std::transform(src.begin(), src.end(),
		std::back_inserter(s),
		[](const Point& p) { return (Point2f)p; });
	perspectiveTransform(s, dst, H);
	return dst;
}

Point2f TopViewMatrix::transformPoint(Point2f pt)
{
	vector<Point2f> src, dst;
	src.push_back(pt);
	dst = transform(src);	
	return dst[0];
}

Point2f TopViewMatrix::transformCOB(bbox box)
{
	Point2f ct = Point2f(box.x+box.w/2, box.y+box.h/2);
	Point2f pt1 = (orientation == 0)?Point2f(box.x, ct.y):Point2f(box.x+box.w,ct.y);
	Point2f pt2 = Point2f(ct.x, box.y+box.h);
	vector<Point2f> src, dst;
	src.push_back(pt1);
	src.push_back(pt2);
	dst = transform(src);
	return (dst[0]+dst[1])/2;
}

Point2f TopViewMatrix::transformCOB(bbox_t box)
{
	Point2f ct = Point2f(box.x + box.w / 2, box.y + box.h / 2);
	Point2f pt1 = Point2f(box.x, ct.y);
	Point2f pt2 = Point2f(ct.x, box.y + box.h);
	vector<Point2f> src, dst;
	src.push_back(pt1);
	src.push_back(pt2);
	dst = transform(src);
	return (dst[0] + dst[1]) / 2;
}

bool TopViewMatrix::IsEmpty()
{
	return H.empty() ? true : false;
}
