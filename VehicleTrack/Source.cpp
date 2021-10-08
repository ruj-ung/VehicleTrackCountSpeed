// VehicleCount.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <windows.h>

#define OPENCV
//#define MYWEIGHT
//#define SHOWCT
#define SHOWTRAJECTORY

#include "../Lib/yolo_v2_class.hpp"	// imported functions from DLL

#include <opencv2/opencv.hpp>			// C++
#include <boost/circular_buffer.hpp>

using namespace cv;

#define buffer 150

typedef boost::circular_buffer<Point> circular_buffer;
struct roi_object{
	unsigned int track_id;        
	boost::circular_buffer<Point> pts{ buffer };
};

const char* winName = "Vehicle Speed";
void onMouse(int event, int x, int y, int f, void*);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void showImage();
void setting();
bool IsAbove(Point p1, Point p2, Point p);
void GetStatus(Point p1, Point p2, Point p, float &nStatus);
void setting1();
void  setDistance();

static Point P1(50, 370);
static Point P2(820, 350);
Point p1(50,370);
Point p2(820, 350);

static Rect cropRect(P1, P2);
char imgName[45];
bool clicked = false;
bool Red_ON = false;
bool isFirstTime = true;

Mat src, img, ROI;
std::vector<std::pair<bbox_t, float>> previous;
std::vector<roi_object> tracks;
int  nCount = 0;
int nCountMC = 0;
int nCountPickup = 0;
int nCountTruck = 0;
int nCountBus = 0;
int nCountVan = 0;
int rows, cols;

// Globals
bool finished = false;
std::vector<Point> vertices;
Rect b;
double distance = 25.0;
double speed;
std::vector<double> speeds;

void SetTransparentColor(cv::Mat& img, std::vector<std::vector<cv::Point>> & roi, cv::Scalar color, double alpha, cv::Mat& out)
{
	cv::Mat layer;

	img.copyTo(layer);

	cv::fillPoly(layer, roi, color);

	cv::addWeighted(layer, alpha, img, 1 - alpha, 0, out);
}
void show_topview(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<Point> vts)
{
	for (auto& i : result_vec)
	{

	}

}

void draw_trackpoints(cv::Mat mat_img, std::vector<bbox_t> result_vec)
{
	for (auto& i : result_vec)
	{
		for (auto& j : tracks)
		{
			if (i.track_id == j.track_id)
			{
				for (int n = 1; n < j.pts.size(); n++)
				{
					//if (pts[i - 1] == NULL || pts[i] == NULL)
					//	continue;

					//otherwise, compute the thickness of the line and
					//draw the connecting lines
					int thickness = 2; // sqrt(30) / float(n + 1) * 2.0;
					line(src, j.pts[n - 1], j.pts[n], Scalar(255, 255, 255), thickness);
				}

			}
		}
	}

}
void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, unsigned int wait_msec = 0) {
	std::vector<std::pair<bbox_t, float>> current;
	cv::Scalar color(60, 160, 260);
	for (auto& i : result_vec) 
	{
#ifndef MYWEIGHT
		if (i.obj_id != 2 && i.obj_id != 3  ) continue; // Car coco
		//if (i.obj_id != 13 && i.obj_id != 6) continue; // Car voc
		//if (i.obj_id != 3771) continue; // Car 9k
#endif // !MYWEIGHT


		cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 3);
		Point p(i.x + i.w / 2, i.y + i.h/2);
#ifdef SHOWCT
		circle(mat_img, p, 3, Scalar(255, 255, 255), CV_FILLED);
#endif // SHOWCT

		float status;
		GetStatus(Point(P1.x, rows-P1.y), Point(P2.x, rows-P2.y), Point(p.x, rows-p.y), status);
		if (status > 0.0)
		{
			for (auto& j : previous)
			{
				if (i.track_id == j.first.track_id && j.second < 0.0)
				{
					//std::cout << "Found" << std::endl;
					if (status - j.second > 20) continue;
#ifndef MYWEIGHT
					if(i.obj_id == 2) nCount++;
					if (i.obj_id == 3) nCountMC++;
#else
					if (i.obj_id == 0) nCountMC++;
					if (i.obj_id == 1) nCount++;
					if (i.obj_id == 2) nCountPickup++;
					if (i.obj_id == 3) nCountTruck++;
					if (i.obj_id == 4) nCountBus++;
					if (i.obj_id == 5) nCountVan++;
#endif // !MYWEIGHT
					//std::cout << nCount << "==============" << i.track_id <<  std::endl;
					//std::cout << i.track_id << " " << status << std::endl;
					//char c = cvWaitKey(0);

				}
			}
		}
		current.push_back(std::make_pair(i, status));
		//std::cout << i.track_id << " " << status << std::endl;
		if (obj_names.size() > i.obj_id)
			putText(mat_img, obj_names[i.obj_id]+std::to_string(i.track_id), cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_PLAIN, 1.0, color, 2);
		//if (i.track_id > 0)
		//	putText(mat_img, std::to_string(i.track_id), cv::Point2f(i.x + 5, i.y + 15), cv::FONT_HERSHEY_PLAIN, 1.0, color, 2);
	}
	previous = current;

	//// Draw transparent box
	//cv::Mat overlay;
	//double alpha = 0.5;

	////mat_img.copyTo(overlay);
	////cv::rectangle(overlay, cv::Rect(0, 0, 150, 70), cv::Scalar(255, 0, 0), -1);
	////cv::addWeighted(overlay, alpha, mat_img, 1 - alpha, 0, mat_img);
	//std::vector<std::vector<Point>> pts{ {Point(0,0),Point(150,0),Point(150,140),Point(0,140)} };
	//SetTransparentColor(mat_img, pts, cv::Scalar(255, 0, 0), alpha, mat_img);
	//
	//putText(mat_img, "Car    : "+std::to_string(nCount), Point(10,20), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	//putText(mat_img, "MC     : " + std::to_string(nCountMC), Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	//putText(mat_img, "Pickup : " + std::to_string(nCountPickup), Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	//putText(mat_img, "Truck  : " + std::to_string(nCountTruck), Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	//putText(mat_img, "Bus    : " + std::to_string(nCountBus), Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	//putText(mat_img, "Van    : " + std::to_string(nCountVan), Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);


	//line(mat_img, P1, P2, Scalar(255, 255, 255), 1, 8);

}
void ShowSpeed(cv::Mat imgFrame)
{
	cv::Scalar color(255, 0, 0);
	putText(imgFrame, "Speed: " + std::to_string(speed) +  " Km/h", Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.2, color, 2.5);
}

void ShowTrafficInfo(cv::Mat imgFrame)
{
	// Draw transparent box
	cv::Mat overlay;
	double alpha = 0.5;
	cv::Scalar color(60, 160, 260);

	//mat_img.copyTo(overlay);
	//cv::rectangle(overlay, cv::Rect(0, 0, 150, 70), cv::Scalar(255, 0, 0), -1);
	//cv::addWeighted(overlay, alpha, mat_img, 1 - alpha, 0, mat_img);
	std::vector<std::vector<Point>> pts{ {Point(0,0),Point(150,0),Point(150,140),Point(0,140)} };
	SetTransparentColor(imgFrame, pts, cv::Scalar(255, 0, 0), alpha, imgFrame);

	putText(imgFrame, "Car    : " + std::to_string(nCount), Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	putText(imgFrame, "MC     : " + std::to_string(nCountMC), Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	putText(imgFrame, "Pickup : " + std::to_string(nCountPickup), Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	putText(imgFrame, "Truck  : " + std::to_string(nCountTruck), Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	putText(imgFrame, "Bus    : " + std::to_string(nCountBus), Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);
	putText(imgFrame, "Van    : " + std::to_string(nCountVan), Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1.2, color, 1.5);

}
void show_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names) {
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
		std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y
			<< ", w = " << i.w << ", h = " << i.h
			<< std::setprecision(3) << ", prob = " << i.prob << ", track_id = " << i.track_id << std::endl;
	}
	std::cout << std::endl;
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
	std::ifstream file(filename);
	std::vector<std::string> file_lines;
	if (!file.is_open()) return file_lines;
	for (std::string line; file >> line;) file_lines.push_back(line);
	std::cout << "object names loaded from " << filename + "\n";
	return file_lines;
}


void readconfig()
{
	Point pt;
	std::ifstream fi("wow.cfg");
	fi >> P1.x >> P1.y >> P2.x >> P2.y;
	while (fi >> pt.x >> pt.y)
	{
		vertices.push_back(pt);
	}
	fi.close();
}

void saveconfig()
{
	std::ofstream fo("wow.cfg");
	if (fo.is_open())
	{
		fo << P1.x << std::endl;
		fo << P1.y << std::endl;
		fo << P2.x << std::endl;
		fo << P2.y << std::endl;
		for (auto& i : vertices)
		{
			fo << i.x << std::endl;
			fo << i.y << std::endl;
		}
		fo.close();
		b = boundingRect(vertices);
	}
	else
		std::cout << "Error when save config" << std::endl;
}
int main()
{
	std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
	Mat image;

	readconfig();
	b = boundingRect(vertices);
	//std::cout << "Enter distance (in metre): ";
	//std::cin >> distance;

#ifndef MYWEIGHT
	auto obj_names = objects_names_from_file("coco.names");
	Detector detector("yolov4.cfg", "yolov4.weights");
	//Detector detector("../yolov3.cfg", "../yolov3.weights");
	//Detector detector("../yolov2.cfg", "../yolov2.weights");
#else
	auto obj_names = objects_names_from_file("../classes.txt");
	Detector detector("../v.cfg", "../v1300.weights");
#endif // !MYWEIGHT

	char fname[MAX_PATH];

	OPENFILENAME ofn;
	ZeroMemory(&fname, sizeof(fname));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;  // If you have a window to center over, put its HANDLE here
	ofn.lpstrFilter = "Video Files\0*.mp4;*.mov\0Any File\0*.*\0";
	ofn.lpstrFile = fname;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = "Select a File, yo!";
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	if (GetOpenFileNameA(&ofn))
	{
		std::cout << "Loading the file: " << fname << std::endl;
	}

	cv::VideoCapture cap(fname);
	double fps = cap.get(CAP_PROP_FPS);

	namedWindow(winName, 0);
	//namedWindow("Topp view", 0);
	int nSkip = 0;
	std::vector<int> previous_ids;

	while (1)
	{
		cap >> src;
		if (src.empty()) break;
		cv::resize(src, src, Size(0, 0), 0.5, 0.5, 1);
		rows = src.size().height;
		cols = src.size().width;
		std::vector<bbox_t> result_vec = detector.detect(src, 0.2);	// Detect all objects in frame
		//std::cout << result_vec.size();


		// Pick only vehicles
		std::vector<bbox_t> temp_vec;
		for (auto& o : result_vec)
		{
			double at = pointPolygonTest(vertices, Point(o.x + o.w / 2, o.y + o.h / 2), false);
			if(at > 0)
			//if(b.contains(Point(o.x+o.w/2, o.y+o.h/2)))
#ifndef MYWEIGHT			
			if (o.obj_id == 2 || o.obj_id == 3 || o.obj_id == 5 || o.obj_id == 7)
#endif
				temp_vec.push_back(o);
		}

		// Tracking and speed calculation
		result_vec = detector.tracking_id(temp_vec, true, 20);
		//std::cout << "result_vec size = " << result_vec.size() << std::endl;
		std::set<int> temp; // current_ids;
		for (bbox_t const &o : result_vec)
		{
			temp.insert(o.track_id);
		}
		std::vector<int> current_ids(temp.begin(), temp.end());

		if (current_ids.size() < previous_ids.size())
		{
			std::cout << "Trig" << std::endl;

			// Find difference (missing element)
			std::vector<int> difference;
			std::set_difference(
				previous_ids.begin(), previous_ids.end(),
				current_ids.begin(), current_ids.end(),
				std::back_inserter(difference)
			);
			//std::cout << difference[0] << std::endl;
			for (auto const& id : difference)
			{
				roi_object track;
				std::vector<roi_object> temp;
				int i;
				for ( i = 0; i < tracks.size(); i++)
				{
					track = tracks[i];
					if (track.track_id == id)
					{
						// Calculate annd show speed
						speed = (distance/1000) / ((track.pts.size()/fps)/3600);
						std::cout << "#" << track.track_id << " speed = " << speed << "km/h" << std::endl;
						speeds.push_back(speed);
						break;
					}
					else
					{
						temp.push_back(track);
					}
				}
				//tracks = temp;				
			}
		}



		previous_ids = current_ids;

		for (auto& i : result_vec)
		{
			bool bExist = false;
			Point p(i.x + i.w / 2, i.y + i.h / 2);
			for (auto& j : tracks)
			{
				if (i.track_id == j.track_id) //add center point to tracking data
				{
					bExist = true; 
					float dist = norm(j.pts[0]-p);
					if(dist < 80)
						j.pts.push_front(p);
					break;
				}
			}
			if (!bExist)  // Add new track id
			{
				roi_object t;
				t.track_id = i.track_id;
				t.pts.push_front(p);
				tracks.push_back(t);
			}
			//std::cout << i.track_id << " ";

		}
		//std::cout << std::endl;
		//std::cout << "Tracks size =   " << tracks.size() << std::endl;
		draw_boxes(src, result_vec, obj_names);
		draw_trackpoints(src, result_vec);
		show_topview(src, result_vec, vertices);
		//cv::rectangle(src, b, cv::Scalar(255, 0, 0), 3);          // Bounding Rectangle of ROI
		//show_result(result_vec, obj_names);
		std::vector<std::vector<Point>> pts{ vertices };
		SetTransparentColor(src, pts, Scalar(0, 0, 255), 0.5, src);  // Draw ROI
		//ShowTrafficInfo(src);
		ShowSpeed(src);

		imshow(winName, src);

		char c = cvWaitKey(1);
		if (c == 's' || c == 'S')
			setting1();
		if (c == 'd' || c == 'D')
			setting();
		if (c == 'r' || c == 'R')
			setDistance();
		//if (c == 32)

		if (c == 27) break;
	}


	cvDestroyAllWindows();
	//std::cout << "Total car = " << nCount << std::endl;
	//std::cout << "Total mc = " << nCountMC << std::endl;
	//std::cout << "Total Pickup = " << nCountPickup << std::endl;
	//std::cout << "Total Truck = " << nCountTruck << std::endl;
	cvWaitKey(0);
}

void  setDistance()
{
	std::cout << "Currennt distance is " << distance << std::endl;
	std::cout << "Enter new distance (in metre): ";
	std::cin >> distance;
}
void setting()
{
	std::cout << "Setting ROI...." << std::endl;
	finished = false;
	vertices.clear();
	setMouseCallback(winName, CallBackFunc, nullptr);
	while (!finished) {
		imshow(winName, src);
		waitKey(50);
	}
	saveconfig();
}

void setting1()
{
	std::cout << "Setting line...." << std::endl;
	setMouseCallback(winName, onMouse, NULL);
	while (true)
	{
		char c = cvWaitKey(33);
		//if (c == 27)
		if (c == 's' || c == 'S')
		{
			setMouseCallback(winName, NULL, NULL);
			break;
		}
	}
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_RBUTTONDOWN) {
		std::cout << "Right mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (vertices.size() < 2) {
			std::cout << "You need a minimum of three points!" << std::endl;
			return;
		}
		// Close polygon
		line(src, vertices[vertices.size() - 1], vertices[0], Scalar(0, 0, 0));
		imshow(winName, src);
		std::vector<std::vector<Point>> pts{ vertices };
		SetTransparentColor(src, pts, Scalar(0, 0, 255), 0.5, src);
		finished = true;
		return;
	}
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "Left mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (vertices.size() == 0) {
			// First click - just draw point
			src.at<Vec3b>(x, y) = Vec3b(255, 0, 0);
		}
		else {
			// Second, or later click, draw line to previous vertex
			line(src, Point(x, y), vertices[vertices.size() - 1], Scalar(0, 0, 0));
		}
		vertices.push_back(Point(x, y));
		return;
	}
}

void onMouse(int event, int x, int y, int f, void*) {


	switch (event) {

	case  CV_EVENT_LBUTTONDOWN:
		clicked = true;
		isFirstTime = false;
		P1.x = x;
		P1.y = y;
		P2.x = x;
		P2.y = y;
		break;

	case  CV_EVENT_LBUTTONUP:
		P2.x = x;
		P2.y = y;
		clicked = false;
		break;

	case  CV_EVENT_MOUSEMOVE:
		if (clicked) {
			P2.x = x;
			P2.y = y;
		}
		break;

	default:   break;


	}


	if (clicked) {
		if (P1.x > P2.x) {
			cropRect.x = P2.x;
			cropRect.width = P1.x - P2.x;
		}
		else {
			cropRect.x = P1.x;
			cropRect.width = P2.x - P1.x;
		}

		if (P1.y > P2.y) {
			cropRect.y = P2.y;
			cropRect.height = P1.y - P2.y;
		}
		else {
			cropRect.y = P1.y;
			cropRect.height = P2.y - P1.y;
		}

	}
	saveconfig();
	showImage();
}

void showImage()
{
	img = src.clone();
	//checkBoundary();
	if (cropRect.width > 0 && cropRect.height > 0)
	{
		ROI = src(cropRect);
		//isFirstTime = false;
		//imshow("cropped", ROI);
	}

	if (!isFirstTime)
	{
		//rectangle(img, cropRect, Scalar(0, 255, 0), 1, 8, 0);
		line(img, P1, P2, Scalar(255, 255, 255), 1, 8);
		if (cropRect.width > 0 && cropRect.height > 0)
		{
			sprintf(imgName, "%d, %d", P1.x, P1.y);
			putText(img, imgName, P1, 1, 0.75, Scalar::all(255));
			sprintf(imgName, "%d, %d", P2.x, P2.y);
			putText(img, imgName, P2, 1, 0.75, Scalar::all(255));
		}

	}
	imshow(winName, img);
}

bool IsAbove(Point p1, Point p2, Point p)
{
	float m = float(p2.y - p1.y) / float(p2.x - p1.x);
	float b = p1.y - m * p1.x;
	float yl = m * p.x + b;
	float delta = yl - p.y;
	return (delta < 0.0) ? true : false;
}

void GetStatus(Point p1, Point p2, Point p, float& nStatus)
{
	float m = float(p2.y - p1.y) / float(p2.x - p1.x);
	float b = p1.y - m * p1.x;
	float yl = m * p.x + b;
	float delta = p.y - yl;
	nStatus = delta;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
