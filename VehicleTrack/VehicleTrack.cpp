// VehicleCount.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
// 22/07/2021 Version 1.3 released
// - Dock/Float the top view
// - On/Off Configuration Information
// - Write log to csv file. 
#include <iostream>
#include <fstream>
#include <windows.h> 

#define OPENCV
#define MYWEIGHT
//#define SHOWCT
#define SHOWTRAJECTORY

#include "../Lib/yolo_v2_class.hpp"	// imported functions from DLL

#include <opencv2/opencv.hpp>			// C++
#include <boost/circular_buffer.hpp>
#include "TopViewMatrix.h"

using namespace cv;

#define buffer 35
#define trajectOffset 6

typedef boost::circular_buffer<Point> circular_buffer;
struct roi_object{
	unsigned int track_id;
	boost::circular_buffer<unsigned int> frameNums{ buffer };
	circular_buffer pts{ buffer };
	boost::circular_buffer<Point2f> mapped_pts{ buffer };
};

const char* winName = "Vehicle Speed Version 1.4 By Rujchai@EnKKU";
void onMouse(int event, int x, int y, int f, void*);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void CallBackFunc2(int event, int x, int y, int flags, void* userdata);
void showImage();
void settingCalibrateROI();
void settingDisplayROI();
bool IsAbove(Point p1, Point p2, Point p);
void GetStatus(Point p1, Point p2, Point p, float &nStatus);
void DrawHelp(cv::Mat src);
void setting1();
void  setDistance();
void  setPixelPerMetre();
void saveconfig();
cv::Scalar id2Color(int id);
bool IsClockwise(vector<Point> vertrices);
cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad);
cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad);

static Point P1(50, 50);
static Point P2(100, 100);

static Rect cropRect(P1, P2);
char imgName[45];
bool clicked = false;
bool isFirstTime = true;

Mat src, img, ROI;
std::vector<std::pair<bbox_t, float>> previous;
std::vector<roi_object> tracks;
int  nCountCar = 0;
int nCountMC = 0;
int nCountPickup = 0;
int nCountTruck = 0;
int nCountBus = 0;
int nCountVan = 0;

// Globals
double fps;
uint max_id = 0;
int rows = 600, cols = 1000;
bool bPause = false;
bool bShowTrajectory = false;
bool bShowCalROI = false;
bool bShowDispROI = false;
bool bDock = true;
bool bShowInfo = true;
bool bShowCounter = true;
bool bShowHelp = false;
std::string path;
std::string configPath;
bool finished = false;
std::vector<Point> vertices{Point(364,208), Point(512,207), Point(813,404), Point(395, 443)};
std::vector<Point> roi_vertices{ Point(364,208), Point(512,207), Point(813,404), Point(395, 443) };
std::vector<Point2f> transformed_roi_vertices;
double g_distance = 25.0;
double cenperpix = 3.5;
double speed;
double pointsize = 18.0;
std::vector<double> speeds;
TopViewMatrix M;
Point2f ct;

double get_clockwise_angle(const Point2f& p, const Point2f& ct)
{
	double angle = 0.0;
	/*calculate angle and return it*/
	cv::Size size(p - ct);
	angle = -atan2(size.height, size.width);
	return angle;
}
bool compare_points(const Point2f& a, const Point2f& b)
{
	return (get_clockwise_angle(a, ct) > get_clockwise_angle(b, ct));
}
void SetTransparentColor(cv::Mat& img, std::vector<std::vector<cv::Point>> & roi, cv::Scalar color, double alpha, cv::Mat& out)
{
	cv::Mat layer;

	img.copyTo(layer);

	cv::fillPoly(layer, roi, color);

	cv::addWeighted(layer, alpha, img, 1 - alpha, 0, out);
}

void draw_topview(cv::Mat topimg, std::vector<bbox_t> result_vec, std::vector<Point> vts)
{
	if (!IsClockwise(vts)) return;
	int thickness = 1; 
	if (!bDock)
	{
		for (auto& i : transformed_roi_vertices)
		{
			circle(topimg, i, 5, Scalar(0, 0, 255), -1);
		}
		line(topimg, transformed_roi_vertices[transformed_roi_vertices.size()-1], transformed_roi_vertices[0], Scalar(255, 255, 255), thickness);
		for (int n = 1; n < transformed_roi_vertices.size(); n++)
		{
			line(topimg, transformed_roi_vertices[n - 1], transformed_roi_vertices[n], Scalar(255, 255, 255), thickness);
		}
	}

	for (auto& i : result_vec)
	{
		Point2f pt = M.transformCOB(i);
		circle(topimg, pt, pointsize, id2Color(i.obj_id), -1);
	}

}

void draw_trackpoints(cv::Mat mat_img, std::vector<bbox_t> result_vec)
{
	for (auto& i : result_vec)
	{
		cv::Scalar color = obj_id_to_color(i.obj_id); // (153, 255, 255);
		cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), id2Color(i.obj_id), 3);
		for (auto& j : tracks)
		{
			if (i.track_id == j.track_id)
			{
				int framediff = j.frameNums[0] - j.frameNums[1];
				if (framediff > 0)
				{
					if (bShowTrajectory)
					{
						if (j.pts.size() <= trajectOffset) break;
						for (int n = 1; n < j.pts.size()-trajectOffset; n++)
						{
							//if (pts[i - 1] == NULL || pts[i] == NULL)
							//	continue;

							//otherwise, compute the thickness of the line and
							//draw the connecting lines
							int thickness = 2; // sqrt(30) / float(n + 1) * 2.0;
							
							line(src, j.pts[n - 1], j.pts[n], id2Color(i.obj_id), thickness);

						}
					}

					//if (j.mapped_pts.size() >= 3)
					{
						// Calculate instant speed
						Point2f pt0 = j.mapped_pts[0];
						Point2f pt1 = j.mapped_pts[1];
						double dist = norm(pt1 - pt0);
						double v = (dist*cenperpix/100)/ framediff * fps * 3600 / 1000;
						char str[80];
						sprintf(str, "%.0f km/h", v);
						putText(src, str /*std::to_string(v)*/, cv::Point2f(i.x+i.w/2-30, i.y - 7), cv::FONT_HERSHEY_PLAIN, 1.0, Scalar(255,255,255), 2);
					}
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
					if(i.obj_id == 2) nCountCar++;
					if (i.obj_id == 3) nCountMC++;
#else
					if (i.obj_id == 0)  nCountMC++;
					if (i.obj_id == 1) nCountCar++;
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
		
		// Show Text of the box
		//if (obj_names.size() > i.obj_id)
			//putText(mat_img, obj_names[i.obj_id]+std::to_string(i.track_id), cv::Point2f(i.x, i.y - 10), cv::FONT_HERSHEY_PLAIN, 1.0, color, 2);
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

cv::Scalar id2Color(int id)
{
	cv::Scalar color = Scalar(255, 255, 255);

#ifndef MYWEIGHT
	if (id == 2) color = obj_id_to_color(4);
	if (id == 3) color = obj_id_to_color(0);
	if (id == 5) color = obj_id_to_color(1);
	if (id == 7) color = obj_id_to_color(2);

#else
	color = obj_id_to_color(id);
	if (id == 1) color = obj_id_to_color(4);
	if (id == 4) color = obj_id_to_color(1);
#endif



	return color;
}

void DrawTrafficInfo(cv::Mat imgFrame, vector<bbox_t> res_vec)
{
	// Draw transparent box
	cv::Mat overlay;
	double alpha = 0.5;
	cv::Scalar color(60, 160, 260);
	
	// Draw transparent background
	cv::Size size(150, 160);
	cv::Point tl = Point(cols - size.width, 0);
	std::vector<std::vector<Point>> pts{ {Point(tl.x,tl.y),Point(tl.x+size.width,0),Point(tl.x+size.width,tl.y+size.height),Point(tl.x,tl.y + size.height)} };
	SetTransparentColor(imgFrame, pts, cv::Scalar(255,0,0), alpha, imgFrame);

	// Draw text obj_id_to_color(i.obj_id)
	putText(imgFrame, "Counter", Point(tl.x + 10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, color, 2.0);
	putText(imgFrame, "MC      : " + std::to_string(nCountMC), Point(tl.x + 10, 40), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(0), 1.5);
	putText(imgFrame, "Car     : " + std::to_string(nCountCar), Point(tl.x + 10, 60), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(4), 1.5);
	putText(imgFrame, "Pickup  : " + std::to_string(nCountPickup), Point(tl.x + 10, 80), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(2), 1.5);
	putText(imgFrame, "Truck   : " + std::to_string(nCountTruck), Point(tl.x + 10, 100), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(3), 1.5);
	putText(imgFrame, "Bus     : " + std::to_string(nCountBus), Point(tl.x + 10, 120), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(1), 1.5);
	putText(imgFrame, "SUV/VAN: " + std::to_string(nCountVan), Point(tl.x + 10, 140), cv::FONT_HERSHEY_PLAIN, 1.2, obj_id_to_color(5), 1.5);
}

void DrawConfigInfo(cv::Mat imgFrame)
{
	// Draw transparent box
	cv::Mat overlay;
	double alpha = 0.5;
	cv::Scalar color(60, 160, 260);

	std::vector<std::vector<Point>> pts{ {Point(0,0),Point(200,0),Point(200,70),Point(0,70)} };
	SetTransparentColor(imgFrame, pts, cv::Scalar(255, 0, 0), alpha, imgFrame);


	putText(imgFrame, "** Config Info **" , Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, color, 2);
	putText(imgFrame, "Distance : " + std::to_string(g_distance), Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(imgFrame, "Cm/Pixel : " + std::to_string(cenperpix), Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
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
	configPath = path + ".cfg";
	std::ifstream fi(configPath);
	if (fi.is_open())
	{
		vertices.clear();
		roi_vertices.clear();
		fi >> g_distance >> P1.x >> P1.y >> P2.x >> P2.y;
		for(int n = 0; n < 4; n++)
		{
			fi >> pt.x >> pt.y;
			vertices.push_back(pt);
		}
		while (fi >> pt.x >> pt.y)
		{
			roi_vertices.push_back(pt);
		}
	}

	// Sort vertrices clockwise
	cv::Moments m = moments(vertices);
	ct = Point2f(m.m10 / m.m00, m.m01 / m.m00);
	std::sort(vertices.begin(), vertices.end(), compare_points);


	fi.close();
}

bool IsValidated()
{
	bool bValid = true;
	for (auto& v : transformed_roi_vertices)
	{
		if (v.x < 0 || v.y < 0 || v.x > cols - 1 || v.y > rows - 1)
			bValid = false;
 	}
	return bValid;
}
std::string getexepath()
{
	char res[MAX_PATH] = { 0 };
	return std::string(res, GetCurrentDirectory(MAX_PATH, res)); //std::string(result, GetModuleFileName(NULL, result, MAX_PATH));
}


void DoCalibrate(vector<Point> source)
{
	vector<Point2f> src, dst,s;
	std::transform(source.begin(), source.end(),
		std::back_inserter(src),
		[](const Point& p) { return (Point2f)p; });

	// transformed quadrangle
	dst.push_back(Point(src[0].x, src[0].y));
	dst.push_back(Point(src[1].x, src[0].y));
	dst.push_back(Point(src[1].x, max(src[2].y, src[3].y)));
	dst.push_back(Point(src[0].x, max(src[2].y, src[3].y)));
	cenperpix = (g_distance * 100.0) / (dst[2].y - dst[1].y);
	M.init(src, dst);

	transformed_roi_vertices = M.transform(roi_vertices);
}
void saveconfig()
{
	// Sort vertrices clockwise
	cv::Moments m = moments(vertices);
	ct = Point2f(m.m10 / m.m00, m.m01 / m.m00);
	std::sort(vertices.begin(), vertices.end(), compare_points);	

	configPath = path + ".cfg";
	std::ofstream fo(configPath);
 	if (fo.is_open())
	{
		fo << g_distance << std::endl;
		fo << P1.x << std::endl;
		fo << P1.y << std::endl;
		fo << P2.x << std::endl;
		fo << P2.y << std::endl;
		for (auto& i : vertices)
		{
			fo << i.x << std::endl;
			fo << i.y << std::endl;
		}
		for (auto& i : roi_vertices)
		{
			fo << i.x << std::endl;
			fo << i.y << std::endl;
		}		
		fo.close();
		DoCalibrate(vertices);
		if (!IsValidated())
		{
			std::cout << "***** ERROR: Display ROI is invalid. Please open the video and set its display ROI again." << std::endl;
		}	
	}
	else
		std::cout << "Error when save config" << std::endl;
}
bool IsClockwise(vector<Point> vertrices) {
	auto area = 0;
	for (int i = 0; i < vertices.size(); i++) {
		int j = (i + 1) % vertices.size();
		area += vertices[i].x * vertices[j].y;
		area -= vertices[j].x * vertices[i].y;
	}
	return area / 2 > 0;
}


void Init()
{
	
}

int main()
{
	std::cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
	Init();
	Mat image;
	std::string exepath = getexepath();
	configPath = exepath + "\\powwow.cfg";


#ifndef MYWEIGHT
	auto obj_names = objects_names_from_file(exepath + "\\coco.names");
	Detector detector(exepath+"\\yolov4.cfg", exepath+"\\yolov4.weights");
	//Detector detector("../yolov3.cfg", "../yolov3.weights");
	//Detector detector("../yolov2.cfg", "../yolov2.weights");
#else
	auto obj_names = objects_names_from_file(exepath +"\\classes.txt");
	//Detector detector(exepath + "\\v.cfg", exepath+"\\v1300.weights");
	Detector detector(exepath + "\\v4.cfg", exepath + "\\v4_2000.weights");
#endif // !MYWEIGHT

	//std::cout << obj_names.size() << std::endl;
	//std::cout << std::endl;
	//std::cout << "Help Info: c = Set calibration ROI, r = Set Display ROI, s = Set line, d = Set distance,\n"
	//	<< "           1 = On/Off trajectory, 2 = On/Off Calibration ROI, 3 = On/Off Display ROI, 4 = Dock/Float Top View,\n"
	//	<< "           5 = On/Off Config Info, + = increase point size, - = decrease point size, [space] = Pause (toggle)\n";
	std::cout << "Press H to show Help panel" << std::endl;
	char fname[MAX_PATH] = { 0 };
	OPENFILENAME ofn;
	ZeroMemory(&fname, sizeof(fname));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;  // If you have a window to center over, put its HANDLE here
	ofn.lpstrFilter = "Video Files\0*.mp4;*.mov\0Any File\0*.*\0";
	ofn.lpstrFile = fname;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = "Select a File";
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	if (GetOpenFileNameA(&ofn))
	{
		std::cout << "Loading the file: " << fname << std::endl;
	}

	// Set the log filename
	std::string fullVideoPath(fname);
	path = fullVideoPath.substr(0, fullVideoPath.find_last_of('.'));
	std::string logpath = path + ".csv";
	std::ofstream flog(logpath);
	if (flog.is_open())
	{
		flog << "Frame No,Vehicle ID,Vehicle Type,x,y,dx,dy,Speed" << std::endl;
	}

	// Check if video opened for the first time
	//if (boost::filesystem::exists(path + ".cfg"))
	ifstream file(path + ".cfg");
	if(!file)
		bShowCalROI = bShowDispROI =  true;
	else
		readconfig();

	// Calibrate the camera
	DoCalibrate(vertices);
	if (!IsValidated())
	{
		roi_vertices = vertices;
		transformed_roi_vertices = M.transform(roi_vertices);
	}

	std::atomic<int> current_fps_cap(0);
	bool const use_kalman_filter = true;   // true - for stationary camera
	track_kalman_t track_kalman;
	// Open video file
	cv::VideoCapture cap(fname);
	if (!cap.isOpened())
	{
		std::cout << "Video open failed" << std::endl;
	}
	fps = cap.get(CAP_PROP_FPS);

	namedWindow(winName, 0);
	int nSkip = 0;
	std::vector<int> previous_ids;
	unsigned int frameNum = 0;

	while (1)
	{
		cap >> src;
		if (src.empty()) break;
		frameNum++;
		//std::cout << "\nFrame No. " << frameNum << " ";
		//cv::resize(src, src, Size(0, 0), 0.5, 0.5, 1);
		resize(src, src, Size(1000, 600), INTER_LANCZOS4);

		//rows = src.size().height;
		//cols = src.size().width;
		std::vector<bbox_t> result_vec = detector.detect(src, 0.2);	// Detect all objects in frame
		//std::cout << "detectSize="  << result_vec.size() << " ";


		// Pick only vehicles
		std::vector<bbox_t> temp_vec;
		for (auto& o : result_vec)
		{
			double at = pointPolygonTest(roi_vertices, Point(o.x + o.w / 2, o.y + o.h / 2), false);
			if(at > 0 && o.prob > 0.6)
#ifndef MYWEIGHT			
			if (o.obj_id == 2 || o.obj_id == 3 || o.obj_id == 5 || o.obj_id == 7)
#endif
				temp_vec.push_back(o);
		}
		//std::cout << "filterSize=" << temp_vec.size() << " ";


		// Tracking and speed calculation
		// track ID by using kalman filter
		if (use_kalman_filter) {
				result_vec = track_kalman.correct(temp_vec);
				//result_vec = track_kalman.predict();
		}
		// track ID by using custom function
		else {
			int frame_story = std::max(5, current_fps_cap.load());
			result_vec = detector.tracking_id(temp_vec, true, frame_story, 40);
		}
		//std::cout << "trackSize=" << result_vec.size() << " ";
		std::set<int> temp; // current_ids;
		for (bbox_t const &o : result_vec)
		{
			temp.insert(o.track_id);
			if (o.track_id > max_id)
			{
				max_id = o.track_id;
#ifndef MYWEIGHT
				switch (o.obj_id)
				{
				case 2:
					nCountCar++;
					break;
				case 3:
					nCountMC++;
					break;
				case 5:
					nCountBus++;
					break;
				case 7:
					nCountPickup++;
					break;
				default:
					break;
				}
#else
				switch (o.obj_id)
				{
				case 0:
					nCountMC++;
					break;
				case 1:
					nCountCar++;
					break;
				case 2:
					nCountPickup++;
					break;
				case 3:
					nCountTruck++;
					break;
				case 4:
					nCountBus++;
					break;
				case 5:
					nCountVan++;
					break;
				default:
					break;
				}
#endif
			}
		}

		//std::cout << "IDsetSize=" << temp.size() << " ";


		////// Calculate and show speeed
		//std::vector<int> current_ids(temp.begin(), temp.end());

		//if (current_ids.size() < previous_ids.size())
		//{
		//	//std::cout << "\nTrig" << " ";

		//	// Find difference (missing element)
		//	std::vector<int> difference;
		//	std::set_difference(
		//		previous_ids.begin(), previous_ids.end(),
		//		current_ids.begin(), current_ids.end(),
		//		std::back_inserter(difference)
		//	);
		//	//std::cout << difference[0] << std::endl;
		//	for (auto const& id : difference)
		//	{
		//		roi_object track;
		//		std::vector<roi_object> temp;
		//		int i;
		//		for ( i = 0; i < tracks.size(); i++)
		//		{
		//			track = tracks[i];
		//			if (track.track_id == id)
		//			{
		//				// Calculate annd show speed
		//				speed = (g_distance/1000) / ((track.pts.size()/fps)/3600);
		//				////std::cout << "ID = " << track.track_id << " speed = " << speed << "km/h" << " ";
		//				speeds.push_back(speed);
		//				break;
		//			}
		//			else
		//			{
		//				temp.push_back(track);
		//			}
		//		}
		//		//tracks = temp;				
		//	}
		//}
		//previous_ids = current_ids;

		// Update Tracks
		for (auto& i : result_vec)
		{
			bool bExist = false;
			Point p(i.x + i.w / 2, i.y + i.h / 2);
			Point2f mapp = M.transformCOB(i);
			for (auto& j : tracks)
			{
				if (i.track_id == j.track_id) //add center point to tracking data
				{
					bExist = true; 
					// Moving average center point
					//if (j.pts.size() >= 4)
					//{
					//	p += j.pts[0] + j.pts[1] + j.pts[2] + j.pts[3];
					//	p /= 5;
					//}
					float dist = norm(j.pts[0] - p);
					if (dist < 80)
					{
						j.frameNums.push_front(frameNum);
						j.pts.push_front(p);
						j.mapped_pts.push_front(mapp);
					}
					break;
				}
			}
			if (!bExist)  // Add new track id
			{
				roi_object t;
				t.track_id = i.track_id;
				t.frameNums.push_front(frameNum);
				t.pts.push_front(p);
				t.mapped_pts.push_front(mapp);
				tracks.push_back(t);
			}
			//std::cout << i.track_id << " ";

		}
		//std::cout << std::endl;
		//std::cout << "Tracks size =   " << tracks.size() << std::endl;

		// Write to log file
		if (flog.is_open())
		{
			if (result_vec.size() > 0)
			{
				double v, x, y, dx, dy;
				for (auto& i : result_vec)
				{
					bool bWrite = true;
					for (auto& j : tracks)
					{
						if (i.track_id == j.track_id)
						{
							// Calculate instant speed
							
							Point2f pt0 = j.mapped_pts[0];
							Point2f pt1 = j.mapped_pts[1];
							int framediff = j.frameNums[0] - j.frameNums[1];

							if (framediff > 0)
							{
								x = pt0.x;
								y = pt0.y;
								dx = pt0.x - pt1.x;
								dy = pt0.y - pt1.y;
								double dist = norm(pt1 - pt0);
								v = (dist * cenperpix / 100.0) / framediff * fps * 3600 / 1000;
							}
							else
								bWrite = false;
						}
					}

					if (bWrite)
					{
						flog << frameNum << "," << i.track_id << "," << obj_names[i.obj_id] << "," << x << "," << y << ","<< dx << "," << dy << "," << v ;
						flog << std::endl;
					}
				}
			}

		}

		Mat top = Mat::zeros(rows, cols, src.type());
		//draw_boxes(src, result_vec, obj_names);
		draw_trackpoints(src, result_vec);
		draw_topview(top, result_vec, roi_vertices);
		//show_result(result_vec, obj_names);
		if (bShowDispROI)
		{
			std::vector<std::vector<Point>> pts{ roi_vertices };
			SetTransparentColor(src, pts, Scalar(0, 0, 255), 0.25, src);  // Draw ROI
		}
		if (bShowCalROI)
		{
			std::vector<std::vector<Point>> pts{ vertices };
			SetTransparentColor(src, pts, Scalar(255, 0, 0), 0.25, src);  // Draw ROI
		}
		if (bShowInfo)
		{
			DrawConfigInfo(src);
		}
		if (bShowCounter)
		{
			DrawTrafficInfo(src, result_vec);
		}
		if (bShowHelp)
		{
			DrawHelp(src);
		}
		//ShowSpeed(src);



		// Show top view
		cv::Rect2f bBox = boundingRect(transformed_roi_vertices);
		int margin = 10;
		//Mat sinput = top(Rect2f(vertices[0].x-margin, vertices[0].y-margin, topViewSize.width+2*margin, topViewSize.height+2*margin));
		Mat sinput = top(Rect2f(bBox.x - margin, bBox.y - margin, bBox.width + 2 * margin, bBox.height + 2 * margin));
		if (sinput.rows > 250)
		{
			float ratio = 250.0 / sinput.rows;
			resize(sinput, sinput, cv::Size(sinput.cols*ratio, sinput.rows*ratio));
		}
		if (sinput.cols > 250)
		{
			float ratio = 250.0 / sinput.cols;
			resize(sinput, sinput, cv::Size(sinput.cols * ratio, sinput.rows * ratio));
		}
		rectangle(sinput, Point(0, 0), Point(80, 20), Scalar(2555, 0, 0), -1);
		putText(sinput, "Top View", Point(2,15), cv::FONT_HERSHEY_PLAIN, 1.0, Scalar(255,255,255), 1);
		if (bDock)
			sinput.copyTo(src(Rect(0, rows-sinput.rows, sinput.cols, sinput.rows)));
		else
			imshow("Top View", top);

		//if (result_vec.size() > 0)
		//{
		//	std::cout << "Frame#" << frameNum << std::endl;
		//	show_result(result_vec, obj_names);
		//}
		imshow(winName, src);



		char c = cvWaitKey(1);
		if (c == 'h' || c == 'H')  // Set Line
			bShowHelp = !bShowHelp;
		if (c == 's' || c == 'S')  // Set Line
			setting1();
		if (c == 'd' || c == 'D') // Set Distance
			setDistance();
		if (c == 'c' || c == 'C') // Set ROI
			settingCalibrateROI();
		if (c == 'r' || c == 'R') // Set ROI
		{
			settingDisplayROI();
		}
		//if (c == 'p' || c == 'P') // Set PPCm
		//	setPixelPerMetre();
		if (c == '1' || c == 't' || c == 'T')
			bShowTrajectory = !bShowTrajectory;
		if (c == '2')
			bShowCalROI = !bShowCalROI;
		if (c == '3' )
			bShowDispROI = !bShowDispROI;
		if (c == '4')
		{
			bDock = !bDock;
			if (bDock) destroyWindow("Top View");
		}
		if (c == '5')
			bShowInfo = !bShowInfo;
		if (c == '6')
			bShowCounter = !bShowCounter;
		if (c == '+' || c == '=')
		{
			pointsize++;
			if (pointsize > 50) pointsize = 50;
		}
		if (c == '-' || c == '_')
		{
			pointsize--;
			if (pointsize < 3) pointsize = 3;
		}
		if (c == 32)              // Toggle Pause 
		{
			while (true)
			{
				if (cvWaitKey(1) == 32) break;
			}
		}

		if (c == 27) break;
	}

	flog.close();
	printf("Counter: MC = %d, Car = %d, Pickup = %d, Truck = %d, Bus = %d, Van = %d \n", nCountMC, nCountCar, nCountPickup, nCountTruck, nCountBus, nCountVan);
	std::cout << "Press any key to continue..." << std::endl;
	cvWaitKey(0);

	cvDestroyAllWindows();
}

void  setPixelPerMetre()
{
	std::cout << "\nCurrent Cm/Pixel is " << cenperpix << std::endl;
	std::cout << "Enter new CmPP : ";
	std::cin >> cenperpix;
	DoCalibrate(vertices);
}

void  setDistance()
{
	std::cout << "\nCurrennt distance is " << g_distance << std::endl;
	std::cout << "Enter new distance (in metre): ";
	std::cin >> g_distance;
	saveconfig();
}
void settingCalibrateROI()
{
	std::cout << "Setting Calibration ROI...." << std::endl;
	finished = false;
	vertices.clear();
	setMouseCallback(winName, CallBackFunc, nullptr);
	while (!finished) {
		imshow(winName, src);
		waitKey(50);
	}
	saveconfig();
}
void settingDisplayROI()
{
	std::cout << "Setting Display ROI...." << std::endl;
	finished = false;
	roi_vertices.clear();
	setMouseCallback(winName, CallBackFunc2, nullptr);
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
		//std::cout << "Right mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (vertices.size() < 4) {
			std::cout << "You need a minimum of four points!" << std::endl;
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
		//std::cout << "Left mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (vertices.size() == 0) {
			// First click - just draw point
			src.at<Vec3b>(x, y) = Vec3b(255, 0, 0);
		}
		else {
			// Second, or later click, draw line to previous vertex
			line(src, Point(x, y), vertices[vertices.size() - 1], Scalar(0, 0, 0));
		}
		if (vertices.size() < 4)
		{
			vertices.push_back(Point(x, y)); 
		}
		if (vertices.size() == 4)
		{
			// Close polygon
			line(src, vertices[vertices.size() - 1], vertices[0], Scalar(0, 0, 0));
			imshow(winName, src);
			std::vector<std::vector<Point>> pts{ vertices };
			SetTransparentColor(src, pts, Scalar(0, 0, 255), 0.5, src);
			finished = true;
		}
		return;
	}
}
void CallBackFunc2(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_RBUTTONDOWN) {
		//std::cout << "Right mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (roi_vertices.size() < 2) {
			std::cout << "You need a minimum of three points!" << std::endl;
			return;
		}
		// Close polygon
		line(src, roi_vertices[roi_vertices.size() - 1], roi_vertices[0], Scalar(0, 0, 0));
		imshow(winName, src);
		std::vector<std::vector<Point>> pts{ roi_vertices };
		SetTransparentColor(src, pts, Scalar(0, 0, 255), 0.5, src);
		finished = true;
		return;
	}
	if (event == EVENT_LBUTTONDOWN) {
		//std::cout << "Left mouse button clicked at (" << x << ", " << y << ")" << std::endl;
		if (roi_vertices.size() == 0) {
			// First click - just draw point
			src.at<Vec3b>(x, y) = Vec3b(255, 0, 0);
		}
		else {
			// Second, or later click, draw line to previous vertex
			line(src, Point(x, y), roi_vertices[roi_vertices.size() - 1], Scalar(0, 0, 0));
		}
		roi_vertices.push_back(Point(x, y));
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

cv::Point2f rotate2d(const cv::Point2f& inPoint, const double& angRad)
{
	cv::Point2f outPoint;
	//CW rotation
	outPoint.x = std::cos(angRad) * inPoint.x - std::sin(angRad) * inPoint.y;
	outPoint.y = std::sin(angRad) * inPoint.x + std::cos(angRad) * inPoint.y;
	return outPoint;
}

cv::Point2f rotatePoint(const cv::Point2f& inPoint, const cv::Point2f& center, const double& angRad)
{
	return rotate2d(inPoint - center, angRad) + center;
}

void DrawHelp(cv::Mat src)
{
	// Draw Transparent Panel
	// Draw transparent box
	cv::Mat overlay;
	double alpha = 0.5;
	cv::Scalar color(255, 0, 0);

	cv::Size size = cv::Size(270, 280);
	cv::Rect rc = Rect((cols-size.width)/2, (rows-size.height)/2, size.width, size.height);
	std::vector<std::vector<Point>> pts{ {Point(rc.x, rc.y),Point(rc.x+rc.width,rc.y),Point(rc.x+rc.width,rc.y+rc.height),Point(rc.x,rc.y+rc.height)} };
	SetTransparentColor(src, pts, cv::Scalar(255, 255, 255), alpha, src);


	// Draw Text
	putText(src, "Help", Point(rc.x+10, rc.y+20), cv::FONT_HERSHEY_PLAIN, 1, color, 2);
	putText(src, "c = Set Calibration ROI", Point(rc.x + 10, rc.y + 40), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "r = Set Display ROI", Point(rc.x + 10, rc.y + 60), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "d = Set Distance", Point(rc.x + 10, rc.y + 80), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "1 = On/Off Trajectory", Point(rc.x + 10, rc.y + 100), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "2 = On/Off Calibration ROI", Point(rc.x + 10, rc.y + 120), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "3 = On/Off Display ROI", Point(rc.x + 10, rc.y + 140), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "4 = Dock/Float Top View", Point(rc.x + 10, rc.y + 160), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "5 = On/Off Conig Info", Point(rc.x + 10, rc.y + 180), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "6 = On/Off Counter", Point(rc.x + 10, rc.y + 200), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "+ = Increase point size", Point(rc.x + 10, rc.y + 220), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "- = Decrease point size", Point(rc.x + 10, rc.y + 240), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);
	putText(src, "Space bar = Pause/Continue", Point(rc.x + 10, rc.y + 260), cv::FONT_HERSHEY_PLAIN, 1, color, 1.5);


}

// 22/07/2021 Version 1.3 released
// - Dock/Float the top view
// - On/Off Configuration Information
// - Write log to csv file. 
// 23/07/2021 Version 1.4 released
// - there are 2 ROIs; calibation ROI abd display ROI
// - Calibration ROI is used for calibrating the camera
// - top view will show area in the display ROI
// - Each video file has its own config file seperately



// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
