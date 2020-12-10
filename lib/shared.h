#include <unistd.h> 
#include <sstream>
#include <time.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

#define M_PI           3.14159265358979323846

#define ROWS 120
#define COLS 160

#define OVERSIZED_BOARD 16

#define CAM_RATIO 0.75f // fieldWidth / distance ==> 15cm / 20cm

#ifdef FOR_ANDROID
	#include <android/log.h>
	#define  LOG_TAG    "ESP32Client"
	#define  ALOG(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#else
	#define  ALOG(...)  printf(__VA_ARGS__)
#endif

using namespace cv;
using namespace std;

struct SegmentMetadata {
	Point start;
	Point end;
	float dist;
};

struct LineMetadata {
	size_t startIndex;
	size_t endIndex;
	size_t contourIndex;
	float gradient;
	float intercept;
	float xIntercept;
	float xGradient;
	float relevance;
	Vec4i bounds;
	std::vector<SegmentMetadata> segments;
	std::vector<cv::Point> line;
};

struct MapOffset {
	float spacing;
	float rotation;
	float north;
	float west;
} mapOffset;

struct FPoint {
	float x;
	float y;
};

struct Square {
	bool occupied;
	float spacing;
	float rotation;
	FPoint center;

	FPoint northEast;
	FPoint northWest;
	FPoint southEast;
	FPoint southWest;

	int x;
	int y;
	int global_x;
	int global_y;
};

enum Orientation {
	NE, NW, SE, SW
};

struct RobotPosition {
	float x;
	float y;
	float z;
};

struct ChessboardToCamera {
	bool calced;
	float distance;
	float squareWidth;
};

struct MinMaxHSV {
	Vec3b min;
	Vec3b max;
	Vec3b average;
	Vec3b sum;
	Vec3b comp;
	unordered_map<string, int> hist;
};

void updateSquareCorner(Square &square, FPoint point, Orientation orientation) {
	switch(orientation) {
		case NE:
			square.northEast = point;
			break;
		case NW:
			square.northWest = point;
			break;
		case SE:
			square.southEast = point;
			break;
		case SW:
			square.southWest = point;
			break;
	}
}

float angleFromGradient(float gradient_first, float gradient_second) {
	return atan(fabs((gradient_first - gradient_second) / (1 + gradient_first * gradient_second)));
}

FPoint pointToFPoint(Point point) {
	return { .x = (float) point.x, .y = (float) point.y };
}

Point fPointToPoint(FPoint point) {
	return Point ((int) point.x, (int) point.y);
}

double fPixelDist( FPoint pt1, FPoint pt2 )
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}


//Find angle at point 2
double angleFromPoints( FPoint pt1, FPoint target_pt2, FPoint pt3 )
{
	FPoint v1 = { .x = target_pt2.x - pt1.x, .y = target_pt2.y - pt1.y};
	FPoint v2 = { .x = target_pt2.x - pt3.x, .y = target_pt2.y - pt3.y};
	return asin((v1.x*v2.y - v1.y*v2.x) / (sqrt(pow(v1.x,2) + pow(v1.y,2))*sqrt(pow(v2.x,2) + pow(v2.y,2))));
}

double absAngleFromPoints( FPoint pt1, FPoint target_pt2, FPoint pt3 ) {
	float dist12 = fPixelDist(pt1, target_pt2);
	float dist13 = fPixelDist(pt1, pt3);
	float dist23 = fPixelDist(target_pt2, pt3);

	return acos((dist12*dist12 + dist23*dist23 - dist13*dist13) / (2*dist12*dist23));
}

double pixelDist( Point pt1, Point pt2 )
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}

double distance( RobotPosition p1, RobotPosition p2 )
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dz = p1.z - p2.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

FPoint locateIntercept(LineMetadata first_line, LineMetadata second_line, bool bounded=false) {
	float x_intercept = (float) ((second_line.intercept - first_line.intercept) / (first_line.gradient - second_line.gradient));
	float y_intercept = (float) (second_line.intercept + second_line.gradient * x_intercept);

	float lowerX1 = (float) first_line.bounds[0], upperX1 = (float) first_line.bounds[2];
	float lowerX2 = (float) second_line.bounds[0], upperX2 = (float) second_line.bounds[2];
	float lowerY1, upperY1, lowerY2, upperY2;
	if(!bounded) {
		return { .x = x_intercept, .y = y_intercept };
	} else {
		if(first_line.bounds[1] < first_line.bounds[3]) {
			lowerY1 = first_line.bounds[1];
			upperY1 = first_line.bounds[3];
		} else {
			lowerY1 = first_line.bounds[3];
			upperY1 = first_line.bounds[1];
		}
		if(second_line.bounds[1] < second_line.bounds[3]) {
			lowerY2 = second_line.bounds[1];
			upperY2 = second_line.bounds[3];
		} else {
			lowerY2 = second_line.bounds[3];
			upperY2 = second_line.bounds[1];
		}

		if(
				x_intercept > lowerX1 && x_intercept < upperX1 &&
				x_intercept > lowerX2 && x_intercept < upperX2 &&
				y_intercept > lowerY1 && y_intercept < upperY1 &&
				y_intercept > lowerY2 && y_intercept < upperY2 ) {
			return { .x = x_intercept, .y = y_intercept };
		}
		float invalid = nan("1");
		return { .x = invalid, .y = invalid };
	}
}

float lineSpacing(LineMetadata first_line, LineMetadata second_line) {
	if(first_line.gradient == 0) first_line.gradient = 0.0001;
	float perpendicular_gradient = -1 / first_line.gradient;

	LineMetadata perpendicular_line = {
		.gradient = perpendicular_gradient,
		.intercept = 0
	};

	FPoint first_intercept = locateIntercept(first_line, perpendicular_line, false);
	FPoint second_intercept = locateIntercept(second_line, perpendicular_line, false);

	float dist = fPixelDist(first_intercept, second_intercept);
	if(isnan(dist)) {
	}
	return (float) fPixelDist(first_intercept, second_intercept);
}

FPoint midPoint(FPoint first, FPoint second) {
	FPoint smaller = first.x < second.x ? first : second;
	FPoint larger = first.x < second.x ? second : first;
	float centerX = first.x + (second.x - first.x) / 2;

	smaller = first.y < second.y ? first : second;
	larger = first.y < second.y ? second : first;
	float centerY = first.y + (second.y - first.y) / 2;

	return { .x = centerX, .y = centerY };
}

FPoint squareCenter(Square square) {
	FPoint firstCenter = midPoint(square.southWest, square.northEast);
	FPoint secondCenter = midPoint(square.northWest, square.southEast);
	return midPoint(firstCenter, secondCenter);
}

FPoint rotatePoint(FPoint point, float angle) {
	/*
	 * Rotation Matrix:
	 * 	| cos(angle) | -sin(angle) |
	 * 	| sin(angle) |  cos(angle) |
	 *
	*/

	float x = point.x * cos(angle) - point.y * sin(angle);
	float y = point.x * sin(angle) + point.y * cos(angle);

	return { .x = x, .y = y };
}

Square rotateSquare(Square square, MapOffset offset) {
	float angle = offset.rotation;
	square.northWest = rotatePoint(square.northWest, angle);
	square.northEast = rotatePoint(square.northEast, angle);
	square.southWest = rotatePoint(square.southWest, angle);
	square.southEast = rotatePoint(square.southEast, angle);

	square.center = squareCenter(square);

	return square;
}

Square translateSquare(Square square, MapOffset offset) {
	square.northWest.x += offset.west;
	square.northEast.x += offset.west;
	square.southWest.x += offset.west;
	square.southEast.x += offset.west;

	square.northWest.y += offset.north;
	square.northEast.y += offset.north;
	square.southWest.y += offset.north;
	square.southEast.y += offset.north;

	if(offset.north || offset.west) { 
		printf("OK: offset: %f %f\n", offset.north, offset.west);
		square.center = squareCenter(square);
	}

	return square;
}
