#define OVERSIZED_BOARD 16

using namespace cv;
using namespace std;

class ColourAnalysis {
	private:
	bool seen[COLS][ROWS] = {};
	MinMaxHSV pieceColour;

	MinMaxHSV squareColour[2];

	int count = 0;

	vector<vector<Point> > drawing;
	Mat lastFrame;

	public:
	ColourAnalysis(Mat& _lastFrame) {
		lastFrame = _lastFrame;
	}

	void executePieceAnalysis(Point2f point, vector<Point>& contour, vector<vector<Point> > &_drawing) {

		drawing = _drawing;
		floodFill(point, contour, pieceColour, 0, true);
		printf("\nMin: %d %d %d\n", pieceColour.min[0], pieceColour.min[1], pieceColour.min[2]);
		printf("Max: %d %d %d\n", pieceColour.max[0], pieceColour.max[1], pieceColour.max[2]);
		_drawing = drawing;
	}

	MinMaxHSV squareColourAnalysis(Square _square, bool colourReferenceType, vector<vector<Point> > &_drawing) {
		drawing = _drawing;
		FPoint southEast = rotatePoint(_square.southEast, -_square.rotation);
		FPoint northWest = rotatePoint(_square.northWest, -_square.rotation);
		FPoint southWest = rotatePoint(_square.southWest, -_square.rotation);
		FPoint northEast = rotatePoint(_square.northEast, -_square.rotation);
		FPoint center = rotatePoint(_square.center, -_square.rotation);

		float gradient = ((float) southEast.y - (float) northWest.y) / ((float) southEast.x - (float) northWest.x);
		float intercept = (float) southEast.y - gradient * ((float) southEast.x);
		printf("HSV Square %f %f: ", southEast.x, northWest.x);
		float pixelOffset = 2;

		int sumHue = 0;
		int sumSaturation = 0;
		int sumValue = 0;
		vector<Point> _contour;
		_contour.push_back(fPointToPoint(southEast));
		_contour.push_back(fPointToPoint(southWest));
		_contour.push_back(fPointToPoint(northWest));
		_contour.push_back(fPointToPoint(northEast));

		floodFill(Point2f(center.x, center.y), _contour, squareColour[colourReferenceType], 3, true);

		int margin = 10;
		squareColour[colourReferenceType].min[0] -= margin;
		squareColour[colourReferenceType].min[1] -= margin;
		squareColour[colourReferenceType].min[2] -= margin;

		squareColour[colourReferenceType].max[0] += margin;
		squareColour[colourReferenceType].max[1] += margin;
		squareColour[colourReferenceType].max[2] += margin;

		//Calc average 
		for(int i = 0; i < 3; i++) squareColour[colourReferenceType].average[i] /= (float) count;

		_drawing = drawing;
		return squareColour[colourReferenceType];

	}

	MinMaxHSV getSquare(bool colourReferenceType) {
		return squareColour[colourReferenceType];
	}

	private:
	void floodFill(Point2f point, vector<Point>& contour, MinMaxHSV &colourRange, float thresh, bool init) {
		if(init) {
			for(int i = 0; i < COLS; i++) memset(seen[i], 0, sizeof(seen[i]));
			count = 0;
		}
		
		if(point.x > COLS || point.y > ROWS || point.x < 0 || point.y < 0) return;
		if(seen[(int)point.x][(int)point.y]) return;
		if(pointPolygonTest(contour, point, !!thresh) < thresh) return;

		seen[(int)point.x][(int)point.y] = true;

		printMarker(Point((int) point.x,(int) point.y), drawing, 1);
		Mat3b hsv;
		Mat3b bgr(lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y)));
		cvtColor(bgr, hsv, COLOR_BGR2HSV); 

		Vec3b hsvColour = hsv.at<Vec3b>(0,0);

		if(init) {
			colourRange.min = hsvColour;
			colourRange.max = hsvColour;
			colourRange.sum = hsvColour;
			count++;
		} else {
			colourRange.max = maxHSV(colourRange.max, hsvColour);
			colourRange.min = minHSV(colourRange.min, hsvColour);

			colourRange.sum[0] += hsvColour[0];
			colourRange.sum[1] += hsvColour[1];
			colourRange.sum[2] += hsvColour[2];

			count++;
		}

		floodFill(Point2f(point.x+1, point.y), contour, colourRange, thresh, false);
		floodFill(Point2f(point.x-1, point.y), contour, colourRange, thresh, false);
		floodFill(Point2f(point.x, point.y+1), contour, colourRange, thresh, false);
		floodFill(Point2f(point.x, point.y-1), contour, colourRange, thresh, false);
	}

	Vec3b maxHSV(Vec3b colour1, Vec3b colour2) {
		int maxHue = max(colour1[0], colour2[0]);
		int maxSaturation = max(colour1[1], colour2[1]);
		int maxValue = max(colour1[2], colour2[2]);
		return Vec3b(maxHue, maxSaturation, maxValue); 
	}

	Vec3b minHSV(Vec3b colour1, Vec3b colour2) {
		int minHue = min(colour1[0], colour2[0]);
		int minSaturation = min(colour1[1], colour2[1]);
		int minValue = min(colour1[2], colour2[2]);
		return Vec3b(minHue, minSaturation, minValue); 
	}

	float colourDistance(Vec3b colour1, Vec3b colour2) {
		return pow(pow(colour1[0] - colour2[0], 2) + pow(colour1[1] - colour2[1], 2) + pow(colour1[2] - colour2[2], 2), 0.5);
	}

	void printMarker(Point point, vector<vector<Point> >& drawing, int size) {
		vector<Point> marker;
		marker.clear();

		for(int x = -size; x < size; x++) {	
			if(point.x+x < 0 || point.x+x > COLS) continue;
			marker.push_back(Point(point.x+x, point.y));
		}
		for(int y = -size; y < size; y++) {	
			if(point.y+y < 0 || point.y+y > ROWS) continue;
			marker.push_back(Point(point.x, point.y+y));
		}
		drawing.push_back(marker);
	}
};
class DetermineChessPieces {
	private:	
	struct ContourMap {
		bool active;
		size_t contour;
		size_t contourSubIndex;
	};

	MinMaxHSV squareColour[2];

	Mat gray_lastFrame;
	Mat lastFrame;

	ContourMap contourMap[COLS][ROWS];
	vector<vector<Point> > contours;
	vector<vector<Point> > drawing;
	vector<Vec4i> hierarchy;

	public:
	void findChessPieces(Mat& _gray_lastFrame, Mat& _lastFrame, vector<vector<Point> >& _contours, vector<Vec4i> _hierarchy, vector<vector<Point> >& _drawing, vector<Square>& _localSquareList) {
		drawing.clear();
		drawing = _drawing;
		gray_lastFrame = _gray_lastFrame;
		lastFrame = _lastFrame;
		hierarchy = _hierarchy;
		//contourMap[COLS][ROWS] = {};
		for(int i = 0; i < COLS; i++) {
			for(int j = 0; j < ROWS; j++) {
				contourMap[i][j].active = false;
			}
		}
		contours.clear();
		contours = _contours;

		for (size_t i = 0; i < contours.size(); i++) {
			for(size_t j = 0; j < contours[i].size(); j++) {
				contourMap[contours[i][j].x][contours[i][j].y].active = true;
				contourMap[contours[i][j].x][contours[i][j].y].contour = i;
				contourMap[contours[i][j].x][contours[i][j].y].contourSubIndex = j;
			}
		}

		int count = 0;
		ColourAnalysis squareColourAnalysis(lastFrame);
		for (int i = 0; i < _localSquareList.size(); i++) {
			//Square square = rotateSquare(_localSquareList[i], {.rotation = -square.rotation});
			Square square = _localSquareList[i];
			FPoint manualCenter = rotatePoint(_localSquareList[i].center, -square.rotation);

			int centerX = (int) manualCenter.x;
			int centerY = (int) manualCenter.y;
			int spacing = (int) square.spacing;


			int spacingOffset = (int)2*spacing/8;
			bool evaluated = false;
			bool containsContours = false;
			vector<int> contoursEvaluated;
			for(int x = centerX - spacingOffset; x < centerX + spacingOffset; x++) {
				for(int y = centerY - spacingOffset; y < centerY + spacingOffset; y++) {
					if(x < COLS && y < ROWS && x >= 0 && y >= 0 && contourMap[x][y].active) {
						
						bool skip = false;
						for(int i = 0; i < contoursEvaluated.size(); i++) { 
							if(contourMap[x][y].contour == contoursEvaluated[i]) {
								skip = true;
								break;
							}
						}
						if(skip) continue;

						if(contours[contourMap[x][y].contour].size() > 2) {
							containsContours = true;
							count++;
							printf("Evaluating COUNT: %d\n", count);
							if(count > 1) {
								printf("Evaluating piece\n");
								evaluated = evaluateChessPiece(
									contourMap[x][y].contour,
									contourMap[x][y].contourSubIndex,
									manualCenter
								);
								//printMarker(Point(x,y), drawing, 20);
							}
						}
						contoursEvaluated.push_back(contourMap[x][y].contour);
					}
				}
			}

			if(!containsContours) {
				printMarker(Point(centerX,centerY), drawing, 2);
				MinMaxHSV result = squareColourAnalysis.squareColourAnalysis(square, i % 2, drawing);
			}
		}

		squareColour[0] = squareColourAnalysis.getSquare(0);
		squareColour[1] = squareColourAnalysis.getSquare(1);
		_drawing = drawing;
	}

	MinMaxHSV getSquareColour(bool blackWhite) {
		return squareColour[blackWhite];
	}	

	private:
	void printMarker(Point point, vector<vector<Point> >& drawing, int size) {
		vector<Point> marker;
		marker.clear();

		for(int x = -size; x < size; x++) {	
			if(point.x+x < 0 || point.x+x > COLS) continue;
			marker.push_back(Point(point.x+x, point.y));
		}
		for(int y = -size; y < size; y++) {	
			if(point.y+y < 0 || point.y+y > ROWS) continue;
			marker.push_back(Point(point.x, point.y+y));
		}
		drawing.push_back(marker);
	}
	bool evaluateChessPiece(size_t contourIndex, size_t contourSubIndex, FPoint center) {
		int count = 0;
		FPoint original = pointToFPoint(contours[contourIndex][contourSubIndex]);
		float angleAscending = 0;
		FPoint ascendingPoint;
		FPoint descendingPoint;
		float angleMin = 0.15;
		float angleMax = 0.3;
		int lastIndex = contours[contourIndex].size() - 1;
		printf("Contour: %d %d - %d %d\n", contours[contourIndex][0].x, contours[contourIndex][0].y, contours[contourIndex][lastIndex].x, contours[contourIndex][lastIndex].y);
		printf("Hierarchy: %d %d %d %d\n", hierarchy[contourIndex][0], hierarchy[contourIndex][1], hierarchy[contourIndex][2], hierarchy[contourIndex][3]);
		printf("Index: %ld\n", contourIndex);

		bool isClosed = hierarchy[contourIndex][2] > 0;
		if(isClosed) {
			//printMarker(fPointToPoint(center), drawing, 3);
			//
			ColourAnalysis(lastFrame).executePieceAnalysis(
				Point2f(center.x, center.y),
				contours[contourIndex],
				drawing
			);
		}

		/*for(size_t i = contourSubIndex; i < contours[contourIndex].size() - 1; i++) {
			FPoint first = pointToFPoint(contours[contourIndex][i]);
			FPoint second = center;
			FPoint third = pointToFPoint(contours[contourIndex][i+1]);
			count++;

			float angle = angleFromPoints(first, center, third);

			angleAscending = angleFromPoints(original, center, third);
			ascendingPoint = third;

			if(angleAscending >  angleMin) {
				ascendingPoint = third;
				break;
			}

			//printf("ASCENDING: angle: %f distance: %f %f %f\n", angleAscending, fPixelDist(first, center), fPixelDist(third, center), fPixelDist(first, third));
		}
		count = 0;
		float angleDecending = 0;
		for(size_t i = contourSubIndex; i > 1; i--) {
			FPoint first = pointToFPoint(contours[contourIndex][i-1]);
			FPoint second = center;
			FPoint third = pointToFPoint(contours[contourIndex][i]);

			count++;

			float angle = angleFromPoints(first, center, third);
			float absAngle = absAngleFromPoints(first, center, third);

			angleDecending = angleFromPoints(original, center, third);
			descendingPoint = third;

			if(angleDecending >  angleMin) break;

			//printf("DECENDING: angle: %f  distance: %f %f %f\n", angleDecending, fPixelDist(first, center), fPixelDist(third, center), fPixelDist(first, third));
		}

		int start = contourSubIndex;
		FPoint A = ascendingPoint;
		FPoint B = pointToFPoint(contours[contourIndex][start]);
		FPoint C = descendingPoint;
		FPoint _center = calculateCircle(A,B,C);
		printf("Center: %f %f\n", _center.x, _center.y);
		printMarker(fPointToPoint(_center), drawing, 10);
		printMarker(fPointToPoint(A), drawing, 5);
		printMarker(fPointToPoint(B), drawing, 5);
		printMarker(fPointToPoint(C), drawing, 5);*/
		return true;
	}

	FPoint calculateCircle(FPoint A, FPoint B, FPoint C) {
		printf("A: %f %f 	B: %f %f	C: %f %f\n", A.x, A.y, B.x, B.y, C.x, C.y);
		float xDelta_a = B.x - A.x;
		float yDelta_a = B.y - A.y;
		float xDelta_b = C.x - B.x;
		float yDelta_b = C.y - B.y;

		float gradientA = yDelta_a / xDelta_a;
		float gradientB = yDelta_b / xDelta_b;
		printf("Grad: %f %f\n", gradientA, gradientB);

		float centerX = (gradientA*gradientB*(A.y - C.y) + gradientB*(A.x + B.x) - gradientA*(B.x + C.x)) / (2 * (gradientB - gradientA));
		float centerY = -(1/gradientA) * (centerX - (A.x + B.x)/2) + (A.y + B.y) / 2;

		return FPoint({ .x = centerX, .y = centerY });
	}
};


