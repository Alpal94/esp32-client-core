#define OVERSIZED_BOARD 16

using namespace cv;
using namespace std;

class DetermineChessPieces {
	private:	
	struct ContourMap {
		bool active;
		size_t contour;
		size_t contourSubIndex;
	};

	Mat gray_lastFrame;

	ContourMap contourMap[COLS][ROWS];
	vector<vector<Point> > contours;
	vector<vector<Point> > drawing;
	public:
	void findChessPieces(Mat& _gray_lastFrame, vector<vector<Point> >& _contours, vector<vector<Point> >& _drawing, vector<Square>& _localSquareList) {
		drawing = _drawing;
		gray_lastFrame = _gray_lastFrame;
		contourMap[COLS][ROWS] = {};
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
		for (int i = 0; i < _localSquareList.size(); i++) {
			Square square = rotateSquare(_localSquareList[i], {.rotation = -square.rotation});
			FPoint manualCenter = rotatePoint(_localSquareList[i].center, -square.rotation);
			int centerX = (int) manualCenter.x;
			int centerY = (int) manualCenter.y;
			int spacing = (int) square.spacing;

			//printMarker(Point(centerX, centerY), drawing, spacing / 2);

			int spacingOffset = (int)2*spacing/8;
			bool evaluated = false;
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

						if(contours[contourMap[x][y].contour].size() > 40) {

							count++;
							printf("COUNT: %d\n", count);
							if(count > 1) {
								printf("Evaluating piece\n");
								evaluated = evaluateChessPiece(
									contourMap[x][y].contour,
									contourMap[x][y].contourSubIndex,
									manualCenter
								);
								printMarker(Point(x,y), drawing, 2);
							}
						}
						contoursEvaluated.push_back(contourMap[x][y].contour);
						if(evaluated) break;
					}
				}
				if(evaluated) break;
			}
			if(evaluated) break;
		}
		_drawing = drawing;
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
		float angleSum = 0;
		/*for(size_t i = contourSubIndex; i < contours[contourIndex].size() - 1; i++) {
			FPoint first = pointToFPoint(contours[contourIndex][i]);
			FPoint second = center;
			FPoint third = pointToFPoint(contours[contourIndex][i+1]);
			count++;
			//if(count > 10) break;

			float angle = angleFromPoints(first, center, third);

			angleSum += angle;
			float angleOriginal = angleFromPoints(original, center, third);
			printf("ASCENDING: angle: %f angleOriginal: %f angleSum: %f distance: %f %f %f\n", angle, angleOriginal, angleSum, fPixelDist(first, center), fPixelDist(third, center), fPixelDist(first, third));
		}*/
		count = 0;
		angleSum = 0;
		/*for(size_t i = contourSubIndex; i > 1; i--) {
			FPoint first = pointToFPoint(contours[contourIndex][i-1]);
			FPoint second = center;
			FPoint third = pointToFPoint(contours[contourIndex][i]);

			count++;
			//if(count > 10) break;

			float angle = angleFromPoints(first, center, third);
			float absAngle = absAngleFromPoints(first, center, third);
			angleSum += angle;
			float angleOriginal = angleFromPoints(original, center, third);
			printf("DECENDING: angle: %f absAngle: %f angleSum: %f distance: %f %f %f\n", angle, absAngle, angleSum, fPixelDist(first, center), fPixelDist(third, center), fPixelDist(first, third));
		}*/

		int start = contourSubIndex;
		FPoint A = pointToFPoint(contours[contourIndex][start-2]);
		FPoint B = pointToFPoint(contours[contourIndex][start]);
		FPoint C = pointToFPoint(contours[contourIndex][start+2]);
		FPoint _center = calculateCircle(A,B,C);
		printf("Center: %f %f\n", _center.x, _center.y);
		printMarker(Point((int)_center.x, (int)_center.y), drawing, 20);
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
