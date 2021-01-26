#define OVERSIZED_BOARD 16

using namespace cv;
using namespace std;
#define FROWS 120
#define FCOLS 160
class ColourAnalysis {
	private:
	bool seen[FCOLS][FROWS] = {};
	MinMaxHSV pieceColour;

	MinMaxHSV squareColour[2];

	int count = 0;

	vector<vector<Point> > drawing;
	Mat lastFrame;

	public:
	ColourAnalysis(Mat& _lastFrame) {

		lastFrame = _lastFrame;

	}

	bool executePieceAnalysis(Point2f point, vector<Point>& contour, vector<vector<Point> > &_drawing) {

		drawing = _drawing;
		floodFill(point, contour, pieceColour, 0, true);
		_drawing = drawing;
		return true;
	}

	MinMaxHSV squareColourAnalysis(Square _square, int colourReferenceType, bool init, vector<vector<Point> > &_drawing) {
		drawing = _drawing;
		FPoint southEast = rotatePoint(_square.southEast, -_square.rotation);
		FPoint northWest = rotatePoint(_square.northWest, -_square.rotation);
		FPoint southWest = rotatePoint(_square.southWest, -_square.rotation);
		FPoint northEast = rotatePoint(_square.northEast, -_square.rotation);
		FPoint center = rotatePoint(_square.center, -_square.rotation);

		float gradient = ((float) southEast.y - (float) northWest.y) / ((float) southEast.x - (float) northWest.x);
		float intercept = (float) southEast.y - gradient * ((float) southEast.x);
		float pixelOffset = 2;

		int sumHue = 0;
		int sumSaturation = 0;
		int sumValue = 0;
		vector<Point> _contour;
		_contour.push_back(fPointToPoint(southEast));
		_contour.push_back(fPointToPoint(southWest));
		_contour.push_back(fPointToPoint(northWest));
		_contour.push_back(fPointToPoint(northEast));

		if(colourReferenceType == 1) floodFill(Point2f(center.x, center.y), _contour, squareColour[colourReferenceType], 5, init);
		else floodFill(Point2f(center.x, center.y), _contour, squareColour[colourReferenceType], 7, init);
		cout << "Black max: " << squareColour[0].max << endl;
		cout << "Black min: " << squareColour[0].min << endl;
		cout << "White max: " << squareColour[1].max << endl;
		cout << "WHite min: " << squareColour[1].min << endl;
		

		/*imshow("hello", lastFrame);
		waitKey(0);*/
		//cout << "Type: " << colourReferenceType << " Square: " << squareColour[colourReferenceType].min << " " << squareColour[colourReferenceType].max << endl;
		//Calc average 
		for(int i = 0; i < 3; i++) squareColour[colourReferenceType].average[i] /= (float) count;

		_drawing = drawing;
		return squareColour[colourReferenceType];

	}

	MinMaxHSV getSquare(bool colourReferenceType) {
		int margin = 1;
		int marginNext = 1;

		/*int minHue = squareColour[colourReferenceType].min[0];
		int minSaturation = squareColour[colourReferenceType].min[1];
		int minValue = squareColour[colourReferenceType].min[2];

		int maxHue = squareColour[colourReferenceType].max[0];
		int maxSaturation = squareColour[colourReferenceType].max[1];
		int maxValue = squareColour[colourReferenceType].max[2];

		squareColour[colourReferenceType].min[0] = (minHue - margin) > 0 ? minHue - margin : 0;
		squareColour[colourReferenceType].min[1] = (minSaturation - marginNext) > 0 ? minSaturation - marginNext : 0;
		squareColour[colourReferenceType].min[2] = (minValue - margin) > 0 ? minValue - margin : 0;

		squareColour[colourReferenceType].max[0] = (maxHue + margin) < 179 ? maxHue + margin : 179;
		squareColour[colourReferenceType].max[1] = (maxSaturation + marginNext) < 255 ? maxHue + marginNext : 255;
		squareColour[colourReferenceType].max[2] = (maxValue + margin) < 255 ? maxValue + margin : 255;*/

		return squareColour[colourReferenceType];
	}

	private:
	void floodFill(Point2f point, vector<Point>& contour, MinMaxHSV &colourRange, float thresh, bool init) {
		if(init) {
			for(int i = 0; i < FCOLS; i++) memset(seen[i], 0, sizeof(seen[i]));
			colourRange.hist.clear();
			count = 0;
		}
		
		if(point.x > FCOLS || point.y > FROWS || point.x < 0 || point.y < 0) return;
		if(seen[(int)point.x][(int)point.y]) return;
		if(pointPolygonTest(contour, point, !!thresh) < thresh) return;

		seen[(int)point.x][(int)point.y] = true;

		Mat3b hsv;
		Mat3b bgr(lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y)));
		cvtColor(bgr, hsv, COLOR_BGR2HSV); 

		Vec3b hsvColour = hsv.at<Vec3b>(0,0);
		//hsvColour = lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y)); 
		//if(thresh == 5) cout << "Type: " << hsvColour << endl;

		cout << "Flood: " << hsvColour << endl;
		string key = "   ";
		for(int i = 0; i < 3; i++) key[i] = (uchar) hsvColour[i] / 5;
		try {
			colourRange.hist.at(key)++;
		} catch (const std::out_of_range& oor) {
			colourRange.hist.insert({key, 1});
		}
		if(init) {
			colourRange.min = hsvColour;
			colourRange.max = hsvColour;
			colourRange.sum = hsvColour;

			count++;
		} else {
				
			//maxDiff(colourRange.max, hsvColour) < 40 && maxDiff(colourRange.min, hsvColour) < 40

				//if(thresh == 5) printMarker(Point((int) point.x,(int) point.y), drawing, 1);
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

	int maxDiff( Vec3b vec1, Vec3b vec2 ) {
		int diff1 = abs(vec1[0] - vec2[0]);
		int diff2 = abs( vec1[1] - vec2[1]);
		int diff3 = abs( vec1[2] - vec2[2]);
		return max(max(diff1, diff2), diff3);
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
			if(point.x+x < 0 || point.x+x > FCOLS) continue;
			marker.push_back(Point(point.x+x, point.y));
		}
		for(int y = -size; y < size; y++) {	
			if(point.y+y < 0 || point.y+y > FROWS) continue;
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
	char chessBoard[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};

	ContourMap contourMap[FCOLS][FROWS];
	vector<vector<Point> > contours;
	vector<vector<Point> > drawing;
	vector<Vec4i> hierarchy;

	public:
	void findChessPieces(
			Mat& _gray_lastFrame,
			Mat& _lastFrame,
			vector<vector<Point> >& _contours,
			vector<Vec4i> _hierarchy,
			vector<vector<Point> >& _drawing,
			vector<Square>& _localSquareList,
			vector<Square>& _globalSquareList
			) {

		drawing.clear();
		drawing = _drawing;

		hierarchy = _hierarchy;

		//contourMap[FCOLS][FROWS] = {};
		for(int i = 0; i < FCOLS; i++) {
			for(int j = 0; j < FROWS; j++) {
				contourMap[i][j].active = false;
			}
		}
		contours.clear();
		contours = _contours;
		for (int i = 0; i < OVERSIZED_BOARD; i++) for(int j = 0; j < OVERSIZED_BOARD; j++) chessBoard[i][j] = 0;
		for (size_t i = 0; i < contours.size(); i++) {
			for(size_t j = 0; j < contours[i].size(); j++) {
				contourMap[contours[i][j].x][contours[i][j].y].active = true;
				contourMap[contours[i][j].x][contours[i][j].y].contour = i;
				contourMap[contours[i][j].x][contours[i][j].y].contourSubIndex = j;
			}
		}
		for (int i = 0; i < _globalSquareList.size(); i++) {
			if(_globalSquareList[i].occupied) {
				int x = _globalSquareList[i].x;
				int y = _globalSquareList[i].y;
				chessBoard[x][y] = '0';
			}
		}
		int count = 0;
		ColourAnalysis squareColourAnalysis(_lastFrame);

		bool firstBlack = false, firstWhite = false;
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
					if(x < FCOLS && y < FROWS && x >= 0 && y >= 0 && contourMap[x][y].active) {
						
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
							if(count > 1) {
								evaluated = evaluateChessPiece(
									contourMap[x][y].contour,
									contourMap[x][y].contourSubIndex,
									manualCenter,
									_lastFrame
								);
								if(evaluated) {
									int x = square.global_x;
									int y = square.global_y;
									chessBoard[x][y] = 'u';
									continue;
								}
							}
						}
						contoursEvaluated.push_back(contourMap[x][y].contour);
					}
					if(evaluated) continue;
				}
				if(evaluated) continue;
			}

			if(!containsContours) {
				if((square.x + square.y) % 2) printMarker(Point(centerX,centerY), drawing, 2);
				else printMarker(Point(centerX,centerY), drawing, 4);
				
				bool init = false;
				if((square.x + square.y) % 2 && !firstBlack) {
					firstBlack = true;
					init = true;
				}
				if((square.x + square.y) % 2 == 0 && !firstWhite) {
					firstWhite = true;
					init = true;
				}
				MinMaxHSV result = squareColourAnalysis.squareColourAnalysis(square, (square.x + square.y) % 2, init, drawing);
			}
		}

		//squareColour[0] = squareColourAnalysis.getSquare(0);
		//squareColour[1] = squareColourAnalysis.getSquare(1);

		//cout << "HSVF Square 0 colour: min: " << squareColour[0].min << " max: " << squareColour[0].max << " Square 1 colour: min: " << squareColour[1].min << " max: " << squareColour[1].max << endl;
		_drawing = drawing;

		/*printf("\nChess peices: \n");
		for(int x = 0; x < OVERSIZED_BOARD; x++) {
			for(int y = 0; y < OVERSIZED_BOARD; y++) {
				if(chessBoard[x][y] == '0') printf(".");
				else if(chessBoard[x][y] == 'u') printf("+");
				else printf(" ");
			}
			printf("\n");
		}*/
	}

	MinMaxHSV getSquareColour(bool blackWhite) {
		return squareColour[blackWhite];
	}	

	private:
	void printMarker(Point point, vector<vector<Point> >& drawing, int size) {
		vector<Point> marker;
		marker.clear();

		for(int x = -size; x < size; x++) {	
			if(point.x+x < 0 || point.x+x > FCOLS) continue;
			marker.push_back(Point(point.x+x, point.y));
		}
		for(int y = -size; y < size; y++) {	
			if(point.y+y < 0 || point.y+y > FROWS) continue;
			marker.push_back(Point(point.x, point.y+y));
		}
		drawing.push_back(marker);
	}
	bool evaluateChessPiece(size_t contourIndex, size_t contourSubIndex, FPoint center, Mat &lastFrame) {
		int count = 0;
		FPoint original = pointToFPoint(contours[contourIndex][contourSubIndex]);
		float angleAscending = 0;
		FPoint ascendingPoint;
		FPoint descendingPoint;
		float angleMin = 0.15;
		float angleMax = 0.3;
		int lastIndex = contours[contourIndex].size() - 1;

		bool isClosed = hierarchy[contourIndex][2] > 0;
		if(isClosed) {
			return ColourAnalysis(lastFrame).executePieceAnalysis(
				Point2f(center.x, center.y),
				contours[contourIndex],
				drawing
			);
		}
		return false;
	}

	FPoint calculateCircle(FPoint A, FPoint B, FPoint C) {
		float xDelta_a = B.x - A.x;
		float yDelta_a = B.y - A.y;
		float xDelta_b = C.x - B.x;
		float yDelta_b = C.y - B.y;

		float gradientA = yDelta_a / xDelta_a;
		float gradientB = yDelta_b / xDelta_b;

		float centerX = (gradientA*gradientB*(A.y - C.y) + gradientB*(A.x + B.x) - gradientA*(B.x + C.x)) / (2 * (gradientB - gradientA));
		float centerY = -(1/gradientA) * (centerX - (A.x + B.x)/2) + (A.y + B.y) / 2;

		return FPoint({ .x = centerX, .y = centerY });
	}
};


