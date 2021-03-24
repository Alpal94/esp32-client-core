#define OVERSIZED_BOARD 16

using namespace cv;
using namespace std;
#define FROWS 120
#define FCOLS 160
#define BOARD 8

class Positions {
	private:
	int noPieces;
	ChessPiece board[BOARD][BOARD];
	ChessPiece oldBoard[BOARD][BOARD];

	int turns;
	bool updating;

	public:
	Positions() {
	}

	void beginBoardUpdate() {
		updating = true;
		for(int i = 0; i < BOARD; i++) {
			for(int j = 0; j < BOARD; j++) {
				oldBoard[i][j] = board[i][j];
				board[i][j].active = false;
			}
		}
	}

	void finishBoardUpdate() {
		resolveBoardUpdate();
		updating = false;
		printBoard();
	}
	
	void resolveBoardUpdate() {
		if(turns == 0) return initBoard();
		bool stateChange;
		Point previous;
		Point newPos;
		for(int i = 0; i < BOARD; i++) {
			for(int j = 0; j < BOARD; j++) {
				bool stayedTheSame = board[i][j].active && oldBoard[i][j].active && board[i][j].colour == oldBoard[i][j].colour;
				bool movedNew = board[i][j].active && !oldBoard[i][j].active;
				bool movedKill = board[i][j].active && oldBoard[i][j].active && board[i][j].colour != oldBoard[i][j].colour;
				bool previousPosition = !board[i][j].active && oldBoard[i][j].active;
				if(stayedTheSame) board[i][j] = oldBoard[i][j]; 
				else if(previousPosition) previous = Point(i, j);
				else if(movedNew || movedKill) newPos = Point(i, j);
				
				if(previousPosition || movedNew || movedKill) stateChange = true;
			}
		}
		if(stateChange) {
			turns++;
			board[newPos.x][newPos.y].type = oldBoard[previous.x][previous.y].type;
		}
	}

	void printBoard() {
		printf("\n");
		for(int row = 0; row < BOARD; row++) {
			for(int col = 0; col < BOARD; col++) {
				if(!board[col][row].active) {
					printf("   ");
					continue;
				}
				if(board[col][row].colour == White) {
					switch(board[col][row].type) {
						case Rook: printf(" ♜ "); break;
						case Knight: printf(" ♞ "); break;
						case Bishop: printf(" ♝ "); break;
						case Queen: printf(" ♛ "); break;
						case King: printf(" ♚ "); break;
						case Pawn: printf(" ♟︎ "); break;
						case Unknown: printf(" - "); break;
						default: printf(" E ");
					}
				} else {
					switch(board[col][row].type) {
						case Rook: printf(" ♖ "); break;
						case Knight: printf(" ♘ "); break;
						case Bishop: printf(" ♗ "); break;
						case Queen: printf(" ♕ "); break;
						case King: printf(" ♔ "); break;
						case Pawn: printf(" ♙ "); break;
						case Unknown: printf(" - "); break;
						default: printf(" E ");
					}
				}
			}
			printf("\n");
		}
	}

	void initBoard() {
		turns++;
		for(int row = 0; row < BOARD; row++) {
			for(int col = 0; col < BOARD; col++) {
				if(row == 0 || row == 7) {
					switch(col) {
						case 0:
						case 7: board[col][row].type = Rook; break;
						case 1:
						case 6: board[col][row].type = Knight; break;
						case 2:
						case 5: board[col][row].type = Bishop; break;
						case 3: board[col][row].type = Queen; break;
						case 4: board[col][row].type = King; break;
					}
				} else if(row == 1 || row == 6) {
					board[col][row].type = Pawn;
				}
			}
		}
	}

	void insertChessPiece(Colour colour, Point position) {
		if(updating) {
			board[position.x][position.y] = {
				.type = Unknown,
				.colour = colour,
				.active = true
			};
		}
	}

};

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

	bool executePieceAnalysis(Point2f point, vector<Point>& contour, vector<vector<Point> > &_drawing) {

		drawing = _drawing;
		floodFill(point, contour, pieceColour, 0, true);
		_drawing = drawing;
		return true;
	}

	MinMaxHSV contourColourAnalysis(Point center, vector<Point>& contour) {
		count = 0;
		MinMaxHSV colour;
		Point2f floatCenter = Point2f((float) center.x, (float) center.y);
		//printf("Starting floodfill: \n\n\n");
		floodFill(floatCenter, contour, colour, 0, true);
		//printf("\n\n\n");
		if(count) {
			colour.average[0] = colour.sum[0] / count;
			colour.average[1] = colour.sum[1] / count;
			colour.average[2] = colour.sum[2] / count;
		}
		return colour;
	}

	MinMaxHSV squareColourAnalysis(Square _square, int colourReferenceType, bool init, vector<vector<Point> > &_drawing) {
		drawing = _drawing;
		FPoint southEast = rotatePoint(_square.southEast, _square.rotation);
		FPoint northWest = rotatePoint(_square.northWest, _square.rotation);
		FPoint southWest = rotatePoint(_square.southWest, _square.rotation);
		FPoint northEast = rotatePoint(_square.northEast, _square.rotation);
		FPoint center = rotatePoint(_square.center, _square.rotation);

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

		/*if(colourReferenceType) printf("WHITE: \n");
		if(colourReferenceType) printf("BLACK: \n");*/
		if(colourReferenceType == 1) floodFill(Point2f(center.x, center.y), _contour, squareColour[colourReferenceType], 5, init);
		else floodFill(Point2f(center.x, center.y), _contour, squareColour[colourReferenceType], 7, init);

		cout << endl;
		if(false) {
			cout << "Black max: " << toGrey(squareColour[0].max) << endl;
			cout << "Black min: " << toGrey(squareColour[0].min) << endl;
			cout << "White max: " << toGrey(squareColour[1].max) << endl;
			cout << "White min: " << toGrey(squareColour[1].min) << endl;
		} else {
			cout << "Black max: " << squareColour[0].max << endl;
			cout << "Black min: " << squareColour[0].min << endl;
			cout << "White max: " << squareColour[1].max << endl;
			cout << "White min: " << squareColour[1].min << endl;
		}
		for(int i = 0; i < 1; i++) {
			printMarker(Point((int) _square.center.x+i,(int) _square.center.y+1), drawing, 40);	
		}

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
		
		if(point.x > COLS || point.y > ROWS || point.x < 0 || point.y < 0) return;
		if(seen[(int)point.x][(int)point.y]) return;
		if(pointPolygonTest(contour, point, !!thresh) < thresh) return;

		seen[(int)point.x][(int)point.y] = true;

		//Mat3b hsv;
		//Mat3b bgr(lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y)));
		//cvtColor(bgr, hsv, COLOR_BGR2HSV); 
		//Vec3b hsvColour = hsv.at<Vec3b>(0,0);
		Vec3b hsvColour = lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y));
		//hsvColour = lastFrame.at<Vec3b>(Point((int)point.x, (int)point.y)); 
		//if(thresh == 5) cout << "Type: " << hsvColour << endl;

		//cout << toGrey(hsvColour) << " (" << point << ") ";
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
			colourRange.sum[0] = hsvColour[0];
			colourRange.sum[1] = hsvColour[1];
			colourRange.sum[2] = hsvColour[2];

			count++;
		} else {
				
			//maxDiff(colourRange.max, hsvColour) < 40 && maxDiff(colourRange.min, hsvColour) < 40

				printMarker(Point((int) point.x,(int) point.y), drawing, 1);
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
		int colour1Grey =  (colour1[0] + colour1[1] + colour1[2]) / 3;
		int colour2Grey =  (colour2[0] + colour2[1] + colour2[2]) / 3;

		if(colour1Grey > colour2Grey) {
			return colour1;
		} else {
			return colour2;
		}
	}

	int toGrey(Vec3b bgr) {
		return (bgr[0] + bgr[1] + bgr[2]) / 3;
	}

	int maxDiff( Vec3b vec1, Vec3b vec2 ) {
		int diff1 = abs(vec1[0] - vec2[0]);
		int diff2 = abs( vec1[1] - vec2[1]);
		int diff3 = abs( vec1[2] - vec2[2]);
		return max(max(diff1, diff2), diff3);
	}

	Vec3b minHSV(Vec3b colour1, Vec3b colour2) {
		int colour1Grey =  (colour1[0] + colour1[1] + colour1[2]) / 3;
		int colour2Grey =  (colour2[0] + colour2[1] + colour2[2]) / 3;

		if(colour1Grey > colour2Grey) {
			return colour2;
		} else {
			return colour1;
		}
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
	char chessBoard[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};

	ContourMap contourMap[FROWS][FCOLS];

	vector<vector<Point> > contours;
	vector<vector<Point> > drawing;
	vector<Vec4i> hierarchy;

	Positions positions;

	public:
	void findChessPieces(
			Mat& _gray_lastFrame,
			Mat& _lastFrame,
			Mat& _detectedEdges,
			vector<vector<Point> >& _contours,
			vector<Vec4i> _hierarchy,
			vector<vector<Point> >& _drawing,
			vector<Square>& _localSquareList,
			vector<Square>& _globalSquareList
			) {
		drawing.clear();
		drawing = _drawing;

		hierarchy = _hierarchy;

		//contourMap[COLS][ROWS] = {};
		/*for(int i = 0; i < COLS; i++) {
			for(int j = 0; j < ROWS; j++) {
				contourMap[i][j].active = false;
			}
		}*/
		printMarker(Point(300, 300), drawing, 90);
		filterChessPieces(_detectedEdges, _localSquareList, _lastFrame, _lastFrame);

		bool found = false;
		int contourNo = 0;
		contours.clear();
		contours = _contours;
		for (int i = 0; i < OVERSIZED_BOARD; i++) for(int j = 0; j < OVERSIZED_BOARD; j++) chessBoard[i][j] = 0;
		/*for (size_t i = 0; i < contours.size(); i++) {
			bool skip = false;
			for (size_t j = 0; j < contours[i].size(); j++) {
				if(skip) {
					skip = false;
					break;
				}
				for (int l = 0; l < _localSquareList.size(); l++) {
					if(found) break;
					Square square = _localSquareList[l];
					FPoint manualCenter = rotatePoint(_localSquareList[l].center, -square.rotation);

					int centerX = (int) manualCenter.x;
					int centerY = (int) manualCenter.y;
					int spacing = (int) square.spacing;

					int spacingOffset = (int)4*spacing/8;
					int lowerY = centerY - spacingOffset;
					int upperY = centerY + spacingOffset;
					int lowerX = centerX - spacingOffset;
					int upperX = centerX + spacingOffset;
								
					printMarker(Point(manualCenter.x, manualCenter.y), drawing, 2);
					if(l == 25) {
						if(contours[i][j].x > lowerX && contours[i][j].x < upperX) {
							if(contours[i][j].y > lowerY && contours[i][j].y < upperY) {
								if(contourNo == 10) {
									vector<vector<Point> > smallContours = shrinkContour(contours[i], 1, (int) (centerX -  spacing/2), (int)(centerY - spacing/2));
									int tSize  = (int) spacing * 1.5 ;
									Mat3b _template(tSize, tSize * 0.5);
									Mat3b _black(tSize, tSize * 0.5);
									_black.setTo(Scalar(0,0,0));
									drawContours( _template, smallContours, 0, Scalar( 255, 255, 255 ), 2, 8, _hierarchy, 0, Point() );
									drawContours( _black, smallContours, 0, Scalar( 255, 255, 255 ), 2, 8, _hierarchy, 0, Point() );

									Mat3b _canny(ROWS, COLS);
									Mat3b dst, mask;
									dst.create(ROWS, COLS);
									mask.create(ROWS, COLS);
									for( int v = 0; v < _contours.size(); v++) {
										drawContours( _canny, contours, v, Scalar( 255, 255, 255 ), 2, 8, _hierarchy, 0, Point() );
									}
									//matchTemplate(_canny, _template, dst, TM_CCORR_NORMED, mask);
									//let result = minMaxLoc(dst, mask);

									imwrite("template.jpg", _template);
									imwrite("source.jpg", _canny);
									imwrite("mask4.jpg", _black);
									//imshow("dst", dst);
									imshow("Mask", _canny);
									waitKey(0);

								}
								contourNo++;
								skip = true;
								break;
							}
						}
					}
				}
				//printf("X: %d Y: %d\n", contours[i][j].x, contours[i][j].y);
			}
		}*/
		printf("FINISHED: %d\n", contourNo);
		return;
		for (int i = 0; i < _globalSquareList.size(); i++) {
			if(_globalSquareList[i].occupied) {
				int x = _globalSquareList[i].x;
				int y = _globalSquareList[i].y;
				chessBoard[x][y] = '0';
			}
		}
		int count = 0;
		printf("Square coloru analysis\n");
		ColourAnalysis squareColourAnalysis(_lastFrame);
		for (int i = 0; i < _localSquareList.size(); i++) {
			Square square = _localSquareList[i];
			if(i == 25) {
				printf("Square rotation: %f\n", square.rotation);
				bool init = true;
				//MinMaxHSV result = squareColourAnalysis.squareColourAnalysis(square, (square.x + square.y) % 2, init, drawing);
			}
		}
		_drawing = drawing;
		return;
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
	struct SquareInfo {
		vector<Point> cContours;
		Square square;
	};
	void filterChessPieces(Mat& edges, vector<Square>& _localSquareList, Mat& frame, Mat originalFrame) {

		if(_localSquareList.size() != 64) return;
		Mat clone = originalFrame.clone();
		ColourAnalysis analysis(clone);
		//imshow("FRAME", frame);
		//waitKey(0);
		for(int i = 0; i < _localSquareList.size(); i++) {
			Square square = _localSquareList[i];
			printSquare(rotateSquare(square, { .rotation = square.rotation }), edges, Scalar(0));
		}

		/*vector<Vec4i> lines;
		HoughLinesP(edges, lines, 1, 2 * CV_PI/180, 30, 20, 5);
		for( size_t i = 0; i < lines.size(); i++) {
			Point pt1 = Point(lines[i][0], lines[i][1]);
			Point pt2 = Point(lines[i][2], lines[i][3]);
			line( edges, pt1, pt2, Scalar(0), 10, LINE_AA);
		}*/
		/*lines.clear();
		HoughLinesP(edges, lines, 1, 2 * CV_PI/180, 30, 20, 5);
		for( size_t i = 0; i < lines.size(); i++) {
			Point pt1 = Point(lines[i][0], lines[i][1]);
			Point pt2 = Point(lines[i][2], lines[i][3]);
			line( edges, pt1, pt2, Scalar(0), 3, LINE_AA);
		}*/
		/*vector<Vec3f> circles;
		HoughCircles(edges, circles, HOUGH_GRADIENT, 30, 1, 30, 10, 1, 30);
		for( size_t i = 0; i < circles.size(); i++) {
			Vec3i c = circles[i];
			Point center = Point(c[0], c[1]);

			int radius = c[2];
			circle( edges, center, radius, Scalar(100), 3, LINE_AA );
		}*/
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		for( int i = 0; i < contours.size(); i++) {
			drawContours( edges, contours, i, Scalar(255), 4, 8);
		}
		//imshow("test", edges);
		//waitKey(0);
		vector<vector<Point> > pieceContours;
		vector<Vec4i> pieceHierarchy;
		findContours( edges, pieceContours, pieceHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		positions.beginBoardUpdate();
		vector<vector<Point> > pieceHulls(pieceContours.size());
		vector<SquareInfo> squareHulls(_localSquareList.size());
		for( int i = 0; i < pieceContours.size(); i++) {
			vector<Point> approxPoly;
			convexHull(pieceContours[i], pieceHulls[i]); 
			approxPolyDP(pieceContours[i], approxPoly, 3.0, true); 
			Moments m = moments(pieceHulls[i]);
			if(m.m00 > 100 && m.m00 < 4800) {
				bool isPiece = false;
				Square square;
				int squareIndex;
				Point center = Point(int(m.m10 / m.m00), int(m.m01 / m.m00));
				for(int j = 0; j < _localSquareList.size(); j++) {
					float shift = -_localSquareList[j].spacing / 5 ;
					Square shrinkSquare = _localSquareList[j];
					shrinkSquare.northEast = shiftPoint(shrinkSquare.northEast, shift, shift);
					shrinkSquare.northWest = shiftPoint(shrinkSquare.northWest, -shift, shift);
					shrinkSquare.southWest = shiftPoint(shrinkSquare.southWest, -shift, -shift);
					shrinkSquare.southEast = shiftPoint(shrinkSquare.southEast, shift, -shift);
					shrinkSquare = rotateSquare(shrinkSquare, { .rotation = shrinkSquare.rotation });

					vector<Point> bounds;
					bounds.push_back(fPointToPoint(shrinkSquare.northEast));
					bounds.push_back(fPointToPoint(shrinkSquare.northWest));
					bounds.push_back(fPointToPoint(shrinkSquare.southWest));
					bounds.push_back(fPointToPoint(shrinkSquare.southEast));
					bounds.push_back(fPointToPoint(shrinkSquare.northEast));
					if(pointPolygonTest(bounds, center, false) > 0) {
						isPiece = true;
						squareIndex = j;
						square = _localSquareList[j];
					}
				}
				if(isPiece) {
					if(!squareHulls[squareIndex].cContours.size()) {
						squareHulls[squareIndex].cContours = pieceContours[i];
					} else {
						for(int p = 0; p < pieceContours[i].size(); p++) {
							squareHulls[squareIndex].cContours.push_back(pieceContours[i][p]);
						}
					}
					squareHulls[squareIndex].square = square;


				}
				//
				/*vector<vector<Point> > all;
				all.clear();
				all.push_back(approxPoly);
				drawContours( originalFrame, all, 0, Scalar(255, 255, 255), 2, 8);*/
			}
		}
		for( int i = 0; i < _localSquareList.size(); i++) {
			vector<Point> hull;
			Square square = squareHulls[i].square;
			if(squareHulls[i].cContours.size()) {
				convexHull(squareHulls[i].cContours, hull); 
				//cout << "HULL: " << squareHulls[i].cContours.size() << endl;
				vector<vector<Point> > hullsPrint;
				hullsPrint.push_back(hull);
				drawContours( frame, hullsPrint, 0, Scalar(255, 255, 255), 2, 8);

				Moments m = moments(hull);
				Point center = Point(int(m.m10 / m.m00), int(m.m01 / m.m00));

				//MinMaxHSV res = analysis.contourColourAnalysis(center, hull);
				Vec3b comp = colourPatch(center, originalFrame);

				//cout << "Comp: " << toGrey(comp) << endl;
				Point position = Point(square.x, square.y); 
				if(toGrey(comp) < 50) {
					//BLACK
					circle(frame, center, 10, Scalar(0, 75, 150), 3, LINE_4, 0);
					positions.insertChessPiece(Black, position);
				} else {
					Vec3b hsvComp = toHSV(comp);
					cout << "HSV: " << hsvComp << endl;
					if(((hsvComp[0] > 0 && hsvComp[0] < 15) || hsvComp[0] > 170) && hsvComp[2] < 100) {
						cout << "Black: " << hsvComp << endl;
						//BLACK
						circle(frame, center, 10, Scalar(0, 75, 150), 3, LINE_4, 0);
						positions.insertChessPiece(Black, position);
					} else {
						//WHITE
						circle(frame, center, 10, Scalar(255, 255, 255), 3, LINE_4, 0);
						positions.insertChessPiece(White, position);

					}
				}
			}
		}
		for( int i = 0; i < pieceHulls.size(); i++) {
			//drawContours( frame, pieceHulls, i, Scalar(255, 255, 255), 2, 8);
			//drawContours( frame, contours, i, Scalar(255, 255, 255), 4, 8);
		}

		positions.finishBoardUpdate();
		//imshow("DETECTED EDGES: ", edges);
		//waitKey(0);
	}

	Point findPointInPolygon(vector<Point> polygon) {
		float shift = 1;
		FPoint onLine = pointToFPoint(polygon[0]);
		Point test1 = fPointToPoint(shiftPoint(onLine, shift, shift));
		Point test2 = fPointToPoint(shiftPoint(onLine, shift, -shift));
		Point test3 = fPointToPoint(shiftPoint(onLine, -shift, -shift));
		Point test4 = fPointToPoint(shiftPoint(onLine, -shift, shift));
		Point test5 = fPointToPoint(shiftPoint(onLine, shift, 0));
		Point test6 = fPointToPoint(shiftPoint(onLine, -shift, 0));
		Point test7 = fPointToPoint(shiftPoint(onLine, 0, shift));
		Point test8 = fPointToPoint(shiftPoint(onLine, 0, -shift));
		if(pointPolygonTest(polygon, test1, false) > 0) return test1;
		if(pointPolygonTest(polygon, test2, false) > 0) return test2;
		if(pointPolygonTest(polygon, test3, false) > 0) return test3;
		if(pointPolygonTest(polygon, test4, false) > 0) return test4;
		if(pointPolygonTest(polygon, test5, false) > 0) return test5;
		if(pointPolygonTest(polygon, test6, false) > 0) return test6;
		if(pointPolygonTest(polygon, test7, false) > 0) return test7;
		if(pointPolygonTest(polygon, test8, false) > 0) return test8;
		return Point(-1, -1);
	}

	Vec3b colourPatch(Point center, Mat &frame) {
		FPoint _center = { .x = (float) center.x, .y = (float) center.y };
		int shift = 5;
		int b = 0, g = 0, r = 0;
		int grey = 0;
		for(float i = -shift; i < shift; i++) {
			for(float j = -shift; j < shift; j++) {
				FPoint shiftPointed = shiftPoint(_center, i,j);
				Point point = fPointToPoint(shiftPoint(_center, i,j));

				grey += greyAt(point, frame);		

				Vec3b bgr = bgrAt(point, frame);
				b += bgr[0];
				g += bgr[1];
				r += bgr[2];
			}
		}
		int count = 4 * shift * shift;
		Vec3b bgrAverage(b / count, g / count, r / count);
		Vec3b hsvAverage = bgrAverage;
		//printf("HSV: %d %d %d   BGR: %d %d %d\n", hsvAverage[0], hsvAverage[1], hsvAverage[2], bgrAverage[0], bgrAverage[1], bgrAverage[2]);
		//printf("Colour: %d ", grey / count);

		return hsvAverage;
	}

	int greyAt(Point pos, Mat &frame) {
		Mat3b bgrMat(frame.at<Vec3b>(pos));
		Vec3b bgr = bgrMat.at<Vec3b>(0,0);
		return toGrey(bgr);
	}

	Vec3b toHSV(Vec3b bgr) {
		Mat3b hsvMat;
		Mat3b bgrMat(bgr);
		cvtColor(bgrMat, hsvMat, COLOR_BGR2HSV);
		return hsvMat.at<Vec3b>(0,0);
	}
	Vec3b bgrAt(Point pos, Mat &frame) {
		Mat3b bgrMat(frame.at<Vec3b>(pos));
		return bgrMat.at<Vec3b>(0,0);
	}

	int toGrey(Vec3b bgr) {
		return (bgr[0] + bgr[1] + bgr[2]) / 3;
	}

	vector<vector<Point> > shrinkContour(vector<Point> contour, float scaleDown, int transformX, int transformY) {
		printf("\nSHRINK: %d %d ",  transformX, transformY);
		for(int i = 0; i < contour.size(); i++) {
			contour[i].x -= transformX;
			contour[i].y -= transformY;
			contour[i].x = (int) contour[i].x * scaleDown;
			contour[i].y = (int) contour[i].y * scaleDown;
			printf("| %d %d |", contour[i].x, contour[i].y);
		}
		vector<vector<Point> > smallContours;
		smallContours.push_back(contour);
		return smallContours;
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
	void printSquare(Square _square, Mat &_display, Scalar colour) {
		int thickness = 20;
		line( _display,
				Point(_square.northEast.x, _square.northEast.y),
				Point(_square.northWest.x, _square.northWest.y),
				colour, thickness, 3);

		line( _display,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.southWest.x, _square.southWest.y),
				colour, thickness, 3);

		line( _display,
				Point(_square.southWest.x, _square.southWest.y),
				Point(_square.northWest.x, _square.northWest.y),
				colour, thickness, 3);

		line( _display,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.northEast.x, _square.northEast.y),
				colour, thickness, 3);
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


