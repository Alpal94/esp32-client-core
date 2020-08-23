#include "lib/shared.h"
#include "lib/determine-chessboard.h"
#include "lib/determine-chess-peices.h"
#include "lib/calibrate-camera.h"
#include "lib/hsv-experiment.h"

#define PRINT_LINES false
#define PRINT_HOUGH_LINES true
#define NEW_LINE_POINTS false
#define PRINT_CONTOURS false
#define HSV_EXPERIMENT false
#define CALIBRATE true
#define READ_CALIBRATION true
#define GRADIENT_VERTICAL 999
#define upper 5
#define lower 5

#define DEBUG_MERGED_LINES true
using namespace cv;
class StreamProcessing {
	private:
	Mat lastFrame;
	Mat display;
	Mat gray_lastFrame;
	Mat HSV_lastFrame;
	Mat next_HSV_lastFrame;
	int frameReference = 0;
	CalibrateCamera calibrate;
	DetermineChessBoard determineChessboard;
	DetermineChessPieces determineChessPieces;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	const char* window_name = "Chess detector";

	Mat laplacianSharpening(Mat src) {
		Mat kernel = (Mat_<float>(3,3) <<
			1,  1, 1,
			1, -8, 1,
			1,  1, 1);
		Mat imgLaplacian;
		filter2D(src, imgLaplacian, CV_32F, kernel);
		Mat sharp;
		src.convertTo(sharp, CV_32F);
		Mat imgResult = sharp - imgLaplacian;
		imgResult.convertTo(imgResult, CV_8UC3);
		imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
		return imgResult;
	}

	Mat evaluateContours(float scale, vector<vector<Point> > &_contours) {
		Mat dst, detected_edges;
		int lowThreshold = 40;
		//const int max_lowThreshold = 100;
		const int ratio = 3;
		const int kernel_size = 3;
		//int thresh = 100;
		//int max_thresh = 255;
		RNG rng(12345);


		resize(gray_lastFrame, gray_lastFrame, Size(), scale, scale);
		resize(lastFrame, lastFrame, Size(), scale, scale);

		//bool useLaplacianSharpening = false;

		bilateralFilter(lastFrame, detected_edges, 5, 30, 30);
		cvtColor( detected_edges, detected_edges, COLOR_BGR2GRAY );
		//adaptiveThreshold(detected_edges, detected_edges, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, 0);

		//blur( detected_edges, detected_edges, Size(3,3) );
		//threshold(detected_edges, detected_edges, 100, 255, THRESH_BINARY);
		//resize(detected_edges, detected_edges, Size(), 0.5, 0.5);
		//resize(detected_edges, detected_edges, Size(), 2.0, 2.0);

		Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

		_contours.clear();
		//CHAIN_APPROX_NONE
		findContours( detected_edges, _contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		Mat drawing;
		bool isContours = false;
		//bool showTrueColour = false;
		if(isContours) {

			drawing = Mat::zeros( detected_edges.size(), CV_8UC3 );
			for( int i = 0; i < _contours.size(); i++) {
				Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				drawContours( drawing, _contours, i, color, 2, 8, hierarchy, 0, Point() );
			}
		}
		return detected_edges;

	}

	void timer(auto start, char* message) {
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		ALOG("\nTime elapsed: %lld (%s)\n", microseconds, message);
	}

	RobotPosition robotPosition = { .x = 18, .y = 12, .z = -5};
	void manageRobot() {

		auto start = std::chrono::high_resolution_clock::now();
		//cvtColor( lastFrame, lastFrame, COLOR_BGR2HSV );
		float scale = 0.1;
		Mat detected_edges = evaluateContours(scale, contours);

		vector<vector<Point> > squares;
		squares.clear();
		//cvtColor( lastFrame, lastFrame, COLOR_BGR2HSV );
		auto startDetermineLines = std::chrono::high_resolution_clock::now();
		vector<LineMetadata> mergedLines = determineLines(detected_edges, squares);
		if(true) {
			timer(startDetermineLines, (char*) "DetermineLines");
			determineChessboard.findChessboardSquares(
					mergedLines,
					squares,
					gray_lastFrame,
					lastFrame,
					display,
					robotPosition
			);
			vector<Square> localSquareList = determineChessboard.getLocalSquareList();
			vector<Square> globalSquareList = determineChessboard.getGlobalSquareList();
			determineChessPieces.findChessPieces(
				gray_lastFrame,
				lastFrame,
				contours,
				hierarchy,
				squares,
				localSquareList,
				globalSquareList
			);

			/*MinMaxHSV blackSquare = determineChessPieces.getSquareColour(0);
			MinMaxHSV whiteSquare = determineChessPieces.getSquareColour(1);

			cout << "Black square: " << blackSquare.min << " " << blackSquare.max << " White square: " << whiteSquare.min << " " << whiteSquare.max << endl;

			printf("Unordered map size: %ld\n", blackSquare.hist.size());

			for ( auto it = blackSquare.hist.begin(); it != blackSquare.hist.end(); ++it ) {
				if(it->second > 1) {
					string key = it->first;
					uchar b = (uchar) key[0];
					uchar g = (uchar) key[1];
					uchar r = (uchar) key[2];
					printf("Key: %d %d %d count: %d\n", b,g,r, it->second);
				}
			}

			robotPosition = traverseChessboard(robotPosition);

			//cvtColor( lastFrame, lastFrame, COLOR_BGR2HSV );
			Mat mask;
			Mat white;
			Mat black;
			Mat next;

			Mat3b bgr;
			Mat3b hsv(whiteSquare.min);
			cvtColor(hsv, bgr, COLOR_HSV2BGR); 
			Vec3b white_bgr_min = whiteSquare.min; //bgr.at<Vec3b>(0,0);

			Mat3b hsv2(whiteSquare.max);
			cvtColor(hsv2, bgr, COLOR_HSV2BGR); 
			Vec3b white_bgr_max = whiteSquare.max; //bgr.at<Vec3b>(0,0);

			Mat3b hsv3(blackSquare.min);
			cvtColor(hsv3, bgr, COLOR_HSV2BGR); 
			Vec3b black_bgr_min = blackSquare.min;//bgr.at<Vec3b>(0,0);

			Mat3b hsv4(blackSquare.max);
			cvtColor(hsv4, bgr, COLOR_HSV2BGR); 
			Vec3b black_bgr_max = blackSquare.max;//bgr.at<Vec3b>(0,0);

			lastFrame.copyTo(next);
			inRange( lastFrame, Scalar(0,0,0), Scalar(255,255,255), mask );
			cvtColor( mask, mask, COLOR_GRAY2BGR );
			next.setTo(Scalar(0,0,0), mask);

			inRange( lastFrame, Scalar(white_bgr_min[0], white_bgr_min[1], white_bgr_min[2]), Scalar(white_bgr_max[0], white_bgr_max[1], white_bgr_max[2]), mask );
			cvtColor( mask, mask, COLOR_GRAY2BGR );
			next.setTo(Scalar(0,255,0), mask);
			//bitwise_or(mask, white, white);

			inRange( lastFrame, black_bgr_min, black_bgr_max, mask );
			cvtColor( mask, mask, COLOR_GRAY2BGR );
			next.setTo(Scalar(0,0,255), mask);
			//bitwise_and(mask, lastFrame, lastFrame);*/

		}
		//bitwise_or(black, white, lastFrame);
		//lastFrame = next;
		if(!HSV_EXPERIMENT) drawSquares(lastFrame, squares, gray_lastFrame.rows, gray_lastFrame.cols);
		resize(lastFrame, lastFrame, Size(), 0.5 / scale, 0.5 / scale);
		resize(detected_edges, detected_edges, Size(), 0.5 / scale, 0.5 / scale);

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		ALOG("\nTime elapsed: %lld\n", microseconds);
	}

	enum TraversalState { Left, Right } traversalState;
	RobotPosition traverseChessboard(RobotPosition position) {
		RobotPosition nextPosition = position;

		switch(traversalState) {
			case TraversalState::Left:
				nextPosition.z -= 0.5;
				break;
			case TraversalState::Right:
				nextPosition.z += 0.5;
				break;
		}
		if(nextPosition.z  < -2.5) {
			traversalState = TraversalState::Right;
		} else if(nextPosition.z  > 2.5) {
			traversalState = TraversalState::Left;
		}
		nextPosition.y = 12;
		nextPosition.x = 18;

		if(STREAM_CAMERA) {
			if(setRobotPosition(nextPosition)) return nextPosition;
			else return position;
		} else {
			return nextPosition;
		}
	}

	/*void traverseChessboard() {
		for(float z = -5; z < 5; z+=2) {
			for(float x = 10; x < 20; x+=0.2) {
				setRobotPosition({.x = x, .y = 10, .z = z});
			}
		}
	}*/

	bool setRobotPosition(RobotPosition _position) {
		printf("SETTING ROBOT POSITION\n");
		CURL *curl;
		CURLcode res;

		curl = curl_easy_init();
		if(curl) {
			printf("CURL Init\n");
			char strBuffer[150];
			sprintf(strBuffer,  "http://chessrobot.local/setpos?x=%f&y=%f&z=%f", _position.x, _position.y, _position.z);
			printf("URL STARTED");
			printf("THIS URL: %s\n", strBuffer);
			curl_easy_setopt(curl, CURLOPT_URL, strBuffer);
			res = curl_easy_perform(curl);
			if(res != CURLE_OK) { 
				printf("NOT OK\n");
				fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
				return false;
			} else {
				printf("OK\n");
			}

			//printf("Robot position set\n");
			curl_easy_cleanup(curl);
			return true;
		}
		return false;
	}

	vector<LineMetadata> determineLines(Mat& edges, vector<vector<Point> >& squares) {
		vector<Vec4i> houghLines;
		vector<LineMetadata> lines;
		HoughLinesP(edges, houghLines, 1, CV_PI/180, 60, 30, 5);
		printf("Hough begin: %ld\n\n", houghLines.size());
		for( size_t i = 0; i < houghLines.size(); i++ ) {
			if(PRINT_HOUGH_LINES) {
				line( lastFrame,
					Point(houghLines[i][0], houghLines[i][1]),
					Point(houghLines[i][2], houghLines[i][3]),
					Scalar(0,0,255), 1, 1);
			}
			float gradient, intercept, xIntercept, xGradient;
			if(twoPointLineCalc(gradient, intercept, xIntercept, xGradient, 
				Point(houghLines[i][0], houghLines[i][1]),
				Point(houghLines[i][2], houghLines[i][3])
			)) {
				LineMetadata data = {
					.gradient = gradient,
					.intercept = intercept,
					.xIntercept = xIntercept,
					.xGradient = xGradient,
					.bounds = houghLines[i]
				};
				lines.push_back(data);
			}
		}
		long long averageTime = 0;
		auto contourTime = std::chrono::high_resolution_clock::now();

		auto elapsed = std::chrono::high_resolution_clock::now() - contourTime;
		averageTime = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		printf("\nAverage time: %lld contours: %ld (CONTOUR)\n", averageTime, contours.size());

		contourTime = std::chrono::high_resolution_clock::now();

		if(PRINT_LINES) {
			for( size_t i = 0; i < lines.size(); i++) {
				squares.push_back(calcPrintLine(lines[i]));
				
			}
		}
		return lines;
	}

	vector<Point> calcPrintLine(LineMetadata line) {
		float gradient = line.gradient;
	       	float intercept = line.intercept;
		float xGradient = line.xGradient;
	       	float xIntercept = line.xIntercept;
		printf("Gradient: %f\n", gradient);

		vector<Point> linePoints;
		linePoints.clear();

		if(false && !isnan(xGradient) && !isnan(xIntercept) && !isinf(xGradient) && !isinf(xIntercept)) {
			for( size_t x = 0; x < ROWS; x++ ) {
				float y = x * xGradient + xIntercept;
				if(y > 0 && y < COLS) {
					Point point(y,x);
					linePoints.push_back(point);
				}
			}
		} else if(!isnan(gradient) && !isnan(intercept) && !isinf(gradient) && !isinf(intercept)) {
			printf("Print normal: %f %f\n", gradient, intercept);
			for( size_t x = 0; x < COLS; x++ ) {
				float y = x * gradient + intercept;
				if(y > 0 && y < ROWS) {
					Point point(x,y);
					linePoints.push_back(point);
				}
			}
		} else {
			printf("Warning: INVALID LINE\n");
		}

		return linePoints;
	}
	static bool sortSegmentsX(SegmentMetadata a, SegmentMetadata b) {
		return a.start.x < b.start.x;
	}
	static bool sortSegmentsY(SegmentMetadata a, SegmentMetadata b) {
		return a.start.y < b.start.y;
	}
	static bool sortLineSegments(pair<Point, Point> a, pair<Point, Point> b) {
		return a.first.x < b.first.x;
	}

	static bool sortLinesGradients(LineMetadata a, LineMetadata b) {
		return a.gradient < b.gradient;
	}

	float gradientToIndex(float gradient) {
		return roundDP(gradient, 1);
	}
	float interceptToIndex(float intercept) {
		int rounded = (int) roundDP(intercept, 0);
		float mod = (float) (rounded - rounded % 2);
		return mod;
	}
	float roundDP(float number, int dp) {
		return round(number*pow(10, dp))/pow(10, dp);
	}

	void calcLine(float gradient, float intercept, vector<Point>& contour, vector<Point>& line, size_t startIndex, size_t endIndex) {
		/*for( size_t j = 0; j < 10; j++) {
			float calcedY = (float) contour[j].x * gradient + intercept;
			float diff = fabs(calcedY - contour[j].y);
			Point linePoint(contour[j].x, (int) calcedY);
			line.push_back(contour[j]);
		}*/

		Point start = contour[startIndex];
		Point end = contour[endIndex];
		size_t jStart, jEnd;
		if(start.x < end.x) {
			jStart = start.x;
			jEnd = end.x;
		} else {
			jStart = end.x;
			jEnd = start.x;
		}
		jEnd += 0;
		if(
				fabs(((float) jStart - (float) jEnd)) > gray_lastFrame.cols || 
				jStart < 0 ||
				jStart > gray_lastFrame.cols ||
				jEnd < 0 ||
				jEnd > gray_lastFrame.cols 

		) return;

		bool isGenerated = true;
		if(isGenerated && pixelDist(start, end) > 0) {
			for( size_t j = jStart; j <= jEnd; j++) {
				float calcedY = (float) j * gradient + intercept;
				Point linePoint(j, (int) calcedY);
				
				if(		linePoint.x < gray_lastFrame.cols &&
						linePoint.y < gray_lastFrame.rows &&
						linePoint.y >= 0 &&
						linePoint.x >= 0 ) {
					
					line.push_back(linePoint);
				}
			}
			if(jStart == jEnd && gradient > 10) {
				for( size_t j = startIndex; j < endIndex; j++) {
					line.push_back(contour[j]);		
				}
			}
		} else {
			for( size_t j = startIndex; j < endIndex; j++) {
				line.push_back(contour[j]);		
			}
		}
	}

	void pureCalcLine(float gradient, float intercept, vector<Point>& line, size_t start=0, size_t end=COLS) {

		for( size_t j = start; j < end; j++) {
			float calcedY = (float) j * gradient + intercept;
			Point linePoint(j, (int) calcedY);
			
			if(		linePoint.x < gray_lastFrame.cols &&
					linePoint.y < gray_lastFrame.rows &&
					linePoint.y >= 0 &&
					linePoint.x >= 0 ) {
				
				line.push_back(linePoint);
			}
			if(gradient >= GRADIENT_VERTICAL && calcedY > 0 && calcedY < COLS) {
				for(size_t z = 0; z < ROWS; z++) {
					Point verticalPoint(j, z);
					line.push_back(verticalPoint);
				}
			}
		}
	}

	size_t calcLineBestFit(float& gradient, float& intercept, float& xIntercept, float& xGradient, vector<Point> contour, int start) {
		int lineEnd = 0;
		bool lineFound = false;
		int upDown = 0;
		int zeroPosition = -1;
		int onePosition = -1;
		int twoPosition = -1;
		/*for(int end = start; end < start + 5; end++) {
			if(end < contour.size()) {
				Point dir = direction(contour[end-1], contour[end]);

				if(zeroPosition > -1 && dir.y == 0) zeroPosition = end;
				if(zeroPosition > -1 && zeroPosition % 3 == end % 3 && dir.y != 0) return false;
				if(onePosition > -1 && dir.y == 1) onePosition = end;
				if(onePosition > -1 && onePosition % 3 == end % 3 && dir.y != 1) return false;
				if(twoPosition > -1 && dir.y == -1) twoPosition = end;
				if(twoPosition > -1 && twoPosition % 3 == end % 3 && dir.y != -1) return false;

				if(!upDown && dir.y) upDown = dir.y;
				if(upDown && !(dir.y == 0 || dir.y == upDown)) return false;
				printf("Direction: %d, %d\n", dir.x, dir.y);
			}
		}*/
		for(int end = start+1; end < start + 20; end++) {

			if(end < contour.size() && pixelDist(contour[start], contour[end]) > 7) {
				float tmpGradient, tmpIntercept;
				float tmpXGradient, tmpXIntercept;
				if(calcGradientIntercept(
						tmpGradient, tmpIntercept, 0.0,
						tmpXGradient, tmpXIntercept, 
						start, end, contour)) {
					lineFound = true;
					gradient = tmpGradient;
					intercept = tmpIntercept;
					lineEnd = end;
				}
				
			}
		}
		if(abs(gradient) > 0.2) return false;
		return lineEnd;
	}

	Point direction(Point point1, Point point2) {
		int distance = (int) pixelDist(point1, point2);
		printf("Distance: %d\n", distance);
		if(distance) {
			int x = (point2.x - point1.x) / distance;
			int y = (point2.y - point1.y) / distance;

			Point dir(x, y);
			return dir;
		} else {
			Point dir(0,0);
			return dir;
		}
	}

	bool twoPointLineCalc(float& gradient, float& intercept, float& xIntercept, float& xGradient, Point point1, Point point2) {
		//if(pixelDist(point1, point2) < 4) return false;

		float x1 = (float) point1.x; float x2 = (float) point2.x;
		float y1 = (float) point1.y; float y2 = (float) point2.y;

		gradient = (y2 - y1) / (x2 - x1);
		xGradient = (x2 - x1) / (y2 - y1);

		if(isnan(gradient) || isinf(gradient)) gradient = GRADIENT_VERTICAL;


		intercept = y1 - x1 * gradient;
		xIntercept = x1 - y1 * xGradient;

		if(isnan(gradient) || isnan(intercept)) return false;
		//if(fabs(gradient) > lower && fabs(gradient) < upper) return false;

		return true;

	}
	
	bool calcGradientIntercept(
			float& gradient, float& intercept, float threshold,
			float& xGradient, float& xIntercept, 
			int start, int end, vector<Point> contour
	) {

		//if(pixelDist(contour[start], contour[end]) < 1) return false;

			
		if(!twoPointLineCalc(gradient, intercept, xIntercept, xGradient, contour[start], contour[end])) {
			return false;
		}

		for ( size_t j = start; j < end; j++) {
			float diff = calcDiff(gradient, intercept, contour[j].x, contour[j].y);
			float xDiff = calcDiff(xGradient, xIntercept, contour[j].y, contour[j].x);
			if(diff > threshold && xDiff > threshold) {
				//return false;
			}
		}

		return true;
	}

	float calcDiff(float gradient, float intercept, int x, int y) {
		float calcedY = (float) x * gradient + intercept;
		float calcedX = (float) ((float) y - intercept) / gradient;
		return min(fabs(calcedY - (float) y), fabs(calcedX - x));
	}
	float calcDiffX(float gradient, float intercept, int x, int y) {
		float calcedX = (float) ((float) y - intercept) / gradient;
		return fabs(calcedX - x);
	}

	static void drawSquares( Mat& image, const vector<vector<Point> >& squares, int rows, int cols) {
		srand (0);
		for (size_t i = 0; i < squares.size(); i++) {
			//const Point* p = &squares[i][0];
		//	int n = (int) squares[i].size();

			//Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
			Vec3b colour(rand() % 255, rand() % 255, rand() % 255);
			for(size_t j = 0; j < squares[i].size(); j++) {
				//circle(image, squares[i][j], 1, color, 1, LINE_4, 0);
				size_t x = squares[i][j].x;
				size_t y = squares[i][j].y;
				if(x < cols && y < rows) {
					image.at<Vec3b>(Point(squares[i][j].x, squares[i][j].y)) = colour;
				}
			}
			//polylines(image, &p, &n, 1, false, color, 1, LINE_4);
		}
	}

	public:
	StreamProcessing() {
		if(HSV_EXPERIMENT) hsv_init();
			
	}
	void processFrame() {
		if(!CALIBRATE) {

			/*cv::cvtColor(lastFrame, lastFrame, COLOR_BGR2YUV);
			std::vector<cv::Mat> channels;
			cv::split(lastFrame, channels);
			cv::equalizeHist(channels[0], channels[0]);
			cv::merge(channels, lastFrame);
			cv::cvtColor(lastFrame, lastFrame, COLOR_YUV2BGR);*/

			if(READ_CALIBRATION) {
				calibrate.calculateFrameFromSavedCalibrationdata(lastFrame);
			}
			cvtColor(lastFrame, HSV_lastFrame, COLOR_BGR2HSV);
			cvtColor(lastFrame, gray_lastFrame, COLOR_BGR2GRAY );
			/*GaussianBlur( gray_lastFrame, gray_lastFrame, Size(5,5), 0 );
			threshold( gray_lastFrame, gray_lastFrame, 0, 255, THRESH_BINARY | THRESH_OTSU);
			Mat shadowFrame;
			float dilationSize = 200;
			resize(gray_lastFrame, shadowFrame, Size(), 1/dilationSize, 1/dilationSize);
			resize(shadowFrame, shadowFrame, Size(), dilationSize, dilationSize);
			Mat element = getStructuringElement(MORPH_RECT, Size(2*dilationSize+1, 2*dilationSize+1), Point(dilationSize,dilationSize)); 
			blur( gray_lastFrame, shadowFrame, Size(20,20) );
			dilate( shadowFrame, shadowFrame, element);

			Mat newImage;
			absdiff(shadowFrame, gray_lastFrame, newImage);*/

			//imshow (window_name, lastFrame);
			//waitKey(0);
			//adaptiveThreshold(gray_lastFrame, gray_lastFrame, 125, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 3, 5);
			//			inRange( lastFrame, Scalar(183,137,143), Scalar(255,255,255), gray_lastFrame );
			//inRange( lastFrame, Scalar(173,127,133), Scalar(255,255,255), gray_lastFrame );
			//inRange( HSV_lastFrame, Scalar(100,40,160), Scalar(150,100,255), gray_lastFrame );
			//inRange( gray_lastFrame, Scalar(160), Scalar(205), gray_lastFrame );
			//namedWindow(window_name, WINDOW_AUTOSIZE );
			manageRobot();
			char fileName[42];
			sprintf(fileName, "recorded/jpeg%d.jpg", frameReference);
			imwrite(fileName, lastFrame);
		}
	}

	void processJPEG( int nSize, char* jpeg) {
		char fileName[42];
		sprintf(fileName, "original/jpeg%d.jpg", frameReference);
		std::ofstream outfile (fileName, std::ofstream::binary);	
		outfile.write(jpeg, nSize);
		outfile.close();

		Mat rawData( 1, nSize, CV_8UC1, (void*) jpeg);
		Mat decodedImage = imdecode(rawData, IMREAD_COLOR);

		if(decodedImage.data == NULL) {
			printf("ERROR DECODING IMAGE");
		} else {
			printf("SUCCESSFULLY DECODED IMAGE: %d", frameReference);
			//calibrate.undistortImage(decodedImage);
			lastFrame = decodedImage;
			frameReference++;
			processFrame();
			if(HSV_EXPERIMENT) {
				//while(frameReference == 3) {
					hsv_processFrame(lastFrame);
					waitKey(0);
				//}
			} else {
				if(!CALIBRATE) {
					imshow (window_name, lastFrame);
				}
				waitKey(1000);
			}
			if(CALIBRATE) {
				calibrate.calculateCalibrationDataFromFrame( decodedImage );
			}
		}

		printf("\nStart: ");
		for(int i = 0; i < 5; i++) {
			printf("%x", jpeg[i]);
		}
		printf("\nEnd: ");
		for(int i = nSize - 9; i < nSize; i++) {
			printf("%x", jpeg[i]);
		}
	}

	Mat *executeChessBoardProcessing( Mat *currFrame) {
		if(currFrame->data == NULL) {
			printf("ERROR DECODING IMAGE");
		} else {
			lastFrame = *currFrame;
			frameReference++;
			processFrame();

			if(CALIBRATE) {
				calibrate.calculateCalibrationDataFromFrame( lastFrame );
			}
		}
		return &lastFrame;
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
