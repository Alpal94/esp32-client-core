#include "lib/shared.h"
#include "lib/determine-chessboard.h"
#include "lib/determine-chess-peices.h"
#include "lib/calibrate-camera.h"

#define PRINT_LINES false
#define GENERATE_LINES false
#define NEW_LINE_POINTS false
#define PRINT_CONTOURS true
#define STREAM_CAMERA false
#define CALIBRATE false
#define upper 90
#define lower 10

#define DEBUG_MERGED_LINES true
using namespace cv;
class StreamProcessing {
	private:
	Mat lastFrame;
	Mat gray_lastFrame;
	int frameReference = 0;
	CalibrateCamera calibrate;
	DetermineChessBoard determineChessboard;
	DetermineChessPieces determineChessPieces;

	vector<vector<Point> > contours;


	//const char* window_name = "Chess detector";

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
		int lowThreshold = 21;
		//const int max_lowThreshold = 100;
		const int ratio = 3;
		const int kernel_size = 3;
		//int thresh = 100;
		//int max_thresh = 255;
		RNG rng(12345);
		vector<Vec4i> hierarchy;

		resize(gray_lastFrame, gray_lastFrame, Size(), scale, scale);
		resize(lastFrame, lastFrame, Size(), scale, scale);

		//bool useLaplacianSharpening = false;

		blur( gray_lastFrame, detected_edges, Size(3,3) );
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

	RobotPosition robotPosition = { .x = 10, .y = 10, .z = 10};
	void manageRobot() {

		auto start = std::chrono::high_resolution_clock::now();

		float scale = 0.1;
		Mat detected_edges = evaluateContours(scale, contours);
		vector<vector<Point> > squares;
		squares.clear();
		vector<LineMetadata> mergedLines = determineLines(detected_edges, squares);
		determineChessboard.findChessboardSquares(
				mergedLines,
				squares,
				gray_lastFrame,
				robotPosition
		);
		vector<Square> localSquareList = determineChessboard.getLocalSquareList();
		determineChessPieces.findChessPieces(
			gray_lastFrame,
			contours,
			squares,
			localSquareList
		);
		robotPosition = traverseChessboard(robotPosition);

		drawSquares(lastFrame, squares, gray_lastFrame.rows, gray_lastFrame.cols);

		resize(lastFrame, lastFrame, Size(), 0.5 / scale, 0.5 / scale);
		resize(detected_edges, detected_edges, Size(), 0.5 / scale, 0.5 / scale);
		//imshow (window_name, lastFrame);

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		ALOG("Time elapsed: %lld\n", microseconds);
	}

	RobotPosition traverseChessboard(RobotPosition position) {
		RobotPosition nextPosition = position;
		if(nextPosition.x < 20 && nextPosition.y < 20) {
			nextPosition.x++;
			nextPosition.y++;
		} else {
			nextPosition.x--;
			nextPosition.y--;
		}
		if(true) return nextPosition;

		if(setRobotPosition(position)) return nextPosition;
		else return position;
	}

	bool setRobotPosition(RobotPosition _position) {
		printf("SETTING ROBOT POSITION\n");
		/*CURL *curl;
		CURLcode res;

		curl = curl_easy_init();
		if(curl) {
			char *strBuffer;
			sprintf(strBuffer,  "http://chessrobot.local/setpos?x=%f&y=%f&z=%f", _position.x, _position.y, _position.z);
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
		}*/
		return false;
	}

	map<float, int> gradientFrequencies;
	map<float, int> interceptFrequencies;
	vector<LineMetadata> determineLines(Mat& edges, vector<vector<Point> >& squares) {
		vector<Point> approx;
		vector<LineMetadata> lines;
		int sampleSize = 5;

		for ( size_t i = 0; i < contours.size(); i++) {
			vector<Point> contour = contours[i];
			approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.0001, false);
			if(PRINT_CONTOURS) {
				squares.push_back(contour);
			}
			float gradient, intercept;
			bool doCalcLines = true;
			if(doCalcLines) {
				for ( size_t j = 0; j < contour.size(); j++) {
					size_t lineEnd = calcLineBestFit(gradient, intercept, sampleSize, contour, j);

					if(lineEnd) {
						vector<Point> line;
						lines.push_back({
							.startIndex = j,
							.endIndex = lineEnd,
							.contourIndex = i,
							.gradient = gradient,
							.intercept = intercept,
							.line = line

						});
						j = lineEnd;
					}
				}
				for( size_t j = 0; j < lines.size(); j++) {
					float gradient = lines[j].gradient;
					float intercept = lines[j].intercept;
					size_t startIndex = lines[j].startIndex;
					size_t endIndex = lines[j].endIndex;
					calcLine(gradient, intercept, contour, lines[j].line, startIndex, endIndex);
				}
			}

			bool showSampleSegment = false;
			if(showSampleSegment) {
				vector <Point> tmp;
				for(size_t j = 10; j < 15; j++) {
					tmp.push_back(contour[j]);
				}	
				squares.push_back(tmp);
			}
		}

		for (size_t i = 0; i < lines.size(); i++) {
			//float gradient = lines[i].gradient;
			//float intercept = lines[i].intercept;
		}

		vector<LineMetadata> mergedLines = recalculateMergedLines(lines, 2.0, 0.06, 10);
		
		vector<pair<float, float> > filteredGradients;
		for (size_t i = 0; i < mergedLines.size(); i++) {
			filteredGradients.push_back(make_pair(mergedLines[i].gradient, mergedLines[i].intercept));	
		}

		if(NEW_LINE_POINTS) {
			//printf("NEW LINES: %ld\n", mergedLines.size());
			for(size_t i = 0; i < mergedLines.size(); i++) {
				if(mergedLines[i].line.size() > lower && mergedLines[i].line.size() < upper) 
					squares.push_back(mergedLines[i].line);
			}
		}

		if(GENERATE_LINES) {
			for (size_t i = 0; i < filteredGradients.size(); i++) {
				vector<Point> line;
				float gradient = filteredGradients[i].first;
				float intercept = filteredGradients[i].second;
				pureCalcLine(gradient, intercept, line);
				squares.push_back(line);
			}
		}

		if(PRINT_LINES) {
			for( size_t i = 0; i < lines.size(); i++) {

				vector<Point> line = lines[i].line;
			//	float gradient = lines[i].gradient;
			//	float intercept = lines[i].intercept;
				//int gradCount = gradientFrequencies[gradientToIndex(gradient)];
		//		int interceptCount = interceptFrequencies[interceptToIndex(intercept)];
				squares.push_back(line);
			}
		}
		return mergedLines;
	}

	vector<LineMetadata> recalculateMergedLines(
			vector<LineMetadata> lines,
			float threshold,
			float angleThreshold,
			float spacingThreshold

		) {
		vector<LineMetadata> _mergedGradients;
		vector<bool> gradientVisited(lines.size());

		//if(DEBUG_MERGED_LINES) printf("Lines size: %ld\n", lines.size());
		for (size_t i = 0; i < lines.size(); i++) {
			float contourIndex = lines[i].contourIndex;
			size_t startIndex = lines[i].startIndex;
			size_t endIndex = lines[i].endIndex;
			Point startPoint = contours[contourIndex][startIndex];
			Point endPoint = contours[contourIndex][endIndex];
			size_t newStartIndex = startPoint.x;
			size_t newEndIndex = endPoint.x;

			vector<Point> newLinePoints;
			for(size_t j = startIndex; j < endIndex; j++) {
			//	float diff = calcDiff(lines[i].gradient, lines[i].intercept, contours[contourIndex][j].x, contours[contourIndex][j].y);
				//if(DEBUG_MERGED_LINES) printf("x: %d y: %d diff: %f\n", contours[contourIndex][j].x, contours[contourIndex][j].y, diff);

				newLinePoints.push_back(contours[contourIndex][j]);
			}

			//float count = 1;
			if(!gradientVisited[i]) {
				float newGradient = 0.0, newIntercept = 0.0;
				//if(DEBUG_MERGED_LINES) printf("START\n");
				for (size_t j = 0; j < lines.size(); j++) {
					if(j == i) continue;
					float oldGrad = lines[i].gradient;
					float nextGrad = lines[j].gradient;
			//		float oldIntercept = lines[i].intercept;
			//		float nextIntercept = lines[j].intercept;

					//if(oldGrad < -5) printf("intercept diff: %f\n", fabs(oldIntercept - nextIntercept));
					if(angleFromGradient(oldGrad, nextGrad) > angleThreshold) continue;
					if(lineSpacing(lines[i], lines[j]) > spacingThreshold) continue;
					//printf("Grad: %f %f \n", nextGrad, oldGrad);

					float nextContourIndex = lines[j].contourIndex;
					size_t nextStartIndex = lines[j].startIndex;
					size_t nextEndIndex = lines[j].endIndex;
					if(nextStartIndex > newStartIndex) newStartIndex = contours[nextContourIndex][nextStartIndex].x;
					if(nextEndIndex < newEndIndex) newEndIndex = contours[nextContourIndex][nextEndIndex].x;


					float tmpGradient, tmpIntercept;
					if(calcGradientIntercept(tmpGradient, tmpIntercept, threshold,
							0, newLinePoints.size(), newLinePoints,
							nextStartIndex, nextEndIndex, contours[nextContourIndex]
						)) {
						newGradient = tmpGradient;
						newIntercept = tmpIntercept;
						//if(DEBUG_MERGED_LINES) printf("FOUND LINE: Grad: %f Intercept: %f Old intercept: %f %f\n", newGradient, newIntercept, lines[j].intercept, lines[i].intercept);
						//if(DEBUG_MERGED_LINES) printf(" i:%ld_j:%ld grad: %f nextgrad: %f", i, j, lines[i].gradient, lines[j].gradient);

						for(size_t z = nextStartIndex; z < nextEndIndex; z++) {
							//if(DEBUG_MERGED_LINES) printf("new x: %d y: %d\n", contours[nextContourIndex][z].x, contours[nextContourIndex][z].y);
							newLinePoints.push_back(contours[nextContourIndex][z]);
						}
					}
				}
				//if(DEBUG_MERGED_LINES) printf("END: grad: %f int: %f points: %ld\n", newGradient, newIntercept, newLinePoints.size());
				if(newGradient && newLinePoints.size() > lower && newLinePoints.size() < upper) {
					//printf("Pushing back new intercept, grad\n");
					_mergedGradients.push_back({
							.startIndex = newStartIndex,
							.endIndex = newEndIndex,
							.gradient = newGradient,
							.intercept = newIntercept,
							.line = newLinePoints
					});
				}
			}
			//printf("\n");
		}
		return _mergedGradients;
	}

	void updateFrequencies(float gradient, float intercept) {
		int gradCount = gradientFrequencies[gradientToIndex(gradient)];
		if(gradientFrequencies.count(gradientToIndex(gradient)) && !isnan(gradCount)) {
			gradientFrequencies[gradientToIndex(gradient)]++;
		} else {
			gradientFrequencies[gradientToIndex(gradient)] = 1;
		}
		int interceptCount = interceptFrequencies[interceptToIndex(intercept)];
		if(interceptFrequencies.count(interceptToIndex(intercept)) && !isnan(interceptCount)) {
			//printf("Intercept count: %d Intercept rounded: %f\n", interceptCount, interceptToIndex(intercept));
			interceptFrequencies[interceptToIndex(intercept)]++;
		} else {
			interceptFrequencies[interceptToIndex(intercept)] = 1;
		}
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
			for( size_t j = jStart; j < jEnd; j++) {
				float calcedY = (float) j * gradient + intercept;
				Point linePoint(j, (int) calcedY);
				
				if(		linePoint.x < gray_lastFrame.cols &&
						linePoint.y < gray_lastFrame.rows &&
						linePoint.y >= 0 &&
						linePoint.x >= 0 ) {
					
					line.push_back(linePoint);
				}
			}
		} else {
			for( size_t j = startIndex; j < endIndex; j++) {
				line.push_back(contour[j]);		
			}
		}
	}

	void pureCalcLine(float gradient, float intercept, vector<Point>& line) {
		size_t start = 0;
		size_t end = gray_lastFrame.cols;

		for( size_t j = start; j < end; j++) {
			float calcedY = (float) j * gradient + intercept;
			Point linePoint(j, (int) calcedY);
			
			if(		linePoint.x < gray_lastFrame.cols &&
					linePoint.y < gray_lastFrame.rows &&
					linePoint.y >= 0 &&
					linePoint.x >= 0 ) {
				
				line.push_back(linePoint);
			}
		}
	}

	size_t calcLineBestFit(float& gradient, float& intercept, int sampleSize, vector<Point> contour, int start) {
		float threshold = 1.0;
		if(sampleSize + start < contour.size()) {
			if(!calcGradientIntercept(gradient, intercept, threshold, start, sampleSize + start, contour)) {
				return false;
			}

			//Check pixel dist is not too small 
			//if(pixelDist(contour[start], contour[sampleSize + start]) > 10) return 0;

			//Find full extention of line
			size_t lineEnd = sampleSize + start;
			//float diff = 0;

			//printf("Line end: %ld contour size: %ld\n", lineEnd, contour.size());
			/*while( lineEnd < contour.size()) {
				//if(pixelDist(contour[start], contour[sampleSize + start]) < 4) return 0;
				diff = calcDiff(gradient, intercept, contour[lineEnd].x, contour[lineEnd].y);
				if(!calcGradientIntercept(gradient, intercept, threshold, start, sampleSize + start, contour)) {

					lineEnd++;
				} else break;
			}*/

			if(pixelDist(contour[start], contour[lineEnd]) < 5) return 0;
			/*if(!calcGradientIntercept(
						gradient, intercept, threshold,
						start, lineEnd, contour
					)) {
				return false;
			}*/
			
			//printf("Line end: %ld Gradient: %f\n", lineEnd, gradient);
			return lineEnd;
		} else return false;
	}


	bool calcGradientIntercept(
			float& gradient, float& intercept, float threshold,
			int start, int end, vector<Point> points_1,
			int sStart = -1, int sEnd = -1, vector<Point> points_2 = {}
	) {

		float noPoints = 0;
		float sumX = 0;
		float sumY = 0;
		float sumX2 = 0;
		float sumXY = 0;
		for ( size_t j = start; j < end; j++) {
			sumX += (float) points_1[j].x;
			sumY += (float) points_1[j].y;
			sumXY += (float) points_1[j].x * points_1[j].y;
			sumX2 += (float) pow(points_1[j].x, 2);
			noPoints++;
		}

		if(!(sStart < 0 || sEnd < 0)) {
			for ( size_t j = sStart; j < sEnd; j++) {
				sumX += (float) points_2[j].x;
				sumY += (float) points_2[j].y;
				sumXY += (float) points_2[j].x * points_2[j].y;
				sumX2 += (float) pow(points_2[j].x, 2);
				noPoints++;
			}
		}

		gradient = ((float) noPoints * (float) sumXY - (float) sumX * (float) sumY) / ((float) noPoints * (float) sumX2 - (float) pow((float) sumX, 2));
		intercept = ((float) sumY - gradient * sumX) / noPoints;

		if(!gradient || isnan(gradient) || isnan(intercept)) return false;

		for ( size_t j = start; j < end; j++) {
			float diff = calcDiff(gradient, intercept, points_1[j].x, points_1[j].y);
			//if(sStart < 0) printf("diff y: %f diff x: %f\n", diff, calcDiffX(gradient, intercept, points_1[j].x, points_1[j].y));
			if(diff > threshold) {
				//if(sStart < 0)printf("\n");
				return false;
			}
		}
		//if(sStart < 0) printf(" PASS\n");


		if(!(sStart < 0 || sEnd < 0)) {
			for ( size_t j = sStart; j < sEnd; j++) {
				float diff = calcDiff(gradient, intercept, points_2[j].x, points_2[j].y);
				if(diff > threshold) {
					return false;
				}
			}
		}

		//printf("diff");
		if(!(sStart < 0 || sEnd < 0)) {
			for ( size_t j = sStart; j < sEnd; j++) {
			//	float diff = calcDiff(gradient, intercept, points_2[j].x, points_2[j].y);
				//printf("diff: %f ", diff);
			}
		}

		//if(sStart < 0) printf("START: %d END: %d GRADIENT: %f\n", start, end, gradient);
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
	void processFrame() {
		if(!CALIBRATE) {
			cvtColor( lastFrame, gray_lastFrame, COLOR_BGR2GRAY );
			//namedWindow(window_name, WINDOW_AUTOSIZE );
			manageRobot();
			char fileName[42];
			sprintf(fileName, "recorded/jpeg%d.jpg", frameReference);
			imwrite(fileName, lastFrame);
		}
		//waitKey(0);
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
};
