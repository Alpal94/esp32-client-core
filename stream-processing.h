#include "lib/shared.h"
#include "lib/determine-chessboard.h"
#include "lib/determine-chess-peices.h"
#include "lib/calibrate-camera.h"
#include "lib/hsv-experiment.h"

#define PRINT_LINES false
#define GENERATE_LINES true
#define NEW_LINE_POINTS false
#define PRINT_CONTOURS false
#define HSV_EXPERIMENT false
#define CALIBRATE false
#define READ_CALIBRATION true
#define upper 90
#define lower 10

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
		findContours( detected_edges, _contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

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

	RobotPosition robotPosition = { .x = 10, .y = 10, .z = 10};
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

		MinMaxHSV blackSquare = determineChessPieces.getSquareColour(0);
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
		//bitwise_and(mask, lastFrame, lastFrame);

		//bitwise_or(black, white, lastFrame);
		//lastFrame = next;
		if(!HSV_EXPERIMENT) drawSquares(lastFrame, squares, gray_lastFrame.rows, gray_lastFrame.cols);
		resize(lastFrame, lastFrame, Size(), 0.5 / scale, 0.5 / scale);
		resize(detected_edges, detected_edges, Size(), 0.5 / scale, 0.5 / scale);

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		ALOG("\nTime elapsed: %lld\n", microseconds);
	}

	RobotPosition traverseChessboard(RobotPosition position) {
		RobotPosition nextPosition = position;
		if(nextPosition.x < 10 && nextPosition.z < 20) {
			nextPosition.x++;
			nextPosition.z++;
		} else {
			nextPosition.x--;
			nextPosition.z--;
		}
		nextPosition.y = 10;
		return position;
		if(setRobotPosition(nextPosition)) return nextPosition;
		else return position;
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

	map<float, int> gradientFrequencies;
	map<float, int> interceptFrequencies;
	vector<LineMetadata> determineLines(Mat& edges, vector<vector<Point> >& squares) {
		vector<Point> approx;
		vector<LineMetadata> lines;
		int sampleSize = 5;

		long long averageTime = 0;
		auto contourTime = std::chrono::high_resolution_clock::now();
		for ( size_t i = 0; i < contours.size(); i++) {
			if(hierarchy[i][2] < 0) continue;
			//if(i < 104 || i > 114) continue;
			printf("Contour index: %ld\n", i);
			vector<Point> contour = contours[i];
			approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.0001, false);
			if(PRINT_CONTOURS) {
				squares.push_back(contour);
			}
			float gradient, intercept;
			bool doCalcLines = true;
			if(doCalcLines) {
				for ( size_t j = 0; j < contour.size(); j++) {
					//if(!(j > 5 && j < 11)) continue;
					//printMarker(contour[11], squares, 10);
					size_t lineEnd = calcLineBestFit(gradient, intercept, sampleSize, contour, j);
					printf("PASS: Gradient: Good: %f %f Lineend: %ld\n", fabs(gradient), intercept, lineEnd);

					if(lineEnd) {
						printf("PASS: Accepting\n");
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


		auto elapsed = std::chrono::high_resolution_clock::now() - contourTime;
		averageTime = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		printf("\nAverage time: %lld contours: %ld (CONTOUR)\n", averageTime, contours.size());

		contourTime = std::chrono::high_resolution_clock::now();

		vector<LineMetadata> mergedLines;
		if(true) {
			mergedLines = recalculateMergedLines(lines, 2.0, 0.06, 10);
			elapsed = std::chrono::high_resolution_clock::now() - contourTime;
			averageTime = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			printf("Recalculate time: %lld (CONTOUR)\n", averageTime);	

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
				for (size_t i = 0; i < mergedLines.size(); i++) {
					vector<Point> line;
					float gradient = mergedLines[i].gradient;
					float intercept = mergedLines[i].intercept;

					size_t s = mergedLines[i].startIndex;
					size_t e = mergedLines[i].endIndex;
					size_t start = s < e ? s : e;
					size_t end = s < e ? e : s;
					printf("MERGED LINE: s: %ld  e: %ld\n", mergedLines[i].startIndex, mergedLines[i].endIndex);
					pureCalcLine(gradient, intercept, line);
					squares.push_back(line);
				}
			}
		}
		if(PRINT_LINES) {
			for( size_t i = 0; i < lines.size(); i++) {

				for(size_t j = 0; j < lines[i].line.size(); j++) {
					printf("x: %d y: %d ", lines[i].line[j].x, lines[i].line[j].y);
				}
				printf("\n");
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
	static bool sortLineSegments(pair<Point, Point> a, pair<Point, Point> b) {
		return a.first.x < b.first.x;
	}

	static bool sortLinesGradients(LineMetadata a, LineMetadata b) {
		return a.gradient < b.gradient;
	}

	vector<LineMetadata> recalculateMergedLines(
			vector<LineMetadata> lines,
			float threshold,
			float angleThreshold,
			float spacingThreshold

		) {
		vector<LineMetadata> _mergedGradients;
		vector<bool> gradientVisited(lines.size());
		sort(lines.begin(), lines.end(), sortLinesGradients);
		//if(DEBUG_MERGED_LINES) printf("Lines size: %ld\n", lines.size());
		for (size_t i = 0; i < lines.size(); i++) {
			float contourIndex = lines[i].contourIndex;
			size_t startIndex = lines[i].startIndex;
			size_t endIndex = lines[i].endIndex;
			Point startPoint = contours[contourIndex][startIndex];
			Point endPoint = contours[contourIndex][endIndex];
			size_t newStartIndex = startPoint.x < endPoint.x ? startPoint.x : endPoint.x;
			size_t newEndIndex = startPoint.x < endPoint.x ? endPoint.x : startPoint.x;
			vector<pair<Point, Point> > segments;

			float gradient = lines[i].gradient;
			float intercept = lines[i].intercept;
			if(gradient > 5) printf("OK Merged: GOOD: %f %f\n", gradient, intercept);

			segments.push_back(make_pair(
				startPoint.x < endPoint.x ? startPoint : endPoint,
				startPoint.x < endPoint.x ? endPoint : startPoint
			));

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
				for (size_t j = i; j < lines.size(); j++) {
					if(j == i) continue;
					float oldGrad = lines[i].gradient;
					float nextGrad = lines[j].gradient;
			//		float oldIntercept = lines[i].intercept;
			//		float nextIntercept = lines[j].intercept;

					//if(oldGrad < -5) printf("intercept diff: %f\n", fabs(oldIntercept - nextIntercept));
					if(nextGrad > 10 && oldGrad > 10) printf("Vertical gradient: %f %f\n", nextGrad, oldGrad);
					if(angleFromGradient(oldGrad, nextGrad) > angleThreshold) break;
					if(lineSpacing(lines[i], lines[j]) > spacingThreshold) continue;
					//printf("Grad: %f %f \n", nextGrad, oldGrad);

					float nextContourIndex = lines[j].contourIndex;
					size_t nextStartIndex = lines[j].startIndex;
					size_t nextEndIndex = lines[j].endIndex;
					if(nextStartIndex > newStartIndex) newStartIndex = contours[nextContourIndex][nextStartIndex].x;
					if(nextEndIndex < newEndIndex) newEndIndex = contours[nextContourIndex][nextEndIndex].x;

					size_t s = contours[nextContourIndex][nextStartIndex].x;
					size_t e = contours[nextContourIndex][nextEndIndex].x;
					segments.push_back(make_pair(
						s < e ? contours[nextContourIndex][nextStartIndex] : contours[nextContourIndex][nextEndIndex],
						s < e ? contours[nextContourIndex][nextEndIndex] : contours[nextContourIndex][nextStartIndex]
					));

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
				bool continuous = true;
				printf("First: START\n");
				sort(segments.begin(), segments.end(), sortLineSegments);
				for(int segment = 0; segment < segments.size()-1; segment++) {
					printf("First: %d Second: %d VS First: %d Second: %d   CALC: %f DIFF: %d\n", segments[segment].first.x, segments[segment].second.x, segments[segment+1].first.x, segments[segment+1].second.x, pixelDist(segments[segment].second, segments[segment+1].first), segments[segment].second.x - segments[segment+1].first.x);
					if(segments[segment].second.x - segments[segment+1].first.x < 0 && pixelDist(segments[segment].second, segments[segment+1].first) > 30) {
						printf("CONTOURS FILTERED");
						continuous = false;
						break;
					}
				}
				float lineDistance = pixelDist(segments[0].first, segments[segments.size()].second);
				//if(DEBUG_MERGED_LINES) printf("END: grad: %f int: %f points: %ld\n", newGradient, newIntercept, newLinePoints.size());
				if(continuous && newLinePoints.size() > lower && newLinePoints.size() < upper && lineDistance > 30) {
					printf("Line distance: %f\n", lineDistance);
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
			if(gradient >= 999 && calcedY > 0 && calcedY < COLS) {
				for(size_t z = 0; z < ROWS; z++) {
					Point verticalPoint(j, z);
					line.push_back(verticalPoint);
				}
			}
		}
	}

	size_t calcLineBestFit(float& gradient, float& intercept, int sampleSize, vector<Point> contour, int start) {
		float threshold = 1.0;
		if(sampleSize + start < contour.size()) {
			size_t lineEnd = sampleSize + start;
			//if(pixelDist(contour[start], contour[lineEnd]) < 5) return 0;

			
			if(!calcGradientIntercept(gradient, intercept, threshold, start, sampleSize + start, contour)) {
				return false;
			}

			if(fabs(gradient) > 5) printf("OK Best: After calc line: %f %f\n", gradient, intercept);

			//Check pixel dist is not too small 
			//if(pixelDist(contour[start], contour[sampleSize + start]) > 10) return 0;

			//Find full extention of line
			//float diff = 0;

			//printf("Line end: %ld contour size: %ld\n", lineEnd, contour.size());
			/*while( lineEnd < contour.size()) {
				//if(pixelDist(contour[start], contour[sampleSize + start]) < 4) return 0;
				diff = calcDiff(gradient, intercept, contour[lineEnd].x, contour[lineEnd].y);
				if(!calcGradientIntercept(gradient, intercept, threshold, start, sampleSize + start, contour)) {

					lineEnd++;
				} else break;
			}*/

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
		if(isnan(gradient)) gradient = 999;

		intercept = ((float) sumY - gradient * sumX) / noPoints;

		if(isnan(gradient) || isnan(intercept)) return false;

		if(fabs(gradient) > 0.1 && fabs(gradient) < 999) return false;

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
				waitKey(0);
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
