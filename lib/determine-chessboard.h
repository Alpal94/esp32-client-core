
using namespace cv;
using namespace std;

class DetermineChessBoard {
	private:	
	vector<Point> squareIntercepts;
	RobotPosition currRobotPosition;
	RobotPosition originalRobotPosition;
	ChessboardToCamera chessboardToCamera;

	Square squareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};
	Square localSquareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};


	bool squareColourDetermined = false;
	Vec3b blackSquareColour;
	Vec3b whiteSquareColour;

	public:
	void findChessboardSquares(vector<LineMetadata> &mergedLines, vector<vector<Point> >& squares, Mat _gray_lastFrame, Mat &_lastFrame, Mat &_display, RobotPosition _robotPosition) {

		vector<vector<Point> > drawing;
		currRobotPosition = _robotPosition;

		squareIntercepts.clear();
		sort(mergedLines.begin(), mergedLines.end(), sortLinesGradient);

		vector<bool> lineVisited((int) mergedLines.size());

		//localSquareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				localSquareMap[i][j].occupied = false;
			}
		}

		vector<LineMetadata> stripped;
		for (size_t i = 0; i < mergedLines.size(); i++) {
			bool duplicate = false;
			for(size_t j = 0; j < stripped.size(); j++) {
				float angle = angleFromGradient(stripped[j].gradient, mergedLines[i].gradient);
				float spacing = lineSpacing(mergedLines[j], mergedLines[i]);
				if(fabs(angle) < 0.03 &&  spacing < 10) {
					duplicate = true;
					break;
				}
			}
			if(!duplicate) {
				stripped.push_back(mergedLines[i]);
			}
		}
		mergedLines = stripped;

		printf("Stripped: %ld %ld\n", stripped.size(), mergedLines.size());
		int noSquares = 0;
		for (size_t i = 0; i < mergedLines.size(); i++) {
			//if(lineVisited[i]) continue;
			float gradient = mergedLines[i].gradient;
			//float intercept = mergedLines[i].intercept;
			
			vector<LineMetadata> parallel_lines;
			vector<LineMetadata> perpendicular_lines;
			parallel_lines.clear();
			perpendicular_lines.clear();
			for (size_t j = 0; j < mergedLines.size(); j++) {
				float gradient_next = mergedLines[j].gradient;
				//float intercept_next = mergedLines[j].intercept;
				
				float angle = angleFromGradient(gradient, gradient_next);
				if(angle < 0.03) {
					parallel_lines.push_back(mergedLines[j]);
				}
				if(angle > 1.4 && angle < 1.7) {
					perpendicular_lines.push_back(mergedLines[j]);
				}
			}
			printf("Parallel lines: %ld\nPerpedicular lines: %ld\n", parallel_lines.size(), perpendicular_lines.size());

			//SLOW RUN HERE
			if(parallel_lines.size() > 1 && perpendicular_lines.size() > 1) {
				float minSpacing = 120;
				float maxSpacing = minSpacing + 60;
				int parallel = 0;
				vector<LineMetadata> squareCandidateParallel;
				vector<LineMetadata> squareCandidatePerpendicular;
				squareCandidateParallel.clear();
				squareCandidatePerpendicular.clear();
				LineMetadata previous_line = {.gradient = 0, .intercept = 0};
				//if(mergedLines[i].gradient < -5) printf("next: intercept: %f\n", mergedLines[i].intercept);
				for(int j = 0; j < parallel_lines.size(); j++) {

					printf("MERGED PARRALEL: %d %d %d %d\n", parallel_lines[i].bounds[0], parallel_lines[i].bounds[1], parallel_lines[i].bounds[2], parallel_lines[i].bounds[3]);
					float spacing = lineSpacing(parallel_lines[j], mergedLines[i]);
					if(spacing > minSpacing && spacing < maxSpacing) {
						parallel++;
						lineVisited[j] = true;
						previous_line = parallel_lines[j];
						squareCandidateParallel.push_back(parallel_lines[j]);

						//pureCalcLine(parallel_lines[j].gradient, parallel_lines[j].intercept, squareIntercepts);
					}
				}
				int perpendicular = 0;
				vector<bool> perpendicularLineInserted((int) perpendicular_lines.size());
				for(int j = 0; j < perpendicular_lines.size(); j++) {
					for(int v = 0; v < perpendicular_lines.size(); v++) {
						if(j == v) continue;
						if(perpendicularLineInserted[j]) continue;
						float spacing = lineSpacing(perpendicular_lines[j], perpendicular_lines[v]);
						if(spacing > minSpacing && spacing < maxSpacing) {
							lineVisited[j] = true;
							perpendicularLineInserted[j] = true;
							squareCandidatePerpendicular.push_back(perpendicular_lines[j]);
							//pureCalcLine(perpendicular_lines[j].gradient, perpendicular_lines[j].intercept, squareIntercepts);
							perpendicular++;
						}
					}
				}
				printf("Perpendicular: %ld\n Parallel: %ld\n", squareCandidatePerpendicular.size(), squareCandidateParallel.size());
				if(parallel > 1 && perpendicular > 1) {
					sort(squareCandidatePerpendicular.begin(), squareCandidatePerpendicular.end(), sortLinesIntercepts);
					sort(squareCandidateParallel.begin(), squareCandidateParallel.end(), sortLinesIntercepts);
					for(int j = 0; j < squareCandidateParallel.size(); j++) {
						for(int z = 0; z < squareCandidatePerpendicular.size() - 1; z+=1) {
							float perpendicularXAxisAngle = angleFromGradient(squareCandidatePerpendicular[z].gradient, 0);
							float parallelXAxisAngle = angleFromGradient(squareCandidateParallel[j].gradient, 0);
							float xAxisAngle = perpendicularXAxisAngle < parallelXAxisAngle ? perpendicularXAxisAngle : parallelXAxisAngle;


							LineMetadata northLine, southLine, westLine, eastLine;
							bool mergedLinesNorthSouth = parallelXAxisAngle > M_PI/2 ? true : false;
							if(mergedLinesNorthSouth) {
								bool isMergedWest = isLineWest(mergedLines[i], squareCandidateParallel[j]);
								westLine = isMergedWest ?  mergedLines[i] : squareCandidateParallel[j];
								eastLine = !isMergedWest ? mergedLines[i] : squareCandidateParallel[j];

								bool is_z1_North = squareCandidatePerpendicular[z+1].intercept > squareCandidatePerpendicular[z].intercept;
								northLine =  is_z1_North ? squareCandidatePerpendicular[z+1] : squareCandidatePerpendicular[z];
								southLine = !is_z1_North ? squareCandidatePerpendicular[z+1] : squareCandidatePerpendicular[z];
							} else {
								bool is_z1_West = isLineWest(squareCandidatePerpendicular[z+1], squareCandidatePerpendicular[z]);
								westLine = is_z1_West ? squareCandidatePerpendicular[z+1] : squareCandidatePerpendicular[z];
								eastLine = !is_z1_West ? squareCandidatePerpendicular[z+1] : squareCandidatePerpendicular[z];

								bool isMergedNorth = mergedLines[i].intercept > squareCandidateParallel[j].intercept;
								northLine = isMergedNorth  ? mergedLines[i] : squareCandidateParallel[j];
								southLine = !isMergedNorth ? mergedLines[i] : squareCandidateParallel[j];
							}

							FPoint northEast = locateIntercept(northLine, eastLine);
							FPoint northWest = locateIntercept(northLine, westLine);
							FPoint southEast = locateIntercept(southLine, eastLine);
							FPoint southWest = locateIntercept(southLine, westLine);

							if(
									isnan(northEast.x) || isnan(northEast.y) || 
									isnan(northWest.x) || isnan(northWest.y) ||
									isnan(southEast.x) || isnan(southEast.y) ||
									isnan(southWest.x) || isnan(southWest.y)
							  ) {
								printf("Warning: Square does not exist within line bounds\n");
								continue;
							}

							float spacingNorth = fPixelDist(northEast, northWest);
							float spacingSouth = fPixelDist(southEast, southWest);
							float spacingWest = fPixelDist(northWest, southWest);
							float spacingEast = fPixelDist(northEast, southEast);

							if(!checkSpacingIsSquare(spacingNorth, spacingSouth, spacingWest, spacingEast)) {
								printf("Warning: not a square.  Spacing NSWE: %f %f %f %f\n", spacingNorth, spacingSouth, spacingWest, spacingEast);
								continue;
							}

							float spacing = (spacingNorth + spacingSouth + spacingWest + spacingEast) / 4;
							FPoint center = squareCenter({
								.northEast = northEast,
								.northWest = northWest,
								.southEast = southEast,
								.southWest = southWest
							});


							noSquares++;
							Square square = {
								.occupied = true,
								.spacing = spacing,
								.rotation = xAxisAngle,
								.center = center,
								.northEast = northEast,
								.northWest = northWest,
								.southEast = southEast,
								.southWest = southWest
							};

							/*pureCalcLine(mergedLines[i].gradient, mergedLines[i].intercept, squareIntercepts);
							pureCalcLine(squareCandidatePerpendicular[z].gradient, squareCandidatePerpendicular[z].intercept, squareIntercepts);
							pureCalcLine(squareCandidatePerpendicular[z+1].gradient, squareCandidatePerpendicular[z+1].intercept, squareIntercepts);
							pureCalcLine(squareCandidateParallel[j].gradient, squareCandidateParallel[j].intercept, squareIntercepts);*/
							if(noSquares) {
								printSquare(rotateSquare(square, { .rotation = 0 }), _lastFrame);
								squareColour(square, _lastFrame, _display);
								printf("Origin Center: %f %f\n", square.center.x, square.center.y);
								insertSquare(&localSquareMap, square, { .spacing = 0, .rotation = xAxisAngle, .north = 0, .west = 0 }, Point(0,0), false);

								/*pureCalcLine(mergedLines[i].gradient, mergedLines[i].intercept, squareIntercepts);
								pureCalcLine(squareCandidatePerpendicular[z].gradient, squareCandidatePerpendicular[z].intercept, squareIntercepts);
								pureCalcLine(squareCandidatePerpendicular[z+1].gradient, squareCandidatePerpendicular[z+1].intercept, squareIntercepts);
								pureCalcLine(squareCandidateParallel[j].gradient, squareCandidateParallel[j].intercept, squareIntercepts);*/
							}

						}
					}

				}
			}
		}
		
		if(squareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2].occupied) {
			//Square globalOrigin = squareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2];
			Square tLocalOrigin = translateSquare(rotateSquare(localSquareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2], mapOffset), mapOffset);
			if(tLocalOrigin.occupied) {

				//bool matched = false;
				Point localOffset;
				if(updateOffset(mapOffset, &localSquareMap, localOffset)) {
					printf("OFFSET: spacing: %f rotation: %f north: %f west: %f\n", mapOffset.spacing, mapOffset.rotation, mapOffset.north, mapOffset.west);
					for(int i = 0; i < OVERSIZED_BOARD; i++) {
						for(int j = 0; j < OVERSIZED_BOARD; j++) {
							if(localSquareMap[i][j].occupied) {
								insertSquare(&squareMap, localSquareMap[i][j], mapOffset, Point(0,0), true);
							}
						}
					}

				} else {
				}
			}
		} else {
			Square localOrigin = localSquareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2];
			originalRobotPosition = currRobotPosition;
			if(localOrigin.occupied) {
				mapOffset.rotation = 0;
				mapOffset.spacing = localOrigin.spacing;
				mapOffset.north = 0;
				mapOffset.west = 0;

				printf("Origin: Assigning square map\n");
				for(int i = 0; i < OVERSIZED_BOARD; i++) {
					for(int j = 0; j < OVERSIZED_BOARD; j++) {
						squareMap[i][j] = localSquareMap[i][j];
					}
				}
			}
		}

		if(calculateChessboardCamera()) {
			//printf("Chessboard camera position calculated\n");
		}

		asciiPrintBoard(&localSquareMap);
		drawing.push_back(squareIntercepts);
		squares = drawing;
		fflush(stdout);
	}

	vector<Square> getLocalSquareList() {
		vector<Square> localSquareList;
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				if(localSquareMap[i][j].occupied) {
					localSquareMap[i][j].x = i;
					localSquareMap[i][j].y = j;
					localSquareList.push_back(localSquareMap[i][j]);
				}
			}
		}
		return localSquareList;
	}

	vector<Square> getGlobalSquareList() {
		vector<Square> globalSquareList;
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				if(squareMap[i][j].occupied) {

					squareMap[i][j].x = i;
					squareMap[i][j].y = j;
					globalSquareList.push_back(squareMap[i][j]);
				}
			}
		}
		return globalSquareList;
	}

	private:	
	bool calculateChessboardCamera() {
		if(
				distance(originalRobotPosition, currRobotPosition) > 10 &&
				abs(originalRobotPosition.z - currRobotPosition.z) == 0.0 &&
				(mapOffset.north > 20 || mapOffset.west > 20)	) {
			double dy = abs(originalRobotPosition.y - currRobotPosition.y);
			double dx = abs(originalRobotPosition.x - currRobotPosition.x);

			double realPixelDistance = mapOffset.north > mapOffset.west ? 
				dy / mapOffset.north : dx / mapOffset.west;

			float squareWidth = realPixelDistance;
			float distance = calcRealDist(realPixelDistance);
			
			chessboardToCamera.calced = true;
			chessboardToCamera.distance = distance;
			chessboardToCamera.squareWidth = squareWidth;
			return true;
		}
		return false;

	}

	float calcRealDist(float realSquareWidth) {
		float fieldWidth = realSquareWidth * COLS;
		return fieldWidth / CAM_RATIO;
	}

	void asciiPrintBoard(Square (*_localSquareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD]) {
		printf("Ascii Board: _\n");
		printf("_\n");
		for(int i = -1; i < 16; i++) {
			for(int j = 0; j < 32; j++) {
				if(i < 0) {
					if(j == 16) printf("    ");
					else {
						if(j%16+1 < 10) {
							printf(" %d ", j%17+1);
						} else {
							printf(" %d", j%17+1);
						}
					}
				} else if(j < -1) { 
					printf("%d ", i);
				} else if(j < 16) {
					if(j%16 == 8 && i%16 == 8 && (*_localSquareMap)[j%16][i%16].occupied) printf(" %c ", 'X');
					else printf(" %d ", (*_localSquareMap)[j%16][i%16].occupied);
				} else {
					if(j == 16) printf("    ");
					if(j%16 == 8 && i%16 == 8 && squareMap[j%16][i%16].occupied) printf(" %c ", 'X');
					else printf(" %d ", squareMap[j%16][i%16].occupied);
				}
			}
			printf("_\n");
		}
	}

	// TODO: If known squares are also kept in a list, could save iterations for efficiency
	bool updateOffset(MapOffset& offset, Square (*_localSquareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], Point &localOffset) {
		float threshold = 2;
		for(int gx = 0; gx < OVERSIZED_BOARD; gx++) {
			for(int gy = 0; gy < OVERSIZED_BOARD; gy++) {
				for(int lx = 0; lx < OVERSIZED_BOARD; lx++) {
					for(int ly = 0; ly < OVERSIZED_BOARD; ly++) {
						if((*_localSquareMap)[lx][ly].occupied && squareMap[gx][gy].occupied) {
							Square tLocal = translateSquare((*_localSquareMap)[lx][ly], offset);
							Square global = squareMap[gx][gy];

							if(fPixelDist(tLocal.center, global.center) < threshold) { 
								offset.north += global.center.y - tLocal.center.y;
								offset.west += global.center.x - tLocal.center.x;

								//printSquare(tLocal);
								//printSquare(global);

								localOffset.x = lx - gx;
								localOffset.y = ly - gy;
								//printf("Creating offset: x: %d %d y: %d %d\n", gx, lx, gy, ly);
								return true;
							}
						}
					}
				}
			}
		}
		return false;
	}

	float calcRotation(Square square) {
		float xAxisAngle_1 = angleFromGradient(gradientFromPoints(square.northWest, square.northEast), 0);
		float xAxisAngle_2 = angleFromGradient(gradientFromPoints(square.northWest, square.southWest), 0);
		float xAxisAngle = xAxisAngle_1 < xAxisAngle_2 ? xAxisAngle_1 : xAxisAngle_2;

		return xAxisAngle;
	}

	bool isLineWest(LineMetadata line, LineMetadata comp) {
		float lineX = -line.intercept / comp.gradient;
		float compX = -comp.intercept / comp.gradient;

		return lineX < compX;
	}
	void insertSquare(Square (*_squareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], Square square, MapOffset offset, Point localOffset, bool debug=false) {

		Square origin = (*_squareMap)[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2];
		Square tSquare = translateSquare(rotateSquare(square, offset), offset);
		//if(debug) printf("OFFSET: spacing: %f rotation: %f north: %f west: %f\n", offset.spacing, offset.rotation, offset.north, offset.west);
		if(debug) printf("ORIGIN: occupied: %d spacing: %f posx: %d posy: %d\n", origin.occupied, origin.spacing, origin.x, origin.y);
		if(origin.occupied) {
			float spacing = origin.spacing;
			FPoint originCenter = origin.center;
			FPoint squareCenter = tSquare.center;

			FPoint vector = { .x = (squareCenter.x - originCenter.x) / spacing, .y = (squareCenter.y - originCenter.y) / spacing };
			if(debug) printf("Origin center: %f,%f Square center: %f,%f Spacing: %f Vector: %f %f\n", originCenter.x, originCenter.y, squareCenter.x, squareCenter.y, spacing, vector.x, vector.y);
			int posX = OVERSIZED_BOARD / 2 + (int) round(vector.x);
			int posY = OVERSIZED_BOARD / 2 + (int) round(vector.y);

			if(posX == OVERSIZED_BOARD / 2 && posY == OVERSIZED_BOARD) return;

			tSquare.global_x = posX;
			tSquare.global_y = posY;
			tSquare.occupied = true;

			//printf("Is occupied: %d x: %d y: %d\n", (*_squareMap)[posX][posY].occupied, posX, posY);
			if(!(*_squareMap)[posX][posY].occupied) {
				if(debug) {
					//printf("Inserting: %d %d ", posX, posY);
				}
				(*_squareMap)[posX][posY] = tSquare;
			} else {
				//printf("Warning: OCCUPIED\n");
			}
		} else {
			if(debug) printf("Origin:  UPDATING ORIGIN\n");
			tSquare.global_x = 0;
			tSquare.global_y = 0;
			tSquare.occupied = true;
			(*_squareMap)[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2] = tSquare;
			//printSquare(square);
		}
	}

	FPoint positionTranslateMapOffset(FPoint position, MapOffset offset) {
		return { .x = (position.x + offset.west), .y = (position.y + offset.north) };
	}

	bool checkSpacingIsSquare(float spacingNorth, float spacingSouth, float spacingWest, float spacingEast) {
		float threshold = 2;
		float spacing[] = { spacingNorth, spacingSouth, spacingWest, spacingEast };
		for(int i = 0; i < 4; i++) {
			for(int j = 0; j < 4; j++) {
				if(i == j) continue;
				if(fabs(spacing[i] - spacing[j]) > threshold) return false;
			}
		}
		return true;
	}

	void pureCalcLine(
		float gradient, float intercept, vector<Point>& line,
		Mat &gray_lastFrame,
		float _start = 0, float _end = COLS
	) {
		float start = _start < _end ? _start : _end;
		float end = _start < _end ? _end : _start;

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

	bool firstSquareColourDetermined = false;
	Vec3b tmpSquareColour;
	void squareColour(Square _square, Mat &lastFrame, Mat &_display) {
		//printMarker(Point((int)_square.center.x, (int)_square.center.y), 10);

		int minHue = 255; int maxHue = 0; int averageHue = 0;
		int minSaturation = 255; int maxSaturation = 0; int averageSaturation = 0;
		int minValue = 255; int maxValue = 0; int averageValue = 0;
		
		int sampleSize = (int) 4;
		for(int i = -sampleSize; i < sampleSize; i++) {
			for(int j = -sampleSize; j < sampleSize; j++) {
				Vec3b currSquareColour = lastFrame.at<Vec3b>(Point((int) _square.center.x + i, (int) _square.center.y + j));


				cv::Mat3b hsv;
				cv::Mat3b bgr(currSquareColour);
				cvtColor(bgr, hsv, COLOR_BGR2HSV); 
				Vec3b hsvColour = hsv.at<Vec3b>(0,0);

				int cvHue = hsvColour.val[0];
				int cvSaturation = hsvColour.val[1];
				int cvValue = hsvColour.val[2];

				minHue = min(minHue, cvHue);
				maxHue = max(maxHue, cvHue);
				averageHue += cvHue;

				minSaturation = min(minSaturation, cvSaturation);
				maxSaturation = max(maxSaturation, cvSaturation);
				averageSaturation += cvSaturation;

				minValue = min(minValue, cvValue);
				maxValue = max(maxValue, cvValue);
				averageValue += cvValue;
			}
		}
  
		averageHue = averageHue /  pow(2 * sampleSize, 2);
		averageSaturation = averageSaturation / pow(2 * sampleSize, 2);
		averageValue = averageValue / pow(2 * sampleSize, 2);
		
		cvtColor(lastFrame, _display, COLOR_BGR2HSV);
		inRange( _display, Scalar(minHue,minSaturation,minValue), Scalar(maxHue,maxSaturation,maxValue), _display );




		if(!firstSquareColourDetermined) {
			//tmpSquareColour = currSquareColour;
			firstSquareColourDetermined = true;
			return;
		}
	}

	void printMarker(Point point, int size, Mat &drawing) {
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

	void printSquare(Square _square, Mat &gray_lastFrame) {
		line( gray_lastFrame,
				Point(_square.northEast.x, _square.northEast.y),
				Point(_square.northWest.x, _square.northWest.y),
				Scalar(0,255,0), 1, 1);

		line( gray_lastFrame,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.southWest.x, _square.southWest.y),
				Scalar(0,255,0), 1, 1);

		line( gray_lastFrame,
				Point(_square.southWest.x, _square.southWest.y),
				Point(_square.northWest.x, _square.northWest.y),
				Scalar(0,255,0), 1, 1);

		line( gray_lastFrame,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.northEast.x, _square.northEast.y),
				Scalar(0,255,0), 1, 1);
	}

	float gradientFromPoints(FPoint _point1, FPoint _point2) {
		return ((float) _point1.y - (float) _point2.y) / ((float) _point1.x - (float) _point2.x);
	}

	static bool sortLinesIntercepts(LineMetadata a, LineMetadata b) {
		return a.intercept < b.intercept;
	}

	static bool sortLinesGradient(LineMetadata a, LineMetadata b) {
		return a.gradient < b.gradient;
	}
};
