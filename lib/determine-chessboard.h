
using namespace cv;
using namespace std;

#define PARALLEL_ANGLE 0.05

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
				if(j == i) continue;
				float angle = angleFromGradient(stripped[j].gradient, mergedLines[i].gradient);
				
				float spacing = fabs(stripped[j].intercept - mergedLines[i].intercept);
				float spacingX = fabs(stripped[j].xIntercept - mergedLines[i].xIntercept);
				//printf("Spacing: %f SpacingX: %f for (%f,%f) (%f,%f)\n", spacing, spacingX, mergedLines[j].intercept, mergedLines[i].intercept, mergedLines[j].xIntercept, mergedLines[i].xIntercept);
				//printf("Spacing: %f or %f. Angle: %f\n", spacing, spacingX, angle);
				//printf("Stripped size:  %ld\n", stripped.size());
				if(angle < PARALLEL_ANGLE && (fabs(spacingX) < 5 || fabs(spacing) < 5)) {
					duplicate = true;
					break;
				}
			}
			if(!duplicate) {
				stripped.push_back(mergedLines[i]);
			}
		}

		//printf("\n");
		mergedLines = stripped;

		vector<Square> squareList;
		squareList.clear();

		printf("Stripped: %ld %ld\n", stripped.size(), mergedLines.size());
		int noSquares = 0;
		for (size_t i = 0; i < mergedLines.size(); i++) {
			//pureCalcLineX(mergedLines[i].xGradient, mergedLines[i].xIntercept, squareIntercepts, _lastFrame);
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
				if(angle < PARALLEL_ANGLE) {
					parallel_lines.push_back(mergedLines[j]);
				}
				///1.57
				if(angle > 1.41 && angle < 1.69) {
					perpendicular_lines.push_back(mergedLines[j]);
				}
			}
			printf("Parallel lines: %ld\nPerpedicular lines: %ld\nAll lines: %ld\n", parallel_lines.size(), perpendicular_lines.size(), mergedLines.size());
			/*for(int j = 0; j < parallel_lines.size(); j++) {
				Point x1 = Point(parallel_lines[j].bounds[0], parallel_lines[j].bounds[1]);
				Point x2 = Point(parallel_lines[j].bounds[2], parallel_lines[j].bounds[3]);
				line(_lastFrame, x1, x2, Scalar(255, 0,0), 3, 3);
			}
			for(int j = 0; j < perpendicular_lines.size(); j++) {
				Point x1 = Point(perpendicular_lines[j].bounds[0], perpendicular_lines[j].bounds[1]);
				Point x2 = Point(perpendicular_lines[j].bounds[2], perpendicular_lines[j].bounds[3]);
				line(_lastFrame, x1, x2, Scalar(0, 255,0), 3, 3);
			}*/

			if(parallel_lines.size() > 1 && perpendicular_lines.size() > 1) {
				float minSpacing = 65;
				float maxSpacing = minSpacing + 25;
				int parallel = 0;
				vector<LineMetadata> squareCandidateParallel;
				vector<LineMetadata> squareCandidatePerpendicular;
				squareCandidateParallel.clear();
				squareCandidatePerpendicular.clear();
				LineMetadata previous_line = {.gradient = 0, .intercept = 0};
				//if(mergedLines[i].gradient < -5) printf("next: intercept: %f\n", mergedLines[i].intercept);
				for(int j = 0; j < parallel_lines.size(); j++) {

					float spacing = lineSpacing(parallel_lines[j], mergedLines[i]);
					printf("Parallel line spacing: %f\n", spacing);
					if(spacing > minSpacing && spacing < maxSpacing) {

						parallel++;
						lineVisited[j] = true;
						previous_line = parallel_lines[j];
						squareCandidateParallel.push_back(parallel_lines[j]);
						//pureCalcLineX(parallel_lines[j].xGradient, parallel_lines[j].xIntercept, squareIntercepts, _lastFrame);
					}
				}
				int perpendicular = 0;
				vector<bool> perpendicularLineInserted((int) perpendicular_lines.size());
				for(int j = 0; j < perpendicular_lines.size(); j++) {
					for(int v = 0; v < perpendicular_lines.size(); v++) {
						if(j == v) continue;
						if(perpendicularLineInserted[j]) continue;
						float spacing = lineSpacing(perpendicular_lines[j], perpendicular_lines[v]);
						//printf("Perpendicular line spacing: %f\n", spacing);
						if(spacing > minSpacing && spacing < maxSpacing) {
							lineVisited[j] = true;
							perpendicularLineInserted[j] = true;
							squareCandidatePerpendicular.push_back(perpendicular_lines[j]);
							//pureCalcLine(perpendicular_lines[j].gradient, perpendicular_lines[j].intercept, _lastFrame);
							//pureCalcLineX(perpendicular_lines[j].xGradient, perpendicular_lines[j].xIntercept, squareIntercepts, _lastFrame);
							perpendicular++;
						}
					}
				}
				printf("Perpendicular: %ld Parallel: %ld\n", squareCandidatePerpendicular.size(), squareCandidateParallel.size());
				if(parallel > 1 && perpendicular > 1) {
					sort(squareCandidatePerpendicular.begin(), squareCandidatePerpendicular.end(), sortLinesIntercepts);
					sort(squareCandidateParallel.begin(), squareCandidateParallel.end(), sortLinesIntercepts);
					for(int j = 0; j < squareCandidateParallel.size(); j++) {
						for(int z = 0; z < squareCandidatePerpendicular.size() - 1; z+=1) {
							float perpendicularXAxisAngle = angleFromGradient(squareCandidatePerpendicular[z].gradient, 0);
							float parallelXAxisAngle = angleFromGradient(squareCandidateParallel[j].gradient, 0);
							float xAxisAngle = perpendicularXAxisAngle < parallelXAxisAngle ? perpendicularXAxisAngle : parallelXAxisAngle;


							LineMetadata northLine, southLine, westLine, eastLine;
							bool mergedLinesNorthSouth = parallelXAxisAngle > M_PI/4 ? true : false;
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
								if(fabs(mergedLines[i].gradient == 0.0) && mergedLines[i].intercept < 300) {
									printf("Warning: not a square.  Spacing NSWE: %f %f %f %f\n", spacingNorth, spacingSouth, spacingWest, spacingEast);
								}
								continue;
							}

							//printf("All spacing: %f %f %f %f\n", spacingNorth, spacingSouth, spacingWest, spacingEast);
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
								.southWest = southWest,
								.lines = { northLine, eastLine, southLine, westLine }
								
							};
							/*if(fabs(mergedLines[i].gradient == 0.0) && mergedLines[i].intercept < 300 && southLine.gradient == 0 && northLine.gradient == 0) {
								if(westLine.xIntercept > 800 && westLine.xIntercept < 1100) {
									pureCalcLine(northLine.gradient, northLine.intercept, _lastFrame);
									pureCalcLine(southLine.gradient, southLine.intercept, _lastFrame);
									pureCalcLine(eastLine.gradient, eastLine.intercept, _lastFrame);
									pureCalcLine(westLine.gradient, westLine.intercept, _lastFrame);
								}
									printSquare(square, _lastFrame);
							}*/
							//pureCalcLine(mergedLines[i].gradient, mergedLines[i].intercept, squareIntercepts, _lastFrame);
							//pureCalcLine(squareCandidatePerpendicular[z].gradient, squareCandidatePerpendicular[z].intercept, squareIntercepts, _lastFrame);
							//pureCalcLine(squareCandidatePerpendicular[z+1].gradient, squareCandidatePerpendicular[z+1].intercept, squareIntercepts, _lastFrame);
							//pureCalcLine(squareCandidateParallel[j].gradient, squareCandidateParallel[j].intercept, squareIntercepts, _lastFrame);
							if(noSquares) {
								//SQUARE COLOUR IS SLOW
								//squareColour(square, _lastFrame, _display);
								printf("Origin Center: %f %f\n", square.center.x, square.center.y);
								squareList.push_back(square);
								//printSquare(square, _lastFrame);
								//pureCalcLine(mergedLines[i].gradient, mergedLines[i].intercept, squareIntercepts);
								//pureCalcLine(squareCandidatePerpendicular[z].gradient, squareCandidatePerpendicular[z].intercept, squareIntercepts);
								//pureCalcLine(squareCandidatePerpendicular[z+1].gradient, squareCandidatePerpendicular[z+1].intercept, squareIntercepts);
								//pureCalcLine(squareCandidateParallel[j].gradient, squareCandidateParallel[j].intercept, squareIntercepts);
							}
							printf("NO Squares: %d\n", noSquares);
						}
					}

				}


			}
		}
		printf("ls: Square list size: %ld\n", squareList.size());
		sort(squareList.begin(), squareList.end(), sortSquares);
		int count = 0;
		for(int i = 0; i < squareList.size(); i++) {
			LineMetadata northLine = squareList[i].lines[0];
			LineMetadata southLine = squareList[i].lines[2];
			float direction = northLine.gradient / fabs(northLine.gradient);
			float northXAxisAngle = direction * angleFromGradient(northLine.gradient, 0);
			float southXAxisAngle = direction * angleFromGradient(southLine.gradient, 0);

			FPoint vector = { .x = (squareList[i].center.x) / squareList[i].spacing, .y = (squareList[i].center.y) / squareList[i].spacing };

			printf("SquarePos: %f %f\n", vector.x, vector.y);
			printf("ls: %f %f %f %f %f\n", northXAxisAngle, southXAxisAngle, northLine.gradient, southLine.gradient, squareList[i].spacing);
		}
		int posX = 0, posY = 0;
		int lastSquareIndex = -1;
		for(int i = 0; i < squareList.size(); i++) {
			LineMetadata northLine = squareList[i].lines[0];
			LineMetadata southLine = squareList[i].lines[2];
			float direction = northLine.gradient / fabs(northLine.gradient);
			float northXAxisAngle = direction * angleFromGradient(northLine.gradient, 0);
			float southXAxisAngle = direction * angleFromGradient(southLine.gradient, 0);



			if(fabs(northLine.gradient - southLine.gradient) > 0.02) {
				continue;
			}

			float upperFilter = 0.08;
			float lowerFilter = 0.0;
			if(northXAxisAngle > upperFilter || northXAxisAngle < lowerFilter || southXAxisAngle < lowerFilter || southXAxisAngle > upperFilter) {
				continue;
			}

			if(i) {
				Square a = squareList[i];
				Square b = squareList[lastSquareIndex];
				float spacing = a.spacing > b.spacing ? b.spacing : a.spacing;


				if(fabs(a.center.y - b.center.y) < spacing * 0.2 && fabs(a.center.x - b.center.x) < spacing * 0.2) {

					continue;
				}
					
				if(fabs(a.center.y - b.center.y) < spacing * 0.2) {
					if(fabs(a.center.x - b.center.x) > spacing * (1 + 0.2)) continue;
					posX++;
				} else {
					posX=0;
					posY++;
				}
			}

			printSquare(squareList[i], _lastFrame);
			lastSquareIndex = i;
			insertSquare(&localSquareMap, posX, posY, squareList[i], { .spacing = 0, .rotation = -squareList[i].rotation, .north = 0, .west = 0 }, Point(0,0), _lastFrame, true);
			count++;
		}
		printf("ls: Square list size stripped: %d\n", count); 
		//Compare squares here	

		//Deprecated
		/*if(squareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2].occupied) {
			//Square globalOrigin = squareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2];
			Square tLocalOrigin = translateSquare(rotateSquare(localSquareMap[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2], mapOffset), mapOffset);
			if(tLocalOrigin.occupied) {

				//bool matched = false;
				Point localOffset;
				if(updateOffset(mapOffset, &localSquareMap, localOffset)) {
					//printf("OFFSET: spacing: %f rotation: %f north: %f west: %f\n", mapOffset.spacing, mapOffset.rotation, mapOffset.north, mapOffset.west);
					for(int i = 0; i < OVERSIZED_BOARD; i++) {
						for(int j = 0; j < OVERSIZED_BOARD; j++) {
							if(localSquareMap[i][j].occupied) {
								insertSquare(&squareMap, localSquareMap[i][j], mapOffset, Point(0,0), _lastFrame, false);
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
		}*/

		/*if(calculateChessboardCamera()) {
			//printf("Chessboard camera position calculated\n");
		}*/

		asciiPrintBoard(&localSquareMap, _lastFrame);
		//calcPerfectBoard(&localSquareMap, _lastFrame);
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

	void asciiPrintBoard(Square (*_localSquareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], Mat &_lastFrame) {
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
					if((*_localSquareMap)[j%16][i%16].occupied) {
						//printSquare(rotateSquare((*_localSquareMap)[j%16][i%16], { .rotation = 0/*-(*_localSquareMap)[j%16][i%16].rotation*/ }), _lastFrame);
					}
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

	void calcPerfectBoard(Square (*_squareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], Mat &_lastFrame) {
		Square start = { .occupied = false };
		int startX, startY;
		FPoint southWest, northEast;
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				if((*_squareMap)[i][j].occupied) {
					start = (*_squareMap)[i][j];

					start = rotateSquare(start, { .spacing = 0, .rotation = -start.rotation, .north = 0, .west = 0 });
					//printSquare((*_squareMap)[i][j], _lastFrame);
					southWest = start.southWest;
					startX = i;
					startY = j;
					break;
				}
				
			}
			if(start.occupied) break;
		}
		int endX, endY, found = false;
		for(int i = OVERSIZED_BOARD - 1; i >=0; i--) {
			for(int j = OVERSIZED_BOARD - 1; j >=0; j--) {
				if((*_squareMap)[i][j].occupied) {
					Square end = (*_squareMap)[i][j];
					end = rotateSquare(end, { .spacing = 0, .rotation = -end.rotation, .north = 0, .west = 0 });
					endX = i;
					endY = j;
					printSquare(end, _lastFrame);
					northEast = end.southWest;
					found = true;
					break;
				}
			}
			if(found) break;
		}

		int width = endX - startX + 1;
		int height = endY - startY + 1;
		float spacing = fabs((northEast.x - southWest.x) / (width));
		printMarker(Point((int) northEast.x, (int)northEast.y), 10, _lastFrame);
		printMarker(Point((int) southWest.x, (int)southWest.y), 10, _lastFrame);
		printf("Width: %d Height: %d [(%d, %d) to (%d, %d)]\n", width, height, startX, startY, endX, endY);

		Square current = start;
		Square rowStart = start;
		printSquare(start, _lastFrame);
		for(int w = 0; w < width; w++) {
			for(int h = 1; h < height; h++) {
				//current = generateNeighboringSquare(current, current.southWest, current.southEast, current.northWest, current.northEast, SW, SE, NW, NE, spacing, _lastFrame);
				current = generateNeighboringSquare(current, current.southEast, current.southWest, current.northEast, current.northWest, SE, SW, NE, NW, spacing, _lastFrame);
				printSquare(current, _lastFrame);

			}
			if(width != w + 1) {
				rowStart = generateNeighboringSquare(rowStart, rowStart.southWest, rowStart.northWest, rowStart.southEast, rowStart.northEast, SW, NW, SE, NE, spacing, _lastFrame);
				current = rowStart;
			}
		}
		
	}
	
	Square generateNeighboringSquare(
			Square square,
			FPoint b1, FPoint b2, FPoint t1, FPoint t2,
			Orientation ob1, Orientation ob2, Orientation ot1, Orientation ot2,
			float spacing,
			Mat &_lastFrame
		) {
		FPoint gradientVector = {
			.x = (b2.x - b1.x),
			.y = (b2.y - b1.y)
		};

		float spacingAdjustment = fabs(spacing / fPixelDist({.x = 0, .y = 0}, gradientVector));

		//printf("Direction: %f %f Adjustment: %f Spacing: %f vector: %f Offset north: %f Offset west: %f Rotation: %f\n", gradientVector.x, gradientVector.y, spacingAdjustment, spacing, fPixelDist({.x = 0, .y = 0}, gradientVector), mapOffset.north, mapOffset.west, square.rotation);

		/*gradientVector.x *= spacingAdjustment;
		gradientVector.y *= spacingAdjustment;*/

		FPoint b3 = {
			.x = b2.x + gradientVector.x * spacingAdjustment,
			.y = b2.y + gradientVector.y * spacingAdjustment
		};

		FPoint t3 = {
			.x = t2.x + gradientVector.x * spacingAdjustment,
			.y = t2.y + gradientVector.y * spacingAdjustment
		};

		Square neighbor = square;
		updateSquareCorner(neighbor, b2, ob1);
		updateSquareCorner(neighbor, t2, ot1);
		updateSquareCorner(neighbor, b3, ob2);
		updateSquareCorner(neighbor, t3, ot2);
		neighbor.center = squareCenter(neighbor);
		//printSquare(neighbor, _lastFrame);
		return neighbor;

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
	void insertSquare(Square (*_squareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], int posX, int posY, Square square, MapOffset offset, Point localOffset, Mat &_lastFrame, bool debug=false) {

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

			tSquare.global_x = posX;
			tSquare.global_y = posY;
			tSquare.occupied = true;

			//printf("Is occupied: %d x: %d y: %d\n", (*_squareMap)[posX][posY].occupied, posX, posY);
			if(!(*_squareMap)[posX][posY].occupied) {
				if(debug) {
					printf("Inserting: %d %d\n", posX, posY);
				}
				(*_squareMap)[posX][posY] = tSquare;
				if(debug) {
//					if(posX == 16 && posY == 13 || posX == 15 && posY == 12 || posX == 15 && posY == 11) {
					//printSquare(tSquare, _lastFrame);
				}
			} else {
				if(posX > 10 && posY < 12) {
					printf("Rejected: %d %d spacing: %f vector:  %f %f\t", posX, posY, origin.spacing, vector.x, vector.y);
					printf("Origin center: %f,%f Square center: %f,%f Spacing: %f Vector: %f %f\n", originCenter.x, originCenter.y, squareCenter.x, squareCenter.y, spacing, vector.x, vector.y);
				}
				printf("Warning: OCCUPIED\n");
			} 
		} else {
			if(debug) printf("Origin:  UPDATING ORIGIN\n");
			tSquare.global_x = 0;
			tSquare.global_y = 0;
			tSquare.occupied = true;
			(*_squareMap)[posX][posY] = tSquare;
			//printSquare(square, _lastFrame);
		}
	}

	FPoint positionTranslateMapOffset(FPoint position, MapOffset offset) {
		return { .x = (position.x + offset.west), .y = (position.y + offset.north) };
	}

	bool checkSpacingIsSquare(float spacingNorth, float spacingSouth, float spacingWest, float spacingEast) {
		float threshold = 15;
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
		float gradient, float intercept, 
		Mat &gray_lastFrame,
		float _start = 0, float _end = COLS
	) {
		float start = _start < _end ? _start : _end;
		float end = _start < _end ? _end : _start;

		
		float calcedY = intercept;
		Point point1(0, (int) calcedY);
		
		calcedY = (float) 1600 * gradient + intercept;
		Point point2(1600, (int) calcedY);
		cv::line( gray_lastFrame,
			point1,
			point2,
			Scalar(0,255,0), 3, 3);
	}

	void pureCalcLineX(
		float gradient, float intercept, vector<Point>& line,
		Mat &gray_lastFrame,
		float _start = 0, float _end = COLS
	) {
		return;
		float start = _start < _end ? _start : _end;
		float end = _start < _end ? _end : _start;

		
		float calcedY = intercept;
		Point point1((int) calcedY, 0);
		
		calcedY = (float) 1200 * gradient + intercept;
		Point point2((int) calcedY, 1200);
		cv::line( gray_lastFrame,
			point1,
			point2,
			Scalar(0,255,0), 3, 3);
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
		//vector<Point> marker;
		//marker.clear();

		line( drawing, Point(point.x-size, point.y), Point(point.x+size, point.y), Scalar(255,0,100), 3, 3);
		line( drawing, Point(point.x, point.y-size), Point(point.x, point.y+size), Scalar(255,0,100), 3, 3);
		/*for(int x = -size; x < size; x++) {	
			if(point.x+x < 0 || point.x+x > COLS) continue;
			marker.push_back(Point(point.x+x, point.y));
		}
		for(int y = -size; y < size; y++) {	
			if(point.y+y < 0 || point.y+y > ROWS) continue;
			marker.push_back(Point(point.x, point.y+y));
		}
		drawing.push_back(marker);*/
	}

	void printSquare(Square _square, Mat &gray_lastFrame) {
		Vec3b colour(rand() % 255, rand() % 255, rand() % 255);
		line( gray_lastFrame,
				Point(_square.northEast.x, _square.northEast.y),
				Point(_square.northWest.x, _square.northWest.y),
				colour, 3, 3);

		line( gray_lastFrame,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.southWest.x, _square.southWest.y),
				colour, 3, 3);

		line( gray_lastFrame,
				Point(_square.southWest.x, _square.southWest.y),
				Point(_square.northWest.x, _square.northWest.y),
				colour, 3, 3);

		line( gray_lastFrame,
				Point(_square.southEast.x, _square.southEast.y),
				Point(_square.northEast.x, _square.northEast.y),
				colour, 3, 3);
	}

	float gradientFromPoints(FPoint _point1, FPoint _point2) {
		return ((float) _point1.y - (float) _point2.y) / ((float) _point1.x - (float) _point2.x);
	}

	static bool sortLinesIntercepts(LineMetadata a, LineMetadata b) {
		if(a.xIntercept > 0 && a.xIntercept < 1600 && b.xIntercept > 0 && b.xIntercept < 1600) return a.xIntercept < b.xIntercept;
		return a.intercept < b.intercept;
	}

	static bool sortLinesGradient(LineMetadata a, LineMetadata b) {
		return a.gradient < b.gradient;
	}

	static bool sortSquares(Square a, Square b) {
		float spacing = a.spacing > b.spacing ? b.spacing : a.spacing;
		if(fabs(a.center.y - b.center.y) < spacing * 0.2) {
			return a.center.x < b.center.x;
		} else {
			return a.center.y < b.center.y;
		}
	}
};
