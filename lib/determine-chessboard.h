
using namespace cv;
using namespace std;

#define PARALLEL_ANGLE 0.03

class DetermineChessBoard {
	private:	
	vector<Point> squareIntercepts;

	Square squareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};
	Square localSquareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};

	bool squareColourDetermined = false;
	Vec3b blackSquareColour;
	Vec3b whiteSquareColour;

	public:
	void findChessboardSquares(vector<LineMetadata> &mergedLines, vector<vector<Point> >& squares, Mat _gray_lastFrame, Mat &_lastFrame, Mat &_display, RobotPosition _robotPosition) {

		vector<vector<Point> > drawing;

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
			float xGradient = mergedLines[i].xGradient;
			//float intercept = mergedLines[i].intercept;
			//if(i < 10 * mergedLines.size() / 16 || i > 11 * mergedLines.size() / 16) continue;
			//26 or 27
			//if(i != 27)  continue;
			printf("SIZE: %ld\n", 10 * mergedLines.size() / 16);
			printf("SIZE: %ld\n", 11 * mergedLines.size() / 16);
			//printLine(mergedLines[i], _lastFrame, Scalar(255, 255, 0));
			vector<LineMetadata> parallel_lines;
			vector<LineMetadata> perpendicular_lines;
			parallel_lines.clear();
			perpendicular_lines.clear();
			for (size_t j = 0; j < mergedLines.size(); j++) {
				float gradient_next = mergedLines[j].gradient;
				float xGradient_next = mergedLines[j].xGradient;
				//float intercept_next = mergedLines[j].intercept;
				
				float perpendicular_angle, parallel_angle;
				if(fabs(gradient) > 5) {
					perpendicular_angle = angleFromGradient(xGradient, gradient_next);
					parallel_angle = angleFromGradient(xGradient, xGradient_next);
				} else {
					perpendicular_angle = angleFromGradient(gradient, xGradient_next);
					parallel_angle = angleFromGradient(gradient, gradient_next);
				}
				printf("DISCOVERED ANGLES: %f %f for %f and %f NEXT: %f, %f\n", perpendicular_angle, parallel_angle, gradient, xGradient, gradient_next, xGradient_next);
				///1.57
				if(fabs(perpendicular_angle) < 0.09) {
					perpendicular_lines.push_back(mergedLines[j]);
				}

				if(fabs(parallel_angle) < PARALLEL_ANGLE) {
					parallel_lines.push_back(mergedLines[j]);
				}
				///1.57

			}
			printf("Parallel lines: %ld Perpedicular lines: %ld All lines: %ld\n", parallel_lines.size(), perpendicular_lines.size(), mergedLines.size());
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

							if((j == 1 && z == 0)) {
								//printLine(northLine, _lastFrame, Scalar(0,0,255));
								//printLine(southLine, _lastFrame, Scalar(0,255,0));
								//printLine(westLine, _lastFrame, Scalar(255,0,0));
								//printLine(eastLine, _lastFrame, Scalar(255,255,255));
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
							if(noSquares) {
								//SQUARE COLOUR IS SLOW (flood fill)
								//squareColour(square, _lastFrame, _display);
								printf("Origin Center: %f %f\n", square.center.x, square.center.y);
								squareList.push_back(square);
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

			if(i) {
				if(lastSquareIndex < 0) lastSquareIndex = 0;
				Square a = squareList[i];
				Square b = squareList[lastSquareIndex];
				printf("Last square index: %d %d\n", lastSquareIndex, i);
				float spacing = a.spacing > b.spacing ? b.spacing : a.spacing;


				if(fabs(a.center.y - b.center.y) < spacing * 0.2 && fabs(a.center.x - b.center.x) < spacing * 0.2) {

					continue;
				}
					
				if(fabs(a.center.y - b.center.y) < spacing * 0.2) {
					if(fabs(a.center.x - b.center.x) > spacing * (1 + 0.2)) continue;
					posX++;
				} else {
					//if(fabs(a.center.y - b.center.y) > spacing * (1 + 0.1)) continue;
					//if(fPixelDist(a.southWest, b.northWest) > 5) continue;
					posX=0;
					posY++;
				}
			}

			//764.972534 798.880493
			//printMarker(Point(885, 557), 10, _lastFrame);
			printMarker(Point(COLS / 2, ROWS / 2), 10, _lastFrame);
			printMarker(Point(764, 798), 10, _lastFrame);
			//1094.459473 514.308594 Center to: 926.229797 515.095703
			//printSquare(squareList[i], _lastFrame);
			//Center from: 846.381348 763.953125 Center to: 846.214661 597.484375
			//printMarker(Point(846, 763), 12, _lastFrame);
			//printMarker(Point(846, 597), 6, _lastFrame);
			lastSquareIndex = i;
			insertSquare(&localSquareMap, posX, posY, squareList[i], { .spacing = 0, .rotation = -squareList[i].rotation, .north = 0, .west = 0 }, Point(0,0), _lastFrame, true);
			count++;
		}
		printf("ls: Square list size stripped: %d\n", count); 
		//Compare squares here	
		/*if(calculateChessboardCamera()) {
			//printf("Chessboard camera position calculated\n");
		}*/

		asciiPrintBoard(&localSquareMap, _lastFrame);
		calcPerfectBoard(&localSquareMap, _lastFrame);
		drawing.push_back(squareIntercepts);

		squares = drawing;
		squareList.clear();
		mergedLines.clear();
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
						//printSquare(rotateSquare((*_localSquareMap)[j%15][i%16], { .rotation = 0/*-(*_localSquareMap)[j%16][i%16].rotation*/ }), _lastFrame);
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
		FPoint centerPoint = { .x = COLS / 2.0, .y = ROWS / 2.0 };
		Square start = { .occupied = false };
		int startX, startY;
		FPoint southWest, northEast;
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				if((*_squareMap)[i][j].occupied) {
					Square square = (*_squareMap)[i][j];
					square = rotateSquare(square, { .spacing = 0, .rotation = square.rotation, .north = 0, .west = 0 });
					float dist = fPixelDist(square.northEast, centerPoint);
					printf("DISTANCE: ne: %f se: %f sw: %f nw: %f\n", fPixelDist(square.northEast, centerPoint), fPixelDist(square.southEast, centerPoint), fPixelDist(square.southWest, centerPoint), fPixelDist(square.northWest, centerPoint));
					if(dist < 15) {
						start = square;
						printf("CENTRE LOCATION: %d %d vs %d %d\n", i, j, square.x, square.y);
						//printSquare(start, _lastFrame);
						break;
					}
				}
				
			}
			if(start.occupied) break;
		}
		printf("\n");
		Square generatedSquareMap[OVERSIZED_BOARD][OVERSIZED_BOARD] = {};
		for(int i = 0; i < OVERSIZED_BOARD; i++) {
			for(int j = 0; j < OVERSIZED_BOARD; j++) {
				(*_squareMap)[i][j].occupied = false;
			}
		}
		generateQuarter(start, _lastFrame, 0, 0, 4, 4, _squareMap, &generatedSquareMap, 3, 3, 0, 0, start.spacing,
			NE, NW, SE, SW,
			SE, SW, NE, NW,
			NW, SW, NE, SE,
			NW, SW, NE, SE
		);
		printf("\n");
		generateQuarter(start, _lastFrame, 1, 0, 5, 4, _squareMap, &generatedSquareMap, 3, 4, 0, 8, start.spacing * 1.0,
			NE, NW, SE, SW,
			SE, SW, NE, NW,
			SW, NW, SE, NE,
			SW, NW, SE, NE
		);
		printf("\n");
		generateQuarter(start, _lastFrame, 1, 1, 5, 5, _squareMap, &generatedSquareMap, 3, 3, 7, 7, start.spacing * 1.05,
			NW, NE, SW, SE,
			SW, SE, NW, NE,
			SW, NW, SE, NE,
			SW, NW, SE, NE
		);
		printf("\n");
		generateQuarter(start, _lastFrame, 0, 1, 4, 5, _squareMap, &generatedSquareMap, 3, 3, 7, 0, start.spacing * 1.05,
			NW, NE, SW, SE,
			SW, SE, NW, NE,
			NW, SW, NE, SE,
			NW, SW, NE, SE
		);
	}

	void generateQuarter(
		Square start, Mat &_lastFrame, int width1, int height1, int width2, int height2,
		Square (*_squareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], 
		Square (*_newSquareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD],
		int startX, int startY, int endX, int endY, float spacing,
		Orientation fvob1, Orientation fvob2, Orientation fvot1, Orientation fvot2,
		Orientation vob1, Orientation vob2, Orientation vot1, Orientation vot2,
		Orientation fhob1, Orientation fhob2, Orientation fhot1, Orientation fhot2,
		Orientation hob1, Orientation hob2, Orientation hot1, Orientation hot2
		) {

		Square current = start;
		Square rowStart = start;

		bool special = false;
		int posX = startX, posY = startY;
		if(height1 == 0 && width1 == 0) {
			printf(" %d_%d ", posX, posY);
			//printSquare(start, _lastFrame);
			localSquareMap[posX][posY] = start;
			special = true;
		}
		for(int w = 0; w < width2; w++) {
			for(int h = 1; h < height2; h++) {

				//if((*_squareMap)[posX][posY].occupied) spacing = (*_squareMap)[posX][posY].spacing;
				current = generateNeighboringSquare(current, orientationToFPoint(current, fvob1), orientationToFPoint(current, fvob2), orientationToFPoint(current, fvot1), orientationToFPoint(current, fvot2), vob1, vob2, vot1, vot2, spacing, _lastFrame);
				if(w >= width1) {
					//printSquare(current, _lastFrame);
					if(special) {
						startX < endX ? posX++ : posX--;
						printf(" %d_%d ", posX, posY);
						localSquareMap[posX][posY] = current;
					} else {
						startX < endX ? posX++ : posX--;
						printf(" %d_%d ", posX, posY);
						localSquareMap[posX][posY] = current;
					}
				}

			}
			if(width2 != w + 1) {
				//if((*_squareMap)[posX][posY].occupied) spacing = (*_squareMap)[posX][posY].spacing;
				printf("SPACING: %f\n", spacing);
				rowStart = generateNeighboringSquare(rowStart, orientationToFPoint(rowStart, fhob1), orientationToFPoint(rowStart, fhob2), orientationToFPoint(rowStart, fhot1), orientationToFPoint(rowStart, fhot2), hob1, hob2, hot1, hot2, spacing, _lastFrame);
				current = rowStart;
				posX = startX;
				if(height1 == 0) {
					//printSquare(current, _lastFrame);
					if(special) { 
						startY < endY ? posY++ : posY--;
						printf(" %d__%d ", posX, posY);
						localSquareMap[posX][posY] = current;
						//if(!specialFirstLine) startY < endY ? posY++ : posY--;
					} else {
						printf(" %d__%d ", posX, posY);
						localSquareMap[posX][posY] = current;
						//if(!specialFirstLine) startY < endY ? posY++ : posY--;
						startY < endY ? posY++ : posY--;
					}
				} else {
					startY < endY ? posY++ : posY--;
				}
			}
		}
	}

	FPoint orientationToFPoint(Square square, Orientation orientation) {
		switch (orientation) {
			case NE: return square.northEast;
			case NW: return square.northWest;
			case SW: return square.southWest;
			case SE: return square.southEast;
			default: return { .x = -1, .y = -1 };
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
		neighbor.occupied = true;
		neighbor.rotation = calcRotation(neighbor);
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
		float lineX = line.xIntercept;
		float compX = comp.xIntercept;
		if(isinf(lineX) || isinf(compX)) {
			lineX = line.intercept; 
			compX = comp.intercept;
		}

		return lineX < compX;
	}
	void insertSquare(Square (*_squareMap)[OVERSIZED_BOARD][OVERSIZED_BOARD], int posX, int posY, Square square, MapOffset offset, Point localOffset, Mat &_lastFrame, bool debug=false) {

		square.x = posX;
		square.y = posY;
		Square origin = (*_squareMap)[OVERSIZED_BOARD/2][OVERSIZED_BOARD/2];
		Square tSquare = translateSquare(rotateSquare(square, offset), offset);

		//printSquare(rotateSquare(tSquare, {.rotation = square.rotation}), _lastFrame);
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
		float _start = 0, float _end = 160
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
		
		calcedY = (float) ROWS * gradient + intercept;
		Point point2((int) calcedY, ROWS);
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

	void printLine(LineMetadata _line, Mat& frame, Scalar color) {
		Point x1 = Point(_line.bounds[0], _line.bounds[1]);
		Point x2 = Point(_line.bounds[2], _line.bounds[3]);
		line(frame, x1, x2, color, 3, 3);
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
