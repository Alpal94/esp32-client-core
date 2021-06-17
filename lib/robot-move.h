using namespace cv;
using namespace std;
// OLD fieldWidth / distance ==> 15cm / 20cm

#define CAM_RATIO 3.26f // fieldWidth / distance ==> 65.18cm / 20cm
#define SQUARE_WIDTH_CM 3.306f
#define CAMERA_TO_CLAW_OFFSET 3.00f
#define HOARM_LENGTH 14.7
#define MAINARM_LENGTH 13.5
#define SWOOP_HEIGHT 9
#define PICKUP_HEIGHT 2

class RobotMove {
	private:
	struct ChessboardToCamera {
		bool calced;
		float distance;
		float squareWidth;
	};

	enum ClawPosition { Open, Ready, Closed };

	RobotPosition defaultPosition = { .x = HOARM_LENGTH + 5, .y = MAINARM_LENGTH + 3, .z = 0, .c = Open};
	RobotPosition robotPosition = defaultPosition;
	RobotPosition currRobotPosition;
	RobotPosition originalRobotPosition;
	ChessboardToCamera chessboardToCamera;
	FenProcessor fen;

	public:
	void init() {
		printf("Start INIT\n");
		setRobotPosition(defaultPosition);
		printf("End INIT\n");
	}
	bool processRobotMove(char *move, vector<Square>& squareList) {
		if(squareList.size() != 64) return false;
		float spacingCenter = 0;
		FPoint boardCenter;
		for(int r = 3; r <= 4; r++) {
			for(int c = 3; c <= 4; c++) {
				Square square = squareList[r + c * 8];
				//square = rotateSquare(square, { .rotation = square.rotation });
				boardCenter = calcCenterFromMiddleForSquares(r, c, square);
				spacingCenter = square.spacing;
			}
		}
		//spacingCenter /= 4;

		float pixelDistanceRatio = calcPixelDistanceRatio(spacingCenter, SQUARE_WIDTH_CM);
		float height = cameraHeightFromFieldWidth(pixelDistanceRatio);

		FPoint realDistCamCenterToBoardCenter = {
			.x = (COLS / 2 - boardCenter.x) * pixelDistanceRatio,
			.y = (ROWS / 2 - boardCenter.y) * pixelDistanceRatio
		};
		int *processedMove = fen.processMove(move);

		Square from = squareList[processedMove[1] + 8 * processedMove[0]];
		//from = rotateSquare(from, { .rotation = from.rotation });

		Square to = squareList[processedMove[3] + 8 * processedMove[2]];
		//to = rotateSquare(to, { .rotation = to.rotation });

		FPoint boardCenterToTargetFrom = {
			.x = (COLS / 2 - from.center.x) * pixelDistanceRatio,
			.y = (ROWS / 2 - from.center.y) * pixelDistanceRatio
		};
		FPoint boardCenterToTargetTo = {
			.x = (COLS / 2 - to.center.x) * pixelDistanceRatio,
			.y = (ROWS / 2 - to.center.y) * pixelDistanceRatio
		};

		printf("RM Pixel distance ratio: %f\n", pixelDistanceRatio);
		printf("RM Processed: %d %d - %d %d\n", processedMove[0], processedMove[1], processedMove[2], processedMove[3]);
		printf("RM Center from: %f %f Center to: %f %f\n", from.center.x, from.center.y, to.center.x, to.center.y);
		printf("RM Proposed move: %s\n", move);
		printf("RM Calculated HEIGHT: %f for fieldWidth: %f\n", height, pixelDistanceRatio * COLS);
		printf("RM Calculated board offset: %f %f\n", realDistCamCenterToBoardCenter.x, realDistCamCenterToBoardCenter.y);

		RobotPosition robotPositionFrom = calculateRobotPositionHover(realDistCamCenterToBoardCenter, boardCenterToTargetFrom, SWOOP_HEIGHT, Ready);
		RobotPosition robotPositionTo = calculateRobotPositionHover(realDistCamCenterToBoardCenter, boardCenterToTargetTo, SWOOP_HEIGHT, Closed);


		RobotPosition position;
		if(setRobotPosition(robotPositionFrom)) {
			robotPosition = robotPositionFrom;
		}
		sleep(3);
		position = verticalShift(robotPosition, PICKUP_HEIGHT, Ready);
		if(setRobotPosition(position)) {
			robotPosition = position;
		}
		sleep(3);
		position = verticalShift(robotPosition, PICKUP_HEIGHT, Closed);
		if(setRobotPosition(position)) {
			robotPosition = position;
		}
		sleep(3);
		position = verticalShift(robotPosition, SWOOP_HEIGHT, Closed);
		if(setRobotPosition(position)) {
			robotPosition = position;
		}
		sleep(3);
		if(setRobotPosition(robotPositionTo)) {
			robotPosition = robotPositionTo;
		}
		sleep(3);
		position = verticalShift(robotPosition, PICKUP_HEIGHT, Closed);
		if(setRobotPosition(position)) {
			robotPosition = position;
		}
		sleep(3);
		position = verticalShift(robotPosition, PICKUP_HEIGHT, Open);
		if(setRobotPosition(position)) {
			robotPosition = position;
		}
		sleep(3);
		if(setRobotPosition(defaultPosition)) {
			robotPosition = defaultPosition;
		}

		/*if(setRobotPosition(robotPositionTo)) {
			robotPosition = robotPositionTo;
		}*/
		printf("RM Default: %f %f %f\n", robotPosition.x, robotPosition.y, robotPosition.z);
		printf("RM Board Center: %f %f\n", realDistCamCenterToBoardCenter.x, realDistCamCenterToBoardCenter.y);
		printf("RM Target From: %f %f %f\n", robotPositionFrom.x, robotPositionFrom.y, robotPositionFrom.z);
		printf("RM Target To: %f %f %f\n", robotPositionTo.x, robotPositionTo.y, robotPositionTo.z);
		printf("RM To: %f %f From: %f %f\n", boardCenterToTargetTo.x, boardCenterToTargetTo.y, boardCenterToTargetFrom.x, boardCenterToTargetFrom.y);

		return true;
	}

	private:
	RobotPosition calculateRobotPositionHover(FPoint realDistCamCenterToBoardCenter, FPoint boardCenterToTarget, float height, ClawPosition c) {
		FPoint move = {
			.x = boardCenterToTarget.x - realDistCamCenterToBoardCenter.x,
			.y = boardCenterToTarget.y - realDistCamCenterToBoardCenter.y
		};
		printf("RM Move: %f %f\n", move.x, move.y);
		printf("RM RB: %f %f\n", robotPosition.x, robotPosition.y);
		//Robot coords conversion --> c.x == r.y, c.x == r.z, r.z == vertical
		return {
			.x = robotPosition.x + move.y - CAMERA_TO_CLAW_OFFSET,
			.y = height,
			.z = robotPosition.z + move.x,
			.c = c
		};

	}

	RobotPosition verticalShift(RobotPosition position, float height, ClawPosition claw) {
		position.y = height;
		position.c = claw;
		return position;
	}

	FPoint calcCenterFromMiddleForSquares(int r, int c, Square square) {
		if(r == 3 && c == 3) {
			printf("SQUARE POS: %f %f\n", square.northEast.x, square.northEast.y);
			return square.northEast;
		} else if(r == 3 && c == 4) {
			printf("SQUARE POS: %f %f\n", square.northWest.x, square.northWest.y);
			return square.northWest;
		} else if(r == 4 && c == 4) {
			printf("SQUARE POS: %f %f\n", square.southWest.x, square.southWest.y);
			return square.southWest;
		} else if(r == 4 && c == 3) {
			printf("SQUARE POS: %f %f\n", square.southEast.x, square.southEast.y);
			return square.southEast;
		}

		return {
			.x = 0,
			.y = 0
		};
	}
	bool setRobotPosition(RobotPosition _position) {
		if(!COMMAND_ROBOT) return true;

		CURL *curl;
		CURLcode res;

		curl = curl_easy_init();
		if(curl) {
			printf("CURL Init\n");
			char strBuffer[BUFSIZ];
			sprintf(strBuffer,  "http://chessrobot.local/setpos?x=%f&y=%f&z=%f&c=%d", _position.x, _position.y, _position.z, _position.c);
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

			curl_easy_cleanup(curl);
			return true;
		}
		return false;
	}

	float calcPixelDistanceRatio(float pixelDist, float realDist) {
		return realDist / pixelDist;
	}

	float cameraHeightFromFieldWidth(float pixelDistanceRatio) {
		float fieldWidth = pixelDistanceRatio * COLS;
		return fieldWidth / CAM_RATIO;
	}

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
			float distance = cameraHeightFromFieldWidth(realPixelDistance);
			
			chessboardToCamera.calced = true;
			chessboardToCamera.distance = distance;
			chessboardToCamera.squareWidth = squareWidth;
			return true;
		}
		return false;

	}

	double distance( RobotPosition p1, RobotPosition p2 )
	{
		double dx = p1.x - p2.x;
		double dy = p1.y - p2.y;
		double dz = p1.z - p2.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}


};
