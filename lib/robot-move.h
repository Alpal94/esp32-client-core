using namespace cv;
using namespace std;

#define CAM_RATIO 0.75f // fieldWidth / distance ==> 15cm / 20cm

class RobotMove {
	private:
	struct ChessboardToCamera {
		bool calced;
		float distance;
		float squareWidth;
	};

	RobotPosition defaultPosition = { .x = 18, .y = 10, .z = -5};
	RobotPosition robotPosition = defaultPosition;
	RobotPosition currRobotPosition;
	RobotPosition originalRobotPosition;
	ChessboardToCamera chessboardToCamera;

	public:
	bool processRobotMove(char *move, vector<Square>& squareList) {
		if(squareList.size() != 64) return false;
		for(int i = 0; i < squareList.size(); i++) {
			int r = i / 8;
			int c = i % 8;

			Square square = squareList[i];
			printf("Spacing: %f\n", square.spacing);
		}





		return true;
	}

	private:
	bool setRobotPosition(RobotPosition _position) {
		CURL *curl;
		CURLcode res;

		curl = curl_easy_init();
		if(curl) {
			printf("CURL Init\n");
			char strBuffer[BUFSIZ];
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

			curl_easy_cleanup(curl);
			return true;
		}
		return false;
	}

	float calcRealDist(float realSquareWidth) {
		float fieldWidth = realSquareWidth * 160;
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
			float distance = calcRealDist(realPixelDistance);
			
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
