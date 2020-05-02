class CalibrateCamera {
	private:
	Mat K, D;
	Mat grayFrame;
	bool KDReadFromFile = 0;
	int board_width = 6;
	int board_height = 4;
	int frameReference = 0;
	bool calibRun = false;
	float square_size = 23.3;
	const char* window_name = "Calibration Program";

	vector< vector< Point3f > > object_points;
	vector< vector< Point2f > > image_points;
	vector< Point2f > corners;


	void convertToBinary(Mat& image) {
		int threshold_value = 146;
		int threshold_type = 3;
		//int const max_value = 255;
		//int const max_type = 4;
		int const max_binary_value = 255;

		threshold( image, image, threshold_value, max_binary_value, threshold_type );
	}

	void runCalibration() {
		vector< Mat > rvecs, tvecs;
		int flag = 0;
		flag |= CALIB_FIX_K4;
		flag |= CALIB_FIX_K5;
		float rms = calibrateCamera(object_points, image_points, grayFrame.size(), K, D, rvecs, tvecs, flag);

		FileStorage fs("calibration_file.txt", FileStorage::WRITE);
		fs << "K" << K;
		fs << "D" << D;
		fs << "board_width" << board_width;
		fs << "board_height" << board_height;
		fs << "square_size" << square_size;
		fs << "dms" << rms;
		printf("Done Calibration with dms: %f\n", rms);
		fs.release();
	}

	Mat calibrationExecute(Mat grayImage) {

		printf("Cali running\n");
		Size board_size = Size(board_width, board_height);
		//int board_n = board_width * board_height;
		bool found = false;
		found = findChessboardCorners(grayImage, board_size, corners,  CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if(found) {
			printf("FOUND\n");
			cornerSubPix(grayImage, corners, cv::Size(5, 5), cv::Size(-1, -1),
				TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.1));
			drawChessboardCorners(grayImage, board_size, corners, found);
		}
		vector< Point3f > obj;
		for (int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
		if (found) {
			printf("Found corners\n");
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
		return grayImage;
	}

	public:
	void calculateCalibrationDataFromFrame(Mat _fullColourFrame) {
		frameReference++;
		float scale = 0.1; 
		bool scaleImage = false;
		if(scaleImage) {
			resize(_fullColourFrame, _fullColourFrame, Size(), scale, scale);
		}
		cvtColor( _fullColourFrame, grayFrame, COLOR_BGR2GRAY );
		//convertToBinary(grayFrame);

		if(readCalibration(K, D)) {
			//undistortImage(grayFrame);
			printf("CALIB TRUE\n");
			calibRun = true;
			imshow (window_name, grayFrame);
		}
		/*if(!calibRun) {
			Mat chessBoardImage = calibrationExecute(grayFrame);
			if(scaleImage) {
				resize(chessBoardImage, chessBoardImage, Size(), 0.5 / scale, 0.5 / scale);
			}
			imshow (window_name, chessBoardImage);
		}
		if(frameReference > 104 && !calibRun) {
			runCalibration();
			calibRun = true;
		}
		if(calibRun) {

			Mat curr = grayFrame.clone();
			undistortImage(curr);

			if(scaleImage) {
				resize(curr, curr, Size(), 0.5 / scale, 0.5 / scale);
			}
			imshow (window_name, curr);
		}*/
	}

	void undistortImage(Mat& image) {
		if(!KDReadFromFile) {
			if(readCalibration(K, D)) {
				KDReadFromFile = 1;
			} else {
				KDReadFromFile = -1;
			}
		}
		if(KDReadFromFile > 0) {
			Mat clone = image.clone();
			undistort(clone, image, K, D);
		}
	}

	bool readCalibration(Mat& cameraMatrix, Mat& distCoeffs) {
		FileStorage fs("calibration_file.txt", FileStorage::READ);
		if(!fs.isOpened()) {
			printf("No YAML File\n");
			return false;
		}

		fs["K"] >> cameraMatrix;
		fs["D"] >> distCoeffs;

		return true;
	}


};
