#define OVERSIZED_BOARD 16

using namespace cv;
using namespace std;
#define FROWS 120
#define FCOLS 160
#define BOARD 8


class HandDetector {
	private:	
	Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorMOG2(2, 16, true);
	Mat background;
	Mat lastFrame;

	public:
	bool isHand(Mat& _lastFrame) {
		lastFrame = _lastFrame;

		Mat fgMask;
		if(background.empty()) {
			background = lastFrame;
		} else {
			pBackSub->apply(background, fgMask);
			pBackSub->apply(background, fgMask);
			pBackSub->apply(lastFrame, fgMask);
			refine(fgMask);

			bool isHand = findHull(fgMask);
			_lastFrame = lastFrame;
			if(isHand) return true;
			else {
				background = lastFrame;
				return false;
			}
		}
		return false;	
	}

	bool findHull(Mat& handMask) {
		Mat edges;
		refine(handMask);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		runCanny(handMask, edges);
		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		bool isHand = false;
		for(int i = 0; i < contours.size(); i++) {
			vector<Point> hull;
			convexHull(contours[i], hull);
			Moments m = moments(hull);
			printf("SIZE: %f\n", m.m00);
			if(m.m00 > 6000) {
				drawHull(contours[i], lastFrame);
				isHand = true;
			}
		}
		return isHand;
	}

	void colourFilter(Mat& handFrame) {
		Mat range1, range2, edges;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		cvtColor(lastFrame, handFrame, COLOR_BGR2HSV);
		//inRange(handFrame, Scalar(0, 0, 100), Scalar(10, 100, 205), range1);
		//inRange(handFrame, Scalar(175, 0, 100), Scalar(255, 100, 205), range2);
		//inRange(handFrame, Scalar(10, 50, 150), Scalar(40, 80, 160), range1);
		inRange(handFrame, Scalar(0,50,50), Scalar(15,255,255), range1);
		inRange(handFrame, Scalar(0, 0, 0), Scalar(0, 0, 0), range2);
		bitwise_or(range1, range2, handFrame);
		refine(handFrame);
		runCanny(handFrame, edges);
		findContours( edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		for(int i = 0; i < contours.size(); i++) {
			vector<Point> hull;
			convexHull(contours[i], hull);
			Moments m = moments(hull);
			printf("SIZE: %f\n", m.m00);
			//if(m.m00 > 1000) {
				drawHull(contours[i], lastFrame);
			//}
			//if(m.m00 > 1000) return false;

		}
	}

	void refine(Mat& hand) {
		erode(hand, hand, getStructuring(10));
		dilate(hand, hand, getStructuring(5));
	}

	Mat getStructuring(int size) {
		return getStructuringElement(MORPH_RECT, Size( 2*size + 1, 2*size + 1), Point( size, size ));
	}

	void runCanny(Mat& frame, Mat& edges) {
		int lowThreshold = 15;
		const int ratio = 3;
		const int kernel_size = 3;

		Canny( frame, edges, lowThreshold, lowThreshold*ratio, kernel_size );
	}

	void drawHull(vector<Point>& hull, Mat &frame) {
		vector<vector<Point> > hullsPrint;
		hullsPrint.push_back(hull);
		drawContours(frame, hullsPrint, 0, Scalar(0, 255, 255), 1, 0);
	}
};


