/**
* @brief This Video Stabilization Code is deisgned for the meeting at 09.09.2015
* @author Ahmet Tavli
*/

#include "ahmetcv\VisionStabilizer.h"

using namespace VisionStabilizer;

/**
* @function main
*/
int main(int, char** argv) {
	Function::DebugScreen();

	string path;

	cout << "Please Enter the Path of the Video (Type a to get hcv.mp4) -->";
	cin >> path; // Example is C:\hcv.mp4

	if (path == "a"){
		path = "C:\\hcv.mp4";
	}

	VideoCapture cap(path);

	if (Function::isVideoOpen(cap)) {
		cout << "\n\n\n\tOpening Video is successful\n\n\n";

		Mat prev_frame, next_frame;
		Mat prev_frameg, next_frameg;

		ofstream originalVideoValues("OriginalVideoValues.txt");

		cap >> prev_frame;
		cvtColor(prev_frame, prev_frameg, COLOR_BGR2GRAY);

		// Not Sure if these are the correct coordinates
		double org_x_prev = prev_frame.at<double>(0, 2);
		double org_y_prev = prev_frame.at<double>(1, 2);

		if (originalVideoValues.is_open()) {
			originalVideoValues << " \t\t Inside Original VÝdeo Values\n  ";
			originalVideoValues << "\t\t\t(x, y, angle values Respectively)\n\n";

			originalVideoValues << org_x_prev << "\t" << org_y_prev << endl;
		}

		// This array will store the Transformation needed params
		vector<TransformParam> transformation_;

		// maxFrames : Maximum Frames of the Video ; fps : Frame Per Second
		const int &maxFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
		const int &fps = cap.get(CV_CAP_PROP_FPS);

		// Calculation of the Percentage and Display to the User
		int counter = 0;

		// 1st ) Next Frame Transformation
		if (nextFrameTransformation.is_open()) {
			nextFrameTransformation << " \t\t Inside Next Frame Transformation Method\n  ";
			nextFrameTransformation << "\t\t\t(x, y, angle values Respectively)\n\n";
		}

		for (;;) {
			cap >> next_frame;
			if (next_frame.data == NULL) {
				break;
			}

			// Not Sure if those are the correct coordinates
			double org_x_next = next_frame.at<double>(0, 2);
			double org_y_next = next_frame.at<double>(1, 2);

			originalVideoValues << org_x_next << "\t" << org_y_next << endl;

			cvtColor(next_frame, next_frameg, COLOR_BGR2GRAY);

			// goodFeaturesToTrack variables
			vector<Point2f> prev_pts;
			int maxCorners = 200;
			double qualityLevel = 0.01;
			double minDistance = 30;
			goodFeaturesToTrack(prev_frameg, prev_pts, maxCorners, qualityLevel, minDistance);

			// calcOpticalFlwPyrLK variables
			vector<Point2f> next_pts;
			vector<uchar> status;
			vector<float> err;
			calcOpticalFlowPyrLK(prev_frameg, next_frameg, prev_pts, next_pts, status, err);

			// estimateRigidTransform variables
			vector<Point2f> prev_pts_err;
			vector<Point2f> next_pts_err;
			bool fullAffine = false;

			// Select the Tranformation Required Variables
			for (int i = 0; i < status.size(); i++) {
				if (status[i]) { // status array ontains only 0 or 1
					prev_pts_err.push_back(prev_pts[i]);
					next_pts_err.push_back(next_pts[i]);
				}
			}

			Mat trans = estimateRigidTransform(prev_pts_err, next_pts_err, fullAffine);

			// Get Transformation Values
			double x_coord = trans.at<double>(0, 2);
			double y_coord = trans.at<double>(1, 2);
			double phi_val = atan2(trans.at<double>(1, 0), trans.at<double>(0, 0));

			// Add Transformation Values to the vector.
			transformation_.push_back(TransformParam(x_coord, y_coord, phi_val));

			// Type to the text file
			nextFrameTransformation << x_coord << "\t" << y_coord << "\t" << phi_val << endl;

			// Consecutive Frames
			next_frame.copyTo(prev_frame);
			next_frameg.copyTo(prev_frameg);
			counter++;
			cout << "\r 1) Processed Frame % " << ((counter * 100) / maxFrames) << "  (" << counter << " / " << maxFrames << " ) " << " Frame per Second is " << fps;

		} // end for

		// Close Raw video Coordinate Values
		originalVideoValues.close();

		// Close the Text
		nextFrameTransformation.close();

		int processed_frame = counter;
		/* Next Frame Transformation ends  */

		// 2nd Get Image Trajectory

		// GetImageTrajectory variables
		double x_ = 0;
		double y_ = 0;
		double phi_ = 0;
		vector<Trajectory> trajectory_;

		if (getImageTrajectory.is_open()){
			getImageTrajectory << " \t\t Inside Get Image Trajectory Method\n  ";
			getImageTrajectory << "\t\t\t(x, y, angle values Respectively)\n\n";
		}

		for (int i = 0; i < transformation_.size(); i++) {
			x_ += transformation_[i].dx;
			y_ += transformation_[i].dy;
			phi_ += transformation_[i].phi;

			trajectory_.push_back(Trajectory(x_, y_, phi_));

			getImageTrajectory << x_ << "\t" << y_ << "\t" << phi_ << endl;
		}

		getImageTrajectory.close();

		/* Get Image Trajectory Ends */

		// 3rd SmoothingTrajectory

		// Smoothing Trajectory Variables
		const int &smoothing_rad = 10;
		vector<Trajectory> smoothing_traj;

		if (smoothTrajectory.is_open()){
			smoothTrajectory << " \t\t Inside Smooth Trajectory Method\n  ";
			smoothTrajectory << "\t\t\t(x, y, angle values Respectively)\n\n";
		}

		for (int i = 0; i < trajectory_.size(); i++) {
			double sum_x = 0;
			double sum_y = 0;
			double sum_a = 0;
			int count = 0;

			for (int j = -smoothing_rad; j <= smoothing_rad; j++) {
				if (i + j >= 0 && i + j < trajectory_.size()) {
					sum_x += trajectory_[i + j].x;
					sum_y += trajectory_[i + j].y;
					sum_a += trajectory_[i + j].phi;

					count++;
				} // end if
			} // end for

			double avg_a = sum_a / count;
			double avg_x = sum_x / count;
			double avg_y = sum_y / count;

			smoothTrajectory << avg_x << "\t" << avg_y << "\t" << avg_a << endl;

			smoothing_traj.push_back(Trajectory(avg_x, avg_y, avg_a));
		}
		smoothTrajectory.close();
		/* Smoothing Trajectory Ends */

		// 4th Step Generating Transformation ( Enhancing )

		if (generateTransformation.is_open()) {
			generateTransformation << " \t\t Inside Generate Transformation Method\n  ";
			generateTransformation << "\t\t\t(x, y, angle values Respectively)\n\n";
		}

		// Generate Transformaton Variables
		x_ = 0;
		y_ = 0;
		phi_ = 0;
		vector<TransformParam> new_transformation_;

		for (int i = 0; i < transformation_.size(); i++) {
			x_ += transformation_[i].dx;
			y_ += transformation_[i].dy;
			phi_ += transformation_[i].phi;

			// target - current
			double diff_x = smoothing_traj[i].x - x_;
			double diff_y = smoothing_traj[i].y - y_;
			double diff_a = smoothing_traj[i].phi - phi_;

			double dx = transformation_[i].dx + diff_x;
			double dy = transformation_[i].dy + diff_y;
			double da = transformation_[i].phi + diff_a;

			new_transformation_.push_back(TransformParam(dx, dy, da));

			generateTransformation << dx << "\t" << dy << "\t" << da << endl;
		} // end for
		generateTransformation.close(); // Close the txt file
		/* Generating Trajectory Ends */

		// 5th Apply Transformation

		if (applyTransformation.is_open()) {
			applyTransformation << " \t\t Inside Next Frame Transformation Method\n  ";
		}

		// Window Name
		const string &winname = "Video Stabilization";

		// Set Video to the 1st Frame
		cap.set(CV_CAP_PROP_POS_FRAMES, 0);

		// new transformation matrix applied to the video
		Mat new_trans(2, 3, CV_64F);

		counter = 0;
		while (counter < processed_frame) {
			cap >> next_frame;
			if (next_frame.data == NULL) {
				break;
			}

			new_trans.at<double>(0, 0) = cos(new_transformation_[counter].phi);
			new_trans.at<double>(0, 1) = -sin(new_transformation_[counter].phi);
			new_trans.at<double>(1, 0) = sin(new_transformation_[counter].phi);
			new_trans.at<double>(1, 1) = cos(new_transformation_[counter].phi);

			new_trans.at<double>(0, 2) = new_transformation_[counter].dx;
			new_trans.at<double>(1, 2) = new_transformation_[counter].dy;

			applyTransformation << new_trans.at<double>(0, 0)
				<< "\t" << new_trans.at<double>(0, 1)
				<< "\t" << new_trans.at<double>(1, 0)
				<< "\t" << new_trans.at<double>(1, 1)
				<< "\t" << new_trans.at<double>(0, 2) // x value
				<< "\t" << new_trans.at<double>(1, 2) << endl; // angle, x , y tranformation values

			Mat display;

			// Applying the Transformation to the video
			warpAffine(next_frame, display, new_trans, next_frame.size());

			if (display.rows > 600) {
				display.rows = 600;
			}

			imshow(winname, display);
			waitKey(20);

			counter++;
		}

		// Close Text File
		applyTransformation.close();
		/* Apply Transformation Ends */
	}

	Function::WaitForUserInteraction();
}