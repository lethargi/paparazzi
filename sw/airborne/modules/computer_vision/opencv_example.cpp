#include "opencv_example.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;



int opencv_example(char* img, int width, int height)
{
	/// Initialize and set variables for use in the progam
		Mat M(width, height, CV_8UC2, img);		// Create a new image, using the original bebop image
		Mat mask, hist, hue, imageROI, backproj;		// Allocate space for the processed images
		/// Histogram setup
			int vmin, vmax, smin;					// Initialize threshold values for histogram
			vmin = 10; vmax = 256; smin = 35;		// Set the threshold values
			int ch[] = { 0,0 };
			int hsize = 16;
			float hranges[] = { 0,180 };
			const float* phranges = hranges;
		/// Reference area setup	
			Rect selection;							// Allocate space for the selection of the image
			Point lefttop, rightbottom;				// Corner points of the reference area
			int refWidth = 25, refHeight = 8;		// Percentage of the screen as reference
		/// Distance search setup
			Rect block;					// Allocate space for block with distance
			int numCols = 16;			// number of cols for distance search
			int numRows = 16;			// number of rows for distance search
			float distThresh = 0.5;		// % of area of block in column that needs to be filled
			/// WARNING: '16' IS HARDCODED, corresponds to numCols!
			int distances[16]; 			// Initialize the array for the output
			Point samplePoint1, samplePoint2;	// lefttop and rightbottom of distance seeking box
			Point distLineL, distLineR;			// Used for plotting the distance lines
			int isSafeToGoForwards = 0;		// End result of the code :)
		
	/// Convert image type to HSV for processing
		cvtColor(M, M, COLOR_YUV2BGR_Y422); 	// Convert from YUV to BGR color space
		//GaussianBlur(M, M, Size(5, 5), 3);	// Gaussian blur the image to even it out
		cvtColor(M, M, COLOR_BGR2HSV);		// Convert from BGR to HSV color space
	
	/// Set up the histogram
		inRange(M,Scalar(0,smin,vmin),Scalar(180,256,vmax),mask);
		hue.create(M.size(),M.depth());
		mixChannels(&M,1,&hue,1, ch,1);
	
	/// Calculate the selection region (reference area corner points)
		lefttop.x = M.cols * (0.5 - float(refWidth) / 100 / 2);		// Lefttop.x of the reference area
		lefttop.y = M.rows * (1 - float(refHeight) / 100);			// Lefttop.y of the reference area
		rightbottom.x = M.cols * (0.5 + float(refWidth) / 100 / 2);	// Bottomright.x of the reference area
		rightbottom.y = M.rows;										// Bottomright.y of the reference area

		selection = Rect(lefttop, rightbottom);		// Define the reference area pixels

	
	/// Calculate the histogram of the reference area
		Mat roi(hue,selection),maskroi(mask,selection);			// Make selection the region of interest
		calcHist(&roi,1,0,maskroi,hist,1,&hsize,&phranges);		// Calculate the histogram of the ROI
		normalize(hist,hist,0,255,NORM_MINMAX);					// Return the normalized histogram
		
	/// Calculate the black and white result
		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		threshold(backproj, backproj, 200, 255, 0);				// Optional threshold
		//Mat element = getStructuringElement(0, Size(3, 3));
		//morphologyEx(backproj, backproj, 1, element);
		//morphologyEx(backproj, backproj, 0, element);
		
	/// Calculate the distances in the frame based on the histogram per column
		int stepSize = backproj.rows / numRows;
		int stepSizeHor = backproj.cols / numCols;

		for (int j = 0; j < numCols; j++)
		{
			for (int i = backproj.rows; i > 0; i -= stepSize)
			{
				samplePoint1.x = j*stepSizeHor; samplePoint1.y = i - stepSize;
				samplePoint2.x = (j + 1)*stepSizeHor;
				samplePoint2.y = i;

				Rect searchSpace = Rect(samplePoint1.x, samplePoint1.y,
					samplePoint2.x - samplePoint1.x, samplePoint2.y - samplePoint1.y);
				imageROI = backproj(searchSpace);
				//printf("imageROI rows: %d , cols: %d\n", imageROI.rows, imageROI.cols);

				int noOfZeros = countNonZero(imageROI);
//int noOfZeros = 50;
				if (noOfZeros < searchSpace.area()*distThresh)
				{
					distances[j] = searchSpace.y;
					break;
				}
				else
				{
					if (i == 0)
					{
						distances[j] = searchSpace.y;
					}
				}

			}
		}

	/// Plot the reference area
		rectangle(backproj, lefttop, rightbottom, CV_RGB(76,84,255));
	
	/// plot the line
		for (int j = 0; j < backproj.cols / numCols; j++)
		{
			distLineL.x = j*(backproj.cols / numCols); distLineL.y = distances[j];
			distLineR.x = (j + 1)*(backproj.cols / numCols); distLineR.y = distLineL.y;
			line(backproj, distLineL, distLineR, CV_RGB(76,84,255));
		}

	/// Output the desired result
		int minimumDist = 0;
		int colMinimum;
		for (int i = 8; i <= 9; i++)
		{
			if (distances[i] > minimumDist)
			{
				minimumDist = distances[i];
				colMinimum = i;
			}
		}

		int distPercentage = float(backproj.rows-minimumDist) / float(backproj.rows) * 100;
		//if (minimumDist > float(backproj.rows) / 2)
		//{
			//distPercentage = 100;
		//}
		//int outputVars[] = { distPercentage, (float(colMinimum) / float(numCols)) * 100 };

		if (distPercentage > 30)
		{
			isSafeToGoForwards = 1;
		}
		else
		{
			isSafeToGoForwards = 0;
		}

		printf("Safe to go forwards = %d\n", isSafeToGoForwards);

	/// Extra magic from MAVLAB - conversion to YUV422
	for (int row=0; row <height; row ++){
		for (int col=0; col <width; col++){
			img[(row*width+col)*2+1] = backproj.at<uint8_t>(row,col);
			img[(row*width+col)*2] = 127;
		}
	}

	return 0;
}
