#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Read the image file
    Mat image = imread("test.jpg");

    // Check for failure
    if (image.empty()) 
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    // Create a window
    namedWindow("OpenCV Test", WINDOW_AUTOSIZE);

    // Show our image inside the created window.
    imshow("OpenCV Test", image);

    // Wait for any keystroke in the window
    waitKey(0);

    return 0;
}