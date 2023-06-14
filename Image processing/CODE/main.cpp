#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat Orginal_Frame, IMG_HSV, ThresholdIMG, canny_OP;
int Min_H = 0, Max_H = 179,Min_S = 60, Max_S = 255, Min_V = 30, Max_V = 255;
int blurSize = 5, BlobSize = 1, element = 20, Thresh = 100, min_rect_region = 1000;

Point2f center;
vector<Point> pts;

int main(int argc, char* argv[])
{
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cout << "Unable to open the web Camera" << endl;
        return -1;
    }

    namedWindow("Control_Window", 1);
    createTrackbar("H LOW", "Control_Window", &Min_H, 179);
    createTrackbar("H HIGH", "Control_Window", &Max_H, 255);
    createTrackbar("S LOW", "Control_Window", &Min_S, 255);
    createTrackbar("S HIGH", "Control_Window", &Max_S, 255);
    createTrackbar("V LOW", "Control_Window", &Min_V, 255);
    createTrackbar("V HIGH", "Control_Window", &Max_V, 255);

    while (1)
    {
        bool bSuccess = cap.read(Orginal_Frame);

        if (!bSuccess)
        {
            cout << "Unable to read the Frame from video stream" << endl;
            break;
        }

        cap >> Orginal_Frame;

        cvtColor(Orginal_Frame, IMG_HSV, COLOR_BGR2HSV); // BGR to HSV

        Mat ThresholdIMG;
        inRange(IMG_HSV, Scalar(Min_H, Min_S, Min_V), Scalar(Max_H, Max_S, Max_V), ThresholdIMG);
        imshow("HSV Image", IMG_HSV);
        imshow("Threshold Image", ThresholdIMG);
        line(ThresholdIMG, { 0, Orginal_Frame.rows }, { Orginal_Frame.cols, Orginal_Frame.rows }, Scalar(0, 30, 0), 380, LINE_8);
        medianBlur(ThresholdIMG, ThresholdIMG, 7);

        erode(ThresholdIMG, ThresholdIMG, getStructuringElement(MORPH_ELLIPSE, Size(element, element)));
        dilate(ThresholdIMG, ThresholdIMG, getStructuringElement(MORPH_ELLIPSE, Size(element, element)));

        std::vector<std::vector<cv::Point> > OutputArray_contours;
        std::vector<cv::Vec4i> OutputArray_hierarchy;

        Canny(ThresholdIMG, canny_OP, Thresh, Thresh * 2, 3);
        imshow("Canny", canny_OP);
        findContours(ThresholdIMG, OutputArray_contours, OutputArray_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        cout << "Number of Contours Formed is:" << OutputArray_contours.size() << endl;

        Mat drawing = Mat::zeros(ThresholdIMG.size(), IMREAD_COLOR);

        vector<RotatedRect> Min_Rect(OutputArray_contours.size());
        for (float i = 0; i < OutputArray_contours.size(); i++)
        {
            float rect_region = contourArea(OutputArray_contours[i]);

            if (rect_region > min_rect_region)
                Min_Rect[i] = minAreaRect(Mat(OutputArray_contours[i]));

                    drawContours(Orginal_Frame, OutputArray_contours, i, Scalar(0, 255, 0), 1, 8, vector<Vec4i>(), 0, Point());

                    Point2f Rect_Points[4];
                    Min_Rect[i].points(Rect_Points);

                     float H = Min_Rect[i].size.height;
                    float W = Min_Rect[i].size.width;

                    if (H > W)
                    {
                       Min_Rect[i].size.height = (float)(0.34) * Min_Rect[i].size.height;
                       Min_Rect[i].center = (Rect_Points[1] + Rect_Points[2]) / 2 + (Rect_Points[0] - Rect_Points[1]) / 6;
                    }
                     else if (W > H)
                    {
                        Min_Rect[i].size.width = (float)(0.34) * Min_Rect[i].size.width;
                        Min_Rect[i].center = (Rect_Points[2] + Rect_Points[3]) / 2 + (Rect_Points[0] - Rect_Points[3]) / 6;
                    }

                    Min_Rect[i].points(Rect_Points);
                     for (int i = 0; i < 4; i++)
                     line(Orginal_Frame, Rect_Points[i], Rect_Points[(i + 1) % 4], Scalar(0, 32, 80), 2, 8);

                }
                imshow("Original_Image", Orginal_Frame);

                if (waitKey(30) == 60)
                {
                    cout << "ESC key is pressed" << endl;
                    break;
                }
            }
            return 0;
        }
