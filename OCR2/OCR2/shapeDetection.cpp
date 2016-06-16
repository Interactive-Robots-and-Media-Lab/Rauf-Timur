/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <cmath>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * Helper function to draw a region of interest box
 */
vector<Point> drawBox(Mat img, vector<Point> points){
    int minX = points[0].x;
    int minY = points[0].y;
    int maxX = points[0].x;
    int maxY = points[0].y;
    int x,y;
    
    for(int k = 1; k<points.size(); k++){
        x = points[k].x;
        y = points[k].y;
        if(x > maxX){
            maxX = x;
        }
        if(y > maxY){
            maxY = y;
        }
        if(x < minX){
            minX = x;
        }
        if(y < minY){
            minY = y;
        }
    }
    
    Point p1(minX,maxY);
    Point p2(maxX,minY);
    rectangle(img, p1, p2,Scalar(255,0,0),2);
    vector<Point> coord;
    coord.push_back(p1);
    coord.push_back(p2);
    return coord;
}


/**
 * Helper function to display multiple images
 * in a one window
 */

Mat createMultiImage(vector<Mat> images){
    if(images.size()==0){
        Mat nothing;
        return nothing;
    }
    long size = images.size();
    int cols = ceil((double)sqrt(size));
    int rows = ceil((double)size/cols);
    int imrows = images[0].rows;
    int imcols = images[0].cols;
    
    Mat doubleImage(imrows*rows, imcols*cols, images[0].type());
    int t = 0;
    for(int i = 0; i <rows; i++){
        for(int j = 0; j <cols; j++){
            images[t].copyTo(doubleImage(Rect(j*imcols,i*imrows, imcols, imrows)));
            if(t==size-1){
                return doubleImage;
            }
            t++;
        }
    }
    
    return doubleImage;
}


/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(Point pt1, Point pt2, Point pt0){
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(Mat& im, const string label, vector<Point>& contour){
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.25;
    int thickness = 1;
    int baseline = 0;
    
    Size text = getTextSize(label, fontface, scale, thickness, &baseline);
    Rect r = boundingRect(contour);
    
    Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}



int main(){

    Mat src, gray, bw, dst;
    vector<vector<Point> > contours;
    vector<Point> approx;
    
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!capture.isOpened()) return -1;

    //************************Variables for ROI images**************************
    vector<Point> recc;
    Mat newIm, subIm, subImBW;
    Rect subRec;
    Size dsize(200,200);
    bool flag = false;
    
    //**************************Variables for tesseract*************************
    char *outText;
    
    //Create tesseract api object
    tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
    // Initialize tesseract-ocr with English, without specifying tessdata path
    if (api->Init(NULL, "eng")) {
        fprintf(stderr, "Could not initialize tesseract.\n");
        exit(1);
    }
    
    while(cvWaitKey(30) != 'q'){
        
        capture >> src;
        if(true){
            // Convert to grayscale
            cvtColor(src, gray, CV_BGR2GRAY);
            
            vector<Mat> images;
            vector<Mat> samples;
            
            // Use Canny instead of threshold to catch squares with gradient shading
            blur( gray, bw, Size(3,3) );
            Canny(gray, bw, 80, 240, 3);
            images.push_back(bw);
            
            
            // Find contours
            findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            
            src.copyTo(dst);
            
            for (int i = 0; i < contours.size(); i++){
                // Approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                
                // Skip small or non-convex objects
                if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
                    continue;
                
                if (approx.size() == 3){
//                    setLabel(dst, "TRI", contours[i]);    // Triangles
                    flag = true;
                }
                else if (approx.size() >= 4 && approx.size() <= 6){
                    // Number of vertices of polygonal curve
                    long vtc = approx.size();
                    
                    // Use the degrees obtained above and the number of vertices
                    // to determine the shape of the contour
                    if (vtc == 4 ){
                        setLabel(dst, "RECT", contours[i]);
                        flag = true;
                    }
                    else if (vtc == 5 )
                        setLabel(dst, "PENTA", contours[i]);
                    else if (vtc == 6 )
                        setLabel(dst, "HEXA", contours[i]);
                }
                else{
                    // Detect and label circles
                    double area = contourArea(contours[i]);
                    Rect r = boundingRect(contours[i]);
                    int radius = r.width / 2;
                    
                    if (abs(1 - ((double)r.width / r.height)) <= 0.2 &&
                        abs(1 - (area / (CV_PI * (radius*radius)))) <= 0.2)
                        setLabel(dst, "CIR", contours[i]);
                }
                
                vector<Point> points = contours[i];
                recc = drawBox(dst, points);
                subRec = Rect(recc[0],recc[1]);
                subIm = dst(subRec);
                subImBW = bw(subRec);
                resize(subIm, newIm , dsize);
                resize(subImBW, subImBW , dsize);
                cvtColor(newIm, newIm, CV_RGB2GRAY);
                samples.push_back(newIm);
                if(flag){
                    threshold(newIm, newIm, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
                    samples.push_back(subImBW);
                    api->SetImage( (uchar*) subImBW.data, subImBW.size().width, subImBW.size().height, subImBW.channels(), subImBW.step1());
                    api->Recognize(0);
                    
                    outText = api->GetUTF8Text();
                    printf("OCR output:\n%s", outText);
                    
                    // Destroy used object and release memory
                    delete [] outText;
                    flag = false;

                }
            }
            
            cvtColor(dst, dst, CV_RGB2GRAY);
            Mat smpls = createMultiImage(samples);
            images.push_back(dst);
            Mat mult = createMultiImage(images);
            imshow("images", mult);
            if(!smpls.empty()){
                imshow("samples", smpls);
            }
            
        }
        else{
            break;
        }
    }
    
    return 0;
}