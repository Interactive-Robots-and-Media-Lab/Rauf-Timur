/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tesseract/baseapi.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>
#include <kinect2io.hpp>
using namespace std;
using namespace cv;

/**
 * Helper function to detect text and letters on an image
 */
vector<Rect> detectLetters(Mat img)
{
    vector<cv::Rect> boundRect;
    Mat img_gray, img_sobel, img_threshold, element;
//    cvtColor(img, img_gray, CV_BGR2GRAY);
    Sobel(img, img_sobel, CV_8U, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    threshold(img_sobel, img_threshold, 0, 255, CV_THRESH_OTSU+CV_THRESH_BINARY);
    element = getStructuringElement(MORPH_RECT, Size(17, 3) );
    morphologyEx(img_threshold, img_threshold, CV_MOP_CLOSE, element); //Does the trick
    vector<vector<Point> > contours;
    findContours(img_threshold, contours, 0, 1);
    vector<vector<Point> > contours_poly( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
        if (contours[i].size()>100)
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            Rect appRect( boundingRect( cv::Mat(contours_poly[i]) ));
            if (appRect.width>appRect.height)
                boundRect.push_back(appRect);
        }
    return boundRect;
}

/**
 * Helper function to draw a region of interest box
 */
vector<Point> drawBox(Mat img, vector<Point> points) {
    int minX = points[0].x;
    int minY = points[0].y;
    int maxX = points[0].x;
    int maxY = points[0].y;
    int x, y;

    for (int k = 1; k < points.size(); k++) {
        x = points[k].x;
        y = points[k].y;
        if (x > maxX) {
            maxX = x;
        }
        if (y > maxY) {
            maxY = y;
        }
        if (x < minX) {
            minX = x;
        }
        if (y < minY) {
            minY = y;
        }
    }

    Point p1(minX, maxY);
    Point p2(maxX, minY);
    rectangle(img, p1, p2, Scalar(255, 0, 0), 2);
    vector<Point> coord;
    coord.push_back(p1);
    coord.push_back(p2);
    return coord;
}

/**
 * Helper function to display multiple images
 * in a one window
 */
Mat createMultiImage(vector<Mat> images) {
    if (images.size() == 0) {
        Mat nothing;
        return nothing;
    }
    long size = images.size();
    int cols = (int) ceil((double) sqrt(size));
    int rows = (int) ceil((double) size / cols);
    int imrows = images[0].rows;
    int imcols = images[0].cols;

    Mat doubleImage(imrows * rows, imcols * cols, images[0].type());
    int t = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            images[t].copyTo(doubleImage(Rect(j * imcols, i * imrows, imcols, imrows)));
            if (t == size - 1) {
                return doubleImage;
            }
            t++;
        }
    }

    return doubleImage;
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(Mat &im, const string label, vector<Point> &contour) {
    int fontface = FONT_HERSHEY_SIMPLEX;
    double scale = 0.25;
    int thickness = 1;
    int baseline = 0;

    Size text = getTextSize(label, fontface, scale, thickness, &baseline);
    Rect r = boundingRect(contour);

    Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
    putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

/**
 * Tesseract wrapper
 * @return
 */
void useTesseract(tesseract::TessBaseAPI *api,Mat newIm) {
    char *outText;
    Mat nIm;
    threshold(newIm, nIm, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    api->SetImage((uchar *) nIm.data, nIm.size().width, nIm.size().height, nIm.channels(), nIm.step1());
    api->Recognize(0);

    outText = api->GetUTF8Text();
    printf("OCR output:\n%s", outText);
    // Destroy used object and release memory
    delete[] outText;
}



int main() {

    bool knct = true;
    Mat bw, dst;
    vector<vector<Point> > contours;
    vector<Point> approx;
    vector<String> shapes = {"NOTHING","ONE","TWO","TRI","RECT","PENTA","HEXA","CIR"};

    //Start video streaming
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!capture.isOpened()) return -1;

        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = 0;
        libfreenect2::PacketPipeline *pipeline = 0;
        std::string serial = "";
        bool enable_rgb = true;
        bool enable_depth = false;

        if (freenect2.enumerateDevices() == 0) {
            std::cout << "no device connected!" << std::endl;
            knct = false;
        } else{
            capture.release();
        }

        if (serial == "") {
            serial = freenect2.getDefaultDeviceSerialNumber();
        }
        if (pipeline) {
            dev = freenect2.openDevice(serial, pipeline);
        }
        else {
            dev = freenect2.openDevice(serial);
        }
        if (dev == 0) {
            std::cout << "failure opening device!" << std::endl;
        }

        int types = 0;
        if (enable_rgb)
            types |= libfreenect2::Frame::Color;
        if (enable_depth)
            types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
        if(knct) {
            dev->setColorFrameListener(&listener);
            dev->setIrAndDepthFrameListener(&listener);
            if (enable_rgb && enable_depth) {
                if (!dev->start())
                    return -1;
            }
            else {
                if (!dev->startStreams(enable_rgb, enable_depth))
                    return -1;
            }
        }



    //************************Variables for ROI images**************************
    Size dsize(300, 300);
    bool flag = false;
    Point a(100,100);
    Point b(250,200);
    Rect subRecc(a,b);

    //**************************Variables for tesseract*************************
    char *outText;

    //Create tesseract api object
    tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();

//    api->Init(, "eng", tesseract::OEM_DEFAULT);
    api->SetVariable("tessedit_char_whitelist","0123456789");
    // Initialize tesseract-ocr with English, without specifying tessdata path
    if (api->Init(NULL, "eng",tesseract::OEM_DEFAULT)) {

        fprintf(stderr, "Could not initialize tesseract.\n");
        exit(1);
    }


    Size size1(1280,720);
    Size size2(200,100);
    int i = 50;
    kinect2reader reader("dataset1_90.bin");

    while (cvWaitKey(30) != 'q') {

        Mat src,ssrc;
        if(knct) {
            if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
            {
                std::cout << "timeout!" << std::endl;
                return -1;
            }

            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];


            Mat src1(rgb->height, rgb->width, CV_8UC4, rgb->data);
            resize(src1, src, size1);
            flip(src,src,1);
            listener.release(frames);
        }else {
            Mat src1;
            src1 = reader.readRgbFrame(i);
            resize(src1, src, size1);
            flip(src,src,1);
            i++;
//            capture >> src;
        }

        // Convert to grayscale
        cvtColor(src, dst, CV_BGR2GRAY);
        cvtColor(src, src, CV_BGR2GRAY);

        vector<Mat> images;
        vector<Mat> samples;

        dst.copyTo(ssrc);
        GaussianBlur(ssrc, ssrc, Size(9,9), 1.5, 1.5);
//        threshold(ssrc, ssrc, 128, 250, THRESH_OTSU);
        // Use Canny instead of threshold to catch squares with gradient shading
        blur(dst, bw, Size(3, 3));
        Canny(bw, bw, 50, 150, 3, true);
        images.push_back(bw);

        // Find contours
        findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


        for (int i = 0; i < contours.size(); i++) {
            // Approximate contour with accuracy proportional
            // to the contour perimeter
            Mat  subIm,subImm;
            Rect subRec,subRecc;
            vector<Point> recc;

            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.2, true);

            // Skip small or non-convex objects
            if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
                continue;

            if (approx.size() == 3) {
                setLabel(dst, shapes[3], contours[i]);
                flag = true;
            }
            else if (approx.size() >= 4 && approx.size() <= 6) {
                // Number of vertices of polygonal curve
                long vtc = approx.size();
                setLabel(dst, shapes[vtc], contours[i]);
//                flag = true;
            }
            else{
                // Detect and label circles
                double area = contourArea(contours[i]);
                Rect r = boundingRect(contours[i]);
                int radius = r.width / 2;

                if (abs(1 - ((double) r.width / r.height)) <= 0.2 &&
                    abs(1 - (area / (CV_PI * (radius * radius)))) <= 0.2)
                    setLabel(dst, shapes[7], contours[i]);
            }


            Mat newIm,newImm,subImm1;

            recc = drawBox(src, contours[i]);
            subRec = Rect(recc[0], recc[1]);
            subIm = ssrc(subRec);
            bitwise_not(ssrc,ssrc);
            resize(subIm, newIm, dsize);
            subRecc = Rect(40,115,210,70);
//            newIm.copyTo(newImm);
            newImm = newIm(subRecc);
            resize(newImm,subImm,size2);
            threshold(subImm,subImm1,128,255,CV_THRESH_BINARY | CV_THRESH_OTSU);
//            vector<Rect> letterBBoxes1=detectLetters(newIm);
//            for(int i=0; i< letterBBoxes1.size(); i++)
//                rectangle(newIm,letterBBoxes1[i],cv::Scalar(0,255,0),3,8,0);
            samples.push_back(subImm1);


            if (flag) {
                useTesseract(api,subImm1);
            }

        }

        images.push_back(dst);
        Mat smpls = createMultiImage(samples);
        Mat mult = createMultiImage(images);
        imshow("images", mult);
        if (!smpls.empty()) {
            imshow("samples", smpls);
        }

    }

    if(knct) {
        dev->stop();
        dev->close();
    }

    return 0;
}