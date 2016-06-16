#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

Mat createMultiImage(vector<Mat> images){
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


int main(){
    
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) return -1;
    
    Mat image, frame, image1,img_bw;
    char *outText;
    
    //Create api object
    tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
    
    // Initialize tesseract-ocr with English, without specifying tessdata path
    if (api->Init(NULL, "eng")) {
        fprintf(stderr, "Could not initialize tesseract.\n");
        exit(1);
    }
    
    int key;
    
    
    while(true){
    
        cap>>frame;
        cvtColor(frame, image, CV_RGB2GRAY);
        threshold(image, img_bw, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        
        vector<Mat> images;
        images.push_back(image);
        images.push_back(img_bw);
        images.push_back(image);
        images.push_back(img_bw);
        images.push_back(image);
        Mat doubleImage = createMultiImage(images);
        
        imshow("tesseract test",doubleImage);
        
        key = cvWaitKey(10);
        if(key==32) {
            api->SetImage( (uchar*) img_bw.data, img_bw.size().width, img_bw.size().height, img_bw.channels(), img_bw.step1());
            api->Recognize(0);
            
            outText = api->GetUTF8Text();
            printf("OCR output:\n%s", outText);
            
            // Destroy used object and release memory
            delete [] outText;
        }
        cvWaitKey(10);
        
    
    }
    
    cap.release();
    return 0;
}