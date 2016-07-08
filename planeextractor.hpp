#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

using namespace cv;

class Plane
{
private:
    double gradX;
    double gradY;
    double dist;
public:
    Plane(double gX = 0, double gY = 0, double d = 0)
    {
        gradX = gX;
        gradY = gY;
        dist = d;
    }
};

class PlaneExtractor
{
private:
    libfreenect2::Freenect2Device* device;
public:
    PlaneExtractor(libfreenect2::Freenect2Device* dev)
    {
        device = dev;
    }

    //    void convert(Mat* img, int* points)
//    {
//        libfreenect2::Freenect2Device::ColorCameraParams ccp = this->device->getColorCameraParams();
//        //TODO converting from depth image to point cloud if needed
//        return;
//    }

    double* getDepthTranslationMatrix(Mat depth, double fieldWidthDegree = 70.6, double fieldHeighDegree = 60.0)
    {
        double pi = 3.141592653589793238463;
        double widthField = pi*fieldWidthDegree/180;
        double heighField = pi*fieldHeighDegree/180;
        double widthStep=(widthField/depth.cols);
        double heighStep=(heighField/depth.rows);
        double* data = new double[depth.cols*depth.rows];
        double i,j;
        int i2=0;
        int j2;
        for (i=-heighField/2; i<heighField;i+=heighStep)
        {
            j2=0;
            for (j=-widthField/2;j<widthField;j+=widthStep)
            {
                data[i2*depth.cols+j2] = cos(i)*cos(j);
                j2++;
            }
            i2++;
        }
        return data;
    }

    Mat resizeMat (Mat* src)
    {
        int factor = 4;
        Size newSize(round(src->cols/factor), round(src->rows/factor));
        Mat ans(newSize,CV_32FC1);
        float* dataPtr = (float*) src->data;
        float* dataPtrAns = (float*) ans.data;
        vector<float> data;
        data.resize(16);
        for(int i = 0; i<ans.rows;i++)
            for(int j = 0; j<ans.cols; j++)
            {
                for (int m = 0;m<factor;m++)
                    for (int n=0;n<factor;n++)
                    {
                        if(dataPtr[(m + factor*i)*(src->cols) + n + factor*j] !=0)
                        {
                            data.push_back(dataPtr[(m + factor*i)*src->cols + n + factor*j]);
                        }
                    }
                dataPtrAns[i*ans.cols + j] = median (data);
                data.erase(data.begin(),data.end());
            }
        return ans;
    }

    //shoud be optimized
    vector<Plane> extractFromSquare(Mat* img)
    {
        int sqrSite = 5;
        Mat classification(img->rows-sqrSite,img->cols-sqrSite,CV_8UC1);
        uchar* Ptr = classification.data;
        float* dataPtr = (float*)img->data;
        double grY, grX, d;
        float metaD[4];
        vector<Plane> planes;
        for (int i=0; i<img->rows-sqrSite; i++)
            for (int j=0; j<img->cols-sqrSite; j++)
            {
                metaD[0]= dataPtr[i*img->cols + j];
                metaD[1]= dataPtr[i*img->cols + j + sqrSite];
                metaD[2]= dataPtr[(i+sqrSite)*img->cols + j];
                metaD[3]= dataPtr[(i+sqrSite)*img->cols + j + sqrSite];
                if(((metaD[0]-metaD[1]-metaD[2]+metaD[3])/(metaD[1]+0.01)<0.002) &&
                        (metaD[0] != 0) && (metaD[1] != 0) && (metaD[2] != 0) && (metaD[3] != 0))
                {
                    grX = (metaD[0]-metaD[1]+metaD[2]-metaD[3]);
                    grY = (metaD[0]+metaD[1]-metaD[2]-metaD[3]);
                    d = metaD[0]+ grX/sqrSite*(img->cols/2-j) + grY/sqrSite*(img->rows/2-i);
                    Plane* cellPlane = new Plane(grX/sqrSite,grY/sqrSite,d);
                    if (planes.size()<1000)
                    {
                        planes.push_back(*cellPlane);
                    }
                    Ptr[i*classification.cols+j] = 255;
                }
                else
                {
                    Ptr[i*classification.cols+j] = 0;
                }
            }
        imshow("qqreq",classification);
        waitKey();
        //print(classification);
        return planes;
    }

    //shoud be optimized
    vector<Plane> extractFromClockwise(Mat* img)
    {
        int sqrSite = 5;// dont change it!
        Mat classification(img->rows-sqrSite,img->cols-sqrSite,CV_8UC1);
        uchar* Ptr = classification.data;
        float* dataPtr = (float*)img->data;
        double grY, grX, d;
        float metaD[12];
        vector<Plane> planes;
        for (int i=0; i<img->rows-sqrSite; i++)
            for (int j=0; j<img->cols-sqrSite; j++)
            {
                metaD[0]= dataPtr[(i+0)*img->cols + j + 2];
                metaD[1]= dataPtr[(i+0)*img->cols + j + 3];
                metaD[2]= dataPtr[(i+1)*img->cols + j + 4];
                metaD[3]= dataPtr[(i+2)*img->cols + j + 4];
                metaD[4]= dataPtr[(i+3)*img->cols + j + 4];
                metaD[5]= dataPtr[(i+4)*img->cols + j + 3];
                metaD[6]= dataPtr[(i+4)*img->cols + j + 2];
                metaD[7]= dataPtr[(i+4)*img->cols + j + 1];
                metaD[8]= dataPtr[(i+3)*img->cols + j + 0];
                metaD[9]= dataPtr[(i+2)*img->cols + j + 0];
                metaD[10]= dataPtr[(i+1)*img->cols + j + 0];
                metaD[11]= dataPtr[(i+0)*img->cols + j + 1];

                if(true)
                {

                    Plane* cellPlane = new Plane(grX/sqrSite,grY/sqrSite,d);
                    if (planes.size()<1000)
                    {
                        planes.push_back(*cellPlane);
                    }
                    Ptr[i*classification.cols+j] = 255;
                }
                else
                {
                    Ptr[i*classification.cols+j] = 0;
                }
            }
        imshow("qqreq",classification);
        waitKey();
        //print(classification);
        return planes;
    }

private:
    void print(Mat img)
    {
        std::cout << std::endl << std::endl;
        uchar* ptr = img.data;
        for (int i = 0; i<img.rows;i++)
        {
            for (int j=0;j<img.cols;j++)
            {
                std::cout << (int)*ptr;
                ptr++;
            }
        }
    }

    void mult(Mat* res, double* a, Mat* b)
    {
        uchar* bPointer = b->datastart;
        for(uchar* resPointer =res->datastart;resPointer<res->dataend;resPointer+=4)
        {
            *resPointer = uchar( (*a)*(*bPointer) );
            a++; bPointer+=4;
        }
    }

    float median(vector<float> data)
    {
        int size = data.size();
        if(size==0)
        {
            return 0;
        }
        else
        {
            float acc = 0;
            int k=0;
            for (uint i = 0; i<data.size(); i++)
            {
                if (data[i]>0)
                {
                    acc+=data[i];
                    k++;
                }
            }
            return acc/k;
        }
    }

    void calibrate (std::vector<Mat> data)
    {
        double* transMatrices = this->getDepthTranslationMatrix(data[0]);
        Mat currentImage;
        data[0].copyTo(currentImage);
        for (uint i=0; i<data.size();i++)
        {
            mult(&currentImage,transMatrices,&data[i]);
            // TODO calibration from seleval frames
        }
    }


};
