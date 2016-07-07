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
        int k = 0;
        for(int i = 0; i<ans.rows;i++)
            for(int j = 0; j<ans.cols; j++)
            {
                k=0;
                for (int m = 0;m<factor;m++)
                    for (int n=0;n<factor;n++)
                    {
                        if(dataPtr[(m + factor*i)*(src->cols) + n + factor*j] !=0)
                        {
                            data[k] = dataPtr[(m + factor*i)*src->cols + n + factor*j];
                            k++;
                        }
                    }
                dataPtrAns[i*ans.cols + j] = median (data);
                data.erase(data.begin(),data.end());
            }
        return ans;
    }
    vector<Plane> extract(Mat* data)
    {
        Mat classification(data->rows,data->cols,CV_8UC1);
        float* Ptr = (float*)classification.data;
        float* dataPtr = (float*)data->data;
        double grY, grX, d;
        float metaD[4];
        vector<Plane> planes;
        for (int i=0; i<data->rows-1; i++)
            for (int j=0; j<data->cols-1; j++)
            {
                metaD[0]= dataPtr[i*data->cols + j];
                metaD[1]= dataPtr[i*data->cols + j + 1];
                metaD[2]= dataPtr[(i+1)*data->cols + j];
                metaD[3]= dataPtr[(i+1)*data->cols + j + 1];
                if((metaD[0]-metaD[1]-metaD[2]+metaD[3])/metaD[1]<0.01)
                {
                    grX = (metaD[0]-metaD[1]+metaD[2]-metaD[3]);
                    grY = (metaD[0]+metaD[1]-metaD[2]-metaD[3]);
                    d = metaD[0]+ grX*(data->cols/2-j) + grY*(data->rows/2-i);
                    Plane* cellPlane = new Plane(grX,grY,d);
                    planes.push_back(*cellPlane);
                    Ptr[i*data->cols+j] = 1;
                }
                else
                {
                    Ptr[i*data->cols+j] = 0;
                }
            }
        return planes;
    }

private:
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
        std::sort_heap(data.begin(),data.end());
        return data[data.size()/2];
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
