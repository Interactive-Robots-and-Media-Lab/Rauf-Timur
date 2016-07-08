#include <iostream>
#include <cstdlib>
#include <fstream>
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
#include <time.h>

using namespace cv;

class kinect2IO
{
private:
    libfreenect2::Freenect2Device* device;
    std::fstream iofile;
    clock_t zeroTime;
    int frame;
    //8 integers at the beginning are:
    //number of frames
    //duration in seconds
    //step size between tuples in bytes
    //offset before the first tuple
    //number of rows of Mat
    //number of cols of Mat
    //status of writing
    // XOR checking byte (is the file in this format or not)
    int getStepSize(Mat *img)
    {
        return img->rows*img->cols*img->elemSize1();
    }
    int getOffset()
    {
        return sizeof(int)*8;
        // +some bytes is the size in bytes for Mat object sirealization
    }
    void writeInfoFrames(Mat img, int numOfFrames)
    {
        iofile << numOfFrames << (int)0 << getStepSize(&img) << getOffset() << img.rows << img.cols << (int)0 << (int)0;
    }
    void writeInfoSeconds(Mat img, int seconds)
    {
        iofile << (int)0 << seconds << getStepSize(&img) << getOffset() << img.rows << img.cols << (int)0 << (int)0;
    }
    void writeData(Mat img)
    {
        iofile.write((char*)img.data,img.rows*img.cols*img.elemSize1());
    }
    void writeTime()
    {
        iofile << (float)((clock()-zeroTime)/CLOCKS_PER_SEC);
    }
//    void writeMetaData()
//    {

//    }
    void writeResult()
    {
        iofile.seekp(0);
        iofile.seekg(0);
        int a[8];
        for (int i=0; i<8; i++)
            iofile >> a[i];
        if (a[0]==0)
        {
            a[0] = frame;
        }
        else
        {
            a[1] = (int)((clock()-zeroTime)/CLOCKS_PER_SEC);
        }
        a[6] = a[0]^a[1]^a[2]^a[3]^a[4]^a[5];
        a[7] = 1;
        iofile.write((char*)a,8*sizeof(int));
    }
    
public:
    kinect2IO (libfreenect2::Freenect2Device* kinectDevice, string filename, std::string arg = "")
    {
        if (arg.length()>0)
        {
            if(arg.length()==1 && arg[0]=='r')
            {
                iofile.open(filename, std::ios::in | std::ios::binary);
                
            }
            else if(arg.length()==1 && arg[0]=='w')
            {
                iofile.open(filename, std::ios::out | std::ios::binary);
            }
            else if(arg.length()==2 && ((arg[0]=='r' && arg[0]=='w') || (arg[0]=='w' && arg[0]=='r')))
            {
                iofile.open(filename, std::ios::out | std::ios::in | std::ios::binary);
            }
        }
        else
        {
            iofile.open(filename, std::ios::out | std::ios::in | std::ios::binary);
        }
        device = kinectDevice;
        zeroTime = clock();
        frame = 0;
    }

    ~kinect2IO()
    {
        iofile.close();
    }

    int framesRecord(Mat img, int numOfFrames)
    {
        int types = 0;
        types |= libfreenect2::Frame::Ir || libfreenect2::Frame::Depth;
        //types |= libfreenect2::Frame::Color;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
        //dev->setColorFrameListener(&listener);
        device->setIrAndDepthFrameListener(&listener);
        listener.release(frames);

        writeInfoFrames(img,numOfFrames);
        for (int i = 0; i<numOfFrames; i++)
        {
            writeTime();
            writeData(img);
            frame++;
            listener.release(frames);
        }
        return 0;
    }
    
    int timeRecord(Mat img, int seconds)
    {
        int types = 0;
        types |= libfreenect2::Frame::Ir || libfreenect2::Frame::Depth;
        //types |= libfreenect2::Frame::Color;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
        //dev->setColorFrameListener(&listener);
        device->setIrAndDepthFrameListener(&listener);
        listener.release(frames);

        writeInfoSeconds(img, seconds);
        for (int i = 0; i<CLOCKS_PER_SEC*seconds; i=clock())
        {
            writeTime();
            writeData(img);
            //writeMetaData();
            frame++;
            listener.release(frames);
        }
        return 0;
    }
    
    Mat readMatTemplate()
    {
        Mat res;
        if (!isCorrect())
        {
            std::cerr << "File has illegal format! Only recording is available." << std::endl;
            return res;
        }
        iofile.seekg(4*sizeof(int));
        // fifth integer from the beginning
        int rows,cols;
        iofile >> rows >> cols;
        res.create(rows,cols,CV_32FC1);
        return res;
    }

    int getFramesCount()
    {
        iofile.seekg(0);
        int a;
        iofile >> a;
        return a;
    }

    int getTimeInSec()
    {
        iofile.seekg(sizeof(int));
        int a;
        iofile >> a;
        return a;
    }

    int loadData(Mat container, int frameNumber)
    {
        iofile.seekg(getOffset()+frameNumber*getStepSize(&container));
        iofile.read((char*)container.data,container.rows*container.cols*4);
        return 0;
    }

    Mat readFirst()
    {
        Mat ans = readMatTemplate();
        loadData(ans,0);
        return ans;
    }

    int isCorrect()
    {
        int a[8];
        iofile.seekg(0);
        for (int i = 0; i<8; i++)
            iofile >> a[i];
        if ((a[7]==1) && ((a[0]^a[1]^a[2]^a[3]^a[4]^a[5])==a[6]))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
};
