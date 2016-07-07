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

using namespace cv;

class kinect2IO
{
private:
    libfreenect2::Freenect2Device* device;
    std::fstream iofile;
    void writeMat(Mat img)
    {

    }
    void writeInfoFrames(Mat img, int numOfFrames)
    {
        iofile << numOfFrames << (int)0;
        writeMat(img);
    }
    void writeInfoSeconds(Mat img, int seconds)
    {
        iofile << (int)0 << seconds;
        writeMat(img);
    }
    void writeData(Mat img)
    {

    }
    void writeTime()
    {

    }
    void writeMetaData()
    {

    }
    
public:
    kinect2IO (libfreenect2::Freenect2Device* dev, std::fstream filename, std::string arg = "")
    {
        if (arg.length()>0)
        {
            if(arg.length()==1 && arg[0]=='r')
            {
                file.open(filename, ios::in | ios::binary);
                
            }
            else if(arg.length()==1 && arg[0]=='w')
            {
                file.open(filename, ios::out | ios::binary);
            }
            else if(arg.length()==2 && ((arg[0]=='r' && arg[0]=='w') || (arg[0]=='w' && arg[0]=='r')))
            {
                file.open(filename, ios::out | ios::in | ios::binary);
            }
        }
        else
        {
            file.open(filename, ios::out | ios::in | ios::binary);
        }
        device = dev;
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
        dev->setIrAndDepthFrameListener(&listener);
        listener.release(frames);

        writeInfo(img, seconds);
        for (int i = 0; i<numOfFrames; i++)
        {
            writeTime();
            writeData(img);
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
        dev->setIrAndDepthFrameListener(&listener);
        listener.release(frames);

        writeInfo(img, seconds);
        for (int i = 0; i<numOfFrames; i++)
        {
            writeTime();
            writeData(img);
            //writeMetaData();
            listener.release(frames);
        }
        return 0;
    }
    
    Mat readMatTemplate()
    {

    }

    int getFramesCount()
    {

    }

    int getTimeInSec()
    {

    }

    int loadData(Mat container, int frameNumber)
    {

    }

    Mat readFirst()
    {
        Mat ans = readMatTemplate();
        loadData(ans,0);
        return ans;
    }
};
