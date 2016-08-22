#include <stdio.h>
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
using namespace std;

class kinect2reader
{
private:
    std::ifstream file;
    std::ifstream paramFile;
    int frames;
    int clocks;
    int seconds;
    int stepSize;
    int offSet;
    int rgbRows;
    int rgbCols;
    int depthRows;
    int depthCols;
    int status;
public:
    kinect2reader(string path)
    {
        file.open(path, std::ios_base::binary | std::ios_base::in);
        paramFile.open("config.txt",std::ios_base::binary | std::ios_base::in);
        int checkBytes;
        paramFile >> frames;
        paramFile >> clocks;
        paramFile >> seconds;
        paramFile >> stepSize;
        paramFile >> offSet;
        paramFile >> rgbRows;
        paramFile >> rgbCols;
        paramFile >> depthRows;
        paramFile >> depthCols;
        paramFile >> status;
        paramFile >> checkBytes;

        if ((frames^clocks^seconds^stepSize^offSet^rgbRows^rgbCols^depthRows^depthCols^status) != checkBytes)
        {
            file.close();
            std::cerr << "File has not a kinect2data format.";
        }
        else
        {
            std::cout << "File opened.";
        }
        if (status != 1)
        {
            std::cout << "Warning! Writing to the file is not complete or file has another format.";
        }
        paramFile.close();
    }

    ~kinect2reader()
    {
        file.close();
    }

    Mat readDepthFrame(int a)
    {
        if (a>=frames)
        {
            std::cerr << "frame #" << a << " doesn't exist!" << std::endl;
            Mat b;
            return b;
        }
        else
        {
            char* data = new char[depthRows*depthCols*4];
            file.seekg(a*stepSize+sizeof(int));
            file.read(data,depthRows*depthCols*4);
            Mat *depth = new Mat(depthRows,depthCols,CV_32FC1,(uchar*)data);
            return *depth;
        }
    }
    int loadDepthFrame(Mat* img, int a)
    {
        if (a>=frames)
        {
            std::cerr << "frame #" << a << " doesn't exist!" << std::endl;
            return -1;
        }
        char* data = (char*)img->data;
        file.seekg(offSet+a*stepSize+sizeof(int));
        file.read(data,depthRows*depthCols*4);
        return 0;
    }

    /**
     * @brief readRgbFrame reads the frame from the file
     * @param a number of Frame
     * @return Frame in cv::Mat format
     */
    Mat readRgbFrame(int a)
    {
        if (a>=frames)
        {
            std::cerr << "frame #" << a << " doesn't exist!" << std::endl;
            Mat b;
            return b;
        }
        char* data = new char[rgbRows*rgbCols*4];
        file.seekg(offSet+a*stepSize+depthRows*depthCols*4+sizeof(int));
        file.read(data,rgbRows*rgbCols*4);
        Mat *rgb = new Mat(rgbRows,rgbCols,CV_8UC4,(uchar*)data);
        return *rgb;
    }
    /**
     * @brief loadRgbFrame updates the data in the frame
     * @param img cv::Mat container
     * @param a number of the frame
     * @return
     */
    int loadRgbFrame(Mat* img, int a)
    {
        if (a>=frames)
        {
            std::cerr << "frame #" << a << " doesn't exist!" << std::endl;
            return -1;
        }
        char* data = (char*)img->data;
        file.seekg(offSet+a*stepSize+depthRows*depthCols*4+sizeof(int));
        file.read(data,rgbRows*rgbCols*4);
        return 0;
    }

};

class kinect2writer
{
private:
    libfreenect2::Freenect2Device* device;
    std::ofstream file;
    std::ofstream paramFile;
    int frames = 0;
    int seconds = 0;
    int stepSize;
    int offSet;
    int rgbRows;
    int rgbCols;
    int depthRows;
    int depthCols;
    int status = 0;
    int checkBytes = 0;
    int zeroTime = clock();
public:
    kinect2writer (libfreenect2::Freenect2Device* kinectDevice)
    {
        device = kinectDevice;
        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
        device->start();
        device->setColorFrameListener(&listener);
        device->setIrAndDepthFrameListener(&listener);
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        rgbRows = rgb->height;
        rgbCols = rgb->width;
        depthRows = depth->height;
        depthCols = depth->width;
        offSet = 0;
        stepSize = (rgbRows*rgbCols+depthRows*depthCols)*4+sizeof(int);
        listener.release(frames);
        device->stop();
    }

    ~kinect2writer()
    {
        if (file.is_open())
            file.close();
    }

    int framesRecord(char* filename, int numOfFrames)
    {
        if(std::remove(filename))
        {
            std::cout << "Rewriting a file..." << std::endl;
        }
        else
        {
            std::cout << "Creating a new file..." << std::endl;
        }
        file.open(filename, std::ios_base::binary | std::ios_base::out);

        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frameMap;
        device->start();
        device->setColorFrameListener(&listener);
        device->setIrAndDepthFrameListener(&listener);
        listener.waitForNewFrame(frameMap,10000);

        for (int i = 0; i<numOfFrames; i++)
        {
            libfreenect2::Frame *rgb = frameMap[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frameMap[libfreenect2::Frame::Depth];
            int curTime = clock()-zeroTime;
            file.write((char*)&curTime,sizeof(int));
            file.write((char*)depth->data,depthRows*depthCols*4);
            file.write((char*)rgb->data,rgbRows*rgbCols*4);
            std::cout << "Frame#" << i+1 << " is written\n" << (clock()-zeroTime)/CLOCKS_PER_SEC << " seconds" << std::endl;
            listener.release(frameMap);
            listener.waitForNewFrame(frameMap,10000);
        }
        listener.release(frameMap);
        device->stop();

        status = 1;
        int clocks = clock()-zeroTime;
        int secs = clocks/CLOCKS_PER_SEC;
        checkBytes = numOfFrames ^ clocks ^ secs ^ stepSize ^ rgbRows ^ rgbCols ^ depthRows ^ depthCols ^ status;

        paramFile.open("config.txt", std::ios_base::out | std::ios_base::binary);
        paramFile << numOfFrames << std::endl;
        paramFile << clocks << std::endl;
        paramFile << secs << std::endl;
        paramFile << stepSize << std::endl;
        paramFile << 0 << std::endl;
        paramFile << rgbRows << std::endl;
        paramFile << rgbCols << std::endl;
        paramFile << depthRows << std::endl;
        paramFile << depthCols << std::endl;
        paramFile << status << std::endl;
        paramFile << checkBytes << std::endl;
        paramFile << 0 << std::endl;
        paramFile.close();

        file.close();
        return 0;
    }

    void close()
    {
        if (file.is_open())
            file.close();
    }
};
