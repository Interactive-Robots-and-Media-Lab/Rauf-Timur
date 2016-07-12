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

//this one doesn't work on linux
//and I dont understand why!
//This class members (don't) allow to read and write
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
    void writeData(Mat* img)
    {
        iofile.write((char*)img->data,img->rows*img->cols*img->elemSize1());
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
    kinect2IO (libfreenect2::Freenect2Device* kinectDevice, string filename)
    {
        std::ofstream file;
        file.open(filename);
        if (!file.is_open())
        {
            std::cout << "Panika";
        }
        file.close();
        iofile.open(filename);
        if (!iofile.is_open())
        {
            std::cout << "Panika";
        }
        device = kinectDevice;
        zeroTime = clock();
        frame = 0;
    }

    ~kinect2IO()
    {
        iofile.close();
    }

    int framesRecord(int numOfFrames)
    {        

        int types = 0;
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;
//        if (!device->start())
//          return -1;
        device->setIrAndDepthFrameListener(&listener);
        if(!listener.waitForNewFrame(frames,10000))
        {
            return -1;
        }

        //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        Mat matDepth(depth->height,depth->width, CV_32FC1,depth->data);
        //Mat matRgb(rgb->height,rgb->width, CV_8UC4,rgb->data);

        writeInfoFrames(matDepth,numOfFrames);
        for (int i = 0; i<numOfFrames; i++)
        {
            writeTime();
            writeData(&matDepth);
            //writeData(&matRgb);
            frame++;
            std::cout << "Frame written" << std::endl;
            listener.release(frames);
            listener.waitForNewFrame(frames,10000);
        }
        listener.release(frames);
        device->stop();
        return 0;
    }
    
    int timeRecord(libfreenect2::SyncMultiFrameListener* listener, Mat* img, int seconds)
    {
        libfreenect2::FrameMap frames;
        device->setColorFrameListener(listener);
        device->setIrAndDepthFrameListener(listener);

        writeInfoSeconds(*img, seconds);
        for (int i = 0; i<CLOCKS_PER_SEC*seconds; i=clock())
        {
            writeTime();
            writeData(img);
            //writeMetaData();
            frame++;
            listener->release(frames);
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

    int close()
    {
        iofile.close();
        return 0;
    }
};

class kinect2writer
{
private:
    libfreenect2::Freenect2Device* device;
    std::ofstream file;
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
        offSet = 11*sizeof(int) + sizeof(float);
        stepSize = (rgbRows*rgbCols+depthRows*depthCols)*4+sizeof(float);
        listener.release(frames);
        device->stop();
    }
    ~kinect2writer()
    {
        if (file.is_open())
            file.close();
    }

    int framesRecord(string filename, int numOfFrames)
    {
        file.open(filename, std::ios_base::binary | std::ios_base::out | std::ios_base::app);

        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frameMap;
        device->start();
        device->setColorFrameListener(&listener);
        device->setIrAndDepthFrameListener(&listener);
        listener.waitForNewFrame(frameMap,10000);

        file.seekp(offSet);
        for (int i = 0; i<numOfFrames; i++)
        {
            libfreenect2::Frame *rgb = frameMap[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frameMap[libfreenect2::Frame::Depth];
            file << (int)(clock()-zeroTime);
            file.write((char*)depth->data,depthRows*depthCols*4);
            file.write((char*)rgb->data,rgbRows*rgbCols*4);
//            for (int h = 0; h<depthRows*depthCols*4; h++)
//                file << depth->data[h];
//            for (int h = 0; h<rgbRows*rgbCols*4; h++)
//                file << rgb->data[h];
            std::cout << "Frame#" << i+1 << "is written\n" << (clock()-zeroTime)/CLOCKS_PER_SEC << " seconds" << std::endl;
            listener.release(frameMap);
            listener.waitForNewFrame(frameMap,10000);
        }
        listener.release(frameMap);
        device->stop();
        file.seekp(0);
        status = 1;
        checkBytes = numOfFrames ^ (int)((clock()-zeroTime)/CLOCKS_PER_SEC) ^ stepSize ^ offSet ^ rgbRows
                     ^ rgbCols ^ depthRows ^ depthCols ^ status;
        file << numOfFrames << (int)((clock()-zeroTime)/CLOCKS_PER_SEC) << stepSize << offSet << rgbRows
             << rgbCols << depthRows << depthCols << status << checkBytes;
        file.close();
        return 0;
    }

    void close()
    {
        if (file.is_open())
            file.close();
    }
};
