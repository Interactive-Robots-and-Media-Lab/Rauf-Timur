#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <planeextractor.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
bool protonect_shutdown = false; ///< Whether the running application should shut down.
void sigint_handler(int s)
{
  protonect_shutdown = true;
}
bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;
void sigusr1_handler(int s)
{
  if (devtopause == 0)
    return;
/// [pause]
  if (protonect_paused)
    devtopause->start();
  else
    devtopause->stop();
  protonect_paused = !protonect_paused;
/// [pause]
}

int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;
  std::cerr << "To pause and unpause: pkill -USR1 Protonect" << std::endl;
  size_t executable_name_idx = program_path.rfind("Protonect");
  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
  std::string serial = "";
  bool enable_rgb = true;
  bool enable_depth = true;
  size_t framemax = -1;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  if (serial == "")
  {
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }
  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }
  devtopause = dev;
  signal(SIGINT,sigint_handler);
#ifdef SIGUSR1
  signal(SIGUSR1, sigusr1_handler);
#endif
  protonect_shutdown = false;
  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  if (enable_rgb && enable_depth)
  {
    if (!dev->start())
      return -1;
  }
  else
  {
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  size_t framecount = 0;

/// [loop start]
  PlaneExtractor PE(dev);
  while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
  {
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "timeout!" << std::endl;
      return -1;
    }
    //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    Mat matDepth(depth->height,depth->width, CV_32FC1,depth->data);
    Mat filteredDepth = PE.resizeMat(&matDepth);
    float* floatData = (float*)filteredDepth.data;
    vector<int> stat;
    for(int i = 0; i<300; i++)
    {
        stat.push_back(0);
    }
    for (int i = 0; i<filteredDepth.rows; i++)
    {
        for (int j = 0; j<filteredDepth.cols;j++)
        {
            std::cout <<*floatData << " ";
            stat[round((*floatData)/10)]++;
            floatData++;
        }
        std::cout << std::endl << std::endl;
    }
    for(int i = 0; i<300; i++)
    {
        std::cerr << stat[i] << " ";
    }

    vector<Plane> planesSquare = PE.extractFromSquare(&filteredDepth);
    //vector<Plane> planesClockwise = PE.extractFromClockwise(&filteredDepth);
    //imshow("paral prj",filteredDepth);
    imwrite("/home/tim/Desktop/calib/test.jpg",matDepth);
    waitKey(0);
    listener.release(frames);

  }
  dev->stop();
  dev->close();
  delete registration;
  return 0;
}
