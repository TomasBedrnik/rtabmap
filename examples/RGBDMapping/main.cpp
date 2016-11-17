/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/OdometryF2M.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QApplication>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "MapBuilder.h"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

using namespace rtabmap;
int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string dir = "";
    std::string rgb_path = "";
    std::string depth_path = "";
    std::string calibration = "";
    float freq = 0;
    bool binary = true;
    bool dir_paths = false;

    for(std::string arg : args)
    {
        if(arg.at(0) != '-' || arg == "help" || arg == "-help" || arg == "--help")
        {
            help = true;
            break;
        }
        arg.erase(arg.begin(),arg.begin()+1);
        if(arg.substr(0,3) == "dir")
        {
            dir = arg.substr(4,arg.length()-4);
            std::cout << "Main path:   " << dir << ";" << std::endl;
        }
        else if(arg.substr(0,1) == "f")
        {
            freq = std::stof(arg.substr(2,arg.length()-2));
            std::cout << "Frequency:   " << freq << "Hz;" << std::endl;
        }
        else if(arg.substr(0,3) == "rgb")
        {
            rgb_path = arg.substr(4,arg.length()-4);
            std::cout << "RGB Image Path:   " << rgb_path << ";" << std::endl;
        }
        else if(arg.substr(0,5) == "depth")
        {
            depth_path = arg.substr(6,arg.length()-6);
            std::cout << "Depth Image Path:   " << depth_path << ";" << std::endl;
        }
        else if(arg.substr(0,11) == "calibration")
        {
            calibration = arg.substr(12,arg.length()-12);
            std::cout << "Calibration Path:   " << calibration << ";" << std::endl;
        }
        else if(arg.substr(0,4) == "text")
        {
            binary = false;
        }
    }

    if(help)
    {
        std::cout << "RTAB-Map tester" << std::endl;
        std::cout << "======================" << std::endl;
        std::cout << "Available Parameters:" << std::endl;
        std::cout << "-dir          -dir=/tmp/grabber" << std::endl;
        std::cout << "     Base tmp directory where data is stored." << std::endl;
        std::cout << "     If not set /tmp/kinect2 is used." << std::endl;
        std::cout << "-f            -f=60" << std::endl;
        std::cout << "     Capture frequency in Hz." << std::endl;
        std::cout << "     If not set or set to 0, data is read as fast as possible." << std::endl;
        std::cout << "-rgb          -rgb=/tmp/rgb.png" << std::endl;
        std::cout << "     RGB image mapped to depth." << std::endl;
        std::cout << "     If not set dir/rgb/rgb.png is used." << std::endl;
        std::cout << "-depth        -depth=/tmp/depth.png" << std::endl;
        std::cout << "     Depth image or xml file." << std::endl;
        std::cout << "     If not set dir/depth/depth.png is used." << std::endl;
        std::cout << "-calibration    -calibration=/tmp/calib.yaml" << std::endl;
        std::cout << "     Calibration .yaml file." << std::endl;
        std::cout << "     If not set, dir /default.yaml is used." << std::endl;
        std::cout << "-text" << std::endl;
        std::cout << "     Store data in textfiles, default is binary form." << std::endl;
        std::cout << " --------------------------------------" << std::endl;
        std::cout << " -help" << std::endl;
        std::cout << "     Print this help." << std::endl;
        return 0;
    }

    if(dir == "")
        dir = "/tmp/kinect2";
    if(depth_path == "")
        depth_path = dir+"/depth/depth.png";
    if(rgb_path == "")
        rgb_path = dir+"/rgb/rgb.png";
    if(calibration == "")
        calibration = dir+"/default.yaml";

    boost::filesystem::path p1 = depth_path;
    boost::filesystem::path p2 = rgb_path;
    if(boost::filesystem::is_directory(p1) && boost::filesystem::is_directory(p2))
    {
        std::cout << "Sources are directories." << std::endl;
        dir_paths = true;
    }
    else if(boost::filesystem::is_regular_file(p1) && boost::filesystem::is_regular_file(p2))
    {
        std::cout << "Sources are images." << std::endl;
    }
    else
    {
        std::cout << "Not supported source files/dirs." << std::endl;
        return -1;
    }

    std::size_t found = calibration.find_last_of("/\\");
    std::string calibration_path = calibration.substr(0,found);
    std::string calibration_name = calibration.substr(found+1);
    if(calibration_name.substr(calibration_name.length()-4,4) != "yaml")
    {
        std::cout << "Calibration fine not found." << std::endl;
        return -1;
    }
    else
    {
        calibration_name = calibration_name.substr(0,calibration_name.length()-5);
    }


    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);


    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward
    Camera * camera = 0;
    Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);

//    if (!CameraFreenect2::available())
//    {
//        UERROR("Not built with Freenect2 support...");
//        exit(-1);
//    }
    //camera = new CameraFreenect2(0, CameraFreenect2::kTypeColor2DepthSD, 0, opticalRotation);
    //camera = new CameraRGBDGrabber("/home/beda/data/skola/_Oulu/kinectData/rgb/","/home/beda/data/skola/_Oulu/kinectData/depth/",1.0,0.0, opticalRotation);
    //camera = new CameraRGBDGrabber("/tmp/kinect2/rgb/rgb.png","/tmp/kinect2/depth/depth.png",1.0,0.0, opticalRotation);
    //camera = new CameraRGBDGrabber("/tmp/kinect2/rgb/rgb.png","/tmp/kinect2/depth_mat/depth_mat.xml",1.0,0.0, opticalRotation);
    if(dir_paths)
        camera = new CameraRGBDImages(rgb_path,depth_path,1.0,freq, opticalRotation);
    else
        camera = new CameraRGBDGrabber(rgb_path,depth_path,1.0,freq, opticalRotation);

    if(!camera->init(calibration_path,calibration_name))
    {
        UERROR("Camera init failed!");
    }

    CameraThread cameraThread(camera);


    // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    // We give it the camera so the GUI can pause/resume the camera
    QApplication app(argc, argv);
    MapBuilder mapBuilder(&cameraThread,dir,binary);

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometryThread odomThread(new OdometryF2M());


    // Create RTAB-Map to process OdometryEvent
    Rtabmap * rtabmap = new Rtabmap();
    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    mapBuilder.registerToEventsManager();

    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();

    mapBuilder.show();
    app.exec(); // main loop

    // remove handlers
    mapBuilder.unregisterFromEventsManager();
    rtabmapThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread.kill();
    odomThread.join(true);
    rtabmapThread.join(true);

    // Save 3D map

//    boost::posix_time::ptime tick = boost::posix_time::microsec_clock::local_time();
//    printf("Saving rtabmap_cloud.pcd...\n");
//    std::map<int, Signature> nodes;
//    std::map<int, Transform> optimizedPoses;
//    std::multimap<int, Link> links;
//    rtabmap->get3DMap(nodes, optimizedPoses, links, true, true);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
//    {
//        Signature node = nodes.find(iter->first)->second;

//        // uncompress data
//        node.sensorData().uncompressData();

//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
//                    node.sensorData(),
//                    4,           // image decimation before creating the clouds
//                    4.0f,        // maximum depth of the cloud
//                    0.01f);  // Voxel grid filtering
//        *cloud += *util3d::transformPointCloud(tmp, iter->second); // transform the point cloud to its pose
//    }
//    if(cloud->size())
//    {
//        printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
//        cloud = util3d::voxelize(cloud, 0.01f);

//        printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
//        boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
//        double diff = (now-tick).total_milliseconds()/1000.0;
//        //printf("Elapsed time = %d\n", diff);
//        std::cout << "Elapsed time = " << diff << std::endl;
//        pcl::io::savePCDFile("/tmp/rtabmap_cloud.pcd", *cloud);
//        //pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
//    }
//    else
//    {
//        printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
//    }

//    // Save trajectory
//    printf("Saving rtabmap_trajectory.txt ...\n");
//    if(optimizedPoses.size() && graph::exportPoses("rtabmap_trajectory.txt", 0, optimizedPoses, links))
//    {
//        printf("Saving rtabmap_trajectory.txt... done!\n");
//    }
//    else
//    {
//        printf("Saving rtabmap_trajectory.txt... failed!\n");
//    }

    return 0;
}
