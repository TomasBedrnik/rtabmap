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

using namespace rtabmap;
int main(int argc, char **argv)
{
    std::vector<std::string> args(argv, argv + argc);
    args.erase(args.begin());
    bool help = false;
    std::string dir = "";
    std::string rgb_image = "";
    std::string depth_image = "";
    std::string calibration = "";
    float freq = 0;

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
        else if(arg.substr(0,9) == "rgb_image")
        {
            rgb_image = arg.substr(10,arg.length()-10);
            std::cout << "RGB Image Path:   " << rgb_image << ";" << std::endl;
        }
        else if(arg.substr(0,11) == "depth_image")
        {
            depth_image = arg.substr(12,arg.length()-12);
            std::cout << "Depth Image Path:   " << depth_image << ";" << std::endl;
        }
        else if(arg.substr(0,11) == "calibration")
        {
            calibration = arg.substr(12,arg.length()-12);
            std::cout << "Calibration Path:   " << calibration << ";" << std::endl;
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
        std::cout << "-rgb_image    -rgb_image=/tmp/rgb.png" << std::endl;
        std::cout << "     RGB image mapped to depth." << std::endl;
        std::cout << "     If not set dir/rgb/rgb.png is used." << std::endl;
        std::cout << "-depth_image    -depth_image=/tmp/depth.png" << std::endl;
        std::cout << "     Depth image or xml file." << std::endl;
        std::cout << "     If not set dir/depth/depth.png is used." << std::endl;
        std::cout << "-calibration    -calibration=/tmp/calib.yaml" << std::endl;
        std::cout << "     Calibration .yaml file." << std::endl;
        std::cout << "     If not set, default is created in dir and used." << std::endl;
        std::cout << " --------------------------------------" << std::endl;
        std::cout << " -help" << std::endl;
        std::cout << "     Print this help." << std::endl;
        return 0;
    }

    if(dir == "")
        dir = "/tmp/kinect2";
    if(depth_image == "")
        depth_image = dir+"/depth/depth.png";
    if(rgb_image == "")
        rgb_image = dir+"/rgb/rgb.png";

    std::string calibration_path = "";
    std::string calibration_name = "";
    if(calibration == "")
    {
        calibration_path = dir;
        calibration_name = "default";
        std::ofstream calibration_file;
        calibration_file.open (calibration_path+"/"+calibration_name+".yaml");

        calibration_file << "%YAML:1.0" << std::endl;
        calibration_file << "camera_name: calib2" << std::endl;
        calibration_file << "image_width: 512" << std::endl;
        calibration_file << "image_height: 424" << std::endl;
        calibration_file << "camera_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 3" << std::endl;
        calibration_file << "   data: [ 3.6110206777979778e+02, 0., 2.6372018528793006e+02, 0.," << std::endl;
        calibration_file << "       3.6097502114915272e+02, 1.7767928198595087e+02, 0., 0., 1. ]" << std::endl;
        calibration_file << "distortion_coefficients:" << std::endl;
        calibration_file << "   rows: 1" << std::endl;
        calibration_file << "   cols: 5" << std::endl;
        calibration_file << "   data: [ 1.1618214944736524e-01, -3.7391857743275664e-01," << std::endl;
        calibration_file << "       -2.3108157640784072e-02, 4.0215076909925294e-03," << std::endl;
        calibration_file << "       3.5294410947770366e-01 ]" << std::endl;
        calibration_file << "distortion_model: plumb_bob" << std::endl;
        calibration_file << "rectification_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 3" << std::endl;
        calibration_file << "   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]" << std::endl;
        calibration_file << "projection_matrix:" << std::endl;
        calibration_file << "   rows: 3" << std::endl;
        calibration_file << "   cols: 4" << std::endl;
        calibration_file << "   data: [ 3.6110206777979778e+02, 0., 2.6372018528793006e+02," << std::endl;
        calibration_file << "       4.0215076909925294e-03, 0., 3.6097502114915272e+02," << std::endl;
        calibration_file << "       1.7767928198595087e+02, 0., 0., 0., 1., 1. ]" << std::endl;

        calibration_file.close();
    }
    else
    {
          std::size_t found = calibration.find_last_of("/\\");
          calibration_path = calibration.substr(0,found);
          calibration_name = calibration.substr(found+1);
          if(calibration_name.substr(calibration_name.length()-4,4) != "yaml")
          {
              std::cout << "Calibration fine not found." << std::endl;
              return -1;
          }
          else
          {
              calibration_name = calibration_name.substr(0,calibration_name.length()-5);
          }
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
    camera = new CameraRGBDGrabber(rgb_image,depth_image,1.0,freq, opticalRotation);

    if(!camera->init(calibration_path,calibration_name))
    {
        UERROR("Camera init failed!");
    }

    CameraThread cameraThread(camera);


    // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    // We give it the camera so the GUI can pause/resume the camera
    QApplication app(argc, argv);
    MapBuilder mapBuilder(&cameraThread,dir);

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
