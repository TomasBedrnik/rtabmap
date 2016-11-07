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

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/utilite/UThread.h>
#include "MapBuilder.h"
#include <pcl/visualization/cloud_viewer.h>
#include <rtabmap/core/OdometryF2M.h>
#include <QApplication>
#include <stdio.h>

using namespace rtabmap;

int main(int argc, char **argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kError);

    int cameraRate = 10000;
    int odomUpdate = 10;
    int mapUpdate = 10;
    std::string dir = "/tmp/kinect2";
    std::string depth_image = dir+"/depth/depth.png";
    std::string rgb_image = dir+"/rgb/rgb.png";

	printf("Camera rate = %d Hz\n", cameraRate);
	printf("Odometry update rate = %d Hz\n", cameraRate/odomUpdate);
	printf("Map update rate = %d Hz\n", (cameraRate/odomUpdate)/mapUpdate);

    std::string calibration_path = dir;
    std::string calibration_name = "default";
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


	Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
    CameraRGBDGrabber camera(rgb_image,depth_image,1.0,cameraRate, opticalRotation);

    if(camera.init(calibration_path,calibration_name))
	{
		OdometryF2M odom;
		Rtabmap rtabmap;
		rtabmap.init();

        QApplication app(argc, argv);
		MapBuilder mapBuilder;
		mapBuilder.show();
		QApplication::processEvents();

		SensorData data = camera.takeImage();
		int cameraIteration = 0;
		int odometryIteration = 0;
		printf("Press \"Space\" in the window to pause\n");
		while(data.isValid() && mapBuilder.isVisible())
		{
			if(cameraIteration++ % odomUpdate == 0)
			{
				OdometryInfo info;
				Transform pose = odom.process(data, &info);

				if(odometryIteration++ % mapUpdate == 0)
				{
					if(rtabmap.process(data, pose))
					{
						mapBuilder.processStatistics(rtabmap.getStatistics());
						if(rtabmap.getLoopClosureId() > 0)
						{
							printf("Loop closure detected!\n");
						}
					}
				}

				mapBuilder.processOdometry(data, pose, info);
			}

			QApplication::processEvents();

			while(mapBuilder.isPaused() && mapBuilder.isVisible())
			{
				uSleep(100);
				QApplication::processEvents();
			}

			data = camera.takeImage();
		}

		if(mapBuilder.isVisible())
		{
			printf("Processed all frames\n");
			app.exec();
		}
	}
	else
	{
		UERROR("Camera init failed!");
	}

	return 0;
}
