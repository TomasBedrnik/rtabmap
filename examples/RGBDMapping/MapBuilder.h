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

#ifndef MAPBUILDER_H_
#define MAPBUILDER_H_

#include <QVBoxLayout>
#include <QtCore/QMetaType>
#include <QAction>

#ifndef Q_MOC_RUN // Mac OS X issue
#include "rtabmap/gui/CloudViewer.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#endif
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

using namespace rtabmap;

// This class receives RtabmapEvent and construct/update a 3D Map
class MapBuilder : public QWidget, public UEventsHandler
{
	Q_OBJECT
public:
	//Camera ownership is not transferred!
        MapBuilder(CameraThread * camera = 0,std::string dir = "/tmp/kinect2",bool binary = true) :
		camera_(camera),
		odometryCorrection_(Transform::getIdentity()),
		processingStatistics_(false),
		lastOdometryProcessed_(true)
	{
		this->setWindowFlags(Qt::Dialog);
		this->setWindowTitle(tr("3D Map"));
		this->setMinimumWidth(800);
		this->setMinimumHeight(600);

		cloudViewer_ = new CloudViewer(this);

		QVBoxLayout *layout = new QVBoxLayout();
		layout->addWidget(cloudViewer_);
		this->setLayout(layout);

		qRegisterMetaType<rtabmap::OdometryEvent>("rtabmap::OdometryEvent");
		qRegisterMetaType<rtabmap::Statistics>("rtabmap::Statistics");

		QAction * pause = new QAction(this);
		this->addAction(pause);
		pause->setShortcut(Qt::Key_Space);
		connect(pause, SIGNAL(triggered()), this, SLOT(pauseDetection()));

                dir_ = dir;
                binary_ = binary;
                boost::filesystem::path path = dir_+"/pcd";
                if ( exists( path ) )
                {
                    boost::filesystem::remove_all(path);
                }
                if (!boost::filesystem::create_directories(path))
                {
                    std::cout << "Cannot create .pcd file directory," << std::endl;
                    std::cout << "POINTCLOUDS WON'T BE STOERED." << std::endl;
                    savePCD_ = false;
                }
	}

	virtual ~MapBuilder()
	{
		this->unregisterFromEventsManager();
	}

protected slots:
	virtual void pauseDetection()
	{
		UWARN("");
		if(camera_)
		{
			if(camera_->isCapturing())
			{
				camera_->join(true);
			}
			else
			{
				camera_->start();
			}
		}
	}

	virtual void processOdometry(const rtabmap::OdometryEvent & odom)
	{
		if(!this->isVisible())
		{
			return;
		}

		Transform pose = odom.pose();
		if(pose.isNull())
		{
			//Odometry lost
			cloudViewer_->setBackgroundColor(Qt::darkRed);

			pose = lastOdomPose_;
		}
		else
		{
			cloudViewer_->setBackgroundColor(cloudViewer_->getDefaultBackgroundColor());
		}
		if(!pose.isNull())
		{
			lastOdomPose_ = pose;

			// 3d cloud
			if(odom.data().depthOrRightRaw().cols == odom.data().imageRaw().cols &&
			   odom.data().depthOrRightRaw().rows == odom.data().imageRaw().rows &&
			   !odom.data().depthOrRightRaw().empty() &&
			   (odom.data().stereoCameraModel().isValidForProjection() || odom.data().cameraModels().size()))
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
					odom.data(),
					2,     // decimation
					4.0f); // max depth
				if(cloud->size())
                                {
                                    //current cloud
                                        if(!cloudViewer_->addCloud("cloudOdom", cloud, odometryCorrection_*pose))
                                        {
                                                UERROR("Adding cloudOdom to viewer failed!");
                                        }
				}
				else
				{
					cloudViewer_->setCloudVisibility("cloudOdom", false);
					UWARN("Empty cloudOdom!");
				}
			}

			if(!odom.pose().isNull())
			{
				// update camera position
                                //and white slownly disappearing line of camera position
                                cloudViewer_->updateCameraTargetPosition(odometryCorrection_*odom.pose());
			}
		}
		cloudViewer_->update();

		lastOdometryProcessed_ = true;
	}


	virtual void processStatistics(const rtabmap::Statistics & stats)
	{
                processingStatistics_ = true;

                //============================
                // Add RGB-D clouds
                //============================
                const std::map<int, Transform> & poses = stats.poses();
                QMap<std::string, Transform> clouds = cloudViewer_->getAddedClouds();
                for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
                {
                        if(!iter->second.isNull())
                        {
                                std::string cloudName = uFormat("cloud%05d", iter->first);

                                // 3d point cloud
                                if(clouds.contains(cloudName))
                                {
                                        // Update only if the pose has changed
                                        Transform tCloud;
                                        cloudViewer_->getPose(cloudName, tCloud);
                                        if(tCloud.isNull() || iter->second != tCloud)
                                        {
                                                if(!cloudViewer_->updateCloudPose(cloudName, iter->second))
                                                {
                                                        UERROR("Updating pose cloud %d failed!", iter->first);
                                                }
                                                std::cout << "UPDATE position: " << cloudName << std::endl;
                                        }
                                        cloudViewer_->setCloudVisibility(cloudName, true);
                                        saveTransform(cloudName,iter->second);


                                }
                                else if(uContains(stats.getSignatures(), iter->first))
                                {
                                        Signature s = stats.getSignatures().at(iter->first);
                                        s.sensorData().uncompressData(); // make sure data is uncompressed
                                        // Add the new cloud
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
                                                        s.sensorData(),
                                                    4,     // decimation
                                                    4.0f); // max depth
                                        if(cloud->size())
                                        {
                                                if(!cloudViewer_->addCloud(cloudName, cloud, iter->second))
                                                {
                                                        UERROR("Adding cloud %d to viewer failed!", iter->first);
                                                }
                                                std::cout << "Add: "<<cloudName<<"|"<<std::endl;
                                                pcl::PCDWriter writer;
                                                std::vector<int> indices;
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                                                pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
                                                writer.write(dir_+"/pcd/tmp.pcd", *outputCloud, binary_);

                                                saveTransform(cloudName,iter->second);
                                                boost::filesystem::rename(dir_+"/pcd/tmp.pcd",dir_+"/pcd/"+cloudName+".pcd");


                                        }
                                        else
                                        {
                                                UWARN("Empty cloud %d!", iter->first);
                                        }
                                }
                        }
                        else
                        {
                                UWARN("Null pose for %d ?!?", iter->first);
                        }
                }

                //============================
                // Add 3D graph (show all poses)
                //============================
                cloudViewer_->removeAllGraphs();
                cloudViewer_->removeCloud("graph_nodes");
                if(poses.size())
                {
                    //lines with green points

                        // Set graph
                        pcl::PointCloud<pcl::PointXYZ>::Ptr graph(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::PointCloud<pcl::PointXYZ>::Ptr graphNodes(new pcl::PointCloud<pcl::PointXYZ>);
                        for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
                        {
                                graph->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
                        }
                        *graphNodes = *graph;


                        // add graph
                        cloudViewer_->addOrUpdateGraph("graph", graph, Qt::gray);
                        cloudViewer_->addCloud("graph_nodes", graphNodes, Transform::getIdentity(), Qt::green);
                        cloudViewer_->setCloudPointSize("graph_nodes", 5);
                }

                odometryCorrection_ = stats.mapCorrection();

                cloudViewer_->update();

                processingStatistics_ = false;
	}

	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("RtabmapEvent") == 0)
		{
			RtabmapEvent * rtabmapEvent = (RtabmapEvent *)event;
			const Statistics & stats = rtabmapEvent->getStats();
			// Statistics must be processed in the Qt thread
			if(this->isVisible())
			{
				QMetaObject::invokeMethod(this, "processStatistics", Q_ARG(rtabmap::Statistics, stats));
			}
		}
		else if(event->getClassName().compare("OdometryEvent") == 0)
		{
			OdometryEvent * odomEvent = (OdometryEvent *)event;
			// Odometry must be processed in the Qt thread
			if(this->isVisible() &&
			   lastOdometryProcessed_ &&
			   !processingStatistics_)
			{
				lastOdometryProcessed_ = false; // if we receive too many odometry events!
				QMetaObject::invokeMethod(this, "processOdometry", Q_ARG(rtabmap::OdometryEvent, *odomEvent));
			}
		}
	}

protected:
	CloudViewer * cloudViewer_;
	CameraThread * camera_;
	Transform lastOdomPose_;
	Transform odometryCorrection_;
	bool processingStatistics_;
	bool lastOdometryProcessed_;
        bool savePCD_ = true;
        std::string dir_;
        bool binary_;
        void saveTransform(std::string cloudName,Transform t)
        {
            if(binary_)
            {
                std::ofstream poseFileBinary;
                poseFileBinary.open (dir_+"/pcd/tmp.trb", std::ios::out | std::ios::binary);
                Eigen::Matrix4f mat = t.toEigen4f();
                for(int row = 0;row<4;row++)
                    for(int col = 0;col<4;col++)
                    {
                        poseFileBinary.write( reinterpret_cast<const char*>( &mat(row,col) ), sizeof( float ));
                    }
                poseFileBinary.close();

                boost::filesystem::rename(dir_+"/pcd/tmp.trb",dir_+"/pcd/"+cloudName+".trb");
            }
            else
            {
                std::ofstream poseFile;
                poseFile.open (dir_+"/pcd/tmp.trt");
                poseFile << t.toEigen4f() << std::endl;
                poseFile.close();

                boost::filesystem::rename(dir_+"/pcd/tmp.trt",dir_+"/pcd/"+cloudName+".trt");
            }
        }
};


#endif /* MAPBUILDER_H_ */
