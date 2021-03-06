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

#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>

#include <pcl/common/io.h>

#include <iostream>
#include <fstream>
#include <cmath>

namespace rtabmap
{

/////////////////////////
// CameraImages
/////////////////////////
CameraImages::CameraImages() :
		_startAt(0),
		_refreshDir(false),
		_rectifyImages(false),
		_bayerMode(-1),
		_isDepth(false),
		_depthScaleFactor(1.0f),
		_count(0),
		_dir(0),
		_countScan(0),
		_scanDir(0),
		_scanLocalTransform(Transform::getIdentity()),
		_scanMaxPts(0),
		_scanDownsampleStep(1),
		_scanVoxelSize(0.0f),
		_scanNormalsK(0),
		_depthFromScan(false),
		_depthFromScanFillHoles(1),
		_depthFromScanFillHolesFromBorder(false),
		_filenamesAreTimestamps(false),
		syncImageRateWithStamps_(true),
		_groundTruthFormat(0),
		_captureDelay(0.0)
	{}
CameraImages::CameraImages(const std::string & path,
					 float imageRate,
					 const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_path(path),
	_startAt(0),
	_refreshDir(false),
	_rectifyImages(false),
	_bayerMode(-1),
	_isDepth(false),
	_depthScaleFactor(1.0f),
	_count(0),
	_dir(0),
	_countScan(0),
	_scanDir(0),
	_scanLocalTransform(Transform::getIdentity()),
	_scanMaxPts(0),
	_scanDownsampleStep(1),
	_scanVoxelSize(0.0f),
	_scanNormalsK(0),
	_depthFromScan(false),
	_depthFromScanFillHoles(1),
	_depthFromScanFillHolesFromBorder(false),
	_filenamesAreTimestamps(false),
	syncImageRateWithStamps_(true),
	_groundTruthFormat(0),
	_captureDelay(0.0)
{

}

CameraImages::~CameraImages()
{
	UDEBUG("");
	if(_dir)
	{
		delete _dir;
	}
	if(_scanDir)
	{
		delete _scanDir;
	}
}

bool CameraImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_lastFileName.clear();
	_lastScanFileName.clear();
	_count = 0;
	_countScan = 0;
	_captureDelay = 0.0;

	UDEBUG("");
	if(_dir)
	{
		_dir->setPath(_path, "jpg ppm png bmp pnm tiff pgm");
	}
	else
	{
		_dir = new UDirectory(_path, "jpg ppm png bmp pnm tiff pgm");
	}
	if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
	{
		_path.append("/");
	}
	if(!_dir->isValid())
	{
		ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
	}
	else if(_dir->getFileNames().size() == 0)
	{
		UWARN("Directory is empty \"%s\"", _path.c_str());
	}
	else
	{
		UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
	}

	// check for scan directory
	if(_scanDir)
	{
		delete _scanDir;
		_scanDir = 0;
	}
	if(!_scanPath.empty())
	{
		UINFO("scan path=%s", _scanPath.c_str());
		_scanDir = new UDirectory(_scanPath, "pcd bin ply"); // "bin" is for KITTI format
		if(_scanPath[_scanPath.size()-1] != '\\' && _scanPath[_scanPath.size()-1] != '/')
		{
			_scanPath.append("/");
		}
		if(!_scanDir->isValid())
		{
			UERROR("Scan directory path is not valid \"%s\"", _scanPath.c_str());
			delete _scanDir;
			_scanDir = 0;
		}
		else if(_scanDir->getFileNames().size() == 0)
		{
			UWARN("Scan directory is empty \"%s\"", _scanPath.c_str());
			delete _scanDir;
			_scanDir = 0;
		}
		else if(_scanDir->getFileNames().size() != _dir->getFileNames().size())
		{
			UERROR("Scan and image directories should be the same size \"%s\"(%d) vs \"%s\"(%d)",
					_scanPath.c_str(),
					(int)_scanDir->getFileNames().size(),
					_path.c_str(),
					(int)_dir->getFileNames().size());
			delete _scanDir;
			_scanDir = 0;
		}
		else
		{
			UINFO("path=%s scans=%d", _scanPath.c_str(), (int)this->imagesCount());
		}
	}

	// look for calibration files
	UINFO("calibration folder=%s name=%s", calibrationFolder.c_str(), cameraName.c_str());
	if(!calibrationFolder.empty() && !cameraName.empty())
	{
		if(!_model.load(calibrationFolder, cameraName))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					cameraName.c_str(), calibrationFolder.c_str());
		}
		else
		{
			UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
					_model.fx(),
					_model.fy(),
					_model.cx(),
					_model.cy());
		}
	}
	_model.setName(cameraName);

	_model.setLocalTransform(this->getLocalTransform());
	if(_rectifyImages && !_model.isValidForRectification())
	{
		UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
		return false;
	}

	bool success = _dir->isValid();
	stamps_.clear();
	groundTruth_.clear();
	if(success)
	{
		if(_filenamesAreTimestamps)
		{
			const std::list<std::string> & filenames = _dir->getFileNames();
			for(std::list<std::string>::const_iterator iter=filenames.begin(); iter!=filenames.end(); ++iter)
			{
				// format is text_12234456.12334_text.png
				std::list<std::string> list = uSplit(*iter, '.');
				if(list.size() == 3)
				{
					list.pop_back(); // remove extension
					std::string decimals = uSplitNumChar(list.back()).front();
					list.pop_back();
					std::string sec = uSplitNumChar(list.back()).back();
					double stamp = uStr2Double(sec + "." + decimals);
					if(stamp > 0.0)
					{
						stamps_.push_back(stamp);
					}
					else
					{
						UERROR("Conversion filename to timestamp failed! (filename=%s)", iter->c_str());
					}
				}
			}
			if(stamps_.size() != this->imagesCount())
			{
				UERROR("The stamps count is not the same as the images (%d vs %d)! "
					   "Converting filenames to timestamps is activated.",
						(int)stamps_.size(), this->imagesCount());
				stamps_.clear();
				success = false;
			}
		}
		else if(timestampsPath_.size())
		{
			std::ifstream file;
			file.open(timestampsPath_.c_str(), std::ifstream::in);
			while(file.good())
			{
				std::string str;
				std::getline(file, str);

				if(str.empty() || str.at(0) == '#' || str.at(0) == '%')
				{
					continue;
				}

				std::list<std::string> strList = uSplit(str, ' ');
				std::string stampStr = strList.front();
				if(strList.size() == 2)
				{
					// format "seconds millisec"
					// the millisec str needs 0-padding if size < 6
					std::string millisecStr = strList.back();
					while(millisecStr.size() < 6)
					{
						millisecStr = "0" + millisecStr;
					}
					stampStr = stampStr+'.'+millisecStr;
				}
				stamps_.push_back(uStr2Double(stampStr));
			}

			file.close();

			if(stamps_.size() != this->imagesCount())
			{
				UERROR("The stamps count (%d) is not the same as the images (%d)! Please remove "
						"the timestamps file path if you don't want to use them (current file path=%s).",
						(int)stamps_.size(), this->imagesCount(), timestampsPath_.c_str());
				stamps_.clear();
				success = false;
			}
		}

		if(groundTruthPath_.size())
		{
			std::map<int, Transform> poses;
			std::map<int, double> stamps;
			if(!graph::importPoses(groundTruthPath_, _groundTruthFormat, poses, 0, &stamps))
			{
				UERROR("Cannot read ground truth file \"%s\".", groundTruthPath_.c_str());
				success = false;
			}
			else if((_groundTruthFormat != 1 && _groundTruthFormat != 5 && _groundTruthFormat != 6 && _groundTruthFormat != 7) && poses.size() != this->imagesCount())
			{
				UERROR("The ground truth count is not the same as the images (%d vs %d)! Please remove "
						"the ground truth file path if you don't want to use it (current file path=%s).",
						(int)poses.size(), this->imagesCount(), groundTruthPath_.c_str());
				success = false;
			}
			else if((_groundTruthFormat == 1 || _groundTruthFormat == 5 || _groundTruthFormat == 6 || _groundTruthFormat == 7) && stamps_.size() == 0)
			{
				UERROR("When using RGBD-SLAM, GPS, MALAGA and ST LUCIA formats for ground truth, images must have timestamps!");
				success = false;
			}
			else if(_groundTruthFormat == 1 || _groundTruthFormat == 5 || _groundTruthFormat == 6 || _groundTruthFormat == 7)
			{
				UDEBUG("");
				//Match ground truth values with images
				groundTruth_.clear();
				std::map<double, int> stampsToIds;
				for(std::map<int, double>::iterator iter=stamps.begin(); iter!=stamps.end(); ++iter)
				{
					stampsToIds.insert(std::make_pair(iter->second, iter->first));
				}
				std::vector<double> values = uValues(stamps);

				int validPoses = 0;
				for(std::list<double>::iterator ster=stamps_.begin(); ster!=stamps_.end(); ++ster)
				{
					Transform pose; // null transform
					std::map<double, int>::iterator endIter = stampsToIds.lower_bound(*ster);
					bool warned = false;
					if(endIter != stampsToIds.end())
					{
						if(endIter->first == *ster)
						{
							pose = poses.at(endIter->second);
						}
						else if(endIter != stampsToIds.begin())
						{
							//interpolate
							std::map<double, int>::iterator beginIter = endIter;
							--beginIter;
							double stampBeg = beginIter->first;
							double stampEnd = endIter->first;
							UASSERT(stampEnd > stampBeg && *ster>stampBeg && *ster < stampEnd);
							if(stampEnd - stampBeg > 10.0)
							{
								warned = true;
								UDEBUG("Cannot interpolate ground truth pose for stamp %f between %f and %f (>10 sec)", 
									*ster,
									stampBeg,
									stampEnd);
							}
							else
							{
								float t = (*ster - stampBeg) / (stampEnd-stampBeg);
								Transform & ta = poses.at(beginIter->second);
								Transform & tb = poses.at(endIter->second);
								if(!ta.isNull() && !tb.isNull())
								{
									++validPoses;
									pose = ta.interpolate(t, tb);
								}
							}
						}
					}
					if(pose.isNull() && !warned)
					{
						UDEBUG("Ground truth pose not found for stamp %f", *ster);
					}
					groundTruth_.push_back(pose);
				}
				if(validPoses != (int)stamps_.size())
				{
					UWARN("%d valid ground truth poses of %d stamps", validPoses, (int)stamps_.size());
				}
			}
			else
			{
				UDEBUG("");
				groundTruth_ = uValuesList(poses);
				if(stamps_.size() == 0 && stamps.size() == poses.size())
				{
					stamps_ = uValuesList(stamps);
				}
				else if(_groundTruthFormat==8 && stamps_.size() == 0 && stamps.size()>0 && stamps.size() != poses.size())
				{
					UERROR("With Karlsruhe ground truth format, timestamps (%d) and poses (%d) should match!", (int)stamps.size(), (int)poses.size());
				}
			}
			UASSERT_MSG(groundTruth_.size() == stamps_.size(), uFormat("%d vs %d", (int)groundTruth_.size(), (int)stamps_.size()).c_str());
		}
	}

	_captureTimer.restart();

	return success;
}

bool CameraImages::isCalibrated() const
{
	return _model.isValidForProjection();
}

std::string CameraImages::getSerial() const
{
	return _model.name();
}

unsigned int CameraImages::imagesCount() const
{
	if(_dir)
	{
		return (unsigned int)_dir->getFileNames().size();
	}
	return 0;
}

std::vector<std::string> CameraImages::filenames() const
{
	if(_dir)
	{
		return uListToVector(_dir->getFileNames());
	}
	return std::vector<std::string>();
}

SensorData CameraImages::captureImage(CameraInfo * info)
{
	if(syncImageRateWithStamps_ && _captureDelay>0.0)
	{
		int sleepTime = (1000*_captureDelay - 1000.0f*_captureTimer.getElapsedTime());
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}
		else if(sleepTime < 0)
		{
			if(this->getImageRate() > 0.0f)
			{
				UWARN("CameraImages: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs). Disable "
					  "source image rate or disable synchronization of capture time with timestamps.", 
					  _captureDelay, _captureTimer.getElapsedTime());
			}
			else
			{
				UWARN("CameraImages: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs).", 
					_captureDelay, _captureTimer.getElapsedTime());
			}
		}

		// Add precision at the cost of a small overhead
		while(_captureTimer.getElapsedTime() < _captureDelay-0.000001)
		{
			//
		}
		_captureTimer.start();
	}
	_captureDelay = 0.0;

	cv::Mat img;
	cv::Mat scan;
	double stamp = UTimer::now();
	Transform groundTruthPose;
	cv::Mat depthFromScan;
	UDEBUG("");
	if(_dir->isValid())
	{
		if(_refreshDir)
		{
			_dir->update();
			if(_scanDir)
			{
				_scanDir->update();
			}
		}
		std::string imageFilePath;
		std::string scanFilePath;
		if(_startAt < 0)
		{
			const std::list<std::string> & fileNames = _dir->getFileNames();
			if(fileNames.size())
			{
				if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
				{
					_lastFileName = *fileNames.rbegin();
					imageFilePath = _path + _lastFileName;
				}
			}
			if(_scanDir)
			{
				const std::list<std::string> & scanFileNames = _scanDir->getFileNames();
				if(scanFileNames.size())
				{
					if(_lastScanFileName.empty() || uStrNumCmp(_lastScanFileName,*scanFileNames.rbegin()) < 0)
					{
						_lastScanFileName = *scanFileNames.rbegin();
						scanFilePath = _scanPath + _lastScanFileName;
					}
				}
			}
		}
		else
		{
			std::string fileName;
			fileName = _dir->getNextFileName();
			if(!fileName.empty())
			{
				imageFilePath = _path + fileName;
				while(_count++ < _startAt && (fileName = _dir->getNextFileName()).size())
				{
					imageFilePath = _path + fileName;
				}
			}
			if(_scanDir)
			{
				fileName = _scanDir->getNextFileName();
				if(!fileName.empty())
				{
					scanFilePath = _scanPath + fileName;
					while(++_countScan < _startAt && (fileName = _scanDir->getNextFileName()).size())
					{
						scanFilePath = _scanPath + fileName;
					}
				}
			}
		}

		if(stamps_.size())
		{
			stamp = stamps_.front();
			stamps_.pop_front();
			if(stamps_.size())
			{
				_captureDelay = stamps_.front() - stamp;
			}
			if(groundTruth_.size())
			{
				groundTruthPose = groundTruth_.front();
				groundTruth_.pop_front();
			}
		}

		if(!imageFilePath.empty())
		{
			ULOGGER_DEBUG("Loading image : %s", imageFilePath.c_str());

#if CV_MAJOR_VERSION >2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
			img = cv::imread(imageFilePath.c_str(), cv::IMREAD_UNCHANGED);
#else
			img = cv::imread(imageFilePath.c_str(), -1);
#endif
			UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d",
					img.cols, img.rows, img.channels(), img.elemSize(), img.total());

			if(_isDepth)
			{
				if(img.type() != CV_16UC1 && img.type() != CV_32FC1)
				{
					UERROR("Depth is on and the loaded image has not a format supported (file = \"%s\"). "
							"Formats supported are 16 bits 1 channel and 32 bits 1 channel.",
							imageFilePath.c_str());
					img = cv::Mat();
				}

				if(_depthScaleFactor > 1.0f)
				{
					img /= _depthScaleFactor;
				}
			}
			else
			{
#if CV_MAJOR_VERSION < 3
				// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
				if(img.depth() != CV_8U)
				{
					// The depth should be 8U
					UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
					IplImage * i = cvLoadImage(imageFilePath.c_str());
					img = cv::Mat(i, true);
					cvReleaseImage(&i);
				}
#endif
				if(img.channels()>3)
				{
                    UWARN("Conversion from 4 channels to 3 channels (file=%s)", imageFilePath.c_str());
					cv::Mat out;
					cv::cvtColor(img, out, CV_BGRA2BGR);
					img = out;
				}
				else if(_bayerMode >= 0 && _bayerMode <=3)
				{
					cv::Mat debayeredImg;
					try
					{
						cv::cvtColor(img, debayeredImg, CV_BayerBG2BGR + _bayerMode);
						img = debayeredImg;
					}
					catch(const cv::Exception & e)
					{
						UWARN("Error debayering images: \"%s\". Please set bayer mode to -1 if images are not bayered!", e.what());
					}
				}

			}

			if(!img.empty() && _model.isValidForRectification() && _rectifyImages)
			{
				img = _model.rectifyImage(img);
			}
		}

		if(!scanFilePath.empty())
		{
			// load without filtering
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::loadCloud(scanFilePath, _scanLocalTransform);
			UDEBUG("Loaded scan=%d points", (int)cloud->size());
			if(_depthFromScan && !img.empty())
			{
				UDEBUG("Computing depth from scan...");
				if(!_model.isValidForProjection())
				{
					UWARN("Depth from laser scan: Camera model should be valid.");
				}
				else if(_isDepth)
				{
					UWARN("Depth from laser scan: Loading already a depth image.");
				}
				else
				{
					depthFromScan = util3d::projectCloudToCamera(img.size(), _model.K(), cloud, _model.localTransform());
					if(_depthFromScanFillHoles!=0)
					{
						util3d::fillProjectedCloudHoles(depthFromScan, _depthFromScanFillHoles>0, _depthFromScanFillHolesFromBorder);
					}
				}
			}
			// filter the scan after registration
			int previousSize = (int)cloud->size();
			if(_scanDownsampleStep > 1 && cloud->size())
			{
				cloud = util3d::downsample(cloud, _scanDownsampleStep);
				UDEBUG("Downsampling scan (step=%d): %d -> %d", _scanDownsampleStep, previousSize, (int)cloud->size());
			}
			previousSize = (int)cloud->size();
			if(_scanVoxelSize > 0.0f && cloud->size())
			{
				cloud = util3d::voxelize(cloud, _scanVoxelSize);
				UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d", _scanVoxelSize, previousSize, (int)cloud->size());
			}
			if(_scanNormalsK > 0 && cloud->size())
			{
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, _scanNormalsK);
				pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields(*cloud, *normals, *cloudNormals);
				scan = util3d::laserScanFromPointCloud(*cloudNormals, _scanLocalTransform.inverse());
			}
			else
			{
				scan = util3d::laserScanFromPointCloud(*cloud, _scanLocalTransform.inverse());
			}
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	if(_model.imageHeight() == 0 || _model.imageWidth() == 0)
	{
		_model.setImageSize(img.size());
	}

	SensorData data(scan, LaserScanInfo(scan.empty()?0:_scanMaxPts, 0, _scanLocalTransform), _isDepth?cv::Mat():img, _isDepth?img:depthFromScan, _model, this->getNextSeqID(), stamp);
	data.setGroundTruth(groundTruthPose);
	return data;
}

/////////////////////////
// CameraGrabberImage
/////////////////////////
CameraGrabberImage::CameraGrabberImage() :
        _startAt(0),
        _refreshDir(false),
        _rectifyImages(false),
        _bayerMode(-1),
        _isDepth(false),
        _depthScaleFactor(1.0f),
        _count(0),
        _dir(0),
        _countScan(0),
        _scanDir(0),
        _scanLocalTransform(Transform::getIdentity()),
        _scanMaxPts(0),
        _scanDownsampleStep(1),
        _scanVoxelSize(0.0f),
        _scanNormalsK(0),
        _depthFromScan(false),
        _depthFromScanFillHoles(1),
        _depthFromScanFillHolesFromBorder(false),
        _filenamesAreTimestamps(false),
        syncImageRateWithStamps_(true),
        _groundTruthFormat(0),
        _captureDelay(0.0),
        _fileXML(false)
    {}
CameraGrabberImage::CameraGrabberImage(const std::string & path,
                     float imageRate,
                     const Transform & localTransform) :
    Camera(imageRate, localTransform),
    _path(path),
    _startAt(0),
    _refreshDir(false),
    _rectifyImages(false),
    _bayerMode(-1),
    _isDepth(false),
    _depthScaleFactor(1.0f),
    _count(0),
    _dir(0),
    _countScan(0),
    _scanDir(0),
    _scanLocalTransform(Transform::getIdentity()),
    _scanMaxPts(0),
    _scanDownsampleStep(1),
    _scanVoxelSize(0.0f),
    _scanNormalsK(0),
    _depthFromScan(false),
    _depthFromScanFillHoles(1),
    _depthFromScanFillHolesFromBorder(false),
    _filenamesAreTimestamps(false),
    syncImageRateWithStamps_(true),
    _groundTruthFormat(0),
    _captureDelay(0.0),
    _fileXML(false)
{

}

CameraGrabberImage::~CameraGrabberImage()
{
    UDEBUG("");
    if(_dir)
    {
        delete _dir;
    }
    if(_scanDir)
    {
        delete _scanDir;
    }
}

bool CameraGrabberImage::init(const std::string & calibrationFolder, const std::string & cameraName)
{
    _lastFileName.clear();
    _lastScanFileName.clear();
    _count = 0;
    _countScan = 0;
    _captureDelay = 0.0;

    UDEBUG("");
    if(_dir)
    {
        _dir->setPath("/tmp/", "jpg ppm png bmp pnm tiff pgm");
    }
    else
    {
        _dir = new UDirectory("/tmp/", "jpg ppm png bmp pnm tiff pgm");
    }

    std::list<std::string> list = uSplit(_path, '.');
    if(list.size() >= 2 && (list.back() == "xml" || list.back() == "XML"))
        _fileXML = true;
//    if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
//    {
//        _path.append("/");
//    }
//    if(!_dir->isValid())
//    {
//        ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
//    }
//    else if(_dir->getFileNames().size() == 0)
//    {
//        UWARN("Directory is empty \"%s\"", _path.c_str());
//    }
//    else
//    {
//        UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
//    }

    // check for scan directory
    if(_scanDir)
    {
        delete _scanDir;
        _scanDir = 0;
    }
    if(!_scanPath.empty())
    {
        UINFO("scan path=%s", _scanPath.c_str());
        _scanDir = new UDirectory(_scanPath, "pcd bin ply"); // "bin" is for KITTI format
        if(_scanPath[_scanPath.size()-1] != '\\' && _scanPath[_scanPath.size()-1] != '/')
        {
            _scanPath.append("/");
        }
        if(!_scanDir->isValid())
        {
            UERROR("Scan directory path is not valid \"%s\"", _scanPath.c_str());
            delete _scanDir;
            _scanDir = 0;
        }
        else if(_scanDir->getFileNames().size() == 0)
        {
            UWARN("Scan directory is empty \"%s\"", _scanPath.c_str());
            delete _scanDir;
            _scanDir = 0;
        }
        else if(_scanDir->getFileNames().size() != _dir->getFileNames().size())
        {
            UERROR("Scan and image directories should be the same size \"%s\"(%d) vs \"%s\"(%d)",
                    _scanPath.c_str(),
                    (int)_scanDir->getFileNames().size(),
                    _path.c_str(),
                    (int)_dir->getFileNames().size());
            delete _scanDir;
            _scanDir = 0;
        }
        else
        {
            UINFO("path=%s scans=%d", _scanPath.c_str(), (int)this->imagesCount());
        }
    }

    // look for calibration files
    UINFO("calibration folder=%s name=%s", calibrationFolder.c_str(), cameraName.c_str());
    if(!calibrationFolder.empty() && !cameraName.empty())
    {
        if(!_model.load(calibrationFolder, cameraName))//HERE!
        {
            UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
                    cameraName.c_str(), calibrationFolder.c_str());
        }
        else
        {
            UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
                    _model.fx(),
                    _model.fy(),
                    _model.cx(),
                    _model.cy());
        }
    }
    _model.setName(cameraName);

    _model.setLocalTransform(this->getLocalTransform());
    if(_rectifyImages && !_model.isValidForRectification())
    {
        UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
        return false;
    }

    bool success = _dir->isValid();
    stamps_.clear();
    groundTruth_.clear();
    if(success)
    {
//        if(_filenamesAreTimestamps)
//        {
//            const std::list<std::string> & filenames = _dir->getFileNames();
//            for(std::list<std::string>::const_iterator iter=filenames.begin(); iter!=filenames.end(); ++iter)
//            {
//                // format is text_12234456.12334_text.png
//                std::list<std::string> list = uSplit(*iter, '.');
//                if(list.size() == 3)
//                {
//                    list.pop_back(); // remove extension
//                    std::string decimals = uSplitNumChar(list.back()).front();
//                    list.pop_back();
//                    std::string sec = uSplitNumChar(list.back()).back();
//                    double stamp = uStr2Double(sec + "." + decimals);
//                    if(stamp > 0.0)
//                    {
//                        stamps_.push_back(stamp);
//                    }
//                    else
//                    {
//                        UERROR("Conversion filename to timestamp failed! (filename=%s)", iter->c_str());
//                    }
//                }
//            }
//            if(stamps_.size() != this->imagesCount())
//            {
//                UERROR("The stamps count is not the same as the images (%d vs %d)! "
//                       "Converting filenames to timestamps is activated.",
//                        (int)stamps_.size(), this->imagesCount());
//                stamps_.clear();
//                success = false;
//            }
//        }
//        else if(timestampsPath_.size())
//        {
//            std::ifstream file;
//            file.open(timestampsPath_.c_str(), std::ifstream::in);
//            while(file.good())
//            {
//                std::string str;
//                std::getline(file, str);

//                if(str.empty() || str.at(0) == '#' || str.at(0) == '%')
//                {
//                    continue;
//                }

//                std::list<std::string> strList = uSplit(str, ' ');
//                std::string stampStr = strList.front();
//                if(strList.size() == 2)
//                {
//                    // format "seconds millisec"
//                    // the millisec str needs 0-padding if size < 6
//                    std::string millisecStr = strList.back();
//                    while(millisecStr.size() < 6)
//                    {
//                        millisecStr = "0" + millisecStr;
//                    }
//                    stampStr = stampStr+'.'+millisecStr;
//                }
//                stamps_.push_back(uStr2Double(stampStr));
//            }

//            file.close();

//            if(stamps_.size() != this->imagesCount())
//            {
//                UERROR("The stamps count (%d) is not the same as the images (%d)! Please remove "
//                        "the timestamps file path if you don't want to use them (current file path=%s).",
//                        (int)stamps_.size(), this->imagesCount(), timestampsPath_.c_str());
//                stamps_.clear();
//                success = false;
//            }
//        }

//        if(groundTruthPath_.size())
//        {
//            std::map<int, Transform> poses;
//            std::map<int, double> stamps;
//            if(!graph::importPoses(groundTruthPath_, _groundTruthFormat, poses, 0, &stamps))
//            {
//                UERROR("Cannot read ground truth file \"%s\".", groundTruthPath_.c_str());
//                success = false;
//            }
//            else if((_groundTruthFormat != 1 && _groundTruthFormat != 5 && _groundTruthFormat != 6 && _groundTruthFormat != 7) && poses.size() != this->imagesCount())
//            {
//                UERROR("The ground truth count is not the same as the images (%d vs %d)! Please remove "
//                        "the ground truth file path if you don't want to use it (current file path=%s).",
//                        (int)poses.size(), this->imagesCount(), groundTruthPath_.c_str());
//                success = false;
//            }
//            else if((_groundTruthFormat == 1 || _groundTruthFormat == 5 || _groundTruthFormat == 6 || _groundTruthFormat == 7) && stamps_.size() == 0)
//            {
//                UERROR("When using RGBD-SLAM, GPS, MALAGA and ST LUCIA formats for ground truth, images must have timestamps!");
//                success = false;
//            }
//            else if(_groundTruthFormat == 1 || _groundTruthFormat == 5 || _groundTruthFormat == 6 || _groundTruthFormat == 7)
//            {
//                UDEBUG("");
//                //Match ground truth values with images
//                groundTruth_.clear();
//                std::map<double, int> stampsToIds;
//                for(std::map<int, double>::iterator iter=stamps.begin(); iter!=stamps.end(); ++iter)
//                {
//                    stampsToIds.insert(std::make_pair(iter->second, iter->first));
//                }
//                std::vector<double> values = uValues(stamps);

//                int validPoses = 0;
//                for(std::list<double>::iterator ster=stamps_.begin(); ster!=stamps_.end(); ++ster)
//                {
//                    Transform pose; // null transform
//                    std::map<double, int>::iterator endIter = stampsToIds.lower_bound(*ster);
//                    bool warned = false;
//                    if(endIter != stampsToIds.end())
//                    {
//                        if(endIter->first == *ster)
//                        {
//                            pose = poses.at(endIter->second);
//                        }
//                        else if(endIter != stampsToIds.begin())
//                        {
//                            //interpolate
//                            std::map<double, int>::iterator beginIter = endIter;
//                            --beginIter;
//                            double stampBeg = beginIter->first;
//                            double stampEnd = endIter->first;
//                            UASSERT(stampEnd > stampBeg && *ster>stampBeg && *ster < stampEnd);
//                            if(stampEnd - stampBeg > 10.0)
//                            {
//                                warned = true;
//                                UDEBUG("Cannot interpolate ground truth pose for stamp %f between %f and %f (>10 sec)",
//                                    *ster,
//                                    stampBeg,
//                                    stampEnd);
//                            }
//                            else
//                            {
//                                float t = (*ster - stampBeg) / (stampEnd-stampBeg);
//                                Transform & ta = poses.at(beginIter->second);
//                                Transform & tb = poses.at(endIter->second);
//                                if(!ta.isNull() && !tb.isNull())
//                                {
//                                    ++validPoses;
//                                    pose = ta.interpolate(t, tb);
//                                }
//                            }
//                        }
//                    }
//                    if(pose.isNull() && !warned)
//                    {
//                        UDEBUG("Ground truth pose not found for stamp %f", *ster);
//                    }
//                    groundTruth_.push_back(pose);
//                }
//                if(validPoses != (int)stamps_.size())
//                {
//                    UWARN("%d valid ground truth poses of %d stamps", validPoses, (int)stamps_.size());
//                }
//            }
//            else
//            {
//                UDEBUG("");
//                groundTruth_ = uValuesList(poses);
//                if(stamps_.size() == 0 && stamps.size() == poses.size())
//                {
//                    stamps_ = uValuesList(stamps);
//                }
//                else if(_groundTruthFormat==8 && stamps_.size() == 0 && stamps.size()>0 && stamps.size() != poses.size())
//                {
//                    UERROR("With Karlsruhe ground truth format, timestamps (%d) and poses (%d) should match!", (int)stamps.size(), (int)poses.size());
//                }
//            }
//            UASSERT_MSG(groundTruth_.size() == stamps_.size(), uFormat("%d vs %d", (int)groundTruth_.size(), (int)stamps_.size()).c_str());
//        }
    }

    _captureTimer.restart();

    return success;
}

bool CameraGrabberImage::isCalibrated() const
{
    return _model.isValidForProjection();
}

std::string CameraGrabberImage::getSerial() const
{
    return _model.name();
}

unsigned int CameraGrabberImage::imagesCount() const
{
//    if(_dir)
//    {
//        return (unsigned int)_dir->getFileNames().size();
//    }
    return 1000;
}

std::vector<std::string> CameraGrabberImage::filenames() const
{
    if(_dir)
    {
        return uListToVector(_dir->getFileNames());
    }
    return std::vector<std::string>();
}

SensorData CameraGrabberImage::captureImage(CameraInfo * info)
{
    if(syncImageRateWithStamps_ && _captureDelay>0.0)
    {
        int sleepTime = (1000*_captureDelay - 1000.0f*_captureTimer.getElapsedTime());
        if(sleepTime > 2)
        {
            uSleep(sleepTime-2);
        }
        else if(sleepTime < 0)
        {
            if(this->getImageRate() > 0.0f)
            {
                UWARN("CameraGrabberImage: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs). Disable "
                      "source image rate or disable synchronization of capture time with timestamps.",
                      _captureDelay, _captureTimer.getElapsedTime());
            }
            else
            {
                UWARN("CameraGrabberImage: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs).",
                    _captureDelay, _captureTimer.getElapsedTime());
            }
        }

        // Add precision at the cost of a small overhead
        while(_captureTimer.getElapsedTime() < _captureDelay-0.000001)
        {
            //
        }
        _captureTimer.start();
    }
    _captureDelay = 0.0;

    cv::Mat img;
    cv::Mat scan;
    double stamp = UTimer::now();
    Transform groundTruthPose;
    cv::Mat depthFromScan;
    UDEBUG("");
    if(_dir->isValid())
    {
        if(_refreshDir)
        {
            _dir->update();
            if(_scanDir)
            {
                _scanDir->update();
            }
        }
        std::string imageFilePath = _path;
        std::string scanFilePath;
//        std::string imageFilePath;
//        std::string scanFilePath;
//        if(_startAt < 0)
//        {
//            const std::list<std::string> & fileNames = _dir->getFileNames();
//            if(fileNames.size())
//            {
//                if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
//                {
//                    _lastFileName = *fileNames.rbegin();
//                    imageFilePath = _path + _lastFileName;
//                }
//            }
//            if(_scanDir)
//            {
//                const std::list<std::string> & scanFileNames = _scanDir->getFileNames();
//                if(scanFileNames.size())
//                {
//                    if(_lastScanFileName.empty() || uStrNumCmp(_lastScanFileName,*scanFileNames.rbegin()) < 0)
//                    {
//                        _lastScanFileName = *scanFileNames.rbegin();
//                        scanFilePath = _scanPath + _lastScanFileName;
//                    }
//                }
//            }
//        }
//        else
//        {
//            std::string fileName;
//            fileName = _dir->getNextFileName();
//            if(!fileName.empty())
//            {
//                imageFilePath = _path + fileName;
//                while(_count++ < _startAt && (fileName = _dir->getNextFileName()).size())
//                {
//                    imageFilePath = _path + fileName;
//                }
//            }
//            if(_scanDir)
//            {
//                fileName = _scanDir->getNextFileName();
//                if(!fileName.empty())
//                {
//                    scanFilePath = _scanPath + fileName;
//                    while(++_countScan < _startAt && (fileName = _scanDir->getNextFileName()).size())
//                    {
//                        scanFilePath = _scanPath + fileName;
//                    }
//                }
//            }
//        }

//        if(stamps_.size())
//        {
//            stamp = stamps_.front();
//            stamps_.pop_front();
//            if(stamps_.size())
//            {
//                _captureDelay = stamps_.front() - stamp;
//            }
//            if(groundTruth_.size())
//            {
//                groundTruthPose = groundTruth_.front();
//                groundTruth_.pop_front();
//            }
//        }

        if(!imageFilePath.empty())
        {
            ULOGGER_DEBUG("Loading image : %s", imageFilePath.c_str());

#if CV_MAJOR_VERSION >2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
            if(!_fileXML)
                img = cv::imread(imageFilePath.c_str(), cv::IMREAD_UNCHANGED);
            else
            {
                cv::FileStorage fs(imageFilePath.c_str(), cv::FileStorage::READ);
                fs["depth_mat"] >> img;
            }
#else
            img = cv::imread(imageFilePath.c_str(), -1);
#endif
            UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d",
                    img.cols, img.rows, img.channels(), img.elemSize(), img.total());

            if(_isDepth)
            {
                if(img.type() != CV_16UC1 && img.type() != CV_32FC1)
                {
                    UERROR("Depth is on and the loaded image has not a format supported (file = \"%s\"). "
                            "Formats supported are 16 bits 1 channel and 32 bits 1 channel.",
                            imageFilePath.c_str());
                    img = cv::Mat();
                }

                if(_depthScaleFactor > 1.0f)
                {
                    img /= _depthScaleFactor;
                }
            }
            else
            {
#if CV_MAJOR_VERSION < 3
                // FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
                if(img.depth() != CV_8U)
                {
                    // The depth should be 8U
                    UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
                    IplImage * i = cvLoadImage(imageFilePath.c_str());
                    img = cv::Mat(i, true);
                    cvReleaseImage(&i);
                }
#endif
                if(img.channels()>3)
                {
                    //UWARN("Conversion from 4 channels to 3 channels (file=%s)", imageFilePath.c_str());
                    cv::Mat out;
                    cv::cvtColor(img, out, CV_BGRA2BGR);
                    img = out;
                }
                else if(_bayerMode >= 0 && _bayerMode <=3)
                {
                    cv::Mat debayeredImg;
                    try
                    {
                        cv::cvtColor(img, debayeredImg, CV_BayerBG2BGR + _bayerMode);
                        img = debayeredImg;
                    }
                    catch(const cv::Exception & e)
                    {
                        UWARN("Error debayering images: \"%s\". Please set bayer mode to -1 if images are not bayered!", e.what());
                    }
                }

            }

            if(!img.empty() && _model.isValidForRectification() && _rectifyImages)
            {
                img = _model.rectifyImage(img);
            }
        }

        if(!scanFilePath.empty())
        {
            // load without filtering
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::loadCloud(scanFilePath, _scanLocalTransform);
            UDEBUG("Loaded scan=%d points", (int)cloud->size());
            if(_depthFromScan && !img.empty())
            {
                UDEBUG("Computing depth from scan...");
                if(!_model.isValidForProjection())
                {
                    UWARN("Depth from laser scan: Camera model should be valid.");
                }
                else if(_isDepth)
                {
                    UWARN("Depth from laser scan: Loading already a depth image.");
                }
                else
                {
                    depthFromScan = util3d::projectCloudToCamera(img.size(), _model.K(), cloud, _model.localTransform());
                    if(_depthFromScanFillHoles!=0)
                    {
                        util3d::fillProjectedCloudHoles(depthFromScan, _depthFromScanFillHoles>0, _depthFromScanFillHolesFromBorder);
                    }
                }
            }
            // filter the scan after registration
            int previousSize = (int)cloud->size();
            if(_scanDownsampleStep > 1 && cloud->size())
            {
                cloud = util3d::downsample(cloud, _scanDownsampleStep);
                UDEBUG("Downsampling scan (step=%d): %d -> %d", _scanDownsampleStep, previousSize, (int)cloud->size());
            }
            previousSize = (int)cloud->size();
            if(_scanVoxelSize > 0.0f && cloud->size())
            {
                cloud = util3d::voxelize(cloud, _scanVoxelSize);
                UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d", _scanVoxelSize, previousSize, (int)cloud->size());
            }
            if(_scanNormalsK > 0 && cloud->size())
            {
                pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, _scanNormalsK);
                pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
                pcl::concatenateFields(*cloud, *normals, *cloudNormals);
                scan = util3d::laserScanFromPointCloud(*cloudNormals, _scanLocalTransform.inverse());
            }
            else
            {
                scan = util3d::laserScanFromPointCloud(*cloud, _scanLocalTransform.inverse());
            }
        }
    }
    else
    {
        UWARN("Directory is not set, camera must be initialized.");
    }

    if(_model.imageHeight() == 0 || _model.imageWidth() == 0)
    {
        _model.setImageSize(img.size());
    }

    SensorData data(scan, LaserScanInfo(scan.empty()?0:_scanMaxPts, 0, _scanLocalTransform), _isDepth?cv::Mat():img, _isDepth?img:depthFromScan, _model, this->getNextSeqID(), stamp);

    data.setGroundTruth(groundTruthPose);

    return data;
}



/////////////////////////
// CameraVideo
/////////////////////////
CameraVideo::CameraVideo(
		int usbDevice,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_rectifyImages(rectifyImages),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

CameraVideo::CameraVideo(
		const std::string & filePath,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_filePath(filePath),
	_rectifyImages(rectifyImages),
	_src(kVideoFile),
	_usbDevice(0)
{
}

CameraVideo::~CameraVideo()
{
	_capture.release();
}

bool CameraVideo::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_guid = cameraName;
	if(_capture.isOpened())
	{
		_capture.release();
	}

	if(_src == kUsbDevice)
	{
		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d", _usbDevice);
		_capture.open(_usbDevice);
	}
	else if(_src == kVideoFile)
	{
		ULOGGER_DEBUG("Camera: filename=\"%s\"", _filePath.c_str());
		_capture.open(_filePath.c_str());
	}
	else
	{
		ULOGGER_ERROR("Camera: Unknown source...");
	}
	if(!_capture.isOpened())
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	else
	{
		if (_guid.empty())
		{
			unsigned int guid = (unsigned int)_capture.get(CV_CAP_PROP_GUID);
			if (guid != 0 && guid != 0xffffffff)
			{
				_guid = uFormat("%08x", guid);
			}
		}

		// look for calibration files
		if(!calibrationFolder.empty() && !_guid.empty())
		{
			if(!_model.load(calibrationFolder, _guid))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
						_guid.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
						_model.fx(),
						_model.fy(),
						_model.cx(),
						_model.cy());
			}
		}
		_model.setLocalTransform(this->getLocalTransform());
		if(_rectifyImages && !_model.isValidForRectification())
		{
			UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
			return false;
		}
	}
	return true;
}

bool CameraVideo::isCalibrated() const
{
	return _model.isValidForProjection();
}

std::string CameraVideo::getSerial() const
{
	return _guid;
}

SensorData CameraVideo::captureImage(CameraInfo * info)
{
	cv::Mat img;
	if(_capture.isOpened())
	{
		if(_capture.read(img))
		{
			if(_model.imageHeight() == 0 || _model.imageWidth() == 0)
			{
				_model.setImageSize(img.size());
			}

			if(_model.isValidForRectification() && _rectifyImages)
			{
				img = _model.rectifyImage(img);
			}
			else
			{
				// clone required
				img = img.clone();
			}
		}
		else if(_usbDevice)
		{
			UERROR("Camera has been disconnected!");
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	return SensorData(img, _model, this->getNextSeqID(), UTimer::now());
}

} // namespace rtabmap
