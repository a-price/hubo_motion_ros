/**
 *
 * \file AchMonitor.h
 * \brief 
 *
 * \author Andrew Price
 * \date May 23, 2013
 *
 * \copyright
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ACHMONITOR_H_
#define ACHMONITOR_H_

#include "hubo_motion_ros/AchROSBridge.h"

template <class DataClass>
class AchMonitor : AchROSBridge<DataClass>
{
public:
	AchMonitor(std::string chanName) : AchROSBridge<DataClass>(chanName){};
	virtual ~AchMonitor();

	/// Reads new data from the ach channel
	virtual ach_status_t updateState();

	/// Waits for new data from the ach channel
	virtual const DataClass&  waitState(const uint32_t millis);
};

//template <class DataClass>
//AchMonitor<DataClass>::AchMonitor(std::string chanName):
//	AchROSBridge(chanName)
//{
//}


template <class DataClass>
AchMonitor<DataClass>::~AchMonitor()
{
	ach_status_t r = ACH_OK;
	r = ach_close(&this->mAchChannel.mAchChan);
}


template <class DataClass>
ach_status_t AchMonitor<DataClass>::updateState()
{
	ach_status_t r = ACH_OK;
	size_t fs = 0;
	r = ach_get( &this->mAchChannel.mAchChan, &this->mAchData, sizeof(this->mAchData), &fs, NULL, ACH_O_LAST );

	if (ACH_STALE_FRAMES != r)
	{
		if (fs != sizeof(this->mAchData))
		{
			ROS_ERROR("Problem reading Ach channel '%s'; data size: %zi != bytes read: %zi. Please check your data struct definitions.",
				this->mAchChannel.mAchChanName.c_str(),sizeof(this->mAchData),fs);
		}

		if (ACH_MISSED_FRAME != r)
		{
			// Potentially do something here, but it's not currently a problem...
		}
		else if(ACH_OK != r)
		{
			ROS_ERROR("Problem reading Ach channel '%s', error: (%d) %s",
				this->mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
		}
	}

	if(ACH_OK == r)
	{
		ROS_INFO_STREAM("[ACH][" << this->mAchChannel.mAchChanName << "] New Data:\n" << this->mAchData << "\n");
	}

	return r;
}

template <class DataClass>
const DataClass& AchMonitor<DataClass>::waitState(const uint32_t millis)
{
	struct timespec waitTime;
	clock_gettime(ACH_DEFAULT_CLOCK, &waitTime);
	waitTime.tv_nsec += millis * 1000 * 1000;
	ach_status_t r = ACH_OK;
	size_t fs = 0;
	r = ach_get( &this->mAchChannel.mAchChan, &this->mAchData, sizeof(this->mAchData), &fs, &waitTime, ACH_O_WAIT );

	if (r == ACH_TIMEOUT)
	{
		ROS_WARN("[ACH][%s] Request timed out.",
			this->mAchChannel.mAchChanName.c_str());
	}
	else if(ACH_STALE_FRAMES == r)
	{
		ROS_WARN("[ACH][%s] Request timed out.",
			this->mAchChannel.mAchChanName.c_str());
	}
	else if(ACH_OK == r)
	{
		ROS_INFO_STREAM("[ACH][" << this->mAchChannel.mAchChanName << "] New Data:\n" << this->mAchData << "\n");
	}
	else
	{
		ROS_ERROR("[ACH][%s] Problem reading channel, error: (%d) %s",
			this->mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
	return this->mAchData;
}

#endif /* ACHMONITOR_H_ */
