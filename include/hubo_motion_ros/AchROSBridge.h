/**
 * \file AchROSBridge.h
 * \brief Provides common functionality for creating and retrieving data to and from Ach channels.
 *
 * \author Andrew Price
 * \date May 20, 2013
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

#ifndef ACHROSBRIDGE_H_
#define ACHROSBRIDGE_H_

#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ach.h>
#include <string>

#include <mutex>
#include <csignal> // TODO: Handle kill signals when listening for ach_o_wait
#include <cstdlib>

#include <ros/ros.h>

class AchChannel
{
public:
	std::mutex mAchLock;
	ach_channel_t mAchChan;
	std::string mAchChanName;

	ach_status_t cancelRequest()
	{
        ach_status_t r = ACH_OK;

//		ach_cancel_attr_t cancel_attrs;
//		ach_cancel_attr_init(&cancel_attrs);
//		r = ach_cancel(&mAchChan, &cancel_attrs);

		return r;
	}
};

volatile bool shutdownRequested = false;

std::set<AchChannel*> lockedChannels;

void ach_safe_shutdown(int parameter)
{
	fprintf(stderr, "Requested Shutdown.\n");
	bool notBusy = true;
	for(std::set<AchChannel*>::iterator i = lockedChannels.begin();
		i != lockedChannels.end(); i++)
	{
		AchChannel* channel = *i;
		notBusy = notBusy && channel->mAchLock.try_lock();
	}

	if (notBusy)
	{
		fprintf(stderr, "Shutting down node.\n");
		shutdownRequested = true;
		//mAchLocked.unlock();
		ros::shutdown();
	}
	else if (shutdownRequested)
	{
		fprintf(stderr, "Hard Exit.\n");
		exit(0);
	}
	else
	{
        fprintf(stderr, "Busy, setting wait.\n");
		for(std::set<AchChannel*>::iterator i = lockedChannels.begin();
			i != lockedChannels.end(); i++)
		{
			AchChannel* channel = *i;
			channel->cancelRequest();
		}
		shutdownRequested = true;
	}
}

#define ACH_ATOMIC_ACCESS(f) \
	this->mAchChannel.mAchLock.lock(); \
	lockedChannels.insert(&this->mAchChannel); \
	{f} \
	lockedChannels.erase(&this->mAchChannel); \
	this->mAchChannel.mAchLock.unlock(); \
	if(shutdownRequested){ros::shutdown();}

template <class DataClass>
class AchROSBridge
{
public:
	//static std::mutex mAchLocked;
	//static bool shutdownRequested = false;

	/// Constructor
	AchROSBridge(std::string chanName);

	/// Destructor
	virtual ~AchROSBridge();

	/// Writes new data to the ach channel
	virtual ach_status_t pushState(const DataClass& data);

	/// Reads new data from the ach channel
	virtual ach_status_t updateState();

	/// Waits for new data from the ach channel
	virtual const DataClass&  waitState(const uint32_t millis);

	/// Returns the data with an option to update first
	virtual const DataClass& getState(bool update = true);

	/// Cancels any put or get commands to the channel.
	virtual ach_status_t cancelRequest();

protected:
//	ach_channel_t mAchChan;
//	std::string mAchChanName;
	AchChannel mAchChannel;
	DataClass mAchData;
};


template <class DataClass>
AchROSBridge<DataClass>::AchROSBridge(std::string chanName)
{
	// Set up signal handler so we don't bail in the middle of an ach read/write
	signal(SIGINT, ach_safe_shutdown);
	shutdownRequested = false;

	if (chanName == "")
	{
		ROS_ERROR("Invalid Ach channel name specified.");
	}

	mAchChannel.mAchChanName = chanName;

	ach_status_t r = ACH_OK;
	r = ach_create(mAchChannel.mAchChanName.c_str(), 10, sizeof(mAchData), NULL);

	if (r == ACH_EEXIST)
	{
		ROS_INFO("Found existing Ach channel '%s'.",
			mAchChannel.mAchChanName.c_str());
	}
	else if( ACH_OK != r )
	{
		ROS_ERROR("Unable to create Ach channel '%s', error: (%d) %s",
			mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
	else
	{
		ROS_INFO("Created new Ach channel '%s'.",
			mAchChannel.mAchChanName.c_str());
	}

	//r = ach_chmod(&mAchChan, 666);

//	if( ACH_OK != r )
//	{
//		ROS_WARN("Unable to modify Ach channel '%s', error: (%d) %s",
//			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
//	}


	memset(&mAchData, 0, sizeof(mAchData));
	r = ach_open( &mAchChannel.mAchChan, mAchChannel.mAchChanName.c_str(), NULL );

	if( ACH_OK != r )
	{
		ROS_ERROR("Unable to open Ach channel '%s', error: (%d) %s",
			mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}

}

template <class DataClass>
AchROSBridge<DataClass>::~AchROSBridge()
{
	ach_status_t r = ACH_OK;
	r = ach_close(&mAchChannel.mAchChan);
	//r = ach_unlink(mAchChanName.c_str());
}


template <class DataClass>
ach_status_t AchROSBridge<DataClass>::pushState(const DataClass& data)
{
	ach_status_t r = ACH_OK;

	mAchData = data;
	ACH_ATOMIC_ACCESS(r = ach_put(&mAchChannel.mAchChan, &mAchData, sizeof(mAchData)););

	if( ACH_OK != r )
	{
		ROS_ERROR("Problem writing Ach channel '%s', error: (%d) %s",
			mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}

	return r;
}

template <class DataClass>
ach_status_t AchROSBridge<DataClass>::updateState()
{
	ach_status_t r = ACH_OK;
	size_t fs = 0;
	ACH_ATOMIC_ACCESS(r = ach_get( &mAchChannel.mAchChan, &mAchData, sizeof(mAchData), &fs, NULL, ACH_O_LAST ););

	if (ACH_STALE_FRAMES != r)
	{
		if (fs != sizeof(mAchData))
		{
			ROS_ERROR("Problem reading Ach channel '%s'; data size: %zi != bytes read: %zi. Please check your data struct definitions.",
				mAchChannel.mAchChanName.c_str(),sizeof(mAchData),fs);
		}

		if (ACH_MISSED_FRAME == r)
		{
			// Potentially do something here, but it's not currently a problem...
		}
		else if(ACH_OK != r)
		{
			ROS_ERROR("Problem reading Ach channel '%s', error: (%d) %s",
				mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
		}
	}

	return r;
}

template <class DataClass>
const DataClass& AchROSBridge<DataClass>::waitState(const uint32_t millis)
{
	ROS_INFO("Waiting for ach channel %s.", mAchChannel.mAchChanName.c_str());
	struct timespec waitTime;
	clock_gettime(ACH_DEFAULT_CLOCK, &waitTime);
	waitTime.tv_nsec += millis * 1000 * 1000;
	ach_status_t r = ACH_OK;
	size_t fs = 0;
	ACH_ATOMIC_ACCESS(r = ach_get( &mAchChannel.mAchChan, &mAchData, sizeof(mAchData), &fs, &waitTime, ACH_O_WAIT | ACH_O_LAST ););

	if (r == ACH_TIMEOUT)
	{
		ROS_INFO("Ach request timed out on channel '%s'.",
			mAchChannel.mAchChanName.c_str());
	}
	else if (r != ACH_OK)
	{
		ROS_ERROR("Problem reading Ach channel '%s', error: (%d) %s",
			mAchChannel.mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
    else
    {
        ROS_INFO("New ach data on channel '%s'.",
            mAchChannel.mAchChanName.c_str());
    }
	return mAchData;
}


template <class DataClass>
const DataClass& AchROSBridge<DataClass>::getState(bool update)
{
	if (update)
	{
		updateState();
	}

	return mAchData;
}

template <class DataClass>
ach_status_t AchROSBridge<DataClass>::cancelRequest()
{
	ach_status_t r = ACH_OK;
	r = mAchChannel.cancelRequest();

	return r;
}

#endif /* ACHROSBRIDGE_H_ */
