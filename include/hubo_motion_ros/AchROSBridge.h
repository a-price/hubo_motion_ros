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
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
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

template <class DataClass>
class AchROSBridge
{
public:
	/// Constructor
	AchROSBridge(std::string chanName);

	/// Destructor
	~AchROSBridge();

	/// Reads new data from the ach channel
	ach_status updateState();

	/// Writes new data to the ach channel
	ach_status pushState(const DataClass& data);

	/// Returns the data with an option to update first
	const DataClass& getState(bool update = true);

protected:
	ach_channel_t mAchChan;
	std::string mAchChanName;
	DataClass mAchData;
};


template <class DataClass>
AchROSBridge<DataClass>::AchROSBridge(std::string chanName)
{
	if (chanName == "")
	{
		fprintf(stderr, "\nInvalid Ach channel name specified.\n");
	}

	mAchChanName = chanName;

	ach_status r = ACH_OK;
	r = ach_create(mAchChanName.c_str(), 10, sizeof(mAchData), NULL);

	if (r == ACH_EEXIST)
	{
		fprintf(stderr, "\nFound existing state channel '%s'.\n",
			mAchChanName.c_str());
	}
	else if( ACH_OK != r )
	{
		fprintf(stderr, "\nUnable to create state channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}

	memset(&mAchData, 0, sizeof(mAchData));
	r = ach_open( &mAchChan, mAchChanName.c_str(), NULL );

	if( ACH_OK != r )
	{
		fprintf(stderr, "\nUnable to open state channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}
}

template <class DataClass>
AchROSBridge<DataClass>::~AchROSBridge()
{
	ach_close(&mAchChan);
}

template <class DataClass>
ach_status AchROSBridge<DataClass>::updateState()
{
	ach_status r = ACH_OK;
	size_t fs = 0;
	r = ach_get( &mAchChan, &mAchData, sizeof(mAchData), &fs, NULL, ACH_O_LAST );

	// Need some error checking here... fs always = 0
//	if (fs != sizeof(mAchData))
//	{
//		fprintf(stderr, "\nProblem reading Ach channel '%s'; data size: %zi != bytes read: %zi. Please check your data struct definitions.\n",
//			mAchChanName.c_str(),sizeof(mAchData),fs);
//	}

	if( ACH_OK != r && ACH_STALE_FRAMES != r)
	{
		fprintf(stderr, "\nProblem reading Ach channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}

	return r;
}

template <class DataClass>
ach_status AchROSBridge<DataClass>::pushState(const DataClass& data)
{
	ach_status r = ACH_OK;

	mAchData = data;
	r = ach_put(&mAchChan, &mAchData, sizeof(mAchData));

	if( ACH_OK != r )
	{
		fprintf(stderr, "\nProblem reading Ach channel '%s', error: (%d) %s\n",
			mAchChanName.c_str(), r, ach_result_to_string((ach_status_t)r));
	}

	return r;
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

#endif /* ACHROSBRIDGE_H_ */
