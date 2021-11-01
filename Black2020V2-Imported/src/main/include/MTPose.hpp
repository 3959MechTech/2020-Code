	/*
 * Pose2D.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Cloie and Cole
 */

#pragma once

#include <mutex>
#include <shared_mutex>

struct MTPoseData{
	double x,y,phi;
	double vx, vy, w;
	double ax, ay, aa;
	double timestamp;
};

class MTPose
{

private:
	MTPoseData _data;
	std::shared_timed_mutex _mutex;


public:

	 MTPose()
	 {
		_data.x = 0;
		_data.y = 0;
		_data.phi = 0;
		_data.vx = 0;
		_data.vy = 0;
		_data.w = 0;
		_data.ax = 0;
		_data.ay = 0;
		_data.aa = 0;
		_data.timestamp = frc::GetTime();
	 };

	void Update(MTPoseData newData)
	{
		//std::unique_lock<std::shared_timed_mutex> lock(_mutex);
		_mutex.lock();
		_data = newData;
		_mutex.unlock();
	};

	MTPoseData Get()
	{
		//std::shared_lock<std::shared_timed_mutex> lock(_mutex);
		MTPoseData temp;
		_mutex.lock_shared();
		temp = _data;
		_mutex.unlock_shared();
		return _data;
	};

};


