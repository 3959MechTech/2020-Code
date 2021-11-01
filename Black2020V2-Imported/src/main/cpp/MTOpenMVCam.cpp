/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MTOpenMVCam.h"

MTOpenMVCam::MTOpenMVCam(double period) 
{
	_thread = new frc::Notifier(&MTOpenMVCam::read, this);
	_port.SetReadBufferSize(MessageLength);
	_port.SetTimeout(.004);
}

void MTOpenMVCam::read()
{
	int recvd = _port.GetBytesReceived();
	if(recvd/MessageLength > 1)
	{
		int bytes2Read = recvd - MessageLength - recvd%MessageLength;
		char olddata[bytes2Read];
		_port.Read(olddata, bytes2Read);
	}

	char data[MessageLength];
	recvd = _port.Read(data, MessageLength);
	
	if(recvd < MessageLength)
	{
		return;
	}

	MTOpenMVData newData;
	newData.isNewData = true;
	if(data[MessageDef::kTargetFound]==1)
	{
		newData.targetFound = true;
	}else
	{
		newData.targetFound = false;
	}
	/*
	newData.x       = (int)((data[MessageDef::kXCenterUpper]<<8) | data[MessageDef::kXCenterLower]);
	newData.y       = (int)((data[MessageDef::kYCenterUpper]<<8) | data[MessageDef::kYCenterLower]);
	newData.Width   = (data[MessageDef::kWidthUpper]<<8)   | data[MessageDef::kWidthLower];
	newData.Height  = (data[MessageDef::kHeightUpper]<<8)  | data[MessageDef::kHeightLower];
	newData.Angle   = data[MessageDef::kAngle];
	*/
}

MTOpenMVData MTOpenMVCam::Get()
{

}
