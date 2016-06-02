#ifndef MOBILECONTROL_H
#define MOBILECONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <iostream>
#include <cstdlib>

#include <time.h>

#include "tserial.h"
#include "Tserial_event.h"

#define YES 0
#define END_OF_DATA 0x0D
#define RIGHT 0
#define LEFT 1
#define WHEELDISTANT 33

#define PHYSICALDISTANCE 4000

using namespace std;

class MobileControl
{
private:
public:
	static MobileControl *m_pThis;
	MobileControl();
	~MobileControl();

	Tserial_event *Mobile_Robot;
	void ConnectMobile();
	void DisconnectMobile();
	void SendCommand(unsigned char *command,int size);
	static void SerialEventManager(uint32 object, uint32 event);
	void DataArrival(int size, unsigned char *buffer);
	int Odometer();
	vector<char> encoder_recv;
	bool IsProcessOver;
	int EncoderData[2];

	time_t start_t, end_t;
	double diff_t;
	double time;

	float LinearVelocity;
	float AngularVelocity;
	float RightVelocity;
	float LeftVelocity;
		
	bool StoreState;
	int LastRIGHT;
	int LastLEFT;
	double LastTIME;
	int cal;

	int HextoDec(char* hex, int len);
	void DataProcess();
	void ComputeVelosity(double new_time, int EncoderRIGHT, int EncoderLEFT );

};

#endif MOBILECONTROL_H