#include "LaserScanner.h"
#include "MobileControl.h"
#include "bot_control.h"
#include "tserial.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <iostream>
#include <string.h>
#include <time.h>
using namespace std;

#define STRAIGHT1000 0
#define STRAIGHT4000 1
#define TUENRIGHT90  2
#define TURNLEFT90   3


#include <cv.h>
#include <highgui.h>
using namespace cv;

LaserScanner *laserModule;
MobileControl *Mobile;

void updateFormThread()
{
	clock_t diff_start, diff_end, diff_time;
	diff_start = clock();
	diff_end = clock();
	while (true)
	{
		diff_end = clock();
		diff_time = diff_end - diff_start;
		diff_start = clock();

		laserModule->updateAndInputEKF(Mobile->time, Mobile->LinearVelocity, M_PI/2, Mobile->AngularVelocity );
		//cout << "hello" << endl;
		Sleep(100);
		cout << "x:" << laserModule->robotState.x << " y" << laserModule->robotState.y << " t:" << laserModule->robotState.theta <<endl;
	}

}

int main(int argc, char *argv[])
{

	
	cout << "Start connect Mobile!!" << endl;
	system("pause");
	cout << endl;

	Mobile = new MobileControl();
	Mobile->ConnectMobile();

	/*cout << "Start connect Laser!!" << endl;
	system("pause");
	cout << endl;
	laserModule = new LaserScanner();
	laserModule->connectLaserDevice();

	int *imageUsing = (int*)malloc(sizeof(int));
	*imageUsing = 0;
	laserModule->shakeHand(imageUsing);

	cout << "Start receive laser data!!" << endl;
	system("pause");
	cout << endl;
	laserModule->startReceiveData();

	//creat update localization thread
	DWORD ThreadId;
	HANDLE Thread;
	Thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)updateFormThread, NULL, 0, &ThreadId);

	*/
	char MoveCommand[6];
	char CleanCommand[2] = "c";
	char StopCommand[2] = "s";
	int v = 0, w = 0;
	char Dispatch[6] = "quit";
	int AccumulateDistance = 0;

	cout << "Send command to Mobile!!(ex:f3R5L),or type 'quit' to leave)" << endl;
	cout << "please enter LinearVelocity" << endl;
	cin >> v;
	cout << "please enter AngularVelocity" << endl;
	cin >> w;
	sprintf(MoveCommand,"f%dR%dL",v,v);

	cout << "type 'quit' to leave" << endl;
	while (true)
	{
		AccumulateDistance = Mobile->Odometer();
		if (strcmp(MoveCommand, Dispatch) == 0)  //disconnect
		{
			Mobile->SendCommand((unsigned char *)StopCommand,1);
			Mobile->DisconnectMobile();
			break;
		}
		else if (AccumulateDistance > 4000)    // stop when reach 4m
		{
			Mobile->SendCommand((unsigned char *)StopCommand,1);
			Mobile->SendCommand((unsigned char *)CleanCommand,1);
			Mobile->DisconnectMobile();
			break;
		}
		else
		{
			Mobile->SendCommand((unsigned char *)MoveCommand, 5 );
		}
		cin >> MoveCommand;
	}
	
	cout << "Disconnect all Device!!" << endl;
	system("pause");
	cout << endl;

	//laserModule->DisconnectLaserDevice();
	//TerminateThread(updateFormThread, 0);
	//delete laserModule;

	Mobile->DisconnectMobile();
	delete Mobile;
	cout << "complete!!" << endl;
	return 0;
}
