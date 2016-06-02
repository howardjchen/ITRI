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
	int v,w;
	char Dispatch[6] = "f0R0L";
	while (true)
	{
		cout << "Send command to Mobile!!(ex:f3R5L),or type 'quit' to leave)" << endl;
		cout << "please enter LinearVelocity" << endl;
		cin >> v;
		cout << "please enter AngularVelocity" << endl;
		cin >> w;

		sprintf(MoveCommand,"f%dR%dL",v,v);

		//cin >> MoveCommand;
		if (strcmp(MoveCommand, Dispatch) == 0)
			break;
		else
			Mobile->SendCommand((unsigned char *)MoveCommand, 5 );
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
