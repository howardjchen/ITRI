#include "MobileControl.h"

MobileControl *MobileControl::m_pThis = 0;

MobileControl::MobileControl()
{
	time = 0;
	m_pThis = this;
	Mobile_Robot = new Tserial_event();

	IsProcessOver = false;
	memset(EncoderData, 0, sizeof(EncoderData));

	diff_t = 0;

	LinearVelocity = 0;
	AngularVelocity = 0;
	RightVelocity = 0;
	LeftVelocity = 0;

	StoreState = true;
	LastRIGHT = 0;
	LastLEFT = 0;
	cal = 0;
}

MobileControl::~MobileControl()
{
	delete Mobile_Robot;
}

void MobileControl::ConnectMobile()
{
	if(Mobile_Robot != 0)
	{
		Mobile_Robot->setManager(SerialEventManager);
		int erreur = Mobile_Robot->connect("COM7", 115200, SERIAL_PARITY_NONE, 8, true);
		if ( !erreur )
		{
			Mobile_Robot->setRxSize(1);	
		}
		else
		{
			cout << "erreur=" << erreur << endl;
			system("pause");
			exit(1);
		}
	}
}

void MobileControl::DisconnectMobile()
{
	Mobile_Robot->disconnect();
}

void MobileControl::SendCommand(unsigned char *command,int size)
{
	//if(Mobile_Robot == 0)
		Mobile_Robot->sendData(command,size);
	//else
		//cout << "Connect Mobile first!!" << endl;
}

void MobileControl::SerialEventManager(uint32 object, uint32 event)
{
	unsigned char *buffer;
	int   size;
	Tserial_event *com;

	com = (Tserial_event *)object;
	if (com != 0)
	{
		switch (event)
		{
			case  SERIAL_CONNECTED:
				printf("Connected ! \n");
				break;
			case  SERIAL_DISCONNECTED:
				printf("Disonnected ! \n");
				break;
			case  SERIAL_DATA_SENT:
				printf("Data sent ! \n");
				break;
			case  SERIAL_RING:
				printf("DRING ! \n");
				break;
			case  SERIAL_CD_ON:
				printf("Carrier Detected ! \n");
				break;
			case  SERIAL_CD_OFF:
				printf("No more carrier ! \n");
				break;
			case  SERIAL_DATA_ARRIVAL:
				size = com->getDataInSize();
				buffer = com->getDataInBuffer();
				com->dataHasBeenRead();
				m_pThis->DataArrival(size, buffer);
				//printf("data arrival\n");
				break;
		}
	}
}

void MobileControl::DataArrival(int size, unsigned char *buffer)
{
	unsigned char recv_tmp = buffer[0];

	if (buffer[0] == 0x0D)	//end of data
	{
		DataProcess();
	}
	else
	{
		encoder_recv.push_back(recv_tmp);
	}

	if (IsProcessOver)
	{
		double start;
		start = clock();
		if (cal > 5)
		{
			ComputeVelosity(start,EncoderData[RIGHT],EncoderData[LEFT]);
			printf("V = %f  W = %f  \n",LinearVelocity,AngularVelocity);
			cal = 0;
		}
		else
			cal++;

		int AccumulateDistance = m_pThis->Odometer(EncoderData[RIGHT],EncoderData[LEFT]);
		printf("AccumulateDistance = %d \n",AccumulateDistance );


		IsProcessOver = false;
		encoder_recv.clear();
	}

}

int MobileControl::Odometer(int RightEncoder, int LeftEncoder)
{

	int DiffEncoder = LeftEncoder - RightEncoder;
	int AccumulateDistance = (RightEncoder+LeftEncoder)/2;
	char StopCommand[2] = "s";
	char CleanCommand[2] = "c";

	if (AccumulateDistance > 1000)
	{
		Mobile_Robot->sendData((unsigned char *)StopCommand,1);
		Mobile_Robot->sendData((unsigned char *)CleanCommand,1);
	}
	return AccumulateDistance;


	/*if (DiffEncoder > 510 && flagg == 0)
	{
		char StopCommand[2] = "s";
		Mobile_Robot->sendData((unsigned char *)StopCommand,1);
		char CleanCommand[2] = "c";
		Mobile_Robot->sendData((unsigned char *)CleanCommand,1);
		flagg = 1;
	}*/

}

int MobileControl::HextoDec(char* hex, int len)
{
	int ret = 0;
	for (int i = 0; i < len; i++) // for each character
	{
		if (hex[i] >= 'A') //if character is ABCDEF
			hex[i] = (char)(hex[i] - 'A' + '\n'); //Minus A for offset + 10 (new line)
		else
			hex[i] = (char)(hex[i] - '0'); //Subtract zero for translation
		ret = (ret << 4) | hex[i]; //shift for every character
	}
	return ret;
}

void MobileControl::DataProcess()
{
	int Data_Length = encoder_recv.size();
	int SPACE = 0x20;
	bool ChangToLeft = false;
	int RightEncoderLength = 0;
	int LeftEncoderLength = 0;


	for (int i = 0; i < Data_Length; ++i)
	{
		if (encoder_recv[i] == SPACE)
		{
			char *tmp_arr_r = new char[RightEncoderLength];
			for (int j = 0; j < RightEncoderLength; ++j)
			{
				tmp_arr_r[j] = encoder_recv[j];
			}
			//EncoderData[RIGHT] = HextoDec(tmp_arr_r,RightEncoderLength);
			EncoderData[RIGHT] = atoi(tmp_arr_r);
			ChangToLeft = true;
		}

		else if (ChangToLeft == false)
		{
			RightEncoderLength++;
		}
		else if(ChangToLeft == true)
		{
			LeftEncoderLength++;
		}
	} 

	//printf("left = %d\n",LeftEncoderLength );
	//printf("right = %d\n",RightEncoderLength );
	//printf("data_length = \n", Data_Length);

	int a = 0;
	char *tmp_arr_l = new char[LeftEncoderLength];
	for (int j = RightEncoderLength+1; j < Data_Length ; ++j)
	{
		tmp_arr_l[a] = encoder_recv[j];
		a++;
	}
	EncoderData[LEFT] = atoi(tmp_arr_l);
	//EncoderData[LEFT] = HextoDec(tmp_arr_l,LeftEncoderLength);
	

	ChangToLeft = false;
	encoder_recv.clear();
	IsProcessOver = true;
}

void MobileControl::ComputeVelosity(double new_time, int EncoderRIGHT, int EncoderLEFT )
{
	int DiffRIGHT;
	int DiffLEFT;
	double DiffTIME;

	if(StoreState == true)
	{
		LastRIGHT = EncoderRIGHT;
		LastLEFT = EncoderLEFT;
		LastTIME = new_time;
		StoreState = false;
	}
	else
	{
		DiffRIGHT = EncoderRIGHT - LastRIGHT;
		DiffLEFT = EncoderLEFT - LastLEFT;
		DiffTIME = (new_time - LastTIME)/CLOCKS_PER_SEC;
		time = DiffTIME;

		//if (DiffRIGHT == 0 && DiffLEFT == 0)
		//{
			RightVelocity = DiffRIGHT/float(DiffTIME);
			LeftVelocity = DiffLEFT/float(DiffTIME);
			LinearVelocity = (RightVelocity + LeftVelocity)/2;
			AngularVelocity = (LeftVelocity - RightVelocity) / WHEELDISTANT;
			
		//}
		/*else
		{
			RightVelocity = DiffRIGHT/DiffTIME;
			LeftVelocity = DiffLEFT/DiffTIME;
			LinearVelocity = (RightVelocity + LeftVelocity)/2;
		}*/
	}

	LastRIGHT = EncoderRIGHT;
	LastLEFT = EncoderLEFT;
	LastTIME = new_time;
}