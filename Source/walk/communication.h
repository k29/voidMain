#ifndef _COMM_H_
#define _COMM_H_

#include <ftdi.h>
#include "commondefswalk.h"

//TODO: implement sleep/timing after or before writing/reading data

class Communication
{
private:
	static const int BROADCAST = 0xfe;
	static const char ftdiID1[];
	static const char ftdiID2[];
	static const int BAUD = 1000000;
	static const int SYNCPACKET_MAXSIZE = 2000;
	struct ftdi_context bodyFTDI1;
	struct ftdi_context bodyFTDI2;
	byte syncPacket[SYNCPACKET_MAXSIZE];
	int syncPacketSize;
public:

	int checksum(byte packet[]);
	int motorSend(int instruction, int dataLength, byte data[], int id);
	int motorReceive(byte data[], int tries=1);
	int addSyncWrite(int location, int dataLength, byte data[], int id);
	int syncFlush();
	int getBaudrate();
	Communication();	
	~Communication();
};

#endif
