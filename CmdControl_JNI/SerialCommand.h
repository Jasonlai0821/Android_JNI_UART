/*
 * SerialCommand.h
 *
 *  Created on: 2017-11-29
 *      Author: Administrator
 */

#ifndef SERIALCOMMAND_H_
#define SERIALCOMMAND_H_
#include <utils/RefBase.h>
//#include <utils/StrongPointer.h>
#include <utils/Mutex.h>
#include "MyUtils.h"
using namespace android;
namespace cmdcontrol {

#define COM0       0
#define COM1       1
#define COM2       2
#define COM3       3

#define INVALID_HANDLE_VALUE	-1
#define VOLUME_NUM 7
#define DEFAULT_COMM_PORT "/dev/ttyS4"
//#define DEFAULT_COMM_PORT "/dev/ttyHSL1"

#define MAX_PACKET_LEN (1*1024)

//
#define NOW_COM 	COM0

#define W_TIMEOUTMS	400		// millisecond
#define R_TIMEOUTMS	500		// millisecond
#define B_TIMEOUTMS	1000    // (millisecond)the maxinum of response time after 'write'
#define MAX_WRITE_BUF_LEN 256

#define MAXlength_serwritedata	MAX_WRITE_BUF_LEN
#define MINlength_serwritedata	4


#define			BC_LEADCODE		0
#define			BC_LEHGTH		1
#define 		BC_CMD			2
#define 		BC_CMD_TYPE		3


#define 		M_LCHEAD		0x5A
#define 		S_LCHEAD		0x5B


#define 		CMD_MASTER_AIR			0x20
#define 		CMD_MASTER_ADLIGHT		0x21
#define 		CMD_MASTER_HEADLAMP		0x22
#define 		CMD_MASTER_EXHAUSTFAN	0x23
#define 		CMD_MASTER_COOLINGFAN	0x24
#define 		CMD_MASTER_TOUCHSCREEN	0x25
#define 		CMD_MASTER_TV			0x26
#define 		CMD_MASTER_RKE			0x27
#define 		CMD_MASTER_SMOKEALARM	0x28
#define 		CMD_MASTER_HBS			0x29
#define 		CMD_MASTER_TS			0x2A
#define 		CMD_MASTER_HS			0x2B


#define			CMD_SLAVE_RESET			0x2C
#define			CMD_SLAVE_RESULT		0x2D


typedef enum {
	SERCMD_unknown = 0,
	SERCMD_md_setparameter = 0x0A,
	SERCMD_md_getparameter = 0x0B,
	SERCMD_sd_response	   = 0x0C
} ser_cmd_e;


typedef struct {
	ser_cmd_e ser_cmd_type;
	unsigned char ser_cmd_data;
	int value;
} ser_data_s;

class SerialCommand: virtual public RefBase {
public:
	SerialCommand();
	virtual ~SerialCommand();
	int iniSer();
	int deInitSer();
	int onExcuteCmd(const char * cmd);
	void setListener(const sp<CmdControlListener>& listener);
	sp<CmdControlListener> getListener() {
		return mListener;
	}

	//static function as tools
	static int SerOpen(int PortNo);

	static int SerWrite_SlaveDevice(int PortNo, const char *w_pBuf,unsigned long TimeOut,int len);
	
	static void SerClear(int PortNo);

	static int SerWrite(int PortNo, unsigned char *pszBuf, unsigned int SendCnt,
			unsigned long TimeOut);

	static void SerClose(int PortNo);
	
	static int SerRead(int PortNo, unsigned char *pszBuf, int ReadCnt,
			unsigned long TimeOutMS);
	
	static int CheckIsValid(const unsigned char *w_pBuf,int len);

	static int StringtoHex(const char *w_pBuf,int len,unsigned char* des_Buf);

	static int getCRCValue(unsigned char* des_Buf,int len);

	static int SerParse_Reponse(const unsigned char *r_pBuf,int len,ser_data_s *pBuf);
	
	static sp<SerialCommand> getInstance();
private:
	SerialCommand(const SerialCommand&);
	SerialCommand& operator=(const SerialCommand&);
	static SerialCommand* mInstance;
	sp<CmdControlListener> mListener;
	bool mSerInitialized;
};

}

#endif /* SERIALCOMMAND_H_ */
