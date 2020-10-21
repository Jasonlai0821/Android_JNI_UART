/*
 * SerialCommand.cpp
 *
 *  Created on: 2017-12-5
 *      Author: Administrator
 */
#define LOG_TAG "SerialCommand"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <utils/StrongPointer.h>
#include "SerialCommand.h"
#include "MyUtils.h"
namespace cmdcontrol {

static int m_serial_handle;
//static sem_t m_singal;
static pthread_t com_read_tid = (pthread_t)0L;


static pthread_mutex_t m_mutex;
static pthread_cond_t m_cond;

static ser_cmd_e Now_cmd_e;
static ser_data_s S_Read;
static unsigned char FLAG_READ_THREAD = 0; /*0: running ,1: need to exit, 2: exited*/

SerialCommand* SerialCommand::mInstance = NULL;

SerialCommand::SerialCommand() :
		mListener(NULL) {
	LOGD("SerialCommand Constructor");
	mInstance = this;
	mSerInitialized = false;
}

SerialCommand::~SerialCommand() {
	LOGD("SerialCommand Destructor");
	mInstance = NULL;
	mListener = NULL;
	mSerInitialized = false;
}

sp<SerialCommand> SerialCommand::getInstance() {
	if (mInstance != NULL) {
		return mInstance;
	}
	return NULL;
}

void SerialCommand::setListener(const sp<CmdControlListener>& listener) {
	mListener = listener;
}
int SerialCommand::SerOpen(int PortNo) {
	int ret;
	struct termios options;

	// m_serial_handle = open(DEFAULT_COMM_PORT, O_RDWR|O_NOCTTY|O_NDELAY);
	m_serial_handle = open(DEFAULT_COMM_PORT, O_RDWR);	//|O_NOCTTY|O_NDELAY);
	if (m_serial_handle == INVALID_HANDLE_VALUE) {
		LOGE("Open file err:%s errno:%d", DEFAULT_COMM_PORT, errno);
		return -1;
	}

//	
	tcgetattr(m_serial_handle, &options);

	bzero(&options,sizeof(struct termios));
	options.c_lflag = 0;
	options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //rockchip add

	/*data bits*/
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag |= (CLOCAL | CREAD); //rockchip modify
	/*parity*/
	options.c_cflag &= ~PARENB; /* ии?б░??o???ии??ии???гд???????ижa? */	
	options.c_iflag &= ~INPCK; /* ии?б░?бнгд???ии??ии???гд???????ижa? */	/*stop bits*/
	options.c_cflag &= ~CSTOPB;

	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	//cfsetispeed(&options, B460800);
	//cfsetospeed(&options, B460800);

	ret = tcsetattr(m_serial_handle, TCSANOW, &options);
	if (ret == -1) {
		LOGE("SetArrt tcsetattr err: %d", errno);
		return -1;
	}
	LOGV("SerOpen() open %s succeed",DEFAULT_COMM_PORT);
	return 0;
}

void SerialCommand::SerClose(int PortNo) {
	if (m_serial_handle != INVALID_HANDLE_VALUE) {
		close(m_serial_handle);
	}
	m_serial_handle = INVALID_HANDLE_VALUE;
}

void SerialCommand::SerClear(int PortNo) {
	LOGV("SerClear: clear the data of uart....");
	tcflush(m_serial_handle, TCIOFLUSH);
	//LOGV("SerClear: clear end....");
}

int SerialCommand::SerWrite(int PortNo, unsigned char *pszBuf,
		unsigned int SendCnt, unsigned long TimeOutMS) {
	int ret = 0;
	struct timeval CurTime;
	fd_set wwrite;

	FD_ZERO(&wwrite);
	FD_SET(m_serial_handle, &wwrite);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(m_serial_handle + 1, NULL, &wwrite, NULL, &CurTime);
	if (ret == 0) {
		LOGE("SerWrite select timeout");
		return -1;
	} else if (ret < 0) {
		LOGE("SerWrite select error: %d", errno);
		return -1;
	}

	ret = write(m_serial_handle, pszBuf, SendCnt);
	if (ret <= 0) {
		SerialCommand::SerClear(NOW_COM);
		LOGE("SerWrite write buffer error:%d ", errno);
		return -1;
	}
	return 0;
}

int SerialCommand::SerRead(int PortNo, unsigned char *pszBuf, int ReadCnt,
		unsigned long TimeOutMS) {
	int ret = 0;
	int index = 0;
	struct timeval CurTime;
	fd_set wread;

	FD_ZERO(&wread);
	FD_SET(m_serial_handle, &wread);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(m_serial_handle + 1, &wread, NULL, NULL, &CurTime);
	if (ret == 0) {
		return -1;
	} else if (ret < 0) {
		LOGE("SerRead select error: %d", errno);
		return -1;
	}

	ret = read(m_serial_handle, pszBuf, ReadCnt);
	if (ret <= 0) {
		LOGE("SerRead read buffer error:%d ", errno);
		return -1;
	}
	return ret;
}

int SerialCommand::getCRCValue(unsigned char* des_Buf,int len)
{
	int sum = 0;
	int crc = 0;
	for(int i = 0; i < len; i++){
		sum += des_Buf[i];
	}

	crc = sum & 0xff;
	LOGD("getCRCValue() crc:0x%x", crc);
	return crc;
}
		

int SerialCommand::StringtoHex(const char *w_pBuf,int len,unsigned char* des_Buf)
{
	int ret = -1;
	int i =0;
	int fu,su;
	for(i = 0; i < len; i= i +2){
		if(w_pBuf[i] >= '0' && w_pBuf[i] <= '9'){
			fu = w_pBuf[i] - '0';
		}else if(w_pBuf[i] >= 'A' && w_pBuf[i] <= 'F'){
			fu = w_pBuf[i] - 'A' + 10;
		}else if(w_pBuf[i] >= 'a' && w_pBuf[i] <= 'f'){
			fu = w_pBuf[i] - 'a' + 10;
		}

		if(w_pBuf[i+1] >= '0' && w_pBuf[i+1] <= '9'){
			su = w_pBuf[i+1] - '0';
		}else if(w_pBuf[i+1] >= 'A' && w_pBuf[i+1] <= 'F'){
			su = w_pBuf[i+1] - 'A' + 10;
		}else if(w_pBuf[i+1] >= 'a' && w_pBuf[i+1] <= 'f'){
			su = w_pBuf[i+1] - 'a' + 10;
		}
		des_Buf[i/2] =fu * 16 + su;
		//LOGE("StringtoHex() fu:0x%x, su:0x%x", fu,su);
	}

	//for(i = 0; i < len / 2; i++){
	//	LOGE("StringtoHex() desBuf:0x%x, i:%d", des_Buf[i],i);
	//}

	des_Buf[len/2] = getCRCValue(des_Buf,len/2);
	return ret;
}


int SerialCommand::CheckIsValid(const unsigned char * w_pBuf, int len)
{
	unsigned char cmd_temp[MAX_WRITE_BUF_LEN] = {0};
	memcpy(cmd_temp,w_pBuf,len);
	int ret = -1;

	if(M_LCHEAD == cmd_temp[BC_LEADCODE] || S_LCHEAD == cmd_temp[BC_LEADCODE]){
		if(cmd_temp[BC_CMD] >= CMD_MASTER_AIR &&  cmd_temp[BC_CMD] <= CMD_SLAVE_RESULT){
			ret = 0;
		}else{
			ret = -1;
		}
		
	}else{
		ret = -1;
	}

	return ret;
}


int SerialCommand::SerWrite_SlaveDevice(int PortNo, const char *w_pBuf,unsigned long TimeOut,int len) {
	unsigned char buffer[MAX_WRITE_BUF_LEN] = {0};
	int ret;
	int b_len = 0;

	//LOGE("SerWrite_SlaveDevice() CMD len:%d",len);
	
	StringtoHex(w_pBuf,len,buffer);

	b_len = (len / 2) + 1; //1:just for crc value
	if(CheckIsValid(buffer,b_len) < 0){
		LOGE("SerWrite_SlaveDevice() CMD is invalid!!!");
		return -1;
	}

	//for(int i = 0; i <b_len; i++){
	//	LOGE("StringtoHex() buffer:0x%x, i:%d", buffer[i],i);
	//}
	ret = SerWrite(PortNo, buffer,b_len, TimeOut);
	if (ret < 0) {
		LOGE("SerWrite err");
		return -1;
	} else {
		if(buffer[BC_CMD_TYPE] == SERCMD_md_getparameter || buffer[BC_CMD_TYPE] == SERCMD_md_setparameter){
			return buffer[BC_CMD_TYPE];//getparameter should sync the result
		}
		return 0;
	}
}

int SerialCommand::SerParse_Reponse(const unsigned char *r_pBuf,int len,ser_data_s *pBuf)
{
	LOGV("SerParse_Reponse() \n");
	unsigned char cmd_temp[MAX_WRITE_BUF_LEN] = {0};
	memcpy(cmd_temp,r_pBuf,len);

	if((cmd_temp[BC_CMD] >= CMD_MASTER_AIR &&  cmd_temp[BC_CMD] <= CMD_MASTER_HS) && cmd_temp[BC_CMD_TYPE] ==SERCMD_md_getparameter){
		LOGV("SerParse_Reponse() response get cmd result");
		pBuf->ser_cmd_type = SERCMD_md_getparameter;
		pBuf->ser_cmd_data = cmd_temp[BC_CMD];
		pBuf->value = cmd_temp[BC_CMD_TYPE+1];
	}else if(cmd_temp[BC_CMD] == CMD_SLAVE_RESET){
		LOGV("SerParse_Reponse() slave device reset the soc");
		pBuf->ser_cmd_type = SERCMD_sd_response;
		pBuf->ser_cmd_data = cmd_temp[BC_CMD];
	}else if(cmd_temp[BC_CMD] == CMD_SLAVE_RESULT){
		LOGV("SerParse_Reponse() response cmd result");
		pBuf->ser_cmd_type = SERCMD_sd_response;
		pBuf->ser_cmd_data = cmd_temp[BC_CMD];
		pBuf->value = cmd_temp[BC_CMD_TYPE];
	}
	
	return 0;
}


static void Clearser_data_s(ser_data_s *pBuf)
{
	pBuf->ser_cmd_type = SERCMD_unknown;
	pBuf->ser_cmd_data = 0;
	pBuf->value = 0;
}

//#define MAX_PACKET_LEN (32)
static void* thread_read_sercom_fun(void *arg)
{
	int ret = 0;
	int n_read_bytes = 0;
	int n_store_bytes = 0;
	unsigned char pReponse[MAX_PACKET_LEN] = {0};
	ser_data_s ser_data;
	LOGV("thread_read_sercom_fun() IN");
	while (1)// var
	{
		int i = 0;
		int n_trim_byte = 0;
		if (1 == FLAG_READ_THREAD){
			LOGD("thread_read_sercom_fun exit thread");
			break;
		}
		memset(pReponse, 0x0, sizeof(pReponse));
		
		n_read_bytes = SerialCommand::SerRead(NOW_COM, pReponse + n_store_bytes, MAX_PACKET_LEN - n_store_bytes, R_TIMEOUTMS);

		LOGV("thread_read_sercom_fun() n_read_bytes =%d\n",n_read_bytes);
		//for(i = 0; i < n_read_bytes; i++){
		//	LOGV("thread_read_sercom_fun() 0x%2x ",pReponse[n_store_bytes+i]);
		//}
		
		if (n_read_bytes <= 0){
			if (n_store_bytes <= 0){
				n_trim_byte++;
				continue;
			}
		} else {
			n_store_bytes += n_read_bytes;
		}

		if (n_trim_byte < MAX_PACKET_LEN){
			if(n_store_bytes >= MAX_PACKET_LEN)
			{
				n_store_bytes = 0;
				n_trim_byte = 0;
				memset(pReponse, 0x0, MAX_PACKET_LEN);
				SerialCommand::SerClear(NOW_COM);
				Clearser_data_s(&ser_data);
				LOGE("thread_read_sercom_fun: messy code......");
				continue;
			}
		}

		if(SerialCommand::CheckIsValid(pReponse,n_store_bytes) < 0){
			LOGE("SerRead() CMD is invalid!!!");
			continue;
		}else{
			//parse the pReponse
			SerialCommand::SerParse_Reponse(pReponse,n_store_bytes,&ser_data);

			LOGE("thread_read_sercom_fun() Now_cmd_e:%d,ser_cmd_type:%d",Now_cmd_e,ser_data.ser_cmd_type);
			if(Now_cmd_e == SERCMD_md_getparameter){
				if(ser_data.ser_cmd_type == SERCMD_md_getparameter){
					LOGV("SerParse_Reponse() return getparameter result ser_data ser_cmd_data:0x%x,value:0x%x",ser_data.ser_cmd_data,ser_data.value);
					S_Read.value = ser_data.value;
					S_Read.ser_cmd_type = Now_cmd_e;
					pthread_cond_signal(&m_cond);
				}
			}else{
				if(ser_data.ser_cmd_type == SERCMD_sd_response){
					S_Read.value = ser_data.value;
					S_Read.ser_cmd_type = Now_cmd_e;
					sp<CmdControlListener> listener = SerialCommand::getInstance()->getListener();
					LOGV("SerParse_Reponse() ser_data ser_cmd_data:0x%x,value:0x%x",ser_data.ser_cmd_data,ser_data.value);
					if(listener != NULL){
						listener->notify(ser_data.ser_cmd_data,ser_data.ser_cmd_type,ser_data.value);
					}
				}
			}
		}
		
		n_store_bytes = 0;
		n_trim_byte = 0;
		n_read_bytes = 0;
		memset(pReponse, 0x0, MAX_PACKET_LEN);
		SerialCommand::SerClear(NOW_COM);
		Clearser_data_s(&ser_data);
		
		usleep(50);
	}

	FLAG_READ_THREAD = 2;
	
	return (void*) 0;
}

int SerialCommand::iniSer() {
	LOGD("Enter iniSer()");
	if(mSerInitialized)
	{
		LOGE("iniSer():initialized already!!!!!!!");
		return 0;
	}
	
	int ret = 0;
	pthread_mutexattr_t ma;

	ret = SerOpen(NOW_COM);
	if (ret < 0) {
		LOGE("iniSer->Seropen Failed");
		return -1;
	}
	SerialCommand::SerClear(NOW_COM);
	pthread_mutexattr_init(&ma);
	pthread_mutexattr_settype(&ma, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&m_mutex, &ma);
	pthread_mutexattr_destroy(&ma);

	mSerInitialized = true;

	pthread_cond_init(&m_cond, NULL);
	Now_cmd_e = SERCMD_unknown;
	memset(&S_Read, 0x0, sizeof(S_Read));

	FLAG_READ_THREAD = 0; /*thread running flag*/
	if(pthread_equal(com_read_tid, (pthread_t)0L)){
		if (pthread_create(&com_read_tid, NULL, thread_read_sercom_fun, NULL) != 0) {
			LOGE("iniSer() pthread_create thread_read_sercom_fun failed");
			return -1;
		}
	}else{
		LOGD("iniSer() thread_read_sercom_fun exist");
	}
	
	LOGD("Leave iniSer()");
	return 0;
}

int SerialCommand::deInitSer() {
	LOGD("Enter deInitSer()");
	FLAG_READ_THREAD = 1;
	if (pthread_equal(com_read_tid, (pthread_t)0L))
	{
		LOGD("deInitSer() thread_read_sercom_fun not exist");
	}else{
		pthread_join(com_read_tid, NULL);
		com_read_tid = (pthread_t)0L;
		LOGD("deInitSer() thread_read_sercom_fun should destory");
	}
	
	while (1) {
		if (FLAG_READ_THREAD == 2) {
			LOGV("deInitSer serial read thread exited");
			break;
		}
		usleep(100000);
	}
	SerClose(NOW_COM);

	mSerInitialized = false;

	pthread_cond_destroy(&m_cond);
	pthread_mutex_destroy(&m_mutex);
	LOGD("Leave deInitSer()");
	return 0;
}

int SerialCommand::onExcuteCmd(const char * cmd)
{
	int ret = 0;	
	int index = 0;
	
	LOGD("Enter onExcuteCmd() cmd:%s",cmd);

	if(!mSerInitialized && cmd != NULL)
	{
		LOGE("onExcuteCmd():ser not initialized already!!!!!!!");
		return -1;
	}

	pthread_mutex_lock(&m_mutex);
	for (index = 0; index < 3; index++){		
		Now_cmd_e = SERCMD_unknown;	
		//check cmd shoud change to Hex data
		ret = SerWrite_SlaveDevice(NOW_COM, cmd, W_TIMEOUTMS,strlen(cmd));
		LOGD("SerWrite_SlaveDevice() ret:%d",ret);
		if (ret >= 0) {
			Now_cmd_e = (ser_cmd_e)ret;
			struct timeval delta;
			struct timespec abstime;
			gettimeofday(&delta, NULL);
			abstime.tv_sec = delta.tv_sec + R_TIMEOUTMS / 1000;
			abstime.tv_nsec = (delta.tv_usec + (R_TIMEOUTMS % 1000) * 1000) * 1000;
			if (abstime.tv_nsec > 1000000000) {
				abstime.tv_sec += 1;
				abstime.tv_nsec -= 1000000000;
			}

			if (pthread_cond_timedwait(&m_cond, &m_mutex, &abstime) == 0) {
				LOGV(" onExcuteCmd:  SUCC...");//getparameter should sync the result to API
				if(ret != 0){
					Now_cmd_e = (ser_cmd_e)ret;
				}else{
					Now_cmd_e = SERCMD_unknown;
				}
				ret = S_Read.value;
				break;
			} else {
				LOGV(" onExcuteCmd:response %d...", index);
				if(ret != 0){
					Now_cmd_e = (ser_cmd_e)ret;
				}else{
					Now_cmd_e = SERCMD_unknown;
				}
				ret = S_Read.value;
				break;
			}
		} else {
			LOGV("onExcuteCmd: SerWrite_SlaveDevice err...");
			Now_cmd_e = SERCMD_unknown;
			ret = -1;
			break;
		}
	}
	
	pthread_mutex_unlock(&m_mutex);
	
	LOGV("Leave onExcuteCmd() ret: %d", ret);
	return ret;

}

} // namespace soundbar

