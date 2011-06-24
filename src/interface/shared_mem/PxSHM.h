#ifndef PXSHM_H
#define PXSHM_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <string>

const int PxSHM_SERVER_MODE = 0x00;
const int PxSHM_CLIENT_MODE = 0x01;

class PxSHM
{
public:
	typedef enum
	{
		SERVER_MODE = 0,
		CLIENT_MODE = 1
	} Mode;

	PxSHM();
	~PxSHM();

	bool init(int key, Mode mode, int infoSize, int dataSize);

	int hashKey(std::string str);

	int write(unsigned int num, unsigned char *s);

	int read(unsigned char *s);

	int nread(int len, unsigned char *s);

	bool bytesWaiting(void);

	int addInfo(unsigned int num, unsigned char *s);

	int getInfo(unsigned char *s);

	long long getMax(void);

	bool setMax(long long max);

	Mode getMode(void);

private:
	unsigned char shmcrc(int num, unsigned char *bytes);

	int shmpos(int num, int mode);

	void shmcpy(unsigned char *data, int len, int off, int mode);

	int writeInfoPacket(int num, unsigned char *data);

	int writeDataPacket(int num, unsigned char *data);

	Mode              mode;      /* server/client mode */
	unsigned char   * mem;       /* shared memory segment */
	unsigned int      key;       /* shared memory key */
	unsigned int      i_size;    /* size of the (static) info buffer */
	unsigned int      d_size;    /* size of the (ringbuffer) data buffer */
	unsigned int      w_off;     /* write offset */
	unsigned int      r_off;     /* read offset */
	unsigned int      i_off;     /* info offset */
};

#endif
