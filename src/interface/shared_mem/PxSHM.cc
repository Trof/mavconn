#include "PxSHM.h"

#include <limits.h>

const int __SHM_WRITE_INFO = 0;
const int __SHM_WRITE      = 1;
const int __SHM_NREAD      = 2;

/*

SHARED MEMORY STRUCTURE:

-- KEY --    --------- STATIC ---------      ---------- DATA ----------
             INDEX        PACKET PACKET      INDEX        PACKET PACKET
00 01 02 03  00 01 02 03  ...    ...         00 01 02 03  ...    ...

             __SHM_INFO + 4                   __SHM_DATA + 4

--------   PACKET ----------
00     01 02    03 04 ... N    N+1
SBYTE  LEN      DATA           CRC

*/

/* Data size of the static segement:     20KB = 20*1024 Bytes */
const int __SHM_INFO = 20*1024;
/* Data size of the ringbuffer segement: 10MB = 10*1048576 Bytes */
const int __SHM_DATA = 10*1048576;

PxSHM::PxSHM()
 : mem(0)
{

}

PxSHM::~PxSHM()
{

}

bool
PxSHM::init(int key, PxSHM::Mode mode, int infoSize, int dataSize)
{
	if (infoSize <= 0)
	{
		i_size = __SHM_INFO;
	}
	else
	{
		i_size = infoSize;
	}

	if (dataSize <= 0)
	{
		d_size = __SHM_DATA;
	}
	else
	{
		d_size = data_size;
	}

	int m, f;
	this->mode = mode;
	if (mode == PxSHM::SERVER_MODE)
	{
		m = IPC_CREAT | 0666;
		f = 0;
	}
	else
	{
		m = 0666;
		f = SHM_RDONLY;
	}

	int shmid;
	key_t shmkey = key;
	if ((shmid = shmget(shmkey, i_size+d_size+12, m)) == -1)
	{
		fprintf(stderr, "# ERROR: can't get a shared memory segment.\n");
		return false;
	}

	if ((mem = (unsigned char *)shmat(shmid, NULL, f)) == (unsigned char *)-1)
	{
		fprintf(stderr, "# ERROR: can't attach shared memory.\n");
		return false;
	}

	r_off = 0;
	w_off = 0;
	if (mode == PxSHM::SERVER_MODE)
	{
		srand(time(0));
		this->key = rand();
		memcpy(mem, &(this->key), 4);

		unsigned int num = 0;
		memcpy(&(mem[4]), &num, 4);
		memcpy(&(mem[i_size+8]), &num, 4);
		fprintf(stderr, "# INFO: allocate %.2f MB of shared memory\n",
				(i_size+d_size+12)/(1024.0*1024.0));
	}

	return true;
}

int
PxSHM::hashKey(std::string str)
{
	int hash = 0;

	for (unsigned int i = 0; i < str.length(); i++)
	{
		hash =  str[i] + (hash << 6) + (hash << 16) - hash;
	}

	return hash;
}

unsigned char
PxSHM::shmcrc(int num, unsigned char *bytes)
{
	unsigned char c = 0;
	for (int i = 0; i < num; i++)
	{
		c += bytes[i];
	}
	return c;
}

const unsigned char __SHM_IDENTIFIER = 0xF;

int
PxSHM::shmpos(int num, int mode)
{
	if (mode == __SHM_WRITE_INFO)
	{
		return (i_off+num+8);
	}
	if (mode == __SHM_WRITE)
	{
		return (((w_off+num)%d_size)+i_size+12);
	}
	if (mode == __SHM_NREAD)
	{
		return (((r_off+num)%d_size)+i_size+12);
	}

	return 0;
}

void
PxSHM::shmcpy(unsigned char *data, int len, int off, int mode)
{
	int part1, part2;

	if (mode == __SHM_WRITE)
	{
		if (w_off+off+len > d_size)
		{
			part1 = d_size - w_off - off;
			part2 = len - part1;
			memcpy(&(mem[shmpos(off,mode)]), data, part1);
			memcpy(&(mem[i_size+12]), &(data[part1]), part2);
		}
		else
		{
			memcpy(&(mem[shmpos(off,mode)]), data, len);
		}
	}

	if (mode == __SHM_NREAD)
	{
		if (r_off+off+len > d_size)
		{
			part1 = d_size - r_off - off;
			part2 = len - part1;
			memcpy(data, &(mem[shmpos(off,mode)]), part1);
			memcpy(&(data[part1]), &(mem[i_size+12]), part2);
		}
		else
		{
			memcpy(data, &(mem[shmpos(off,mode)]), len);
		}
	}
}

int
PxSHM::writeInfoPacket(int num, unsigned char *data)
{
	if (i_off+num+2 > i_size)
	{
		return 0;
	}

	mem[shmpos(0,__SHM_WRITE_INFO)] = __SHM_IDENTIFIER;

	char bytes[2];
	memcpy(bytes, &num, 2);
	mem[shmpos(1,__SHM_WRITE_INFO)] = bytes[0];
	mem[shmpos(2,__SHM_WRITE_INFO)] = bytes[1];
	for (int i = 0; i < num; i++)
	{
		mem[shmpos(3+i,__SHM_WRITE_INFO)] = data[i];
	}
	mem[shmpos(3+num,__SHM_WRITE_INFO)] = shmcrc(num, data);
	i_off += num+4;
	memcpy(&(mem[4]), &i_off, 4);
	return num;
}

int
PxSHM::writeDataPacket(int num, unsigned char *data)
{
	mem[shmpos(0,__SHM_WRITE)] = __SHM_IDENTIFIER;
	shmcpy((unsigned char *) (&num), 4, 1, __SHM_WRITE);
	shmcpy(data, num, 5, __SHM_WRITE);
	mem[shmpos(5+num,__SHM_WRITE)] = shmcrc(num, data);
	w_off = (w_off+num+6)%d_size;
	memcpy(&(mem[i_size+8]), &w_off, 4);
	return num;
}

int
PxSHM::write(unsigned int num, unsigned char *s)
{
	return writeDataPacket(num, s);
}

int
PxSHM::addInfo(unsigned int num, unsigned char *s)
{
	return writeInfoPacket(num, s);
}

int
PxSHM::getInfo(unsigned char *s)
{
	unsigned int o;
	memcpy(&o, &(mem[4]), 4);
	if (o != i_off)
	{
		if (mem[i_off+8] == __SHM_IDENTIFIER)
		{
			unsigned short num;

			memcpy(&num, &(mem[i_off+9]), 2);
			for (unsigned short i = 0; i < num; i++)
			{
				s[i] = mem[i_off+11+i];
			}

			if (mem[i_off+11+num] == shmcrc(num, s))
			{
				i_off += num+4;
				return num;
			}
			else
			{
				fprintf(stderr, "# WARNING: crc error, re-initialize shm\n");
				return -1;
			}
		}
		else
		{
			fprintf(stderr, "# WARNING: corrupt packet, re-initialize shm\n");
			return -1;
		}
	}
	return 0;
}

int
PxSHM::nread(int len, unsigned char *s)
{
	unsigned int shmkey, off;
	memcpy(&shmkey, mem, 4);
	memcpy(&off, &(mem[i_size+8]), 4);
	if (key != shmkey)
	{
		r_off = off;
		key = shmkey;
		return 0;
	}
	if (off != r_off)
	{
		if (mem[shmpos(0,__SHM_NREAD)] != __SHM_IDENTIFIER)
		{
			fprintf(stderr, "# WARNING: corrupt packet, re-initialize shm\n");
			r_off = off;
			return 0;
		}

		unsigned int num;
		shmcpy((unsigned char *)(&num), 4, 1, __SHM_NREAD);

		unsigned int n;
		if (num > (unsigned int)len)
		{
			n = len;
		}
		else
		{
			n = num;
		}
		shmcpy(s, n, 5, __SHM_NREAD);
		if (n == num)
		{
			if (mem[shmpos(5+num,__SHM_NREAD)] == shmcrc(num, s))
			{
				r_off = (r_off+num+6)%d_size;
				return num;
			}
			else
			{
				fprintf(stderr, "# WARNING: crc error packet, re-initialize shm\n");
				r_off = off;
				return -1;
			}
		}
		else
		{
			r_off = (r_off+num+6)%d_size;
			return n;
		}
	}
	return 0;
}

int
PxSHM::read(unsigned char *s)
{
	return nread(INT_MAX, s);
}

bool
PxSHM::bytesWaiting()
{
	unsigned int off;
	memcpy(&off, &(mem[i_size+8]), 4);
	return (off != (unsigned int)r_off);
}

const char PROC_SHM_MAX[] = "/proc/sys/kernel/shmmax";

long long
PxSHM::getMax(void)
{
	FILE    * fp = NULL;
	int       n = 0;
	long long max = -1;
	char      number[100];

	if ((fp = fopen(PROC_SHM_MAX,"r")) == 0)
	{
		fprintf(stderr, "# ERROR: can't open shm info: %s\n", PROC_SHM_MAX);
		return -1;
	}
	else
	{
		n = fscanf(fp, "%s", number);
		if (n != 1)
		{
			max = -1;
		}
		else
		{
			max = atoll(number);
		}
		fclose(fp);
	}
	return max;
}

bool
PxSHM::setMax(long long max)
{
	FILE *fp = NULL;

	if ((fp = fopen(PROC_SHM_MAX, "w")) == 0)
	{
		fprintf(stderr, "# ERROR: can't open shm info: %s\n", PROC_SHM_MAX);
		return false;
	}
	else
	{
		fprintf(fp, "%lld\n", max);
		fclose(fp);
	}
	return true;
}

PxSHM::Mode
PxSHM::getMode(void)
{
	return mode;
}
