/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009-2011 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
* @file
*   @brief Shared memory interface.
*
*   This interface has two modes: client and server. The shared memory
*   structure consists of one static buffer, and one dynamic ringbuffer.
*
*   @author Lionel Heng  <hengli@inf.ethz.ch>
*
*/

#include "PxSHM.h"

#include <limits.h>
#include <string.h>

/*

SHARED MEMORY STRUCTURE:

-- KEY --    --------- STATIC ---------      ---------- DATA ----------
             OFFSET       PACKET PACKET      OFFSET       PACKET PACKET
00 01 02 03  00 01 02 03  ...    ...         00 01 02 03  ...    ...

             i_size + 4                      d_size + 4

--------   PACKET ----------
00     01 02 03 04    05 06 ... N-1    N
SBYTE  LEN            DATA             CRC

*/

PxSHM::PxSHM()
 : mem(0)
 , key(0)
 , i_size(0)
 , d_size(0)
 , w_off(0)
 , r_off(0)
 , i_off(0)
{

}

PxSHM::~PxSHM()
{

}

bool
PxSHM::init(int key, PxSHM::Type type, int infoMaxPacketSize, int infoQueueLength,
			int dataMaxPacketSize, int dataQueueLength)
{
	if (infoMaxPacketSize <= 0)
	{
		fprintf(stderr, "# ERROR: Info packet size is invalid.\n");
		return false;
	}
	if (infoQueueLength <= 0)
	{
		fprintf(stderr, "# ERROR: Info queue length is invalid.\n");
		return false;
	}
	if (dataMaxPacketSize <= 0)
	{
		fprintf(stderr, "# ERROR: Data packet size is invalid.\n");
		return false;
	}
	if (dataQueueLength <= 0)
	{
		fprintf(stderr, "# ERROR: Data queue length is invalid.\n");
		return false;
	}

	i_size = infoMaxPacketSize * infoQueueLength;
	d_size = dataMaxPacketSize * dataQueueLength;

	int m, f;
	this->type = type;
	if (type == SERVER_TYPE)
	{
		m = IPC_CREAT | 0666;
		f = 0;
	}
	else if (type == CLIENT_TYPE)
	{
		m = IPC_CREAT | 0666;
		f = SHM_RDONLY;
	}
	else
	{
		fprintf(stderr, "# ERROR: Unknown type.\n");
		return false;
	}

	int shmid;
	key_t shmkey = key;
	if ((shmid = shmget(shmkey, i_size + d_size + 12, m)) == -1)
	{
		fprintf(stderr, "# ERROR: Unable to get a shared memory segment.\n");
		return false;
	}

	if ((mem = (unsigned char *)shmat(shmid, NULL, f)) == (unsigned char *)-1)
	{
		fprintf(stderr, "# ERROR: Unable to attach shared memory.\n");
		return false;
	}

	r_off = 0;
	w_off = 0;
	if (type == SERVER_TYPE)
	{
		srand(time(0));
		this->key = rand();
		memcpy(mem, &(this->key), 4);

		unsigned int num = 0;
		memcpy(&(mem[4]), &num, 4);
		memcpy(&(mem[i_size + 8]), &num, 4);

		fprintf(stderr, "# INFO: allocate %.2f MB of shared memory\n",
				(i_size + d_size + 12) / (1024.0 * 1024.0));
	}

	return true;
}

int
PxSHM::hashKey(const std::string& str) const
{
	int hash = 0;

	for (unsigned int i = 0; i < str.length(); i++)
	{
		hash =  str[i] + (hash << 6) + (hash << 16) - hash;
	}

	return hash;
}

const uint8_t __SHM_IDENTIFIER = 0xF;

int
PxSHM::readInfoPacket(std::vector<uint8_t>& data)
{
	unsigned int o;
	memcpy(&o, &(mem[4]), 4);
	if (o != i_off)
	{
		// check packet magic ID
		if (mem[i_off + 8] == __SHM_IDENTIFIER)
		{
			// read packet size
			unsigned int payloadSizeInBytes;
			memcpy(&payloadSizeInBytes, &(mem[i_off + 9]), 4);

			data.resize(payloadSizeInBytes);

			// read packet payload
			memcpy(&(data[0]), mem + i_off + 13, payloadSizeInBytes);

			// validate packet CRC
			if (mem[i_off + 13 + payloadSizeInBytes] == crc(data))
			{
				i_off += payloadSizeInBytes + 6;
				return payloadSizeInBytes;
			}
			else
			{
				fprintf(stderr, "# WARNING: packet CRC error.\n");
				return -1;
			}
		}
		else
		{
			fprintf(stderr, "# WARNING: corrupt packet.\n");
			return -1;
		}
	}
	return 0;
}

int
PxSHM::writeInfoPacket(const std::vector<uint8_t>& data)
{
	if (i_off + data.size() + 6 > i_size)
	{
		return 0;
	}

	// write packet magic ID
	mem[pos(0,WRITE_INFO)] = __SHM_IDENTIFIER;

	// write packet payload size
	char bytes[4];
	size_t num = data.size();
	memcpy(bytes, &num, 4);
	mem[pos(1,WRITE_INFO)] = bytes[0];
	mem[pos(2,WRITE_INFO)] = bytes[1];
	mem[pos(3,WRITE_INFO)] = bytes[2];
	mem[pos(4,WRITE_INFO)] = bytes[3];

	// write packet payload data
	for (size_t i = 0; i < num; i++)
	{
		mem[pos(5 + i,WRITE_INFO)] = data[i];
	}

	// write packet CRC
	mem[pos(5 + num,WRITE_INFO)] = crc(data);
	i_off += num + 6;

	// update offset
	memcpy(&(mem[4]), &i_off, 4);

	return num;
}

int
PxSHM::readDataPacket(std::vector<uint8_t>& data)
{
	unsigned int shmkey, off;
	memcpy(&shmkey, mem, 4);
	memcpy(&off, &(mem[i_size + 8]), 4);
	if (key != shmkey)
	{
		r_off = off;
		key = shmkey;
		return 0;
	}
	if (off != r_off)
	{
		// read packet magic ID
		if (mem[pos(0,READ_DATA)] != __SHM_IDENTIFIER)
		{
			fprintf(stderr, "# WARNING: corrupt packet.\n");
			r_off = off;
			return 0;
		}

		// read packet size
		unsigned int payloadSizeInBytes;
		copyFromSHM(reinterpret_cast<uint8_t *>(&payloadSizeInBytes), 4, 1);

		// read packet payload
		copyFromSHM(data, payloadSizeInBytes, 5);

		// check packet crc
		if (mem[pos(5 + payloadSizeInBytes,READ_DATA)] == crc(data))
		{
			r_off = (r_off + payloadSizeInBytes + 6) % d_size;
			return payloadSizeInBytes;
		}
		else
		{
			fprintf(stderr, "# WARNING: packet CRC error.\n");
			// reset
			r_off = off;
			return -1;
		}
	}
	return 0;
}

int
PxSHM::writeDataPacket(const std::vector<uint8_t>& data)
{
	// write packet magic ID (1 byte)
	mem[pos(0,WRITE_DATA)] = __SHM_IDENTIFIER;
	// write size of packet (4 bytes)
	size_t num = data.size();
	copyToSHM(reinterpret_cast<uint8_t *>(&num), 4, 1);
	// write packet payload (num bytes)
	copyToSHM(data, 5);
	// write packet CRC (1 byte)
	mem[pos(5 + num,WRITE_DATA)] = crc(data);
	w_off = (w_off + num + 6) % d_size;

	// update write offset
	memcpy(&(mem[i_size + 8]), &w_off, 4);

	return num;
}

bool
PxSHM::bytesWaiting(void) const
{
	unsigned int off;
	memcpy(&off, &(mem[i_size + 8]), 4);
	return (off != r_off);
}

const char PROC_SHM_MAX[] = "/proc/sys/kernel/shmmax";

long long
PxSHM::getMax(void) const
{
	FILE* fp = NULL;
	long long max = -1;

	if ((fp = fopen(PROC_SHM_MAX,"r")) == 0)
	{
		fprintf(stderr, "# ERROR: can't open shm info: %s\n", PROC_SHM_MAX);
		return -1;
	}
	else
	{
		char number[100];
		int n = fscanf(fp, "%s", number);
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
PxSHM::setMax(long long max) const
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

PxSHM::Type
PxSHM::getType(void) const
{
	return type;
}

uint8_t
PxSHM::crc(const std::vector<uint8_t>& data) const
{
	uint8_t c = 0;
	for (size_t i = 0; i < data.size(); i++)
	{
		c += data[i];
	}
	return c;
}

int
PxSHM::pos(int num, PxSHM::Mode mode) const
{
	if (mode == WRITE_INFO)
	{
		return (i_off + num + 8);
	}
	if (mode == WRITE_DATA)
	{
		return (((w_off + num) % d_size) + i_size + 12);
	}
	if (mode == READ_DATA)
	{
		return (((r_off + num) % d_size) + i_size + 12);
	}

	return 0;
}

void
PxSHM::copyToSHM(const uint8_t* data, int len, int off)
{
	if (w_off + off + len > d_size)
	{
		int part1 = d_size - w_off - off;
		int part2 = len - part1;
		memcpy(&(mem[pos(off,WRITE_DATA)]), data, part1);
		memcpy(&(mem[i_size + 12]), &(data[part1]), part2);
	}
	else
	{
		memcpy(&(mem[pos(off,WRITE_DATA)]), data, len);
	}
}

void
PxSHM::copyToSHM(const std::vector<uint8_t>& data, int off)
{
	copyToSHM(&(data[0]), data.size(), off);
}

void
PxSHM::copyFromSHM(uint8_t* data, int len, int off) const
{
	if (r_off + off + len > d_size)
	{
		int part1 = d_size - r_off - off;
		int part2 = len - part1;
		memcpy(data, &(mem[pos(off,READ_DATA)]), part1);
		memcpy(&(data[part1]), &(mem[i_size + 12]), part2);
	}
	else
	{
		memcpy(data, &(mem[pos(off,READ_DATA)]), len);
	}
}

void
PxSHM::copyFromSHM(std::vector<uint8_t>& data, int len, int off) const
{
	if (data.size() < static_cast<size_t>(len))
	{
		data.resize(len);
	}

	copyFromSHM(&(data[0]), len, off);
}
