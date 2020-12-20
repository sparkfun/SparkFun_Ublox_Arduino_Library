/*
Copyright (c) 2020 Balamurugan Kandan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef STREAM_H__
#define STREAM_H__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <pty.h>

#include "Common.h"
#include "Print.h"

class Stream : public Print {
public:
	Stream(char *devName) : m_IsAvailable(false), m_IsConnected(false), m_SerialFd(-1), m_baudRate(115200), m_devName({'\0'}) {
		strcpy(m_devName, devName);
		open_comm(devName);
		m_recvdByte = '\0';
	}

	Stream() : m_IsAvailable(false), m_IsConnected(false), m_SerialFd(-1) {
		open_comm(create_pseudo_com());
		m_recvdByte = '\0';
	}

	virtual ~Stream() {
		close_comm();
	}

	void close_comm() {
		close(m_SerialFd);
		m_IsAvailable = false;
		m_IsConnected = false;
		m_SerialFd = -1;
		printf ("Stream : Closed\n");
	}

	int get_baud(int baud)
	{
		switch (baud) {
		case 1200:
			return B1200;
		case 2400:
			return B2400;
		case 4800:
			return B4800;
		case 9600:
			return B9600;
		case 19200:
			return B19200;
		case 38400:
			return B38400;
		case 57600:
			return B57600;
		case 115200:
			return B115200;
		case 230400:
			return B230400;
		case 460800:
			return B460800;
		case 921600:
			return B921600;
		default: 
			return B115200;
	}
	}

	bool open_comm(char *devName) {
		m_SerialFd = open(devName, O_NOCTTY|O_RDWR|O_NONBLOCK);
		if (m_SerialFd != -1) {
			printf ("\n%s is connected successfully!!!\n", devName);
			// Get device info
			struct termios newtio;
			memset(&newtio,0,sizeof(newtio));
			cfmakeraw(&newtio);
			newtio.c_cflag |= get_baud(m_baudRate);
			tcflush(m_SerialFd, TCIFLUSH);
			tcsetattr(m_SerialFd,TCSANOW,&newtio);
			m_IsConnected = true;
		} else {
			printf("Unable to open GNSS on: %s\n",devName);
		}
		return m_IsConnected;
	}

	bool isConnected() { return m_IsConnected; }
	bool available() {
	    int iRxLen = 0;
		iRxLen = ::read(m_SerialFd, (void *)&m_recvdByte, sizeof(m_recvdByte));
	    if (iRxLen > 0) {
	    	m_IsAvailable = true;
	        // Debugging purpose
	        /*
	        int iPos = 0;
	        while (iPos < iRxLen) {
				iPos++;
			}
			*/
		} else {
			m_IsAvailable = false;
	    }
		return m_IsAvailable;
	}

	size_t write(uint8_t byte) {
		m_IsAvailable = false;
		return ::write(m_SerialFd, (void *)&byte, sizeof(byte));
	}

	size_t write(uint8_t *byte, size_t num_of_bytes) {
		m_IsAvailable = false;
		return ::write(m_SerialFd, byte, num_of_bytes);
	}

	uint8_t read() {
		return m_recvdByte;
	}

	int getFD() {
		return m_SerialFd;
	}

	void begin(uint16_t baud_rate) {
		m_baudRate = baud_rate;
		close_comm();
		open_comm(m_devName);
	}

private:
	bool m_IsAvailable;
	bool m_IsConnected;
	int m_SerialFd;
	uint8_t m_recvdByte;
	uint16_t m_baudRate;
	char m_devName[SHORT_BUFF];

	char* create_pseudo_com() {
		static char name[SHORT_BUFF] = {'\0'};
		struct termios tt = {'\0'};
		int master = 0, slave = 0;

		if (tcgetattr (STDIN_FILENO, &tt) < 0) {
			printf("Cannot get terminal attributes of stdin\n");
		}
		cfmakeraw (&tt);
		if (openpty (&master, &slave, name, &tt, NULL /*ws*/) < 0) {
			printf("Cannot open pty\n");
		}

		return name;
	}
};

//typedef Stream Serial;
extern Stream Serial;

#endif // STREAM__