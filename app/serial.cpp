#include "serial.h"

#ifdef _WIN32
	#include <Windows.h>
	typedef HANDLE serial_handle_t;
	#define DEFAULT_COM_PORT "\\\\.\\COM1"
	#pragma warning(disable:4996)
#else
	#include <unistd.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <sys/stat.h>
	#include <sys/eventfd.h>
	#include <sys/poll.h>
	typedef int serial_handle_t;
	#define INVALID_HANDLE_VALUE (-1)
#endif

struct Serial::Handle {
	serial_handle_t serial_fd = INVALID_HANDLE_VALUE;
	serial_handle_t cancel_fd = INVALID_HANDLE_VALUE;
	Handle() = default;
	Handle(serial_handle_t serial_fd, serial_handle_t cancel_fd)
		: serial_fd(serial_fd)
		, cancel_fd(cancel_fd)
	{
	}
};

Serial::Serial()
{
}

bool Serial::open(Option *option)
{
	if (handle_) return false;
	
	option_ = *option;
#ifdef _WIN32
	serial_handle_t serial_fd;
	DCB dcb;
	serial_fd = CreateFileA(option_.port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (serial_fd == INVALID_HANDLE_VALUE) {
		return false;
	}

	GetCommState(serial_fd, &dcb);
	dcb.BaudRate = option_.speed;
	dcb.fBinary = TRUE;
	dcb.fParity = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fErrorChar = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	dcb.fAbortOnError = FALSE;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	if (!SetCommState(serial_fd, &dcb)) {
		CloseHandle(serial_fd);
		return false;
	}
	PurgeComm(serial_fd, PURGE_RXCLEAR | PURGE_TXCLEAR);

	handle_ = std::make_shared<Handle>(serial_fd, nullptr);
	return true;
#else
	int speed;
	serial_handle_t serial_fd;
	serial_handle_t cancel_fd;
	struct termios attr;
	
	int oflag = O_RDWR | O_NOCTTY;// | O_NONBLOCK;
	serial_fd = ::open(option->port.c_str(), oflag);
	if (serial_fd < 0) {
		return false;
	}
	cancel_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (cancel_fd < 0) {
		::close(serial_fd);
		return false;
	}

	tcgetattr(serial_fd, &attr);
	option->saveattr = attr;

	switch (option->speed) {
//	case      50:	speed =      B50;		break;
//	case      75:	speed =      B75;		break;
//	case     110:	speed =     B110;		break;
//	case     134:	speed =     B134;		break;
//	case     150:	speed =     B150;		break;
// 	case     200:	speed =     B200;		break;
	case     300:	speed =     B300;		break;
	case     600:	speed =     B600;		break;
	case    1200:	speed =    B1200;		break;
//	case    1800:	speed =    B1800;		break;
	case    2400:	speed =    B2400;		break;
	case    4800:	speed =    B4800;		break;
	case    9600:	speed =    B9600;		break;
	case   19200:	speed =   B19200;		break;
	case   38400:	speed =   B38400;		break;
	case   57600:	speed =   B57600;		break;
	case  115200:	speed =  B115200;		break;
	case  230400:	speed =  B230400;		break;
//	case  460800:	speed =  B460800;		break;
//	case  500000:	speed =  B500000;		break;
//	case  576000:	speed =  B576000;		break;
//	case  921600:	speed =  B921600;		break;
//	case 1000000:	speed = B1000000;		break;
//	case 1152000:	speed = B1152000;		break;
//	case 1500000:	speed = B1500000;		break;
//	case 2000000:	speed = B2000000;		break;
//	case 2500000:	speed = B2500000;		break;
//	case 3000000:	speed = B3000000;		break;
//	case 3500000:	speed = B3500000;		break;
//	case 4000000:	speed = B4000000;		break;
	default:
		speed = B38400;
		break;
	}

	cfsetispeed(&attr, speed);
	cfsetospeed(&attr, speed);
	cfmakeraw(&attr);

	attr.c_cflag &= ~CSIZE;
	attr.c_cflag |= CS8 | CLOCAL | CREAD;
	attr.c_iflag = 0;
	attr.c_oflag = 0;
	attr.c_lflag = 0;
	attr.c_cc[VMIN] = 1;
	attr.c_cc[VTIME] = 0;

	if (tcsetattr(serial_fd, TCSANOW, &attr) != 0) {
		::close(serial_fd);
		::close(cancel_fd);
		return false;
	}
	tcflush(serial_fd, TCIOFLUSH);

	handle_ = std::make_shared<Handle>(serial_fd, cancel_fd);
	return true;
#endif
}

void Serial::cancel()
{
	if (!handle_) return;
	
#ifdef _WIN32
#else
	uint64_t one = 1;
	::write(handle_->cancel_fd, &one, sizeof(one));
#endif
}

void Serial::close()
{
	if (!handle_) return;
		
#ifdef _WIN32
	CloseHandle(handle_->serial_fd);
#else
	cancel();
	tcsetattr(handle_->serial_fd, TCSANOW, &option_.saveattr);
	::close(handle_->serial_fd);
	::close(handle_->cancel_fd);
#endif
	handle_.reset();
}

int Serial::write(const void *ptr, int len)
{
#ifdef _WIN32
	DWORD bytes = 0;
	if (WriteFile(handle_->serial_fd, ptr, len, &bytes, NULL)) {
		return bytes;
	}
	return -1;
#else
	return ::write(handle_->serial_fd, ptr, len);
#endif
}

int Serial::read(void *ptr, int len, int timeout)
{
#ifdef _WIN32
	DWORD bytes = 0;
	COMMTIMEOUTS cto;
	if (!GetCommTimeouts(handle_->serial_fd, &cto)) {
		return -1;
	}
	if (timeout < 0) {
		cto.ReadIntervalTimeout = 0;
		cto.ReadTotalTimeoutMultiplier = 0;
		cto.ReadTotalTimeoutConstant = 0;
	} else if (timeout == 0) {
		cto.ReadIntervalTimeout = MAXDWORD;
		cto.ReadTotalTimeoutMultiplier = 0;
		cto.ReadTotalTimeoutConstant = 0;
	} else {
		cto.ReadIntervalTimeout = 0;
		cto.ReadTotalTimeoutMultiplier = 0;
		cto.ReadTotalTimeoutConstant = timeout;
	}
	if (!SetCommTimeouts(handle_->serial_fd, &cto)) {
		return -1;
	}
	if (ReadFile(handle_->serial_fd, ptr, len, &bytes, NULL)) {
		return bytes;
	}
	return -1;
#else
	pollfd fds[] = {
		{handle_->serial_fd, POLLIN, 0},
		{handle_->cancel_fd, POLLIN, 0},
	};
	while (1) {
		int r = poll(fds, 2, timeout);
		if (r == 0) return 0; // timeout
		if (r < 0) {
			if (errno == EINTR) continue;
			break;
		}
		if (fds[1].revents & POLLIN) {
			break;
		}
		if (fds[0].revents & (POLLIN | POLLERR | POLLHUP)) {
			ssize_t n = ::read(handle_->serial_fd, ptr, len);
			if (n > 0) {
				return n;
			} else if (n < 0) {
				if (errno != EINTR) break;
			}
		}
	}
	return -1;
#endif
}
