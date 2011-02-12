#include <sys/socket.h>
#include "syscall.h"
#include "socketcall.h"

int getpeername(int fd, struct sockaddr *addr, socklen_t *len)
{
	unsigned long args[] = { fd, (unsigned long)addr, (unsigned long)len };
	return syscall2(__NR_socketcall, SYS_GETPEERNAME, (long)args);
}
