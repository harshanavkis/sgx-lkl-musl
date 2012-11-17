struct msghdr
{
	void *msg_name;
	int msg_namelen;
	struct iovec *msg_iov;
	unsigned long msg_iovlen;
	void *msg_control;
	unsigned long  msg_controllen;
	unsigned msg_flags;
};

#define SO_DEBUG        1
#define SO_REUSEADDR    2
#define SO_TYPE         3
#define SO_ERROR        4
#define SO_DONTROUTE    5
#define SO_BROADCAST    6
#define SO_SNDBUF       7
#define SO_RCVBUF       8
#define SO_SNDBUFFORCE  32
#define SO_RCVBUFFORCE  33
#define SO_KEEPALIVE    9
#define SO_OOBINLINE    10
#define SO_NO_CHECK     11
#define SO_PRIORITY     12
#define SO_LINGER       13
#define SO_BSDCOMPAT    14
#define SO_RCVLOWAT     16
#define SO_SNDLOWAT     17
#define SO_RCVTIMEO     18
#define SO_SNDTIMEO     19
#define SO_PASSCRED     20
#define SO_PEERCRED     21
 