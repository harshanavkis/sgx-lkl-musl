#define _BSD_SOURCE
#include <unistd.h>
#include "syscall.h"
#include <stdarg.h>

#undef syscall

/*

Derived from https://filippo.io/linux-syscall-table/ using the following JavaScript, executed in Chrome's debug console:

const tbl = document.querySelector('.tbls-table');
const rows = tbl.querySelectorAll('.tbls-entry-collapsed');
let countBuckets = {0: [], 1: [], 2: [], 3: [], 4: [], 5: [], 6: []};
rows.forEach((row) => {
	let arguments = 0;
	if (row.nextElementSibling.classList.contains('tbls-arguments-collapsed')) {
		arguments = row.nextElementSibling.querySelectorAll('td > table strong').length;
	}
	let syscallno = row.querySelector('td').innerText;
	countBuckets[arguments].push(syscallno);
});

let o = '\tswitch (n) {\n';
const letters = ['a','b','c','d','e','f'];
for (let i = 0; i <= 6; i++) {
	for (let syscallno of countBuckets[i]) {
		o += '\tcase ' + syscallno + ':\n';
	}
	o += '\t\treturn __syscall_ret(__syscall(calln';
	if (i > 0)
		o += ',' + letters.slice(0, i).join(',');
	o += '));\n';
}
o += '\t}\n';
o += '\tabort();\n';
console.log(o);
*/

static const short syscall_remap_len = 547;
static const short syscall_remap[] = {
	63, /* read - x86-64 syscall: 0 */
	64, /* write - x86-64 syscall: 1 */
	1024, /* open - x86-64 syscall: 2 */
	57, /* close - x86-64 syscall: 3 */
	1049, /* stat - x86-64 syscall: 4 */
	1051, /* fstat - x86-64 syscall: 5 */
	1050, /* lstat - x86-64 syscall: 6 */
	1068, /* poll - x86-64 syscall: 7 */
	1057, /* lseek - x86-64 syscall: 8 */
	1058, /* mmap - x86-64 syscall: 9 */
	226, /* mprotect - x86-64 syscall: 10 */
	215, /* munmap - x86-64 syscall: 11 */
	214, /* brk - x86-64 syscall: 12 */
	-1, /* not implemented in x86-64 */
	135, /* rt_sigprocmask - x86-64 syscall: 14 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	67, /* pread64 - x86-64 syscall: 17 */
	68, /* pwrite64 - x86-64 syscall: 18 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	1033, /* access - x86-64 syscall: 21 */
	1040, /* pipe - x86-64 syscall: 22 */
	1067, /* select - x86-64 syscall: 23 */
	124, /* sched_yield - x86-64 syscall: 24 */
	216, /* mremap - x86-64 syscall: 25 */
	227, /* msync - x86-64 syscall: 26 */
	232, /* mincore - x86-64 syscall: 27 */
	233, /* madvise - x86-64 syscall: 28 */
	194, /* shmget - x86-64 syscall: 29 */
	196, /* shmat - x86-64 syscall: 30 */
	195, /* shmctl - x86-64 syscall: 31 */
	23, /* dup - x86-64 syscall: 32 */
	1041, /* dup2 - x86-64 syscall: 33 */
	1061, /* pause - x86-64 syscall: 34 */
	101, /* nanosleep - x86-64 syscall: 35 */
	102, /* getitimer - x86-64 syscall: 36 */
	1059, /* alarm - x86-64 syscall: 37 */
	103, /* setitimer - x86-64 syscall: 38 */
	172, /* getpid - x86-64 syscall: 39 */
	1046, /* sendfile - x86-64 syscall: 40 */
	198, /* socket - x86-64 syscall: 41 */
	203, /* connect - x86-64 syscall: 42 */
	202, /* accept - x86-64 syscall: 43 */
	206, /* sendto - x86-64 syscall: 44 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	210, /* shutdown - x86-64 syscall: 48 */
	200, /* bind - x86-64 syscall: 49 */
	201, /* listen - x86-64 syscall: 50 */
	204, /* getsockname - x86-64 syscall: 51 */
	205, /* getpeername - x86-64 syscall: 52 */
	199, /* socketpair - x86-64 syscall: 53 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	220, /* clone - x86-64 syscall: 56 */
	1079, /* fork - x86-64 syscall: 57 */
	1071, /* vfork - x86-64 syscall: 58 */
	-1, /* not implemented in x86-64 */
	93, /* exit - x86-64 syscall: 60 */
	260, /* wait4 - x86-64 syscall: 61 */
	129, /* kill - x86-64 syscall: 62 */
	160, /* uname - x86-64 syscall: 63 */
	190, /* semget - x86-64 syscall: 64 */
	193, /* semop - x86-64 syscall: 65 */
	191, /* semctl - x86-64 syscall: 66 */
	197, /* shmdt - x86-64 syscall: 67 */
	186, /* msgget - x86-64 syscall: 68 */
	189, /* msgsnd - x86-64 syscall: 69 */
	188, /* msgrcv - x86-64 syscall: 70 */
	187, /* msgctl - x86-64 syscall: 71 */
	1052, /* fcntl - x86-64 syscall: 72 */
	32, /* flock - x86-64 syscall: 73 */
	82, /* fsync - x86-64 syscall: 74 */
	83, /* fdatasync - x86-64 syscall: 75 */
	1048, /* truncate - x86-64 syscall: 76 */
	1047, /* ftruncate - x86-64 syscall: 77 */
	1065, /* getdents - x86-64 syscall: 78 */
	17, /* getcwd - x86-64 syscall: 79 */
	49, /* chdir - x86-64 syscall: 80 */
	50, /* fchdir - x86-64 syscall: 81 */
	1034, /* rename - x86-64 syscall: 82 */
	1030, /* mkdir - x86-64 syscall: 83 */
	1031, /* rmdir - x86-64 syscall: 84 */
	1064, /* creat - x86-64 syscall: 85 */
	1025, /* link - x86-64 syscall: 86 */
	1026, /* unlink - x86-64 syscall: 87 */
	1036, /* symlink - x86-64 syscall: 88 */
	1035, /* readlink - x86-64 syscall: 89 */
	1028, /* chmod - x86-64 syscall: 90 */
	52, /* fchmod - x86-64 syscall: 91 */
	1029, /* chown - x86-64 syscall: 92 */
	55, /* fchown - x86-64 syscall: 93 */
	1032, /* lchown - x86-64 syscall: 94 */
	166, /* umask - x86-64 syscall: 95 */
	169, /* gettimeofday - x86-64 syscall: 96 */
	163, /* getrlimit - x86-64 syscall: 97 */
	165, /* getrusage - x86-64 syscall: 98 */
	179, /* sysinfo - x86-64 syscall: 99 */
	153, /* times - x86-64 syscall: 100 */
	-1, /* not implemented in x86-64 */
	174, /* getuid - x86-64 syscall: 102 */
	116, /* syslog - x86-64 syscall: 103 */
	176, /* getgid - x86-64 syscall: 104 */
	146, /* setuid - x86-64 syscall: 105 */
	144, /* setgid - x86-64 syscall: 106 */
	175, /* geteuid - x86-64 syscall: 107 */
	177, /* getegid - x86-64 syscall: 108 */
	154, /* setpgid - x86-64 syscall: 109 */
	173, /* getppid - x86-64 syscall: 110 */
	1060, /* getpgrp - x86-64 syscall: 111 */
	157, /* setsid - x86-64 syscall: 112 */
	145, /* setreuid - x86-64 syscall: 113 */
	143, /* setregid - x86-64 syscall: 114 */
	158, /* getgroups - x86-64 syscall: 115 */
	159, /* setgroups - x86-64 syscall: 116 */
	147, /* setresuid - x86-64 syscall: 117 */
	148, /* getresuid - x86-64 syscall: 118 */
	149, /* setresgid - x86-64 syscall: 119 */
	150, /* getresgid - x86-64 syscall: 120 */
	155, /* getpgid - x86-64 syscall: 121 */
	151, /* setfsuid - x86-64 syscall: 122 */
	152, /* setfsgid - x86-64 syscall: 123 */
	156, /* getsid - x86-64 syscall: 124 */
	90, /* capget - x86-64 syscall: 125 */
	91, /* capset - x86-64 syscall: 126 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	133, /* rt_sigsuspend - x86-64 syscall: 130 */
	-1, /* not implemented in x86-64 */
	1063, /* utime - x86-64 syscall: 132 */
	1027, /* mknod - x86-64 syscall: 133 */
	1077, /* uselib - x86-64 syscall: 134 */
	92, /* personality - x86-64 syscall: 135 */
	1070, /* ustat - x86-64 syscall: 136 */
	1056, /* statfs - x86-64 syscall: 137 */
	1055, /* fstatfs - x86-64 syscall: 138 */
	-1, /* not implemented in x86-64 */
	141, /* getpriority - x86-64 syscall: 140 */
	140, /* setpriority - x86-64 syscall: 141 */
	118, /* sched_setparam - x86-64 syscall: 142 */
	121, /* sched_getparam - x86-64 syscall: 143 */
	119, /* sched_setscheduler - x86-64 syscall: 144 */
	120, /* sched_getscheduler - x86-64 syscall: 145 */
	125, /* sched_get_priority_max - x86-64 syscall: 146 */
	126, /* sched_get_priority_min - x86-64 syscall: 147 */
	127, /* sched_rr_get_interval - x86-64 syscall: 148 */
	228, /* mlock - x86-64 syscall: 149 */
	229, /* munlock - x86-64 syscall: 150 */
	230, /* mlockall - x86-64 syscall: 151 */
	231, /* munlockall - x86-64 syscall: 152 */
	58, /* vhangup - x86-64 syscall: 153 */
	-1, /* not implemented in x86-64 */
	41, /* pivot_root - x86-64 syscall: 155 */
	1078, /* _sysctl - x86-64 syscall: 156 */
	167, /* prctl - x86-64 syscall: 157 */
	-1, /* not implemented in x86-64 */
	171, /* adjtimex - x86-64 syscall: 159 */
	164, /* setrlimit - x86-64 syscall: 160 */
	51, /* chroot - x86-64 syscall: 161 */
	81, /* sync - x86-64 syscall: 162 */
	89, /* acct - x86-64 syscall: 163 */
	170, /* settimeofday - x86-64 syscall: 164 */
	40, /* mount - x86-64 syscall: 165 */
	39, /* umount2 - x86-64 syscall: 166 */
	224, /* swapon - x86-64 syscall: 167 */
	225, /* swapoff - x86-64 syscall: 168 */
	142, /* reboot - x86-64 syscall: 169 */
	161, /* sethostname - x86-64 syscall: 170 */
	162, /* setdomainname - x86-64 syscall: 171 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	105, /* init_module - x86-64 syscall: 175 */
	106, /* delete_module - x86-64 syscall: 176 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	60, /* quotactl - x86-64 syscall: 179 */
	42, /* nfsservctl - x86-64 syscall: 180 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	178, /* gettid - x86-64 syscall: 186 */
	213, /* readahead - x86-64 syscall: 187 */
	5, /* setxattr - x86-64 syscall: 188 */
	6, /* lsetxattr - x86-64 syscall: 189 */
	7, /* fsetxattr - x86-64 syscall: 190 */
	8, /* getxattr - x86-64 syscall: 191 */
	9, /* lgetxattr - x86-64 syscall: 192 */
	10, /* fgetxattr - x86-64 syscall: 193 */
	11, /* listxattr - x86-64 syscall: 194 */
	12, /* llistxattr - x86-64 syscall: 195 */
	13, /* flistxattr - x86-64 syscall: 196 */
	14, /* removexattr - x86-64 syscall: 197 */
	15, /* lremovexattr - x86-64 syscall: 198 */
	16, /* fremovexattr - x86-64 syscall: 199 */
	130, /* tkill - x86-64 syscall: 200 */
	1062, /* time - x86-64 syscall: 201 */
	98, /* futex - x86-64 syscall: 202 */
	122, /* sched_setaffinity - x86-64 syscall: 203 */
	123, /* sched_getaffinity - x86-64 syscall: 204 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	1, /* io_destroy - x86-64 syscall: 207 */
	4, /* io_getevents - x86-64 syscall: 208 */
	-1, /* not implemented in x86-64 */
	3, /* io_cancel - x86-64 syscall: 210 */
	-1, /* not implemented in x86-64 */
	18, /* lookup_dcookie - x86-64 syscall: 212 */
	1042, /* epoll_create - x86-64 syscall: 213 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	234, /* remap_file_pages - x86-64 syscall: 216 */
	61, /* getdents64 - x86-64 syscall: 217 */
	96, /* set_tid_address - x86-64 syscall: 218 */
	128, /* restart_syscall - x86-64 syscall: 219 */
	192, /* semtimedop - x86-64 syscall: 220 */
	1053, /* fadvise64 - x86-64 syscall: 221 */
	-1, /* not implemented in x86-64 */
	110, /* timer_settime - x86-64 syscall: 223 */
	108, /* timer_gettime - x86-64 syscall: 224 */
	109, /* timer_getoverrun - x86-64 syscall: 225 */
	111, /* timer_delete - x86-64 syscall: 226 */
	112, /* clock_settime - x86-64 syscall: 227 */
	113, /* clock_gettime - x86-64 syscall: 228 */
	114, /* clock_getres - x86-64 syscall: 229 */
	115, /* clock_nanosleep - x86-64 syscall: 230 */
	94, /* exit_group - x86-64 syscall: 231 */
	1069, /* epoll_wait - x86-64 syscall: 232 */
	21, /* epoll_ctl - x86-64 syscall: 233 */
	131, /* tgkill - x86-64 syscall: 234 */
	1037, /* utimes - x86-64 syscall: 235 */
	-1, /* not implemented in x86-64 */
	235, /* mbind - x86-64 syscall: 237 */
	237, /* set_mempolicy - x86-64 syscall: 238 */
	236, /* get_mempolicy - x86-64 syscall: 239 */
	180, /* mq_open - x86-64 syscall: 240 */
	181, /* mq_unlink - x86-64 syscall: 241 */
	182, /* mq_timedsend - x86-64 syscall: 242 */
	183, /* mq_timedreceive - x86-64 syscall: 243 */
	-1, /* not implemented in x86-64 */
	185, /* mq_getsetattr - x86-64 syscall: 245 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	217, /* add_key - x86-64 syscall: 248 */
	218, /* request_key - x86-64 syscall: 249 */
	219, /* keyctl - x86-64 syscall: 250 */
	30, /* ioprio_set - x86-64 syscall: 251 */
	31, /* ioprio_get - x86-64 syscall: 252 */
	1043, /* inotify_init - x86-64 syscall: 253 */
	27, /* inotify_add_watch - x86-64 syscall: 254 */
	28, /* inotify_rm_watch - x86-64 syscall: 255 */
	238, /* migrate_pages - x86-64 syscall: 256 */
	56, /* openat - x86-64 syscall: 257 */
	34, /* mkdirat - x86-64 syscall: 258 */
	33, /* mknodat - x86-64 syscall: 259 */
	54, /* fchownat - x86-64 syscall: 260 */
	1066, /* futimesat - x86-64 syscall: 261 */
	1054, /* newfstatat - x86-64 syscall: 262 */
	35, /* unlinkat - x86-64 syscall: 263 */
	38, /* renameat - x86-64 syscall: 264 */
	37, /* linkat - x86-64 syscall: 265 */
	36, /* symlinkat - x86-64 syscall: 266 */
	78, /* readlinkat - x86-64 syscall: 267 */
	53, /* fchmodat - x86-64 syscall: 268 */
	48, /* faccessat - x86-64 syscall: 269 */
	72, /* pselect6 - x86-64 syscall: 270 */
	73, /* ppoll - x86-64 syscall: 271 */
	97, /* unshare - x86-64 syscall: 272 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	76, /* splice - x86-64 syscall: 275 */
	77, /* tee - x86-64 syscall: 276 */
	84, /* sync_file_range - x86-64 syscall: 277 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	88, /* utimensat - x86-64 syscall: 280 */
	22, /* epoll_pwait - x86-64 syscall: 281 */
	1045, /* signalfd - x86-64 syscall: 282 */
	85, /* timerfd_create - x86-64 syscall: 283 */
	1044, /* eventfd - x86-64 syscall: 284 */
	47, /* fallocate - x86-64 syscall: 285 */
	86, /* timerfd_settime - x86-64 syscall: 286 */
	87, /* timerfd_gettime - x86-64 syscall: 287 */
	242, /* accept4 - x86-64 syscall: 288 */
	74, /* signalfd4 - x86-64 syscall: 289 */
	19, /* eventfd2 - x86-64 syscall: 290 */
	20, /* epoll_create1 - x86-64 syscall: 291 */
	24, /* dup3 - x86-64 syscall: 292 */
	59, /* pipe2 - x86-64 syscall: 293 */
	26, /* inotify_init1 - x86-64 syscall: 294 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	241, /* perf_event_open - x86-64 syscall: 298 */
	-1, /* not implemented in x86-64 */
	262, /* fanotify_init - x86-64 syscall: 300 */
	263, /* fanotify_mark - x86-64 syscall: 301 */
	261, /* prlimit64 - x86-64 syscall: 302 */
	264, /* name_to_handle_at - x86-64 syscall: 303 */
	265, /* open_by_handle_at - x86-64 syscall: 304 */
	266, /* clock_adjtime - x86-64 syscall: 305 */
	267, /* syncfs - x86-64 syscall: 306 */
	-1, /* not implemented in x86-64 */
	268, /* setns - x86-64 syscall: 308 */
	168, /* getcpu - x86-64 syscall: 309 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	272, /* kcmp - x86-64 syscall: 312 */
	273, /* finit_module - x86-64 syscall: 313 */
	274, /* sched_setattr - x86-64 syscall: 314 */
	275, /* sched_getattr - x86-64 syscall: 315 */
	276, /* renameat2 - x86-64 syscall: 316 */
	277, /* seccomp - x86-64 syscall: 317 */
	278, /* getrandom - x86-64 syscall: 318 */
	279, /* memfd_create - x86-64 syscall: 319 */
	-1, /* not implemented in x86-64 */
	280, /* bpf - x86-64 syscall: 321 */
	-1, /* not implemented in x86-64 */
	282, /* userfaultfd - x86-64 syscall: 323 */
	283, /* membarrier - x86-64 syscall: 324 */
	284, /* mlock2 - x86-64 syscall: 325 */
	285, /* copy_file_range - x86-64 syscall: 326 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	288, /* pkey_mprotect - x86-64 syscall: 329 */
	289, /* pkey_alloc - x86-64 syscall: 330 */
	290, /* pkey_free - x86-64 syscall: 331 */
	291, /* statx - x86-64 syscall: 332 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	-1, /* not implemented in x86-64 */
	134, /* rt_sigaction - x86-64 syscall: 512 */
	139, /* rt_sigreturn - x86-64 syscall: 513 */
	29, /* ioctl - x86-64 syscall: 514 */
	65, /* readv - x86-64 syscall: 515 */
	66, /* writev - x86-64 syscall: 516 */
	207, /* recvfrom - x86-64 syscall: 517 */
	211, /* sendmsg - x86-64 syscall: 518 */
	212, /* recvmsg - x86-64 syscall: 519 */
	221, /* execve - x86-64 syscall: 520 */
	117, /* ptrace - x86-64 syscall: 521 */
	136, /* rt_sigpending - x86-64 syscall: 522 */
	137, /* rt_sigtimedwait - x86-64 syscall: 523 */
	138, /* rt_sigqueueinfo - x86-64 syscall: 524 */
	132, /* sigaltstack - x86-64 syscall: 525 */
	107, /* timer_create - x86-64 syscall: 526 */
	184, /* mq_notify - x86-64 syscall: 527 */
	104, /* kexec_load - x86-64 syscall: 528 */
	95, /* waitid - x86-64 syscall: 529 */
	99, /* set_robust_list - x86-64 syscall: 530 */
	100, /* get_robust_list - x86-64 syscall: 531 */
	75, /* vmsplice - x86-64 syscall: 532 */
	239, /* move_pages - x86-64 syscall: 533 */
	69, /* preadv - x86-64 syscall: 534 */
	70, /* pwritev - x86-64 syscall: 535 */
	240, /* rt_tgsigqueueinfo - x86-64 syscall: 536 */
	243, /* recvmmsg - x86-64 syscall: 537 */
	269, /* sendmmsg - x86-64 syscall: 538 */
	270, /* process_vm_readv - x86-64 syscall: 539 */
	271, /* process_vm_writev - x86-64 syscall: 540 */
	208, /* setsockopt - x86-64 syscall: 541 */
	209, /* getsockopt - x86-64 syscall: 542 */
	0, /* io_setup - x86-64 syscall: 543 */
	2, /* io_submit - x86-64 syscall: 544 */
	281, /* execveat - x86-64 syscall: 545 */
	286, /* preadv2 - x86-64 syscall: 546 */
	287, /* pwritev2 - x86-64 syscall: 547 */
};

long syscall(long n, ...)
{
	va_list ap;
	syscall_arg_t a,b,c,d,e,f;
	va_start(ap, n);
	a=va_arg(ap, syscall_arg_t);
	b=va_arg(ap, syscall_arg_t);
	c=va_arg(ap, syscall_arg_t);
	d=va_arg(ap, syscall_arg_t);
	e=va_arg(ap, syscall_arg_t);
	f=va_arg(ap, syscall_arg_t);
	va_end(ap);

	if (n > syscall_remap_len) {
		fprintf(stderr, "SGX-MUSL-LKL WARN: x86-64 syscall %d is greater than known syscall table length %d\n", n, syscall_remap_len);
		errno = ENOSYS;
		return -ENOSYS;
	}
	long calln = syscall_remap[n];
	if (calln == -1) {
		fprintf(stderr, "SGX-MUSL-LKL WARN: x86-64 syscall %d has no known mapping\n", n);
		errno = ENOSYS;
		return -ENOSYS;
	}

	/* AUTOGENERATED CODE BEGINS BELOW: */
	switch (n) {
	case 15:
	case 24:
	case 34:
	case 39:
	case 57:
	case 58:
	case 102:
	case 104:
	case 107:
	case 108:
	case 110:
	case 111:
	case 112:
	case 152:
	case 153:
	case 162:
	case 174:
	case 177:
	case 178:
	case 180:
	case 181:
	case 182:
	case 183:
	case 184:
	case 185:
	case 186:
	case 214:
	case 215:
	case 219:
	case 236:
	case 253:
		return __syscall_ret(__syscall(calln));
	case 3:
	case 12:
	case 22:
	case 25:
	case 32:
	case 37:
	case 60:
	case 63:
	case 67:
	case 74:
	case 75:
	case 80:
	case 81:
	case 84:
	case 87:
	case 95:
	case 99:
	case 100:
	case 105:
	case 106:
	case 121:
	case 122:
	case 123:
	case 124:
	case 134:
	case 135:
	case 145:
	case 146:
	case 147:
	case 151:
	case 156:
	case 159:
	case 161:
	case 163:
	case 168:
	case 172:
	case 201:
	case 205:
	case 207:
	case 211:
	case 213:
	case 218:
	case 225:
	case 226:
	case 231:
	case 241:
	case 272:
	case 284:
	case 291:
	case 294:
	case 306:
	case 323:
	case 331:
		return __syscall_ret(__syscall(calln,a));
	case 4:
	case 5:
	case 6:
	case 11:
	case 21:
	case 33:
	case 35:
	case 36:
	case 48:
	case 50:
	case 62:
	case 68:
	case 73:
	case 76:
	case 77:
	case 79:
	case 82:
	case 83:
	case 85:
	case 86:
	case 88:
	case 90:
	case 91:
	case 96:
	case 97:
	case 98:
	case 109:
	case 113:
	case 114:
	case 115:
	case 116:
	case 125:
	case 126:
	case 127:
	case 130:
	case 131:
	case 132:
	case 136:
	case 137:
	case 138:
	case 140:
	case 142:
	case 143:
	case 148:
	case 149:
	case 150:
	case 155:
	case 160:
	case 164:
	case 166:
	case 167:
	case 170:
	case 171:
	case 176:
	case 197:
	case 198:
	case 199:
	case 200:
	case 206:
	case 224:
	case 227:
	case 228:
	case 229:
	case 235:
	case 244:
	case 252:
	case 255:
	case 273:
	case 283:
	case 287:
	case 290:
	case 293:
	case 300:
	case 305:
	case 308:
	case 319:
	case 324:
	case 330:
		return __syscall_ret(__syscall(calln,a,b));
	case 0:
	case 1:
	case 2:
	case 7:
	case 8:
	case 10:
	case 16:
	case 19:
	case 20:
	case 26:
	case 27:
	case 28:
	case 29:
	case 30:
	case 31:
	case 38:
	case 41:
	case 42:
	case 43:
	case 46:
	case 47:
	case 49:
	case 51:
	case 52:
	case 59:
	case 64:
	case 65:
	case 71:
	case 72:
	case 78:
	case 89:
	case 92:
	case 93:
	case 94:
	case 103:
	case 117:
	case 118:
	case 119:
	case 120:
	case 129:
	case 133:
	case 139:
	case 141:
	case 144:
	case 154:
	case 158:
	case 173:
	case 175:
	case 187:
	case 194:
	case 195:
	case 196:
	case 203:
	case 204:
	case 209:
	case 210:
	case 212:
	case 217:
	case 222:
	case 234:
	case 238:
	case 245:
	case 251:
	case 254:
	case 258:
	case 261:
	case 263:
	case 266:
	case 268:
	case 269:
	case 274:
	case 282:
	case 292:
	case 304:
	case 309:
	case 313:
	case 314:
	case 317:
	case 318:
	case 321:
	case 325:
		return __syscall_ret(__syscall(calln,a,b,c));
	case 13:
	case 14:
	case 17:
	case 18:
	case 40:
	case 53:
	case 61:
	case 66:
	case 69:
	case 101:
	case 128:
	case 169:
	case 179:
	case 191:
	case 192:
	case 193:
	case 220:
	case 221:
	case 223:
	case 230:
	case 232:
	case 233:
	case 240:
	case 246:
	case 249:
	case 256:
	case 257:
	case 259:
	case 262:
	case 264:
	case 267:
	case 276:
	case 277:
	case 278:
	case 280:
	case 285:
	case 286:
	case 288:
	case 289:
	case 297:
	case 302:
	case 307:
	case 315:
	case 329:
		return __syscall_ret(__syscall(calln,a,b,c,d));
	case 23:
	case 54:
	case 55:
	case 56:
	case 70:
	case 157:
	case 165:
	case 188:
	case 189:
	case 190:
	case 208:
	case 216:
	case 239:
	case 242:
	case 243:
	case 247:
	case 248:
	case 250:
	case 260:
	case 265:
	case 271:
	case 295:
	case 296:
	case 298:
	case 299:
	case 301:
	case 303:
	case 312:
	case 316:
	case 320:
	case 322:
		return __syscall_ret(__syscall(calln,a,b,c,d,e));
	case 9:
	case 44:
	case 45:
	case 202:
	case 237:
	case 270:
	case 275:
	case 279:
	case 281:
	case 310:
	case 311:
	case 326:
	case 327:
	case 328:
		return __syscall_ret(__syscall(calln,a,b,c,d,e,f));
	}
	abort();
}
