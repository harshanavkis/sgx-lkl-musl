#include <time.h>
#include <errno.h>
#include <stdint.h>
#include "syscall.h"
#include "atomic.h"
#include "sgxlkl_debug.h"

#ifdef VDSO_CGT_SYM

static void *volatile vdso_func;

static int cgt_init(clockid_t clk, struct timespec *ts)
{
	void *p = __vdsosym(VDSO_CGT_VER, VDSO_CGT_SYM);
	int (*f)(clockid_t, struct timespec *) =
		(int (*)(clockid_t, struct timespec *))p;
	a_cas_p(&vdso_func, (void *)cgt_init, p);
	return f ? f(clk, ts) : -ENOSYS;
}

static void *volatile vdso_func = (void *)cgt_init;

#endif

/* Straight from linux::arch/x86/include/asm/vgtod.h */
typedef uint64_t gtod_long_t;
#define CLOCK_REALTIME			0
#define CLOCK_MONOTONIC			1
#define CLOCK_PROCESS_CPUTIME_ID	2
#define CLOCK_THREAD_CPUTIME_ID		3
#define CLOCK_MONOTONIC_RAW		4
#define CLOCK_REALTIME_COARSE		5
#define CLOCK_MONOTONIC_COARSE		6
#define CLOCK_BOOTTIME			7
#define CLOCK_REALTIME_ALARM		8
#define CLOCK_BOOTTIME_ALARM		9
/*
 * The driver implementing this got removed. The clock ID is kept as a
 * place holder. Do not reuse!
 */
#define CLOCK_SGI_CYCLE			10
#define CLOCK_TAI			11
#define VDSO_BASES	(CLOCK_TAI + 1)
struct vdso_timestamp {
	uint64_t	sec;
	uint64_t	nsec;
};
/*  linux/include/vdso/datapage.h  */
struct vdso_data {
	uint32_t seq;

	int32_t clock_mode;
	uint64_t	cycle_last;
	uint64_t	mask;
	uint32_t	mult;
	uint32_t	shift;

	struct vdso_timestamp	basetime[VDSO_BASES];
	//union {
	//	struct vdso_timestamp	basetime[VDSO_BASES];
	//	struct timens_offset	offset[VDSO_BASES];
	//};

	int32_t		tz_minuteswest;
	int32_t		tz_dsttime;
	int32_t		hrtimer_res;
	int32_t		__unused;
};

/* Straight from linux::arch/x86/include/asm/vvar.h */
static int __vdso_data_offset = 128;

static inline uint32_t
__iter_div_u64_rem(uint64_t dividend, uint32_t divisor, uint64_t *remainder)
{
	uint32_t ret = 0;

	while (dividend >= divisor) {
		/* The following asm() prevents the compiler from
		   optimising this loop into a modulo operation.  */
		__asm__("" : "+rm"(dividend));

		dividend -= divisor;
		ret++;
	}

	*remainder = dividend;

	return ret;
}

static int vdso_read_begin(const struct vdso_data *s)
{
	unsigned ret;

retry:
	ret = s->seq;
	if (ret & 1)
		goto retry;


	a_barrier();
	return ret;
}

static int vdso_read_retry(const struct vdso_data *s, unsigned start)
{
	a_barrier();
	return s->seq != start;
}

enum vdso_clock_mode {
	VDSO_CLOCKMODE_NONE,
	VDSO_CLOCKMODE_TSC,
	VDSO_CLOCKMODE_PVCLOCK,
	VDSO_CLOCKMODE_HVCLOCK,
	VDSO_CLOCKMODE_MAX,
	/* Indicator for time namespace VDSO */
	VDSO_CLOCKMODE_TIMENS = INT_MAX,
};


#ifndef SGXLKL_HW

static uint64_t rdtsc_ordered(void)
{
	uint64_t low, high, ret;
	a_barrier();
	__asm("rdtscp" : "=a"(low), "=d"(high) : : "rcx");
	return (high << 32) + low;
}

static uint64_t vgetsns(const volatile struct vdso_data *s, int volatile *mode)
{
	uint64_t v;
	uint64_t cycles;
	if (s->clock_mode == VDSO_CLOCKMODE_TSC) {
		uint64_t rdtsc = (uint64_t)rdtsc_ordered();
		uint64_t last = s->cycle_last;
		cycles = (rdtsc >= last) ? rdtsc : last;
	} else
		return 0;

	v = (cycles - s->cycle_last) & s->mask;
	return v * s->mult;
}
#endif /* SGXLKL_HW */

int __clock_gettime(clockid_t clk, struct timespec *ts)
{
	int r;
#ifdef VDSO_CGT_SYM
	int (*f)(clockid_t, struct timespec *) =
		(int (*)(clockid_t, struct timespec *))vdso_func;
	if (f) {
		r = f(clk, ts);
		if (!r) return r;
		if (r == -EINVAL) return __syscall_ret(r);
		/* Fall through on errors other than EINVAL. Some buggy
		 * vdso implementations return ENOSYS for clocks they
		 * can't handle, rather than making the syscall. This
		 * also handles the case where cgt_init fails to find
		 * a vdso function to use. */
	}
#endif

	if (libc.vvar_base && (clk == CLOCK_REALTIME || clk == CLOCK_MONOTONIC ||
	                       clk == CLOCK_REALTIME_COARSE || clk == CLOCK_MONOTONIC_COARSE)) {
		volatile struct vdso_data *ptr;
		unsigned seq;
		uint64_t ns;

                ptr = (struct vdso_data *)((char *)libc.vvar_base + __vdso_data_offset);

//		do {
			//seq = vdso_read_begin(ptr);
		seq = ptr->seq;
		if (ptr->clock_mode == VDSO_CLOCKMODE_TIMENS) {
			fprintf(stderr, "Getting the time inside a container not supported right now!\n");
			exit(1);
		}
		ts->tv_sec = ptr->basetime[clk].sec;
		ns = ptr->basetime[clk].nsec;
		if ((clk == CLOCK_REALTIME || clk == CLOCK_MONOTONIC)) {
#ifndef SGXLKL_HW
			// This requires (efficient) RDTSC support which we don't have
			// in SGX v1 where RDTSC instructions are illegal.
			ns += vgetsns(ptr, &ptr->vclock_mode);
#endif /* SGXLKL_HW */
			ns >>= ptr->shift;
		}
//		} while (vdso_read_retry(ptr, seq));

		ts->tv_sec += __iter_div_u64_rem(ns, 1000000000L, &ns);
		ts->tv_nsec = ns;

		return __syscall_ret(0);
	}

	r = __syscall(SYS_clock_gettime, clk, ts);
	if (r == -ENOSYS) {
		if (clk == CLOCK_REALTIME) {
			__syscall(SYS_gettimeofday, ts, 0);
			ts->tv_nsec = (int)ts->tv_nsec * 1000;
			return 0;
		}
		r = -EINVAL;
	}
	return __syscall_ret(r);
}

weak_alias(__clock_gettime, clock_gettime);
