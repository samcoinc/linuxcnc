#ifndef RTAPI_USPACE_HH
#define RTAPI_USPACE_HH
#include <sys/fsuid.h>
#include <unistd.h>

struct WithRoot
{
    WithRoot() { if(!level) setfsuid(geteuid()); level++; }
    ~WithRoot() { --level; if(!level) setfsuid(getuid()); }
    static int level;
};

struct RtapiApp
{

    RtapiApp(int policy = SCHED_OTHER) : policy(policy), period(0) {}

    int prio_highest();
    int prio_lowest();
    int prio_next_higher(int prio);
    int prio_next_lower(int prio);
    long clock_set_period(long int period_nsec);
    int task_new(void (*taskcode)(void*), void *arg,
            int prio, int owner, unsigned long int stacksize, int uses_fp);
    virtual int task_delete(int id) = 0;
    virtual int task_start(int task_id, unsigned long period_nsec) = 0;
    virtual int task_pause(int task_id) = 0;
    virtual int task_resume(int task_id) = 0;
    virtual void wait() = 0;
    virtual unsigned char do_inb(unsigned int port) = 0;
    virtual void do_outb(unsigned char value, unsigned int port) = 0;
    virtual int run_threads(int fd, int (*callback)(int fd)) = 0;
    int policy;
    long period;
};

#define MAX_TASKS  64
#define TASK_MAGIC    21979	/* random numbers used as signatures */

struct rtapi_task {
  int magic;			/* to check for valid handle */
  int owner;
  pthread_t thr;                /* thread's context */
  size_t stacksize;
  int prio;
  long period;
  struct timespec nextstart;
  unsigned ratio;
  void *arg;
  void (*taskcode) (void*);	/* pointer to task function */
};

extern struct rtapi_task task_array[MAX_TASKS];

#define WITH_ROOT WithRoot root
#endif
