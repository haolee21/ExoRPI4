#ifndef COMMON_HPP
#define COMMON_HPP

// this is the
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h> // Needed for mlockall()
#include <unistd.h>   // needed for sysconf(int name);
#include <malloc.h>
#include <sys/time.h>     // needed for getrusage
#include <sys/resource.h> // needed for getrusage
#include <pthread.h>
#include <limits.h>
#include <vector>
#include <sched.h>

#define PRE_ALLOCATION_SIZE (100 * 1024 * 1024) /* 100MB pagefault free buffer */
#define MY_STACK_SIZE (1024 * 1024)              /* 1MB for each thread. */
class RT
//this is the RT class for Preempt-RT kernel, it is a singleton class because I don't think one system can have two stable RT thread
//The class only starts the pthread, making sure enough memory is assigned
{
private:
    RT();
    bool init_flag = false;
    static void show_new_pagefault_count(const char *logtext,
                                         const char *allowed_maj, const char *allowed_min);
    static void prove_thread_stack_use_is_safe(int stacksize);
    static void configure_malloc_behavior(void);
    static void reserve_process_memory(int size);
 

public:
    static RT &GetInstance();
    RT(const RT &) = delete;

    ~RT();

    static int StartThread(pthread_t &thread,void*(*)(void*),int32_t priority);

    static void Init();
    static const int32_t RT_PRIORITY = 80;
};
#endif
