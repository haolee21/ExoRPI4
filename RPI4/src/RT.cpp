#include "RT.hpp"
#include <iostream>
void RT::show_new_pagefault_count(const char *logtext,
                                  const char *allowed_maj, const char *allowed_min)
{
    static int last_majflt = 0, last_minflt = 0;
    struct rusage usage;

    getrusage(RUSAGE_SELF, &usage);

    printf("%-30.30s: Pagefaults, Major:%ld (Allowed %s), "
           "Minor:%ld (Allowed %s)\n",
           logtext,
           usage.ru_majflt - last_majflt, allowed_maj,
           usage.ru_minflt - last_minflt, allowed_min);

    last_majflt = usage.ru_majflt;
    last_minflt = usage.ru_minflt;
}
void RT::prove_thread_stack_use_is_safe(int stacksize)
{
    volatile char buffer[stacksize];
    
    buffer[0]=0;
    /* Prove that this thread is behaving well */
    for (int i = 0; i < stacksize; i += sysconf(_SC_PAGESIZE))
    {
        /* Each write to this buffer shall NOT generate a 
   			pagefault. */
        buffer[i] = i;
    }

    show_new_pagefault_count("Caused by using thread stack", "0", "0");
}
void RT::configure_malloc_behavior(void)
{
    /* Now lock all current and future pages 
   	   from preventing of being paged */
    if (mlockall(MCL_CURRENT | MCL_FUTURE))
        perror("mlockall failed:");

    /* Turn off malloc trimming.*/
    mallopt(M_TRIM_THRESHOLD, -1);

    /* Turn off mmap usage. */
    mallopt(M_MMAP_MAX, 0);
}

void RT::reserve_process_memory(int size)
{
    int i;
    char *buffer;

    buffer = (char *)malloc(size);

    /* Touch each page in this piece of memory to get it mapped into RAM */
    for (i = 0; i < size; i += sysconf(_SC_PAGESIZE))
    {
        /* Each write to this buffer will generate a pagefault.
   		   Once the pagefault is handled a page will be locked in
   		   memory and never given back to the system. */
        buffer[i] = 0;
    }

    /* buffer will now be released. As Glibc is configured such that it 
   	   never gives back memory to the kernel, the memory allocated above is
   	   locked for this process. All malloc() and new() calls come from
   	   the memory pool reserved and locked above. Issuing free() and
   	   delete() does NOT make this locking undone. So, with this locking
   	   mechanism we can build C++ applications that will never run into
   	   a major/minor pagefault, even with swapping enabled. */
    free(buffer);
}

RT &RT::GetInstance()
{
    static RT instance;
    return instance;
}
void RT::Init()
{
    if (!RT::GetInstance().init_flag)
    { //we only lock the memory once
        RT::GetInstance().init_flag = true;
        RT::show_new_pagefault_count("Initial count", ">=0", ">=0");
        RT::configure_malloc_behavior();
        RT::show_new_pagefault_count("mlockall() generated", ">=0", ">=0");
        RT::reserve_process_memory(PRE_ALLOCATION_SIZE);
        RT::show_new_pagefault_count("malloc() and touch generated", ">=0", ">=0");
        RT::reserve_process_memory(PRE_ALLOCATION_SIZE);
        RT::show_new_pagefault_count("2nd malloc() and use generated", "0", "0");
        printf("\n\nLook at the output of ps -leyf, and see that the "
               "RSS is now about %d [MB]\n",
               PRE_ALLOCATION_SIZE / (1024 * 1024));
    }
    else
    {
        std::cout << "RT has already setup\n";
    }
}

int RT::StartThread(pthread_t &thread, void *(*task)(void *),int priority)
{
    if (!RT::GetInstance().init_flag)
    {
        RT::Init();
    }

    pthread_attr_t attr;
    struct sched_param param;
    /* init to default values */
    if (pthread_attr_init(&attr))
    {
        std::cout << "init pthread attributes failed\n";

        exit(1);
    }

    if (pthread_attr_setschedpolicy(&attr, SCHED_RR))
    {
        std::cout << "pthread setschedpolicy failed\n";
        exit(1);
    }

    param.sched_priority = priority;
    if (pthread_attr_setschedparam(&attr, &param))
    {
        std::cout << "pthread setschedparam failed\n";
        exit(1);
    }

    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED))
    {
        std::cout << "pthread setinheritsched failed\n";
        exit(1);
    }

    /* Set the requested stacksize for this thread */
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + MY_STACK_SIZE))
    {
        std::cout << "pthread setstacksize failed\n";
        exit(1);
    }
    /* And finally start the actual thread */
    // The RT class will not monitor the thread, it is the user's duty to join the threads when it ends
    pthread_create(&thread, &attr, task, NULL);
    std::cout<<"RT thread created\n";
    return 0;
}

RT::RT()
{
}
RT::~RT()
{
}