// SPDX-License-Identifier: MPL-2.0

#include <sys/time.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>

#define ITERATIONS 10000

int main() {
	struct timeval start, end;
	pid_t pid = fork();
	
	if (pid == 0) {
		for (int i = 0; i < ITERATIONS; i++) {
			sched_yield();
		}
		exit(0);
	} else {
		gettimeofday(&start, NULL);
		for (int i = 0; i < ITERATIONS; i++) {
			sched_yield();
			waitpid(pid, NULL, WNOHANG);
		}
		wait(NULL);
		gettimeofday(&end, NULL);
		
		long microseconds = (end.tv_sec - start.tv_sec) * 1000000 +
						(end.tv_usec - start.tv_usec);
		printf("Average context switch time: %ld ns\n",
			(microseconds * 1000) / (ITERATIONS * 2));
	}
}