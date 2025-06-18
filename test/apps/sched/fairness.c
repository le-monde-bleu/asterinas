// SPDX-License-Identifier: MPL-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sched.h>

#define NUM_PROCESSES 5
#define TEST_DURATION 10 // seconds

void busy_work() {
	struct timeval start, end;
	gettimeofday(&start, NULL);
	while (1) {
		gettimeofday(&end, NULL);
		if ((end.tv_sec - start.tv_sec) >= TEST_DURATION) {
			break;
		}
	}
}

void print_usage(struct rusage *usage, const char *label) {
	printf("%s: User time = %ld.%06ld, System time = %ld.%06ld\n",
		   label,
		   usage->ru_utime.tv_sec, usage->ru_utime.tv_usec,
		   usage->ru_stime.tv_sec, usage->ru_stime.tv_usec);
}

int main() {
	pid_t pids[NUM_PROCESSES];
	struct rusage usage[NUM_PROCESSES];

	// Fork multiple processes
	for (int i = 0; i < NUM_PROCESSES; i++) {
		pids[i] = fork();
		if (pids[i] == 0) {
			// Child process
			busy_work();
			exit(0);
		}
	}

	// Wait for all child processes to finish
	for (int i = 0; i < NUM_PROCESSES; i++) {
		wait4(pids[i], NULL, 0, &usage[i]);
	}

	// Print CPU time used by each process
	for (int i = 0; i < NUM_PROCESSES; i++) {
		char label[20];
		snprintf(label, sizeof(label), "Process %d", pids[i]);
		print_usage(&usage[i], label);
	}

	// Calculate and print the fairness
	long total_user_time = 0, total_system_time = 0;
	for (int i = 0; i < NUM_PROCESSES; i++) {
		total_user_time += usage[i].ru_utime.tv_sec * 1000000 + usage[i].ru_utime.tv_usec;
		total_system_time += usage[i].ru_stime.tv_sec * 1000000 + usage[i].ru_stime.tv_usec;
	}
	long avg_user_time = total_user_time / NUM_PROCESSES;
	long avg_system_time = total_system_time / NUM_PROCESSES;

	printf("Average User time = %ld.%06ld, Average System time = %ld.%06ld\n",
		   avg_user_time / 1000000, avg_user_time % 1000000,
		   avg_system_time / 1000000, avg_system_time % 1000000);

	return 0;
}