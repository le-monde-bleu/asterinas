// SPDX-License-Identifier: MPL-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/resource.h>

#define NUM_THREADS 100
#define TEST_DURATION 10 // seconds

void *busy_work(void *arg) {
	struct timeval start, end;
	gettimeofday(&start, NULL);
	while (1) {
		gettimeofday(&end, NULL);
		if ((end.tv_sec - start.tv_sec) >= TEST_DURATION) {
			break;
		}
	}
	return NULL;
}

int main() {
	pthread_t threads[NUM_THREADS];
	struct rusage usage;

	// Create multiple threads
	for (int i = 0; i < NUM_THREADS; i++) {
		if (pthread_create(&threads[i], NULL, busy_work, NULL) != 0) {
			perror("Failed to create thread");
			exit(EXIT_FAILURE);
		}
	}

	// Wait for all threads to finish
	for (int i = 0; i < NUM_THREADS; i++) {
		pthread_join(threads[i], NULL);
	}

	// Get resource usage
	getrusage(RUSAGE_SELF, &usage);

	// Print CPU time used by the process
	printf("User time = %ld.%06ld, System time = %ld.%06ld\n",
		   usage.ru_utime.tv_sec, usage.ru_utime.tv_usec,
		   usage.ru_stime.tv_sec, usage.ru_stime.tv_usec);

	return 0;
}