// SPDX-License-Identifier: MPL-2.0

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>

void print_process_info(const char *label) {
	printf("%s: PID=%d, PPID=%d\n", label, getpid(), getppid());
}

void create_child_process(const char *label, int level) {
	pid_t pid = fork();
	if (pid < 0) {
		perror("fork failed");
		exit(EXIT_FAILURE);
	} else if (pid == 0) {
		// Child process
		print_process_info(label);
		if (level > 0) {
			char new_label[256];
			snprintf(new_label, sizeof(new_label), "%s-child", label);
			create_child_process(new_label, level - 1);
		}
		exit(EXIT_SUCCESS);
	} else {
		// Parent process
		wait(NULL);
	}
}

int main() {
	printf("Testing multi-level fork:\n");
	print_process_info("Parent");
	create_child_process("Level0", 0); // Create a 1-level process hierarchy
	create_child_process("Level1", 1); // Create a 2-level process hierarchy
	create_child_process("Level2", 2); // Create a 3-level process hierarchy
	create_child_process("Level3", 3); // Create a 4-level process hierarchy
	create_child_process("Level10", 10); // Create a 11-level process hierarchy
	create_child_process("Level50", 50); // Create a 51-level process hierarchy
	return 0;
}
