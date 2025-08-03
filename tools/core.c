#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/stat.h>

typedef struct {
	uint16_t	r[8];
	uint16_t	psw;
} CPUSTATE;

int main(int argc, char** argv)
{
	if(argc != 2) {
		fprintf(stderr, "Usage: %s dump.core\n", *argv);
		return 1;
	}

	const char* filename = argv[1];

	struct stat buf;
	if(stat(filename, &buf) < 0) {
		fprintf(stderr, "stat failed: %s\n", strerror(errno));
		return 1;
	}

	const size_t trailer = 9 * 2;
	size_t memsize = buf.st_size - trailer;

	printf("RAM=%02dkW\n", memsize / 2 / 1024);

	FILE* in = fopen(filename, "rb");
	if(!in) {
		fprintf(stderr, "Failed to open file %s: %s\n", filename,
				strerror(errno));
		return 1;
	}

	fseek(in, memsize, SEEK_SET);

	CPUSTATE state;
	fread(&state, 9 * 2, 1, in);

	fclose(in);

	for(int i = 0; i < 8; i++) {
		printf("R%d=%06o\n", i, state.r[i]);
	}
	printf("RS=%06o\n", state.psw);

	return 0;
}
