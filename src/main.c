#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include "lsi11.h"
#include "trace.h"

/* #define DEBUG */

struct termios original_tio;

volatile int running;

/* RX02 double density bootstrap */
const char* odt_input =
	"2000/012701\r"
	"2002/177170\r"
	"2004/012700\r"
	"2006/100240\r"
	"2010/005002\r"
	"2012/012705\r"
	"2014/000200\r"
	"2016/012704\r"
	"2020/000401\r"
	"2022/012703\r"
	"2024/177172\r"
	"2026/030011\r"
	"2030/001776\r"
	"2032/100440\r"
	"2034/012711\r"
	"2036/000407\r"
	"2040/030011\r"
	"2042/001776\r"
	"2044/100433\r"
	"2046/110413\r"
	"2050/000304\r"
	"2052/030011\r"
	"2054/001776\r"
	"2056/110413\r"
	"2060/000304\r"
	"2062/030011\r"
	"2064/001776\r"
	"2066/100422\r"
	"2070/012711\r"
	"2072/000403\r"
	"2074/030011\r"
	"2076/001776\r"
	"2100/100415\r"
	"2102/010513\r"
	"2104/030011\r"
	"2106/001776\r"
	"2110/100411\r"
	"2112/010213\r"
	"2114/060502\r"
	"2116/060502\r"
	"2120/122424\r"
	"2122/120427\r"
	"2124/000007\r"
	"2126/003737\r"
	"2130/005000\r"
	"2132/005007\r"
	"2134/000000\r"
	"2000G";

void LSI11ConsoleSend(DLV11J* dlv, const char c)
{
	DLV11JSend(dlv, 3, c);
}

void LSI11ConsoleSendString(DLV11J* dlv, const char* s)
{
	for(; *s; s++) {
		LSI11ConsoleSend(dlv, *s);
	}
}

void console_print(unsigned char c)
{
	printf("%c", c);
	fflush(stdout);
}

void sigint_handler(int signum)
{
	running = 0;
}

#define	READ(addr)		lsi.bus.read(lsi.bus.user, (addr))
#define	WRITE(addr, val)	lsi.bus.write(lsi.bus.user, (addr), (val))

int main(int argc, char** argv)
{
	LSI11 lsi;
	MSV11D msv11;
	RXV21 rxv21;
	DLV11J dlv11;
	BDV11 bdv11;

	fd_set fds;
	struct timeval tv;

	struct timespec now;
	struct timespec last;

	FILE* floppy_file;
	u8* floppy;

	struct termios tio;
	struct sigaction sa;

	const char* floppy_filename = NULL;
	const char* load_file = NULL;
	int halt = 0;
	int bootstrap = 0;
	const char* trace_file = NULL;
	int compress = 0;

#ifndef DEBUG
	if(tcgetattr(0, &original_tio) == -1) {
		printf("Failed to retrieve TTY configuration\n");
		return 1;
	}
#endif

	MSV11DInit(&msv11);
	DLV11JInit(&dlv11);
	BDV11Init(&bdv11);
	RXV21Init(&rxv21);

	argc--;
	argv++;

	for(; argc; argc--, argv++) {
		if(!strcmp("-h", *argv)) {
			halt = 1;
		} else if(!strcmp("-b", *argv)) {
			bootstrap = 1;
		} else if(!strcmp("-z", *argv)) {
			compress = 1;
		} else if(!strcmp("-l", *argv) && argc > 1) {
			load_file = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-f", *argv) && argc > 1) {
			floppy_filename = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-t", *argv) && argc > 1) {
			trace_file = argv[1];
			argc--;
			argv++;
		} else if(**argv != '-') {
			load_file = *argv;
		} else {
			printf("Unknown option: %s\n", *argv);
			return 1;
		}
	}

	floppy = (u8*) malloc(77 * 26 * 256);
	if(!floppy) {
		printf("Error: cannot allocate memory\n");
		return 1;
	}

	if(floppy_filename) {
		floppy_file = fopen(floppy_filename, "rb");
		if(!floppy_file) {
			printf("Error: cannot open file %s\n", floppy_filename);
			free(floppy);
			return 1;
		}
		fread(floppy, 77 * 26 * 256, 1, floppy_file);
		fclose(floppy_file);
	} else {
		memset(floppy, 0, 77 * 26 * 256);
	}

	RXV21SetData(&rxv21, floppy);

	dlv11.channel[3].receive = console_print;

	if(trace_file) {
		TRCINIT(trace_file);
		if(compress) {
			trc.flags |= TRACE_COMPRESS;
		}
	}
	/* trc.flags |= TRACE_PRINT; */

	LSI11Init(&lsi);
	LSI11InstallModule(&lsi, 1, &msv11.module);
	LSI11InstallModule(&lsi, 2, &rxv21.module);
	LSI11InstallModule(&lsi, 3, &dlv11.module);
	LSI11InstallModule(&lsi, 4, &bdv11.module);
	LSI11Reset(&lsi);

	if(bootstrap) {
		LSI11ConsoleSendString(&dlv11, odt_input);
	}
	if(load_file) {
		/* execute absolute loader binary */
		/* const char* filename = "VKAAC0.BIC"; */
		/* const char* filename = "VKADC0.BIC"; */
		const char* filename = load_file;
		size_t size;
		u16 len;
		u16 addr;
		u8 cksum;
		size_t bytes = 0;
		FILE* f = fopen(filename, "rb");
		if(!f) {
			printf("error opening file!\n");
			return 1;
		}
		fseek(f, 0, SEEK_END);
		size = ftell(f);
		fseek(f, 0, SEEK_SET);
		printf("loading %s [%ld bytes]...\n", filename, size);
		fflush(stdout);
		while(bytes < size) {
			int c;
			while((c = fgetc(f)) != EOF) {
				if(c == 1) {
					break;
				}
			}
			if(c == 1) {
				c = fgetc(f);
			}
			if(c == EOF) {
				printf("error: unexpected EOF\n");
				fclose(f);
				free(floppy);
				return 1;
			} else if(c != 0) {
				printf("error: invalid signature! [%02x]\n", c);
				fclose(f);
				free(floppy);
				return 1;
			}
			fread(&len, 2, 1, f);
			fread(&addr, 2, 1, f);
			bytes += len;
			printf("%06o: %d bytes\n", addr, len - 6);
			fread(&msv11.data[addr], len - 6, 1, f);
			TRCMemoryDump(&msv11.data[addr], addr, len - 6);
			fread(&cksum, 1, 1, f);
			if(len == 6) {
				if((addr & 1) == 0) {
					lsi.cpu.r[7] = addr;
					/* LSI11ConsoleSendString(&dlv11, "P"); */
					lsi.cpu.state = 1;
				} else {
					/* LSI11ConsoleSendString(&dlv11, "200G"); */
					lsi.cpu.r[7] = 0200;
					lsi.cpu.state = 1;
				}
				break;
			}
		}
		fclose(f);
	}

#ifndef DEBUG
	sa.sa_handler = sigint_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	if(sigaction(SIGINT, &sa, NULL) == -1) {
		printf("WARNING: failed to set SIGINT signal handler\n");
	}

	/* set raw mode */
	tio = original_tio;
	tio.c_iflag &= ~(ICRNL | INPCK | ISTRIP);
	tio.c_oflag &= ~OPOST;
	tio.c_cflag |= CS8;
	tio.c_lflag &= ~(ECHO | ICANON | IEXTEN);
	tcsetattr(0, TCSANOW, &tio);
#endif

	running = 1;

	if(halt) {
		lsi.cpu.state = 0;
	} else if(!bootstrap && !halt && !load_file) {
		lsi.cpu.state = 1;
	}

	clock_gettime(CLOCK_MONOTONIC, &last);

	while(running) {
		unsigned int i;
		double dt;

		FD_ZERO(&fds);
		FD_SET(0, &fds);
		tv.tv_sec = 0;
		tv.tv_usec = 0;

		if(select(1, &fds, NULL, NULL, &tv) == 1) {
			char c;
			read(0, &c, 1);
#ifdef DEBUG
			if(c == '\n')
				c = '\r';
#endif
			DLV11JSend(&dlv11, 3, c);
		}

		for(i = 0; i < 1000; i++)
			LSI11Step(&lsi);

		clock_gettime(CLOCK_MONOTONIC, &now);
		dt = (now.tv_sec - last.tv_sec) +
			(now.tv_nsec - last.tv_nsec) / 1e9;
		last = now;
		BDV11Step(&bdv11, dt);
	}

	free(floppy);

	DLV11JDestroy(&dlv11);
	BDV11Destroy(&bdv11);
	RXV21Destroy(&rxv21);
	MSV11DDestroy(&msv11);
	LSI11Destroy(&lsi);

	tcsetattr(0, TCSANOW, &original_tio);

	printf("\n");

	if(trace_file) {
		TRCFINISH();
	}

	return 0;
}
