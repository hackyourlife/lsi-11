#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>

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

#define DISK_SIZE (2 * 40 * 512 * 256)
static u32 rl02_serial = 0;
void init_rl02(u8* disk)
{
	if(rl02_serial == 0) {
		struct timespec now;
		clock_gettime(CLOCK_REALTIME, &now);
		rl02_serial = (now.tv_sec & 0xFFFFFFFFL) ^ ((now.tv_sec >> 32) & 0xFFFFFFFFL);
	}

	u16 serial_lo = rl02_serial & 0x7F;
	u16 serial_hi = (rl02_serial >> 15) & 0x7F;
	rl02_serial++;

	memset(disk, 0, DISK_SIZE);
	memset(disk + 0x9fd800, 0xFF, 0x400);
	memset(disk + 0x9fec00, 0xFF, 0x400);

	disk[0x9fd800] = (unsigned char)  serial_lo;
	disk[0x9fd801] = (unsigned char) (serial_lo >> 8);
	disk[0x9fd802] = (unsigned char)  serial_hi;
	disk[0x9fd803] = (unsigned char) (serial_hi >> 8);
	disk[0x9fd804] = 0;
	disk[0x9fd805] = 0;
	disk[0x9fd806] = 0;
	disk[0x9fd807] = 0;

	disk[0x9fec00] = (unsigned char)  serial_lo;
	disk[0x9fec01] = (unsigned char) (serial_lo >> 8);
	disk[0x9fec02] = (unsigned char)  serial_hi;
	disk[0x9fec03] = (unsigned char) (serial_hi >> 8);
	disk[0x9fec04] = 0;
	disk[0x9fec05] = 0;
	disk[0x9fec06] = 0;
	disk[0x9fec07] = 0;
}

#define	READ(addr)		lsi.bus.read(lsi.bus.user, (addr))
#define	WRITE(addr, val)	lsi.bus.write(lsi.bus.user, (addr), (val))

#define	BOOT_NONE	0
#define	BOOT_RX02	1
#define	BOOT_RL02	2

int main(int argc, char** argv)
{
	LSI11 lsi;
	MSV11D msv11;
	RXV21 rxv21;
	RLV12 rlv12;
	DLV11J dlv11;
	BDV11 bdv11;

	fd_set fds;
	struct timeval tv;

	struct timespec now;
	struct timespec last;

	const char* self = *argv;
	FILE* floppy_file;
	u8* floppy0;
	u8* floppy1;
	int den0 = 1;
	int den1 = 1;
	u8* disk0 = NULL;
	u8* disk1 = NULL;
	u8* disk2 = NULL;
	u8* disk3 = NULL;

	int bootdev = BOOT_NONE;
	u16 dialog = BDV11_DIALOG;
	u16 tests = BDV11_CPU_TEST | BDV11_MEM_TEST;

	struct termios tio;
	struct sigaction sa;

	const char* floppy_filename0 = NULL;
	const char* floppy_filename1 = NULL;
	const char* disk_filename0 = NULL;
	const char* disk_filename1 = NULL;
	const char* disk_filename2 = NULL;
	const char* disk_filename3 = NULL;
	const char* load_file = NULL;
	int halt = 0;
	int bootstrap = 0;
	const char* trace_file = NULL;
	int compress = 1;

	int dump_disk0 = 0;
	int dump_disk1 = 0;
	int dump_disk2 = 0;
	int dump_disk3 = 0;

	int exit_on_halt = 0;
	int no_sigint = 0;

	int notty = 0;
	const char* console_input = NULL;

#ifndef DEBUG
	if(tcgetattr(0, &original_tio) == -1) {
		printf("Failed to retrieve TTY configuration\n");
		notty = 1;
	}
#endif

	argc--;
	argv++;

	for(; argc; argc--, argv++) {
		if(!strcmp("-h", *argv)) {
			halt = 1;
		} else if(!strcmp("-b", *argv)) {
			bootstrap = 1;
		} else if(!strcmp("-nz", *argv)) {
			compress = 0;
		} else if(!strcmp("-x", *argv)) {
			exit_on_halt = 1;
		} else if(!strcmp("-l", *argv) && argc > 1) {
			load_file = argv[1];
			argc--;
			argv++;
		} else if((!strcmp("-f", *argv) || !strcmp("-f0", *argv)) && argc > 1) {
			bootdev = BOOT_RX02;
			floppy_filename0 = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-f1", *argv) && argc > 1) {
			floppy_filename1 = argv[1];
			argc--;
			argv++;
		} else if((!strcmp("-d", *argv) || !strcmp("-d0", *argv)) && argc > 1) {
			bootdev = BOOT_RL02;
			disk_filename0 = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-d1", *argv) && argc > 1) {
			disk_filename1 = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-d2", *argv) && argc > 1) {
			disk_filename2 = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-d3", *argv) && argc > 1) {
			disk_filename3 = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-sd0", *argv) || !strcmp("-sd", *argv)) {
			dump_disk0 = 1;
		} else if(!strcmp("-sd1", *argv)) {
			dump_disk1 = 1;
		} else if(!strcmp("-sd2", *argv)) {
			dump_disk2 = 1;
		} else if(!strcmp("-sd3", *argv)) {
			dump_disk3 = 1;
		} else if(!strcmp("-q", *argv)) {
			dialog = 0;
		} else if(!strcmp("-n", *argv)) {
			tests = 0;
		} else if(!strcmp("-i", *argv)) {
			no_sigint = 1;
		} else if(!strcmp("-t", *argv) && argc > 1) {
			trace_file = argv[1];
			argc--;
			argv++;
		} else if(!strcmp("-c", *argv) && argc > 1) {
			console_input = argv[1];
			argc--;
			argv++;
		} else if(**argv != '-') {
			load_file = *argv;
		} else if(!strcmp("--help", *argv)) {
			printf("Usage: %s [OPTIONS] [FILE]\n"
					"\n"
					"OPTIONS\n"
					"  -h              Halt CPU\n"
					"  -x              Exit on HALT\n"
					"  -b              Enter RX02 double density bootstrap program\n"
					"  -l file.bin     Load file.bin in absolute loader format\n"
					"  -f file.rx2     Load RX02 floppy image from file.rx2\n"
					"  -d file.rl2     Load RL02 disk image from file.rl2\n"
					"  -sd             Store updated RL02 disk in file\n"
					"  -q              Skip BDV11 console test\n"
					"  -n              Skip BDV11 CPU and memory tests\n"
					"  -i              Ignore SIGINT, only CTRL-E exits the emulator\n"
					"  -t file.trc     Record execution trace to file.trc\n"
					"  -nz             Don't use delta compression for execution trace\n"
					"\n"
					"The optional last argument FILE is equivalent to -l file\n", self);
			return 0;
		} else {
			printf("Unknown option: %s\n", *argv);
			return 1;
		}
	}

	MSV11DInit(&msv11);
	DLV11JInit(&dlv11);
	BDV11Init(&bdv11);
	RXV21Init(&rxv21);
	RLV12Init(&rlv12);

	switch(bootdev) {
		default:
		case BOOT_NONE:
			BDV11SetSwitch(&bdv11, tests | dialog);
			break;
		case BOOT_RX02:
			BDV11SetSwitch(&bdv11, tests | dialog | BDV11_RX02);
			break;
		case BOOT_RL02:
			BDV11SetSwitch(&bdv11, tests | dialog | BDV11_RL01);
			break;
	}

	floppy0 = (u8*) malloc(77 * 26 * 256);
	floppy1 = (u8*) malloc(77 * 26 * 256);
	if(!floppy0 || !floppy1) {
		printf("Error: cannot allocate memory\n");
		return 1;
	}

	if(floppy_filename0) {
		struct stat buf;
		if(stat(floppy_filename0, &buf) == -1) {
			printf("Error: cannot stat file %s: %s\n", floppy_filename0, strerror(errno));
			free(floppy0);
			free(floppy1);
			return 1;
		}
		floppy_file = fopen(floppy_filename0, "rb");
		if(!floppy_file) {
			printf("Error: cannot open file %s\n", floppy_filename0);
			free(floppy0);
			free(floppy1);
			return 1;
		}
		int is_rx01 = buf.st_size == 77 * 26 * 128;
		int is_rx02 = buf.st_size == 77 * 26 * 256;
		den0 = is_rx02;
		if(!is_rx01 && !is_rx02) {
			printf("Error: %s is not a RX01 or RX02 disk image\n", floppy_filename0);
		} else {
			fread(floppy0, buf.st_size, 1, floppy_file);
		}
		fclose(floppy_file);
	} else {
		memset(floppy0, 0, 77 * 26 * 256);
	}

	if(floppy_filename1) {
		struct stat buf;
		if(stat(floppy_filename1, &buf) == -1) {
			printf("Error: cannot stat file %s: %s\n", floppy_filename1, strerror(errno));
			free(floppy0);
			free(floppy1);
			return 1;
		}
		floppy_file = fopen(floppy_filename1, "rb");
		if(!floppy_file) {
			printf("Error: cannot open file %s\n", floppy_filename1);
			free(floppy0);
			free(floppy1);
			return 1;
		}
		int is_rx01 = buf.st_size == 77 * 26 * 128;
		int is_rx02 = buf.st_size == 77 * 26 * 256;
		den1 = is_rx02;
		if(!is_rx01 && !is_rx02) {
			printf("Error: %s is not a RX01 or RX02 disk image\n", floppy_filename1);
		} else {
			fread(floppy1, buf.st_size, 1, floppy_file);
		}
		fclose(floppy_file);
	} else {
		memset(floppy1, 0, 77 * 26 * 256);
	}

	RXV21SetData0(&rxv21, floppy0, den0);
	RXV21SetData1(&rxv21, floppy1, den1);

	if(disk_filename0) {
		disk0 = (u8*) malloc(DISK_SIZE);
		init_rl02(disk0);

		FILE* file = fopen(disk_filename0, "rb");
		if(!file) {
			printf("Error: cannot open file %s: %s\n", disk_filename0, strerror(errno));
		} else {
			fread(disk0, DISK_SIZE, 1, file);
			fclose(file);
		}

		RLV12SetData0(&rlv12, disk0);
	}

	if(disk_filename1) {
		disk1 = (u8*) malloc(DISK_SIZE);
		init_rl02(disk1);

		FILE* file = fopen(disk_filename1, "rb");
		if(!file) {
			printf("Error: cannot open file %s: %s\n", disk_filename1, strerror(errno));
		} else {
			fread(disk1, DISK_SIZE, 1, file);
			fclose(file);
		}

		RLV12SetData1(&rlv12, disk1);
	}

	if(disk_filename2) {
		disk2 = (u8*) malloc(DISK_SIZE);
		init_rl02(disk2);

		FILE* file = fopen(disk_filename2, "rb");
		if(!file) {
			printf("Error: cannot open file %s: %s\n", disk_filename2, strerror(errno));
		} else {
			fread(disk2, DISK_SIZE, 1, file);
			fclose(file);
		}

		RLV12SetData2(&rlv12, disk2);
	}

	if(disk_filename3) {
		disk3 = (u8*) malloc(DISK_SIZE);
		init_rl02(disk3);

		FILE* file = fopen(disk_filename3, "rb");
		if(!file) {
			printf("Error: cannot open file %s: %s\n", disk_filename3, strerror(errno));
		} else {
			fread(disk3, DISK_SIZE, 1, file);
			fclose(file);
		}

		RLV12SetData3(&rlv12, disk3);
	}

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
	LSI11InstallModule(&lsi, 2, &rlv12.module);
	LSI11InstallModule(&lsi, 3, &rxv21.module);
	LSI11InstallModule(&lsi, 4, &dlv11.module);
	LSI11InstallModule(&lsi, 5, &bdv11.module);
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
				free(floppy0);
				free(floppy1);
				return 1;
			} else if(c != 0) {
				printf("error: invalid signature! [%02x]\n", c);
				fclose(f);
				free(floppy0);
				free(floppy1);
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
	if(!notty) {
		tio = original_tio;
		tio.c_iflag &= ~(ICRNL | INPCK | ISTRIP);
		tio.c_oflag &= ~OPOST;
		tio.c_cflag |= CS8;
		tio.c_lflag &= ~(ECHO | ICANON | IEXTEN);
		if(no_sigint) {
			tio.c_lflag &= ~ISIG;
		}
		tcsetattr(0, TCSANOW, &tio);
	}
#endif

	running = 1;

	if(halt) {
		lsi.cpu.state = 0;
	} else if(!bootstrap && !halt && !load_file) {
		lsi.cpu.state = 1;
	}

	clock_gettime(CLOCK_MONOTONIC, &last);

	const char* console_in = console_input;
	int delay = 0;
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

			if(no_sigint && c == 5) {
				running = 0;
			}
#ifdef DEBUG
			if(c == '\n')
				c = '\r';
#endif
			DLV11JSend(&dlv11, 3, c);
		}

		if(delay++ >= 500 && console_in && *console_in) {
			LSI11ConsoleSend(&dlv11, *console_in);
			console_in++;
			delay = 0;
		}

		for(i = 0; i < 1000; i++)
			LSI11Step(&lsi);

		if(exit_on_halt && lsi.cpu.state == 0) {
			/* make sure ODT finishes its prompt */
			for(i = 0; i < 32; i++)
				LSI11Step(&lsi);

			running = 0;
		}

		clock_gettime(CLOCK_MONOTONIC, &now);
		dt = (now.tv_sec - last.tv_sec) +
			(now.tv_nsec - last.tv_nsec) / 1e9;
		last = now;
		BDV11Step(&bdv11, dt);
		DLV11JStep(&dlv11);
		RXV21Step(&rxv21);
		RLV12Step(&rlv12);
	}

	free(floppy0);
	free(floppy1);

	DLV11JDestroy(&dlv11);
	BDV11Destroy(&bdv11);
	RXV21Destroy(&rxv21);
	MSV11DDestroy(&msv11);
	LSI11Destroy(&lsi);

	if(!notty) {
		tcsetattr(0, TCSANOW, &original_tio);
	}

	printf("\n");

	if(disk0) {
		if(dump_disk0) {
			printf("dumping RL02 unit 0 to %s\n", disk_filename0);
			FILE* file = fopen(disk_filename0, "wb");
			fwrite(disk0, DISK_SIZE, 1, file);
			fclose(file);
		}
		free(disk0);
	}
	if(disk1) {
		if(dump_disk1) {
			printf("dumping RL02 unit 1 to %s\n", disk_filename1);
			FILE* file = fopen(disk_filename1, "wb");
			fwrite(disk1, DISK_SIZE, 1, file);
			fclose(file);
		}
		free(disk1);
	}
	if(disk2) {
		if(dump_disk2) {
			printf("dumping RL02 unit 2 to %s\n", disk_filename2);
			FILE* file = fopen(disk_filename2, "wb");
			fwrite(disk2, DISK_SIZE, 1, file);
			fclose(file);
		}
		free(disk2);
	}
	if(disk3) {
		if(dump_disk3) {
			printf("dumping RL02 unit 3 to %s\n", disk_filename3);
			FILE* file = fopen(disk_filename3, "wb");
			fwrite(disk2, DISK_SIZE, 1, file);
			fclose(file);
		}
		free(disk3);
	}

	if(trace_file) {
		TRCFINISH();
	}

	return 0;
}
