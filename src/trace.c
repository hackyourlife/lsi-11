#include <stdio.h>
#include <string.h>

#include "lsi11.h"
#include "trace.h"

TRACE trc = { 0 };

#define	DST	stderr

int TRACEOpen(TRACE* trace, const char* filename)
{
	char header[6] = { 'X', 'T', 'R', 'C', 0, 65 };
	trace->file = fopen(filename, "wb");
	if(!trace->file) {
		return -1;
	}
	trace->step = 0;
	memset(trace->last_r, 0, 7 * sizeof(u16));
	trace->last_psw = 0;
	trace->flags = TRACE_WRITE | TRACE_FIRST_Z;
	fwrite(&header, 6, 1, trace->file);
	return 0;
}

void TRACEClose(TRACE* trace)
{
	fclose(trace->file);
}

void TRACEStep(TRACE* trace, u16* r, u16 psw, u16* insn)
{
	int i;
	char buf[64];

	if(trace->flags & TRACE_PRINT) {
#define	PSW_GET(x)	(psw & (x))
#define	PSW_C		0x01
#define	PSW_V		0x02
#define	PSW_Z		0x04
#define	PSW_N		0x08
#define	PSW_T		0x10
#define	PSW_PRIO	0x80
		LSI11Disassemble(insn, r[7], buf);
		fprintf(DST, "PC=%06o PSW [%c%c%c%c%c%c] SP=%06o [%06o %06o] %06o => %s\n",
			r[7],
			PSW_GET(PSW_PRIO) ? 'P' : '-',
			PSW_GET(PSW_T) ? 'T' : '-',
			PSW_GET(PSW_N) ? 'N' : '-',
			PSW_GET(PSW_Z) ? 'Z' : '-',
			PSW_GET(PSW_V) ? 'V' : '-',
			PSW_GET(PSW_C) ? 'C' : '-',
			r[6], r[0], r[1],
			*insn, buf);
#undef	PSW_GET
#undef	PSW_C
#undef	PSW_V
#undef	PSW_Z
#undef	PSW_N
#undef	PSW_T
#undef	PSW_PRIO
	}

	if(trace->flags & TRACE_WRITE) {
		if((trace->flags & TRACE_COMPRESS) && !(trace->flags & TRACE_FIRST_Z)) {
			int len = LSI11InstructionLength(insn);
			int cnt = len;
			int off = 0;
			u16 mask = 0;
			for(i = 0; i < 7; i++) {
				if(trace->last_r[i] != r[i]) {
					mask |= (1 << i);
					cnt++;
				}
			}
			if(trace->last_psw != psw) {
				mask |= 0x80;
				cnt++;
			}

			if(trace->step < 0xFFFFFFFFL) {
				TRACE_CPUZS rec;
				rec.magic = U32B(MAGIC_CPUZ);
				rec.step = U32B((u32) trace->step++);
				rec.pc = U16B(r[7]);
				rec.mask = U16B((len << 8) | mask);
				for(i = 0; i < len; i++) {
					rec.data[off++] = U16B(insn[i]);
				}
				for(i = 0; i < 7; i++) {
					if(trace->last_r[i] != r[i]) {
						rec.data[off++] = U16B(r[i]);
						trace->last_r[i] = r[i];
					}
				}
				if(trace->last_psw != psw) {
					rec.data[off++] = U16B(psw);
					trace->last_psw = psw;
				}
				fwrite(&rec, 12 + cnt * 2, 1, trace->file);
			} else {
				TRACE_CPUZ rec;
				rec.magic = U32B(MAGIC_CPUZ);
				rec.step = U64B(trace->step++);
				rec.pc = U16B(r[7]);
				rec.mask = U16B(0x8000 | (len << 8) | mask);
				for(i = 0; i < len; i++) {
					rec.data[off++] = U16B(insn[i]);
				}
				for(i = 0; i < 7; i++) {
					if(trace->last_r[i] != r[i]) {
						rec.data[off++] = U16B(r[i]);
						trace->last_r[i] = r[i];
					}
				}
				if(trace->last_psw != psw) {
					rec.data[off++] = U16B(psw);
					trace->last_psw = psw;
				}
				fwrite(&rec, 16 + cnt * 2, 1, trace->file);
			}
		} else {
			TRACE_CPU rec;
			trace->flags &= ~TRACE_FIRST_Z;
			rec.magic = U32B(MAGIC_CPU0);
			rec.psw = U16B(psw);
			rec.step = U64B(trace->step++);
			memcpy(&rec.insn, insn, 6);
			memcpy(&rec.r, r, 16);
			for(i = 0; i < 3; i++) {
				rec.insn[i] = U16B(rec.insn[i]);
			}
			for(i = 0; i < 8; i++) {
				rec.r[i] = U16B(rec.r[i]);
			}
			fwrite(&rec, sizeof(rec), 1, trace->file);
		}
	}
}

void TRACECPUEvent(TRACE* trace, int type, u16 value)
{
	TRACE_CPUEVT rec;

	if(trace->flags & TRACE_PRINT) {
		switch(type) {
			case TRC_CPU_TRAP:
				fprintf(DST, "[KD11] handling trap: %o\n", value);
				break;
			case TRC_CPU_HALT:
				fprintf(DST, "[KD11] HALT @ %06o\n", value);
				break;
			case TRC_CPU_WAIT:
				fprintf(DST, "[KD11] HALT @ %06o\n", value);
				break;
			case TRC_CPU_RUN:
				fprintf(DST, "[KD11] resume execution at %06o\n", value);
				break;
			case TRC_CPU_DBLBUS:
				fprintf(DST, "[KD11] double bus error [%06o]\n", value);
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_CPU1);
		rec.type = U16B(type);
		rec.value = U16B(value);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACEBus(TRACE* trace, u16 type, u16 address, u16 value)
{
	TRACE_BUS rec;

	if(trace->flags & TRACE_IGNORE_BUS) {
		return;
	}

	if(trace->flags & TRACE_PRINT) {
		switch(type) {
			case TRC_BUS_RD:
				fprintf(DST, "[QBUS] read  %06o = %06o\n", address, value);
				break;
			case TRC_BUS_RDFAIL:
				fprintf(DST, "[QBUS] read %06o timed out\n", address);
				break;
			case TRC_BUS_WR:
				fprintf(DST, "[QBUS] write %06o = %06o\n", address, value);
				break;
			case TRC_BUS_WRFAIL:
				fprintf(DST, "[QBUS] write %06o = %06o timed out\n", address, value);
				break;
			case TRC_BUS_RESET:
				fprintf(DST, "[QBUS] reset\n");
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_BUS0);
		rec.type = U16B(type);
		rec.addr = U16B(address);
		rec.value = U16B(value);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACEMemoryDump(TRACE* trace, u8* ptr, u16 address, u16 length)
{
	TRACE_MEMDUMP rec;

	if(trace->flags & TRACE_IGNORE_BUS) {
		return;
	}

	if(trace->flags & TRACE_PRINT) {
		int i = 0;
		fprintf(DST, "[MEM] dump %d from %06o:\n", length, address);
		for(i = 0; i < length; i++) {
			if((i % 8) == 7) {
				fprintf(DST, "\n");
			}
			fprintf(DST, " %03o", ptr[i]);
		}
		fprintf(DST, "\n");
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_BUS1);
		rec.addr = U16B(address);
		rec.len = U16B(length);
		fwrite(&rec, sizeof(rec), 1, trace->file);
		fwrite(ptr, length, 1, trace->file);
	}
}

static const char* get_trap_name(int n) {
	switch(n) {
		case 0004: /* Bus error, Time out */
			return "Bus error / time out";
			break;
		case 0010: /* Reserved */
			return "Reserved instruction";
		case 0014: /* BPT trap instruction, T bit */
			return "BPT trap instruction / T bit";
		case 0020: /* IOT executed */
			return "IOT executed";
		case 0024: /* Power fail / restart */
			return "Power fail / restart";
		case 0030: /* EMT executed */
			return "EMT executed";
		case 0034: /* TRAP executed */
			return "TRAP executed";
		case 0060: /* Console input device */
			return "Console input device";
		case 0064: /* Console output device */
			return "Console output device";
		case 0100: /* External event line interrupt */
			return "External event line interrupt";
		case 0244: /* FIS trap */
			return "FIS trap";
		default:
			return NULL;
	}
}

void TRACETrap(TRACE* trace, int n, int cause)
{
	TRACE_TRAP rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name;
		switch(cause) {
			case TRC_TRAP:
				name = get_trap_name(n);
				if(name) {
					fprintf(DST, "[KD11] TRAP %o: %s\n", n, name);
				} else {
					fprintf(DST, "[KD11] TRAP %o\n", n);
				}
				break;
			case TRC_TRAP_T:
				fprintf(DST, "[KD11] TRAP %o: T bit\n", n);
				break;
			case TRC_TRAP_RADDR:
				fprintf(DST, "[KD11] TRAP %o: get address on mode 0\n", n);
				break;
			case TRC_TRAP_ILL:
				fprintf(DST, "[KD11] TRAP %o: illegal instruction\n", n);
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_TRAP);
		rec.trap = U16B(n);
		rec.cause = U16B(cause);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACEIrq(TRACE* trace, int n, int type)
{
	TRACE_IRQ rec;

	if(trace->flags & TRACE_IGNORE_BUS) {
		return;
	}

	if(trace->flags & TRACE_PRINT) {
		switch(type) {
			case TRC_IRQ_OK:
				fprintf(DST, "[QBUS] interrupt request %o\n", n);
				break;
			case TRC_IRQ_FAIL:
				fprintf(DST, "[QBUS] interrupt request %o denied\n", n);
				break;
			case TRC_IRQ_SIG:
				fprintf(DST, "[QBUS] signaling irq %o\n", n);
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_IRQ0);
		rec.trap = U16B(n);
		rec.type = U16B(type);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACEDLV11(TRACE* trace, int channel, int type, u16 value)
{
	TRACE_DLV11 rec;

	if(trace->flags & TRACE_PRINT) {
		switch(type) {
			case TRC_DLV11_RX:
				fprintf(DST, "[DLV11J] CH%d: receive %06o [%c]\n", channel, value, value >= 0x20 ? value : '.');
				break;
			case TRC_DLV11_TX:
				fprintf(DST, "[DLV11J] CH%d: transmit %06o [%c]\n", channel, value, value >= 0x20 ? value : '.');
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_DLV1);
		rec.channel = channel;
		rec.type = type;
		rec.value = U16B(value);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

static const char* rxv21_get_cmd_name(int func)
{
	switch(func) {
		case TRC_RXV21_FILL:
			return "Fill Buffer";
		case TRC_RXV21_EMPTY:
			return "Empty Buffer";
		case TRC_RXV21_WRITE:
			return "Write Sector";
		case TRC_RXV21_READ:
			return "Read Sector";
		case TRC_RXV21_SET_MD:
			return "Set Media Density";
		case TRC_RXV21_STATUS:
			return "Read Status";
		case TRC_RXV21_WRITE_DD:
			return "Write Deleted Data Sector";
		case TRC_RXV21_READ_ERR:
			return "Read Error Code";
		default:
			return "unknown";
	}
}

void TRACERXV21Command(TRACE* trace, int commit, int type, u16 rx2cs)
{
	TRACE_RXV21CMD rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name = rxv21_get_cmd_name(type);
		fprintf(DST, "[RXV21] Execute command: %s\n", name);
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_RX2C);
		rec.type = type;
		rec.commit = commit;
		rec.rx2cs = U16B(rx2cs);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACERXV21Step(TRACE* trace, int type, int step, u16 rx2db)
{
	TRACE_RXV21STEP rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name = rxv21_get_cmd_name(type);
		fprintf(DST, "[RXV21] Processing command %s: step %d with data %06o\n",
				name, step, rx2db);
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_RX2S);
		rec.type = type;
		rec.step = step;
		rec.rx2db = U16B(rx2db);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACERXV21DMA(TRACE* trace, int type, u16 rx2wc, u16 rx2ba)
{
	TRACE_RXV21DMA rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name = rxv21_get_cmd_name(type);
		fprintf(DST, "[RXV21] DMA transfer [%s]: %06o words to %06o\n",
				name, rx2wc, rx2ba);
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_RX2D);
		rec.type = U16B(type);
		rec.rx2wc = U16B(rx2wc);
		rec.rx2ba = U16B(rx2ba);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

static const char* rxv21_get_error_name(int type)
{
	switch(type) {
		case TRC_RXV21_WC_OVFL:
			return "Word Count overflow";
		case TRC_RXV21_DEN_ERR:
			return "Density mismatch";
		case TRC_RXV21_TRACK_NO:
			return "Tried to access a track greater than 76";
		case TRC_RXV21_SECT_NO:
			return "Desired sector could not be found after looking at 52 headers (2 revolutions)";
		default:
			return "unknwn";
	}
}

void TRACERXV21Error(TRACE* trace, int type, u16 info)
{
	TRACE_RXV21ERR rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name = rxv21_get_error_name(type);
		switch(type) {
			case TRC_RXV21_WC_OVFL:
			case TRC_RXV21_TRACK_NO:
			case TRC_RXV21_SECT_NO:
				fprintf(DST, "[RXV21] Error: %s [%o/%d]\n",
						name, info, info);
				break;
			default:
			case TRC_RXV21_DEN_ERR:
				fprintf(DST, "[RXV21] Error: %s\n", name);
				break;
		}
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_RX2E);
		rec.type = U16B(type);
		rec.info = U16B(info);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}

void TRACERXV21Disk(TRACE* trace, int type, int drive, int density, u16 rx2sa, u16 rx2ta)
{
	TRACE_RXV21DISK rec;

	if(trace->flags & TRACE_PRINT) {
		const char* name;
		switch(type) {
			case TRC_RXV21_READ:
				name = "read";
				break;
			case TRC_RXV21_WRITE:
				name = "write";
				break;
			case TRC_RXV21_WRITE_DD:
				name = "write (delete data)";
				break;
			default:
				name = "???";
				break;
		}
		fprintf(DST, "[RXV21] %s sector SEC=%d, TR=%d [drive %d, %s density]\n",
				name, rx2sa, rx2ta, drive,
				density ? "double" : "single");
		fflush(DST);
	}

	if(trace->flags & TRACE_WRITE) {
		rec.magic = U32B(MAGIC_RX2A);
		rec.type = U16B(type);
		rec.drive = drive;
		rec.density = density;
		rec.rx2sa = U16B(rx2sa);
		rec.rx2ta = U16B(rx2ta);
		fwrite(&rec, sizeof(rec), 1, trace->file);
	}
}
