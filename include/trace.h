#ifndef __TRACE_H__
#define __TRACE_H__

#include <stdio.h>

#include "types.h"

#define	MAGIC_CPU0		0x43505530
#define	MAGIC_CPUZ		0x4350555a
#define	MAGIC_CPU1		0x43505531
#define	MAGIC_BUS0		0x42555330
#define	MAGIC_BUS1		0x42555331
#define	MAGIC_TRAP		0x54524150
#define	MAGIC_IRQ0		0x49525130
#define	MAGIC_RX2A		0x52583241
#define	MAGIC_RX2C		0x52583243
#define	MAGIC_RX2D		0x52583244
#define	MAGIC_RX2E		0x52583245
#define	MAGIC_RX2S		0x52583253
#define	MAGIC_DLV1		0x444C5631

#define	TRC_CPU_TRAP		0
#define	TRC_CPU_HALT		1
#define	TRC_CPU_WAIT		2
#define	TRC_CPU_RUN		3
#define	TRC_CPU_DBLBUS		4
#define	TRC_CPU_ODT_P		5
#define	TRC_CPU_ODT_G		5

#define	TRC_TRAP		0
#define	TRC_TRAP_ILL		1
#define	TRC_TRAP_RADDR		2
#define	TRC_TRAP_T		3

#define	TRC_BUS_RD		0
#define	TRC_BUS_WR		1
#define	TRC_BUS_RDFAIL		2
#define	TRC_BUS_WRFAIL		3
#define	TRC_BUS_RESET		4

#define	TRC_IRQ_OK		0
#define	TRC_IRQ_FAIL		1
#define	TRC_IRQ_SIG		2

#define	TRC_DLV11_RX		0
#define	TRC_DLV11_TX		1
#define	TRC_DLV11_RDY		2
#define	TRC_DLV11_SEI		3
#define	TRC_DLV11_CLI		4

#define	TRC_RXV21_FILL		0
#define	TRC_RXV21_EMPTY		1
#define	TRC_RXV21_WRITE		2
#define	TRC_RXV21_READ		3
#define	TRC_RXV21_SET_MD	4
#define	TRC_RXV21_STATUS	5
#define	TRC_RXV21_WRITE_DD	6
#define	TRC_RXV21_READ_ERR	7

#define	TRC_RXV21_WC_OVFL	0
#define	TRC_RXV21_DEN_ERR	1
#define	TRC_RXV21_TRACK_NO	2
#define	TRC_RXV21_SECT_NO	3

#define	TRACE_WRITE		1
#define	TRACE_IGNORE_BUS	2
#define	TRACE_PRINT		4
#define	TRACE_COMPRESS		8
#define	TRACE_FIRST_Z		16

#define	TRCSETIGNBUS()	(trc.flags |= TRACE_IGNORE_BUS)
#define	TRCCLRIGNBUS()	(trc.flags &= ~TRACE_IGNORE_BUS)
#define	IFTRC()		if(trc.flags)
#define	__IFTRC(x)	{ IFTRC() x; }
#define	TRCStep(r, psw, insn) \
		__IFTRC(TRACEStep(&trc, r, psw, insn))
#define	TRCCPUEvent(event, info) \
		__IFTRC(TRACECPUEvent(&trc, event, info))
#define	TRCBus(type, addr, val) \
		__IFTRC(TRACEBus(&trc, type, addr, val))
#define	TRCMemoryDump(ptr, addr, val) \
		__IFTRC(TRACEMemoryDump(&trc, ptr, addr, len))
#define	TRCIRQ(n, type) \
		__IFTRC(TRACEIrq(&trc, n, type))
#define	TRCTrap(n, cause) \
		__IFTRC(TRACETrap(&trc, n, cause))
#define	TRCDLV11(type, channel, value) \
		__IFTRC(TRACEDLV11(&trc, channel, type, value))
#define	TRCRXV21CMD(type, rx2cs) \
		__IFTRC(TRACERXV21Command(&trc, 0, type, rx2cs))
#define	TRCRXV21CMDCommit(type, rx2cs) \
		__IFTRC(TRACERXV21Command(&trc, 1, type, rx2cs))
#define	TRCRXV21Step(type, step, rx2db) \
		__IFTRC(TRACERXV21Step(&trc, type, step, rx2db))
#define	TRCRXV21DMA(type, rx2wc, rx2ba) \
		__IFTRC(TRACERXV21DMA(&trc, type, rx2wc, rx2ba))
#define	TRCRXV21Disk(type, drive, density, rx2sa, rx2ta) \
		__IFTRC(TRACERXV21Disk(&trc, type, drive, density, rx2sa, rx2ta))
#define	TRCRXV21Error(type, info) \
		__IFTRC(TRACERXV21Error(&trc, type, info))

#define	TRCINIT(name)	TRACEOpen(&trc, name);
#define	TRCFINISH()	TRACEClose(&trc);

typedef struct {
	u32	magic;
	u16	r[8];
	u16	psw;
	u16	insn[3];
	u32	pad;
	u64	step;
} TRACE_CPU;

typedef struct {
	u32	magic;
	u16	pc;
	u16	mask;
	u64	step;
	u16	data[11];
} TRACE_CPUZ;

typedef struct {
	u32	magic;
	u16	pc;
	u16	mask;
	u32	step;
	u16	data[11];
} TRACE_CPUZS;

typedef struct {
	u32	magic;
	u16	type;
	u16	value;
} TRACE_CPUEVT;

typedef struct {
	u32	magic;
	u16	addr;
	u16	value;
	u16	type;
	u16	pad;
} TRACE_BUS;

typedef struct {
	u32	magic;
	u16	addr;
	u16	len;
} TRACE_MEMDUMP;

typedef struct {
	u32	magic;
	u16	trap;
	u16	cause;
} TRACE_TRAP;

typedef struct {
	u32	magic;
	u16	trap;
	u16	type;
} TRACE_IRQ;

typedef struct {
	u32	magic;
	u16	rx2cs;
	u16	rx2ta;
	u16	rx2sa;
	u16	rx2wc;
	u16	rx2ba;
	u16	rx2es;
	u16	command;
	u16	status;
} TRACE_RX02;

typedef struct {
	u32	magic;
	u8	channel;
	u8	type;
	u16	value;
} TRACE_DLV11;

typedef struct {
	u32	magic;
	u8	type;
	u8	commit;
	u16	rx2cs;
} TRACE_RXV21CMD;

typedef struct {
	u32	magic;
	u8	type;
	u8	step;
	u16	rx2db;
} TRACE_RXV21STEP;

typedef struct {
	u32	magic;
	u16	type;
	u16	rx2wc;
	u16	rx2ba;
	u16	pad;
} TRACE_RXV21DMA;

typedef struct {
	u32	magic;
	u16	type;
	u8	drive;
	u8	density;
	u16	rx2sa;
	u16	rx2ta;
} TRACE_RXV21DISK;

typedef struct {
	u32	magic;
	u16	type;
	u16	info;
} TRACE_RXV21ERR;

typedef struct {
	FILE*	file;
	u64	step;
	int	flags;
	u16	last_psw;
	u16	last_r[7];
} TRACE;

extern TRACE trc;

int	TRACEOpen(TRACE* trace, const char* filename);
void	TRACEClose(TRACE* trace);
void	TRACEStep(TRACE* trace, u16* r, u16 psw, u16* insn);
void	TRACECPUEvent(TRACE* trace, int type, u16 value);
void	TRACEBus(TRACE* trace, u16 type, u16 address, u16 value);
void	TRACEMemoryDump(TRACE* trace, u8* ptr, u16 address, u16 length);
void	TRACETrap(TRACE* trace, int n, int cause);
void	TRACEIrq(TRACE* trace, int n, int type);
void	TRACERX02(TRACE* trace, u16 command, u16 status);
void	TRACEDLV11(TRACE* trace, int channel, int type, u16 value);
void	TRACERXV21Command(TRACE* trace, int commit, int type, u16 rx2cs);
void	TRACERXV21Step(TRACE* trace, int type, int step, u16 rx2db);
void	TRACERXV21DMA(TRACE* trace, int type, u16 rx2wc, u16 rx2ba);
void	TRACERXV21Disk(TRACE* trace, int type, int drive, int density, u16 rx2sa, u16 rx2ta);
void	TRACERXV21Error(TRACE* trace, int type, u16 info);

#endif
