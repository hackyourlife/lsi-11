#ifndef __LSI_11_H__
#define __LSI_11_H__

#include "types.h"

#define	_BV(x)	(1 << (x))

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define	U16B(x)			(x)
#define	U32B(x)			(x)
#define	U64B(x)			(x)
#define	U16L(x)			__builtin_bswap16(x)
#define	U32L(x)			__builtin_bswap32(x)
#define	U64L(x)			__builtin_bswap64(x)
#else
#define	U16B(x)			__builtin_bswap16(x)
#define	U32B(x)			__builtin_bswap32(x)
#define	U64B(x)			__builtin_bswap64(x)
#define	U16L(x)			(x)
#define	U32L(x)			(x)
#define	U64L(x)			(x)
#endif

/* Main memory size: 32kW / 64kB */
#define	MSV11D_SIZE		(65536 - 2 * 4096)

/* DLV11-J input buffer */
#define	DLV11J_BUF		2048

/* Backplane size */
#define	LSI11_SIZE		8

/* QBUS interrupt request delay */
#define	QBUS_DELAY		20

/* QBUS interrupt request delay jitter */
#define	QBUS_DELAY_JITTER	10

/* LTC rate */
#define	LTC_RATE		50
#define	LTC_TIME		(1.0 / LTC_RATE)

/* BDV11 switches */
#define	_A(x)		(1 << ((x) - 1))
#define	_B(x)		(1 << ((x) + 7))

#define	BDV11_CPU_TEST	_A(1)
#define	BDV11_MEM_TEST	_A(2)
#define	BDV11_DECNET	_A(3)
#define	BDV11_DIALOG	_A(4)
#define	BDV11_LOOP	_B(1)
#define	BDV11_RK05	_A(8)
#define	BDV11_RL01	_A(7)
#define	BDV11_RX01	_A(6)
#define	BDV11_RX02	(_A(6) | _A(7))
#define	BDV11_ROM	_A(5)

#define	BDV11_EXT_DIAG	_B(2)
#define	BDV11_2780	_B(3)
#define	BDV11_PROG_ROM	_B(4)

typedef struct QBUS QBUS;
struct QBUS {
	void*	user;
	u16	trap;
	u16	delay;
	u16	irq;
	u16	nxm;
	int	(*interrupt)(QBUS* self, int n);
	void	(*reset)(QBUS* self);
	u16	(*read)(void* user, u16 addr);
	void	(*write)(void* user, u16 addr, u16 value);
	u8	(*read8)(void* user, u16 addr);
	void	(*write8)(void* user, u16 addr, u8 value);
	u16	(*readDMA)(void* user, u16 addr, BOOL* nxm);
	BOOL	(*writeDMA)(void* user, u16 addr, u16 value);
	u8	(*read8DMA)(void* user, u16 addr, BOOL* nxm);
	BOOL	(*write8DMA)(void* user, u16 addr, u8 value);
};

typedef struct {
	QBUS*	bus;
	void*	self;
	u16	(*read)(void* self, u16 addr);
	void	(*write)(void* self, u16 addr, u16 value);
	u8	(*read8)(void* self, u16 addr);
	void	(*write8)(void* self, u16 addr, u8 value);
	u8	(*responsible)(void* self, u16 addr);
	void	(*reset)(void* self);
	int	irq;
} QBUSMod;

typedef struct {
	u16	addr;
	u16	val;
	u8	input;
	u8	state;
	u8	next;
	u8	buf[16];
	u8	buf_r;
	u8	buf_sz;
} KD11ODT;

typedef struct KD11 KD11;
struct KD11 {
	u16	r[8];
	u16	psw;
	KD11ODT	odt;
	u8	state;
	u16	trap;
	void	(*coredump)(KD11* self, QBUS* bus);
};


typedef struct {
	KD11	cpu;
	QBUS	bus;
	QBUSMod*backplane[LSI11_SIZE];
} LSI11;

typedef struct {
	u16	rcsr;
	u16	rbuf;
	u16	xcsr;
	u16	xbuf;

	u16	base;
	u16	vector;

	u8*	buf;
	u16	buf_r;
	u16	buf_w;
	u16	buf_size;
	void	(*receive)(unsigned char c);
} DLV11Ch;

/* peripherals */
typedef struct {
	QBUSMod	module;
	u8*	data;
} MSV11D;

typedef struct {
	QBUSMod	module;
	DLV11Ch	channel[4];
	u16	base;
} DLV11J;

typedef struct {
	QBUSMod	module;
	u16	pcr;
	u16	scratch;
	u16	option;
	u16	display;
	u16	ltc;
	u16	sw;
	float	time;
} BDV11;

typedef struct {
	QBUSMod	module;
	u16	base;
	u16	vector;

	u16	rx2cs;
	u16	rx2db;

	u16	rx2ta;	/* RX Track Address */
	u16	rx2sa;	/* RX Sector Address */
	u16	rx2wc;	/* RX Word Count Register */
	u16	rx2ba;	/* RX Bus Address Register */
	u16	rx2es;	/* RX Error and Status */

	u16	state;
	u16	error;

	u16	nxm;

	u8	den0;
	u8	den1;

	u16	buffer[128];
	u8*	data0;
	u8*	data1;
} RXV21;

typedef struct {
	QBUSMod	module;
	u16	csr;
	u16	bar;
	u16	dar;
	u16	bae;
	u16	wc;

	u16	state;
	u16	fifo[8];
	u16	error;

	BOOL	nxm;

	u16	hs[4];
	u16	ca[4];
	u16	sa[4];

	u16	dma[256];

	u16	opi_timer;

	u16*	data0;
	u16*	data1;
	u16*	data2;
	u16*	data3;
} RLV12;

void MSV11DInit(MSV11D* msv);
void MSV11DDestroy(MSV11D* msv);

void DLV11JInit(DLV11J* dlv);
void DLV11JDestroy(DLV11J* dlv);
void DLV11JSend(DLV11J* dlv, int channel, unsigned char c);
void DLV11JStep(DLV11J* dlv);

void BDV11Init(BDV11* bdv);
void BDV11Destroy(BDV11* bdv);
void BDV11Step(BDV11* bdv, float dt);
void BDV11SetSwitch(BDV11* bdv, u16 sw);

void RXV21Init(RXV21* rx);
void RXV21Destroy(RXV21* rx);
void RXV21SetData0(RXV21* rx, u8* data, int den);
void RXV21SetData1(RXV21* rx, u8* data, int den);
void RXV21Step(RXV21* rx);

void RLV12Init(RLV12* rl);
void RLV12Step(RLV12* rl);
void RLV12SetData0(RLV12* rl, u8* data);
void RLV12SetData1(RLV12* rl, u8* data);
void RLV12SetData2(RLV12* rl, u8* data);
void RLV12SetData3(RLV12* rl, u8* data);

/* KD11 subroutines */
void KD11Init(KD11* kd11);
void KD11Reset(KD11* kd11);
void KD11Step(KD11* kd11, QBUS* bus);
void KD11Trap(KD11* kd11, int n);
void KD11SetCoredumpHandler(KD11* kd11, void (*coredump)(KD11*, QBUS*));

/* LSI-11 subroutines */
void LSI11Init(LSI11* lsi);
void LSI11Destroy(LSI11* lsi);
void LSI11InstallModule(LSI11* lsi, int slot, QBUSMod* module);
void LSI11Reset(LSI11* lsi);
void LSI11Step(LSI11* lsi);

/* LSI-11 disassembler */
int  LSI11Disassemble(const u16* insn, u16 pc, char* buf);
int  LSI11InstructionLength(const u16* insn);

#endif
