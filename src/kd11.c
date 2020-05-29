#include <string.h>
#include <stdlib.h>

#include "lsi11.h"
#include "trace.h"

/* ODT states */
#define	ODT_STATE_INIT		0
#define	ODT_STATE_WAIT		1
#define	ODT_STATE_ADDR		2
#define	ODT_STATE_REG		3
#define	ODT_STATE_REG_WAIT	4
#define	ODT_STATE_VAL		5
#define	ODT_STATE_REG_VAL	6
#define	ODT_STATE_WR		7

/* CPU states */
#define	STATE_HALT		0
#define	STATE_RUN		1
#define	STATE_WAIT		2
#define	STATE_INHIBIT_TRACE	3

#define	PSW_C			_BV(0)
#define	PSW_V			_BV(1)
#define	PSW_Z			_BV(2)
#define	PSW_N			_BV(3)
#define	PSW_T			_BV(4)
#define	PSW_PRIO		_BV(7)

#define	PSW_GET(x)		(((kd11->psw) & (x)) ? 1 : 0)
#define	PSW_SET(x)		((kd11->psw) |= (x))
#define	PSW_CLR(x)		((kd11->psw) &= ~(x))
#define	PSW_EQ(x, v) { \
	if(v) { \
		PSW_SET(x); \
	} else { \
		PSW_CLR(x); \
	} \
}

#define	TRAP(n)		KD11Trap(kd11, n)

typedef struct {
	u16	rn:3;
	u16	mode:3;
	u16	opcode:10;
} KD11INSN1;

typedef struct {
	u16	dst_rn:3;
	u16	dst_mode:3;
	u16	src_rn:3;
	u16	src_mode:3;
	u16	opcode:4;
} KD11INSN2;

typedef struct {
	u16	offset:8;
	u16	opcode:8;
} KD11INSNBR;

typedef struct {
	u16	rn:3;
	u16	mode:3;
	u16	r:3;
	u16	opcode:7;
} KD11INSNJSR;

typedef struct {
	u16	rn:3;
	u16	opcode:13;
} KD11INSNRTS;

typedef struct {
	u16	nn:6;
	u16	opcode:10;
} KD11INSNMARK;

typedef struct {
	u16	offset:6;
	u16	rn:3;
	u16	opcode:7;
} KD11INSNSOB;

void KD11Init(KD11* kd11)
{
	memset(kd11, 0, sizeof(KD11));
}

void KD11Reset(KD11* kd11)
{
	kd11->r[7] = 0173000;
	kd11->psw = 0;
	kd11->trap = 0;
	kd11->state = STATE_HALT;
	kd11->odt.state = ODT_STATE_INIT;
}

#define	READ(addr)		(bus->read(bus->user, (addr)))
#define	WRITE(addr, val)	(bus->write(bus->user, (addr), (val)))
#define	CHECK()			{ \
	if(kd11->trap && kd11->trap <= 010) \
		return; \
	}

static void KD11ODTClear(KD11ODT* odt)
{
	odt->buf_sz = 0;
}

static void KD11ODTWrite(KD11ODT* odt, u8 c)
{
	odt->buf[odt->buf_sz++] = c;
	odt->buf_r = 0;
}

static void KD11ODTWriteOctal(KD11ODT* odt, u16 val)
{
	int i;
	odt->buf[odt->buf_sz++] = ((val >> 15) & 0x7) + '0';
	for(i = 0; i < 5; i++) {
		odt->buf[odt->buf_sz++] = ((val >> 12) & 7) + '0';
		val <<= 3;
	}
}

static void KD11ODTInputError(KD11ODT* odt)
{
	odt->state = ODT_STATE_WR;
	odt->next = ODT_STATE_WAIT;
	odt->buf[0] = '?';
	odt->buf[1] = '\r';
	odt->buf[2] = '\n';
	odt->buf[3] = '@';
	odt->buf_r = 0;
	odt->buf_sz = 4;
}

void KD11ODTStep(KD11* kd11, QBUS* bus)
{
	/* odt */
	KD11ODT* odt = &kd11->odt;
	switch(odt->state) {
		case ODT_STATE_INIT:
			KD11ODTClear(odt);
			KD11ODTWrite(odt, '\r');
			KD11ODTWrite(odt, '\n');
			KD11ODTWriteOctal(odt, kd11->r[7]);
			KD11ODTWrite(odt, '\r');
			KD11ODTWrite(odt, '\n');
			KD11ODTWrite(odt, '@');
			odt->next = ODT_STATE_WAIT;
			odt->state = ODT_STATE_WR;
			break;
		case ODT_STATE_WAIT:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 c = READ(0177562);
				WRITE(0177566, (u8) c);
				switch((u8) c) {
					case '$':
					case 'R':
						odt->state = ODT_STATE_REG;
						break;
					case '0':
					case '1':
					case '2':
					case '3':
					case '4':
					case '5':
					case '6':
					case '7':
						odt->state = ODT_STATE_ADDR;
						odt->addr = ((u8) c) - '0';
						break;
					case 'G':
						KD11ODTClear(odt);
						KD11ODTWrite(odt, '\r');
						KD11ODTWrite(odt, '\n');
						KD11ODTWrite(odt, '@');
						odt->next = ODT_STATE_WAIT;
						odt->state = ODT_STATE_WR;
						break;
					case 'P':
						odt->state = ODT_STATE_INIT;
						kd11->state = STATE_RUN;
						TRCCPUEvent(TRC_CPU_ODT_P, kd11->r[7]);
						break;
					default:
						KD11ODTInputError(odt);
						break;
				}
			}
			break;
		case ODT_STATE_WR:
			if(READ(0177564) & 0x80) {
				WRITE(0177566, odt->buf[odt->buf_r++]);
				if(odt->buf_r == odt->buf_sz) {
					odt->state = odt->next;
				}
			}
			break;
		case ODT_STATE_ADDR:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 ch = READ(0177562);
				u8 c = (u8) ch;
				WRITE(0177566, c);
				if((u8) c == '/') { /* delimit */
					u16 val = READ(odt->addr);
					KD11ODTClear(odt);
					KD11ODTWriteOctal(odt, val);
					KD11ODTWrite(odt, ' ');
					odt->next = ODT_STATE_VAL;
					odt->state = ODT_STATE_WR;
					odt->input = 0;
				} else if(c >= '0' && c <= '7') {
					odt->addr <<= 3;
					odt->addr |= c - '0';
				} else if(c == 'G') {
					odt->state = ODT_STATE_INIT;
					kd11->r[7] = odt->addr;
					kd11->state = STATE_RUN;
					TRCCPUEvent(TRC_CPU_ODT_G, odt->addr);
				} else {
					KD11ODTInputError(odt);
				}
			}
			break;
		case ODT_STATE_REG:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 ch = READ(0177562);
				u8 c = (u8) ch;
				WRITE(0177566, c);
				odt->state = ODT_STATE_REG_WAIT;
				if(c >= '0' && c <= '7') {
					odt->addr = c - '0';
				} else if(c == 'S') {
					odt->addr = 8;
				} else {
					KD11ODTInputError(odt);
				}
			}
			break;
		case ODT_STATE_VAL:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 ch = READ(0177562);
				u8 c = (u8) ch;
				WRITE(0177566, c);
				if(c == '\r' || c == '\n') {
					if(odt->input) {
						WRITE(odt->addr, odt->val);
					}
				} else if(c >= '0' && c <= '7') {
					odt->val <<= 3;
					odt->val |= c - '0';
					odt->input = 1;
				} else {
					KD11ODTInputError(odt);
				}
				if(c == '\r') {
					KD11ODTClear(odt);
					KD11ODTWrite(odt, '\r');
					KD11ODTWrite(odt, '\n');
					KD11ODTWrite(odt, '@');
					odt->state = ODT_STATE_WR;
					odt->next = ODT_STATE_WAIT;
				} else if(c == '\n') {
					u16 val;

					odt->addr += 2;
					odt->val = 0;
					val = READ(odt->addr);

					KD11ODTClear(odt);
					KD11ODTWrite(odt, '\r');
					KD11ODTWrite(odt, '\n');
					KD11ODTWriteOctal(odt, odt->addr);
					KD11ODTWrite(odt, '/');
					KD11ODTWriteOctal(odt, val);
					KD11ODTWrite(odt, ' ');

					odt->next = ODT_STATE_VAL;
					odt->state = ODT_STATE_WR;
					odt->input = 0;
				}
			}
			break;
		case ODT_STATE_REG_WAIT:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 ch = READ(0177562);
				u8 c = (u8) ch;
				WRITE(0177566, c);
				if(c == '/') {
					u16 val;
					if(odt->addr < 8) {
						val = kd11->r[odt->addr];
					} else {
						val = kd11->psw;
					}
					KD11ODTClear(odt);
					KD11ODTWriteOctal(odt, val);
					KD11ODTWrite(odt, ' ');
					odt->val = 0;
					odt->next = ODT_STATE_REG_VAL;
					odt->state = ODT_STATE_WR;
					odt->input = 0;
				} else {
					KD11ODTInputError(odt);
				}
			}
			break;
		case ODT_STATE_REG_VAL:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 ch = READ(0177562);
				u8 c = (u8) ch;
				WRITE(0177566, c);
				if(c == '\r' || c == '\n') {
					if(odt->input) {
						if(odt->addr == 8) {
							kd11->psw = odt->val;
						} else {
							kd11->r[odt->addr] = odt->val;
						}
					}
				} else if(c >= '0' && c <= '7') {
					odt->val <<= 3;
					odt->val |= c - '0';
					odt->input = 1;
				} else {
					KD11ODTInputError(odt);
				}
				if(c == '\r' || (c == '\n' && odt->addr == 7)) {
					KD11ODTClear(odt);
					KD11ODTWrite(odt, '\r');
					KD11ODTWrite(odt, '\n');
					KD11ODTWrite(odt, '@');
					odt->state = ODT_STATE_WR;
					odt->next = ODT_STATE_WAIT;
				} else if(c == '\n') {
					u16 val;

					odt->addr++;
					odt->val = 0;
					val = kd11->r[odt->addr];

					KD11ODTClear(odt);
					KD11ODTWrite(odt, '\r');
					KD11ODTWrite(odt, 'R');
					KD11ODTWrite(odt, odt->addr + '0');
					KD11ODTWrite(odt, '/');
					KD11ODTWriteOctal(odt, val);
					KD11ODTWrite(odt, ' ');

					odt->next = ODT_STATE_REG_VAL;
					odt->state = ODT_STATE_WR;
					odt->input = 0;
				}
			}
			break;
	}
}

#define	CHECKREAD()		{ \
	if(kd11->trap && kd11->trap <= 010) \
		return 0; \
	}

u16 KD11CPUReadW(KD11* kd11, QBUS* bus, u16 dst, u16 mode, int inc)
{
	u16 addr;
	switch(mode) {
		case 0: /* Register */
			return kd11->r[dst];
		case 1: /* Register indirect */
			return READ(kd11->r[dst]);
		case 2: /* Autoincrement */
			addr = kd11->r[dst];
			if(inc) {
				kd11->r[dst] += 2;
				kd11->r[dst] &= 0xFFFE;
			}
			return READ(addr);
		case 3: /* Autoincrement indirect */
			addr = kd11->r[dst];
			if(inc) {
				kd11->r[dst] += 2;
				kd11->r[dst] &= 0xFFFE;
			}
			addr = READ(addr);
			CHECKREAD();
			return READ(addr);
		case 4: /* Autodecrement */
			if(inc) {
				kd11->r[dst] -= 2;
				addr = kd11->r[dst];
				kd11->r[dst] &= 0xFFFE;
			} else {
				addr = kd11->r[dst] - 2;
			}
			return READ(addr);
		case 5: /* Autodecrement indirect */
			if(inc) {
				kd11->r[dst] -= 2;
				addr = kd11->r[dst];
				kd11->r[dst] &= 0xFFFE;
			} else {
				addr = kd11->r[dst] - 2;
			}
			addr = READ(addr);
			CHECKREAD();
			return READ(addr);
		case 6: /* Index */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			if(inc) {
				kd11->r[7] += 2;
			} else if(dst == 7) {
				addr += 2;
			}
			addr += kd11->r[dst];
			return READ(addr);
		case 7: /* Index indirect */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			if(inc) {
				kd11->r[7] += 2;
			} else if(dst == 7) {
				addr += 2;
			}
			addr += kd11->r[dst];
			addr = READ(addr);
			CHECKREAD();
			return READ(addr);
		default:
			return 0;
	}
}

#define	READ8(addr) (((addr) & 1) ? (u8) (READ((addr) & 0xFFFE) >> 8) : \
				((u8) READ(addr & 0xFFFE)))

u8 KD11CPUReadB(KD11* kd11, QBUS* bus, u16 dst, u16 mode, int inc)
{
	u16 addr;
	switch(mode) {
		case 0: /* Register */
			return (u8) kd11->r[dst];
		case 1: /* Register indirect */
			return READ8(kd11->r[dst]);
		case 2: /* Autoincrement */
			addr = kd11->r[dst];
			if(inc) {
				if(dst == 6 || dst == 7) {
					kd11->r[dst] += 2;
				} else {
					kd11->r[dst]++;
				}
			}
			return READ8(addr);
		case 3: /* Autoincrement indirect */
			addr = kd11->r[dst];
			if(inc) {
				kd11->r[dst] += 2;
			}
			addr = READ(addr);
			CHECKREAD();
			return READ8(addr);
		case 4: /* Autodecrement */
			if(inc) {
				if(dst == 6 || dst == 7) {
					kd11->r[dst] -= 2;
				} else {
					kd11->r[dst]--;
				}
				addr = kd11->r[dst];
			} else {
				if(dst == 6 || dst == 7) {
					addr = kd11->r[dst] - 2;
				} else {
					addr = kd11->r[dst] - 1;
				}
			}
			return READ8(addr);
		case 5: /* Autodecrement indirect */
			if(inc) {
				kd11->r[dst] -= 2;
				addr = kd11->r[dst];
			} else {
				addr = kd11->r[dst] - 2;
			}
			addr = READ(addr);
			CHECKREAD();
			return READ8(addr);
		case 6: /* Index */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			if(inc) {
				kd11->r[7] += 2;
			} else if(dst == 7) {
				addr += 2;
			}
			addr += kd11->r[dst];
			return READ8(addr);
		case 7: /* Index indirect */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			if(inc) {
				kd11->r[7] += 2;
			} else if(dst == 7) {
				addr += 2;
			}
			addr += kd11->r[dst];
			addr = READ(addr);
			CHECKREAD();
			return READ8(addr);
		default:
			return 0;
	}
}

void KD11CPUWriteW(KD11* kd11, QBUS* bus, u16 dst, u16 mode, u16 val)
{
	u16 addr;
	switch(mode) {
		case 0: /* Register */
			kd11->r[dst] = val;
			break;
		case 1: /* Register indirect */
			WRITE(kd11->r[dst], val);
			break;
		case 2: /* Autoincrement */
			kd11->r[dst] &= 0xFFFE;
			addr = kd11->r[dst];
			kd11->r[dst] += 2;
			WRITE(addr, val);
			break;
		case 3: /* Autoincrement indirect */
			kd11->r[dst] &= 0xFFFE;
			addr = kd11->r[dst];
			kd11->r[dst] += 2;
			addr = READ(addr);
			CHECK();
			WRITE(addr, val);
			break;
		case 4: /* Autodecrement */
			kd11->r[dst] &= 0xFFFE;
			kd11->r[dst] -= 2;
			addr = kd11->r[dst];
			WRITE(addr, val);
			break;
		case 5: /* Autodecrement indirect */
			kd11->r[dst] &= 0xFFFE;
			kd11->r[dst] -= 2;
			addr = kd11->r[dst];
			addr = READ(addr);
			CHECK();
			WRITE(addr, val);
			break;
		case 6: /* Index */
			addr = READ(kd11->r[7]);
			CHECK();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			WRITE(addr, val);
			break;
		case 7: /* Index indirect */
			addr = READ(kd11->r[7]);
			CHECK();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			addr = READ(addr);
			CHECK();
			WRITE(addr, val);
			break;
	}
}

#define	WRITE8(addr, val)	{ \
	u16 aaddr = addr & 0xFFFE; \
	u16 tmp = READ(aaddr); \
	if(addr & 1) { \
		tmp = (tmp & 0x00FF) | (val << 8); \
	} else { \
		tmp = (tmp & 0xFF00) | val; \
	} \
	WRITE(aaddr, tmp); \
}

void KD11CPUWriteB(KD11* kd11, QBUS* bus, u16 dst, u16 mode, u8 val)
{
	u16 addr;
	switch(mode) {
		case 0: /* Register */
			kd11->r[dst] = (kd11->r[dst] & 0xFF00) | val;
			break;
		case 1: /* Register deferred */
			WRITE8(kd11->r[dst], val);
			break;
		case 2: /* Autoincrement */
			addr = kd11->r[dst];
			if(dst == 6 || dst == 7) {
				kd11->r[dst] += 2;
			} else {
				kd11->r[dst]++;
			}
			WRITE8(addr, val);
			break;
		case 3: /* Autoincrement deferred */
			addr = kd11->r[dst];
			kd11->r[dst] += 2;
			addr = READ(addr);
			CHECK();
			WRITE8(addr, val);
			break;
		case 4: /* Autodecrement */
			if(dst == 6 || dst == 7) {
				kd11->r[dst] -= 2;
			} else {
				kd11->r[dst]--;
			}
			addr = kd11->r[dst];
			WRITE8(addr, val);
			break;
		case 5: /* Autodecrement deferred */
			kd11->r[dst] -= 2;
			addr = kd11->r[dst];
			addr = READ(addr);
			CHECK();
			WRITE8(addr, val);
			break;
		case 6: /* Index */
			addr = READ(kd11->r[7]);
			CHECK();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			WRITE8(addr, val);
			break;
		case 7: /* Index deferred */
			addr = READ(kd11->r[7]);
			CHECK();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			addr = READ(addr);
			CHECK();
			WRITE8(addr, val);
			break;
	}
}

u16 KD11CPUGetAddr(KD11* kd11, QBUS* bus, u16 dst, u16 mode)
{
	u16 addr;
	switch(mode) {
		case 0: /* Register */
			TRCTrap(4, TRC_TRAP_RADDR);
			TRAP(4); /* illegal instruction */
		case 1: /* Register indirect */
			return kd11->r[dst];
		case 2: /* Autoincrement */
			addr = kd11->r[dst];
			kd11->r[dst] += 2;
			return addr;
		case 3: /* Autoincrement indirect */
			addr = kd11->r[dst];
			kd11->r[dst] += 2;
			addr = READ(addr);
			return addr;
		case 4: /* Autodecrement */
			kd11->r[dst] -= 2;
			addr = kd11->r[dst];
			return addr;
		case 5: /* Autodecrement indirect */
			kd11->r[dst] -= 2;
			addr = kd11->r[dst];
			addr = READ(addr);
			return addr;
		case 6: /* Index */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			return addr;
		case 7: /* Index indirect */
			addr = READ(kd11->r[7]);
			CHECKREAD();
			kd11->r[7] += 2;
			addr += kd11->r[dst];
			addr = READ(addr);
			return addr;
		default:
			return 0;
	}
}

#define	CPUREADW(rn, mode)	KD11CPUReadW(kd11, bus, rn, mode, 1); \
				CHECK()
#define	CPUREADB(rn, mode)	KD11CPUReadB(kd11, bus, rn, mode, 1); \
				CHECK()
#define	CPUREADNW(rn, mode)	KD11CPUReadW(kd11, bus, rn, mode, 0); \
				CHECK()
#define	CPUREADNB(rn, mode)	KD11CPUReadB(kd11, bus, rn, mode, 0); \
				CHECK()
#define CPUWRITEW(rn, mode, v)	KD11CPUWriteW(kd11, bus, rn, mode, v); \
				CHECK()
#define CPUWRITEB(rn, mode, v)	KD11CPUWriteB(kd11, bus, rn, mode, v); \
				CHECK()

typedef union {
	float	f32;
	u32	u32;
} FLOAT;

void KD11CPUStep(KD11* kd11, QBUS* bus)
{
	u16 tmp, tmp2;
	u16 src, dst;
	s32 tmps32;
	FLOAT f1, f2, f3;
	u8 unknown = 0;

	u16 insn = READ(kd11->r[7]);
	KD11INSN1* insn1 = (KD11INSN1*) &insn;
	KD11INSN2* insn2 = (KD11INSN2*) &insn;
	KD11INSNBR* insnbr = (KD11INSNBR*) &insn;
	KD11INSNJSR* insnjsr = (KD11INSNJSR*) &insn;
	KD11INSNRTS* insnrts = (KD11INSNRTS*) &insn;
	KD11INSNMARK* insnmark = (KD11INSNMARK*) &insn;
	KD11INSNSOB* insnsob = (KD11INSNSOB*) &insn;

	kd11->r[7] += 2;

	CHECK();

	/* single operand instructions */
	switch(insn & 0177700) {
		case 0005000: /* CLR */
			CPUWRITEW(insn1->rn, insn1->mode, 0);
			PSW_CLR(PSW_N | PSW_V | PSW_C);
			PSW_SET(PSW_Z);
			break;
		case 0105000: /* CLRB */
			CPUWRITEB(insn1->rn, insn1->mode, 0);
			PSW_CLR(PSW_N | PSW_V | PSW_C);
			PSW_SET(PSW_Z);
			break;
		case 0005100: /* COM */
			tmp = CPUREADNW(insn1->rn, insn1->mode);
			tmp = ~tmp;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_CLR(PSW_V);
			PSW_SET(PSW_C);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105100: /* COMB */
			tmp = CPUREADNB(insn1->rn, insn1->mode);
			tmp = ~tmp;
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_CLR(PSW_V);
			PSW_SET(PSW_C);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !((u8) tmp));
			break;
		case 0005200: /* INC */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp = src + 1;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 077777)
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105200: /* INCB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp = (u8) (src + 1);
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 000177)
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0005300: /* DEC */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp = src - 1;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0100000)
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105300: /* DECB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp = (u8) (src - 1);
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0000200)
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0005400: /* NEG */
			tmp = CPUREADNW(insn1->rn, insn1->mode);
			if(tmp != 0100000) {
				tmp = -tmp;
			}
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, tmp == 0100000)
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_C, tmp);
			break;
		case 0105400: /* NEGB */
			tmp = CPUREADNB(insn1->rn, insn1->mode);
			if(tmp != 0200) {
				tmp = -tmp;
			}
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, tmp == 0200)
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_C, tmp);
			break;
		case 0005700: /* TST */
			tmp = CPUREADW(insn1->rn, insn1->mode);
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105700: /* TSTB */
			tmp = CPUREADB(insn1->rn, insn1->mode);
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0006200: /* ASR */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp = src;
			if(tmp & 0x8000) {
				tmp >>= 1;
				tmp |= 0x8000;
			} else {
				tmp >>= 1;
			}
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 1);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0106200: /* ASRB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp = src;
			if(tmp & 0x80) {
				tmp >>= 1;
				tmp |= 0x80;
			} else {
				tmp >>= 1;
			}
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 1);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0006300: /* ASL */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp = src << 1;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x8000);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0106300: /* ASLB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp = (u8) (src << 1);
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x80);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0006000: /* ROR */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C);
			tmp = src >> 1;
			if(tmp2) {
				tmp |= 0x8000;
			}
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x0001);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0106000: /* RORB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C);
			tmp = src >> 1;
			if(tmp2) {
				tmp |= 0x80;
			}
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x01);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0006100: /* ROL */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C);
			tmp = src << 1;
			if(tmp2) {
				tmp |= 0x0001;
			}
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x8000);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0106100: /* ROLB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C);
			tmp = (u8) (src << 1);
			if(tmp2) {
				tmp |= 0x01;
			}
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_C, src & 0x80);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
			break;
		case 0000300: /* SWAB */
			tmp = CPUREADNW(insn1->rn, insn1->mode);
			tmp = ((tmp & 0x00FF) << 8) | ((tmp >> 8) & 0xFF);
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !((u8) tmp));
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			break;
		case 0005500: /* ADC */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C) ? 1 : 0;
			tmp = src + tmp2;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0077777 && PSW_GET(PSW_C));
			PSW_EQ(PSW_C, src == 0177777 && PSW_GET(PSW_C));
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105500: /* ADCB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C) ? 1 : 0;
			tmp = (u8) (src + tmp2);
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0177 && PSW_GET(PSW_C));
			PSW_EQ(PSW_C, src == 0377 && PSW_GET(PSW_C));
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0005600: /* SBC */
			src = CPUREADNW(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C) ? 1 : 0;
			tmp = src - tmp2;
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0100000);
			PSW_EQ(PSW_C, !src && PSW_GET(PSW_C));
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0105600: /* SBCB */
			src = CPUREADNB(insn1->rn, insn1->mode);
			tmp2 = PSW_GET(PSW_C) ? 1 : 0;
			tmp = (u8) (src - tmp2);
			CPUWRITEB(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_V, src == 0200);
			PSW_EQ(PSW_C, !src && PSW_GET(PSW_C));
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0006700: /* SXT */
			if(PSW_GET(PSW_N)) {
				tmp = 0xFFFF;
			} else {
				tmp = 0;
			}
			CPUWRITEW(insn1->rn, insn1->mode, tmp);
			PSW_EQ(PSW_Z, !PSW_GET(PSW_N));
			PSW_CLR(PSW_V);
			break;
		case 0106700: /* MFPS */
			tmp = (u8) kd11->psw;
			if(insn1->mode == 0) {
				kd11->r[insn1->rn] = (s8) kd11->psw;
			} else {
				CPUWRITEB(insn1->rn, insn1->mode, tmp);
			}
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !(tmp & 0xFF));
			PSW_CLR(PSW_V);
			break;
		case 0106400: /* MTPS */
			tmp = CPUREADB(insn1->rn, insn1->mode);
			kd11->psw = (kd11->psw & PSW_T) | (tmp & ~PSW_T);
			break;
		case 0000100: /* JMP */
			tmp = KD11CPUGetAddr(kd11, bus, insn1->rn, insn1->mode);
			CHECK();
			kd11->r[7] = tmp;
			break;
		case 0006400: /* MARK */
			kd11->r[6] = kd11->r[7] + 2 * insnmark->nn;
			kd11->r[7] = kd11->r[5];
			kd11->r[5] = READ(kd11->r[6]);
			kd11->r[6] += 2;
			break;
		default:
			unknown = 1;
			break;
	}

	if(!unknown) {
		return;
	}

	unknown = 0;

	/* double operand instructions */
	switch(insn & 0170000) {
		case 0010000: /* MOV */
			tmp = CPUREADW(insn2->src_rn, insn2->src_mode);
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0110000: /* MOVB */
			tmp = CPUREADB(insn2->src_rn, insn2->src_mode);
			tmp = (s8) tmp;
			if(insn2->dst_mode == 0) {
				kd11->r[insn2->dst_rn] = tmp;
			} else {
				CPUWRITEB(insn2->dst_rn, insn2->dst_mode, tmp);
			}
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0020000: /* CMP */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADW(insn2->dst_rn, insn2->dst_mode);
			tmp = src - dst;
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x8000) != (dst & 0x8000)) \
					&& ((dst & 0x8000) == (tmp & 0x8000)));
			PSW_EQ(PSW_C, ((u32) src - (u32) dst) & 0x10000);
			break;
		case 0120000: /* CMPB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADB(insn2->dst_rn, insn2->dst_mode);
			tmp = (u8) (src - dst);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x80) != (dst & 0x80)) \
					&& ((dst & 0x80) == (tmp & 0x80)));
			PSW_EQ(PSW_C, (src - dst) & 0x100);
			break;
		case 0060000: /* ADD */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = src + dst;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x8000) == (dst & 0x8000)) \
					&& ((dst & 0x8000) != (tmp & 0x8000)));
			PSW_EQ(PSW_C, ((u32) src + (u32) dst) & 0x10000);
			break;
		case 0160000: /* SUB */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = dst - src;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x8000) != (dst & 0x8000)) \
					&& ((src & 0x8000) == (tmp & 0x8000)));
			PSW_EQ(PSW_C, ((u32) dst - (u32) src) & 0x10000);
			break;
		case 0030000: /* BIT */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADW(insn2->dst_rn, insn2->dst_mode);
			tmp = src & dst;
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0130000: /* BITB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADB(insn2->dst_rn, insn2->dst_mode);
			tmp = src & dst;
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0040000: /* BIC */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = ~src & dst;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0140000: /* BICB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNB(insn2->dst_rn, insn2->dst_mode);
			tmp = (u8) (~src & dst);
			CPUWRITEB(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0050000: /* BIS */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = src | dst;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0150000: /* BISB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNB(insn2->dst_rn, insn2->dst_mode);
			tmp = src | dst;
			CPUWRITEB(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		default:
			unknown = 1;
			break;
	}

	if(!unknown) {
		return;
	}

	unknown = 0;

	/* XOR/JSR */
	switch(insn & 0177000) {
		case 0074000: /* XOR */
			src = kd11->r[insnjsr->r];
			dst = CPUREADNW(insnjsr->rn, insnjsr->mode);
			tmp = src ^ dst;
			CPUWRITEW(insnjsr->rn, insnjsr->mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 0004000: /* JSR */
			dst = KD11CPUGetAddr(kd11, bus, insnjsr->rn, insnjsr->mode);
			src = kd11->r[insnjsr->r];
			CHECK();
			kd11->r[6] -= 2;
			WRITE(kd11->r[6], src);
			kd11->r[insnjsr->r] = kd11->r[7];
			kd11->r[7] = dst;
			break;
		case 0077000: /* SOB */
			kd11->r[insnsob->rn]--;
			if(kd11->r[insnsob->rn]) {
				kd11->r[7] -= 2 * insnsob->offset;
			}
			break;
		case 0070000: /* MUL */
			dst = kd11->r[insnjsr->r];
			src = CPUREADW(insnjsr->rn, insnjsr->mode);
			tmps32 = (s32) (s16) dst * (s16) src;
			kd11->r[insnjsr->r] = (u16) (tmps32 >> 16);
			kd11->r[insnjsr->r | 1] = (u16) tmps32;
			PSW_CLR(PSW_V);
			PSW_EQ(PSW_N, tmps32 < 0);
			PSW_EQ(PSW_Z, !tmps32);
			PSW_EQ(PSW_C, (tmps32 >= 0x7FFF) || (tmps32 < -0x8000));
			break;
		case 0071000: /* DIV */
			tmps32 = (kd11->r[insnjsr->r] << 16)
				| kd11->r[insnjsr->r | 1];
			src = CPUREADW(insnjsr->rn, insnjsr->mode);
			if(src == 0) {
				PSW_SET(PSW_C);
				PSW_SET(PSW_V);
			} else {
				s32 quot = tmps32 / (s16) src;
				s32 rem = tmps32 % (s16) src;
				PSW_CLR(PSW_C);
				if((s16) quot != quot) {
					PSW_SET(PSW_V);
				} else {
					kd11->r[insnjsr->r] = (u16) quot;
					kd11->r[insnjsr->r | 1] = (u16) rem;
					PSW_EQ(PSW_Z, !quot);
					PSW_EQ(PSW_N, quot < 0);
				}
			}
			break;
		case 0072000: /* ASH */
			dst = kd11->r[insnjsr->r];
			src = CPUREADW(insnjsr->rn, insnjsr->mode);
			if(src & 0x20) { /* negative; right */
				src = (~src & 0x1F) + 1;
				s16 stmp = (s16) dst;
				s16 stmp2 = stmp >> (src - 1);
				stmp >>= src;
				tmp = (u16) stmp;
				PSW_EQ(PSW_C, stmp2 & 1);
				PSW_CLR(PSW_V);
			} else if((src & 0x1F) == 0) {
				/* nothing */
				PSW_CLR(PSW_V);
				PSW_CLR(PSW_C);
				tmp = dst;
			} else { /* positive, left */
				s16 mask = 0;
				src &= 0x1F;
				tmp = dst << src;
				if(src > 0) {
					mask = 0x8000;
					mask >>= src;
					tmp2 = dst & mask;
					PSW_EQ(PSW_V, !((tmp2 == 0) || (((tmp2 & mask) | ~mask) == 0xFFFF)));
				} else {
					PSW_CLR(PSW_V);
				}
				PSW_EQ(PSW_C, (dst << (src - 1)) & 0x8000);
				if((dst & 0x8000) != (tmp & 0x8000)) {
					PSW_SET(PSW_V);
				}
			}
			kd11->r[insnjsr->r] = tmp;
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			break;
		case 0073000: /* ASHC */
			dst = kd11->r[insnjsr->r];
			tmps32 = (kd11->r[insnjsr->r] << 16)
				| kd11->r[insnjsr->r | 1];
			src = CPUREADW(insnjsr->rn, insnjsr->mode);
			if((src & 0x3F) == 0x20) { /* negative; 32 right */
				PSW_EQ(PSW_C, tmps32 & 0x80000000);
				PSW_CLR(PSW_V);
				if(PSW_GET(PSW_C)) {
					tmps32 = 0xFFFFFFFF;
				} else {
					tmps32 = 0;
				}
			} else if(src & 0x20) { /* negative; right */
				s32 stmp2;
				src = (~src & 0x1F) + 1;
				stmp2 = tmps32 >> (src - 1);
				tmps32 >>= src;
				PSW_EQ(PSW_C, stmp2 & 1);
			} else if((src & 0x1F) == 0) {
				/* nothing */
				PSW_CLR(PSW_V);
				PSW_CLR(PSW_C);
			} else { /* positive, left */
				s32 stmp2;
				src &= 0x1F;
				stmp2 = tmps32 << (src - 1);
				tmps32 <<= src;
				PSW_EQ(PSW_C, stmp2 & 0x80000000);
				PSW_EQ(PSW_V, !!(dst & 0x8000)
						!= !!(tmps32 & 0x80000000));
			}
			kd11->r[insnjsr->r] = (u16) (tmps32 >> 16);
			kd11->r[insnjsr->r | 1] = (u16) tmps32;
			PSW_EQ(PSW_N, tmps32 & 0x80000000);
			PSW_EQ(PSW_Z, !tmps32);
			break;
		default:
			unknown = 1;
			break;
	}

	if(!unknown) {
		return;
	}

	unknown = 0;

	/* RTS */
	switch(insn & 0177770) {
		case 0000200: /* RTS */
			kd11->r[7] = kd11->r[insnrts->rn];
			kd11->r[insnrts->rn] = READ(kd11->r[6]);
			kd11->r[6] += 2;
			break;
		case 0075000: /* FADD */
			f1.u32 = (READ(kd11->r[insnrts->rn] + 4) << 16)
				| READ(kd11->r[insnrts->rn] + 6);
			f2.u32 = (READ(kd11->r[insnrts->rn]) << 16)
				| READ(kd11->r[insnrts->rn] + 2);
			f3.f32 = f1.f32 + f2.f32;
			/* TODO: result <= 2**-128 -> result = 0 */
			/* TODO: implement traps */
			WRITE(kd11->r[insnrts->rn] + 4,
					(u16) (f3.u32 >> 16));
			WRITE(kd11->r[insnrts->rn] + 6, (u16) f3.u32);
			PSW_EQ(PSW_N, f3.f32 < 0);
			PSW_EQ(PSW_Z, f3.f32 == 0);
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			break;
		case 0075010: /* FSUB */
			f1.u32 = (READ(kd11->r[insnrts->rn] + 4) << 16)
				| READ(kd11->r[insnrts->rn] + 6);
			f2.u32 = (READ(kd11->r[insnrts->rn]) << 16)
				| READ(kd11->r[insnrts->rn] + 2);
			f3.f32 = f1.f32 - f2.f32;
			/* TODO: result <= 2**-128 -> result = 0 */
			/* TODO: implement traps */
			WRITE(kd11->r[insnrts->rn] + 4,
					(u16) (f3.u32 >> 16));
			WRITE(kd11->r[insnrts->rn] + 6, (u16) f3.u32);
			PSW_EQ(PSW_N, f3.f32 < 0);
			PSW_EQ(PSW_Z, f3.f32 == 0);
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			break;
		case 0075020: /* FMUL */
			f1.u32 = (READ(kd11->r[insnrts->rn] + 4) << 16)
				| READ(kd11->r[insnrts->rn] + 6);
			f2.u32 = (READ(kd11->r[insnrts->rn]) << 16)
				| READ(kd11->r[insnrts->rn] + 2);
			f3.f32 = f1.f32 * f2.f32;
			/* TODO: result <= 2**-128 -> result = 0 */
			/* TODO: implement traps */
			WRITE(kd11->r[insnrts->rn] + 4,
					(u16) (f3.u32 >> 16));
			WRITE(kd11->r[insnrts->rn] + 6, (u16) f3.u32);
			PSW_EQ(PSW_N, f3.f32 < 0);
			PSW_EQ(PSW_Z, f3.f32 == 0);
			PSW_CLR(PSW_V);
			PSW_CLR(PSW_C);
			break;
		case 0075030: /* FDIV */
			f1.u32 = (READ(kd11->r[insnrts->rn] + 4) << 16)
				| READ(kd11->r[insnrts->rn] + 6);
			f2.u32 = (READ(kd11->r[insnrts->rn]) << 16)
				| READ(kd11->r[insnrts->rn] + 2);
			if(f2.f32 != 0) {
				f3.f32 = f1.f32 / f2.f32;
				/* TODO: result <= 2**-128 -> result = 0 */
				/* TODO: implement traps */
				WRITE(kd11->r[insnrts->rn] + 4,
						(u16) (f3.u32 >> 16));
				WRITE(kd11->r[insnrts->rn] + 6,
						(u16) f3.u32);
				PSW_EQ(PSW_N, f3.f32 < 0);
				PSW_EQ(PSW_Z, f3.f32 == 0);
				PSW_CLR(PSW_V);
				PSW_CLR(PSW_C);
			}
			break;
		default:
			unknown = 1;
			break;
	}

	if(!unknown) {
		return;
	}

	unknown = 0;

	/* branches */
	switch(insn & 0177400) {
		case 0000400: /* BR */
			kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			break;
		case 0001000: /* BNE */
			if(!PSW_GET(PSW_Z)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0001400: /* BEQ */
			if(PSW_GET(PSW_Z)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0100000: /* BPL */
			if(!PSW_GET(PSW_N)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0100400: /* BMI */
			if(PSW_GET(PSW_N)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0102000: /* BVC */
			if(!PSW_GET(PSW_V)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0102400: /* BVS */
			if(PSW_GET(PSW_V)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0103000: /* BCC */
			if(!PSW_GET(PSW_C)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0103400: /* BCS */
			if(PSW_GET(PSW_C)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0002000: /* BGE */
			if((PSW_GET(PSW_N) ^ PSW_GET(PSW_V)) == 0) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0002400: /* BLT */
			if(PSW_GET(PSW_N) ^ PSW_GET(PSW_V)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0003000: /* BGT */
			if((PSW_GET(PSW_Z) || (PSW_GET(PSW_N) ^ PSW_GET(PSW_V))) == 0) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0003400: /* BLE */
			if(PSW_GET(PSW_Z) || (PSW_GET(PSW_N) ^ PSW_GET(PSW_V))) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0101000: /* BHI */
			if(!PSW_GET(PSW_C) && !PSW_GET(PSW_Z)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0101400: /* BLOS */
			if(PSW_GET(PSW_C) || PSW_GET(PSW_Z)) {
				kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
			}
			break;
		case 0104000: /* EMT */
			TRCTrap(030, TRC_TRAP);
			TRAP(030);
			break;
		case 0104400: /* TRAP */
			TRCTrap(034, TRC_TRAP);
			TRAP(034);
			break;
		default:
			unknown = 1;
			break;
	}

	if(!unknown) {
		return;
	}

	unknown = 0;

	/* misc instructions without operands */
	switch(insn) {
		case 0000003: /* BPT */
			TRCTrap(014, TRC_TRAP);
			TRAP(014);
			break;
		case 0000004: /* IOT */
			TRCTrap(020, TRC_TRAP);
			TRAP(020);
			break;
		case 0000002: /* RTI */
			kd11->r[7] = READ(kd11->r[6]);
			kd11->r[6] += 2;
			CHECK();
			kd11->psw = READ(kd11->r[6]);
			kd11->r[6] += 2;
			CHECK();
			break;
		case 0000006: /* RTT */
			kd11->r[7] = READ(kd11->r[6]);
			kd11->r[6] += 2;
			CHECK();
			kd11->psw = READ(kd11->r[6]);
			kd11->r[6] += 2;
			CHECK();
			kd11->state = STATE_INHIBIT_TRACE;
			break;
		case 0000000: /* HALT */
			TRCCPUEvent(TRC_CPU_HALT, kd11->r[7]);
			kd11->state = STATE_HALT;
			kd11->odt.state = ODT_STATE_INIT;
			break;
		case 0000001: /* WAIT */
			TRCCPUEvent(TRC_CPU_WAIT, kd11->r[7]);
			kd11->state = STATE_WAIT;
			break;
		case 0000005: /* RESET */
			bus->reset(bus);
			break;
		default:
			unknown = 1;
			break;
	}

	/* CLN/CLZ/CLV/CLC/CCC/SEN/SEZ/SEV/SEC/SCC */
	if(unknown && ((insn & 0177740) == 0000240)) {
		tmp = insn & 017;
		if(insn & 020) {
			kd11->psw |= tmp;
		} else {
			kd11->psw &= ~tmp;
		}
	} else if(unknown) {
		TRCTrap(010, TRC_TRAP_ILL);
		TRAP(010);
	}
}

void KD11HandleTraps(KD11* kd11, QBUS* bus)
{
	u16 trap = kd11->trap;

	if(!PSW_GET(PSW_PRIO)) {
		kd11->trap = bus->trap;
		bus->trap = 0;
	} else if(bus->trap == 004) {
		kd11->trap = bus->trap;
		bus->trap = 0;
	} else {
		kd11->trap = 0;
	}

	/* ignore traps if in HALT mode */
	if(kd11->state == STATE_HALT) {
		return;
	}

	if(!trap) {
		if(kd11->trap) {
			trap = kd11->trap;
			kd11->trap = 0;
		} else {
			return;
		}
	}

	/* trap instructions have highest priority */
	if((kd11->trap == 030 || kd11->trap == 034)
			&& !(trap == 030 || trap == 034)) {
		u16 tmp = kd11->trap;
		kd11->trap = trap;
		trap = tmp;
	}

	TRCCPUEvent(TRC_CPU_TRAP, trap);

	kd11->r[6] -= 2;
	WRITE(kd11->r[6], kd11->psw);
	if(bus->trap == 004 || bus->trap == 010) {
		TRCCPUEvent(TRC_CPU_DBLBUS, kd11->r[6]);
		bus->trap = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->r[6] -= 2;
	WRITE(kd11->r[6], kd11->r[7]);
	if(bus->trap == 004 || bus->trap == 010) {
		TRCCPUEvent(TRC_CPU_DBLBUS, kd11->r[6]);
		bus->trap = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->r[7] = READ(trap);
	if(bus->trap == 004 || bus->trap == 010) {
		TRCCPUEvent(TRC_CPU_DBLBUS, trap);
		bus->trap = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->psw = READ(trap + 2);
	if(bus->trap == 004 || bus->trap == 010) {
		TRCCPUEvent(TRC_CPU_DBLBUS, trap + 2);
		bus->trap = 0;
		kd11->state = STATE_HALT;
		return;
	}

	/* resume execution if in WAIT state */
	if(kd11->state == STATE_WAIT) {
		TRCCPUEvent(TRC_CPU_RUN, kd11->r[7]);
		kd11->state = STATE_RUN;
	}
}

void KD11Step(KD11* kd11, QBUS* bus)
{
	switch(kd11->state) {
		case STATE_HALT:
			KD11ODTStep(kd11, bus);
			break;
		case STATE_RUN:
			IFTRC() {
				TRCSETIGNBUS();
				u16 bus_trap = bus->trap;
				u16 cpu_trap = kd11->trap;
				u16 code[3];
				code[0] = READ(kd11->r[7] + 0);
				code[1] = READ(kd11->r[7] + 2);
				code[2] = READ(kd11->r[7] + 4);
				TRCStep(kd11->r, kd11->psw, code);
				bus->trap = bus_trap;
				kd11->trap = cpu_trap;
				TRCCLRIGNBUS();
			}
			KD11CPUStep(kd11, bus);
			if(kd11->state == STATE_INHIBIT_TRACE) {
				kd11->state = STATE_RUN;
			} else if(!kd11->trap && (kd11->psw & PSW_T)) {
				TRCTrap(014, TRC_TRAP_T);
				TRAP(014);
			}
			KD11HandleTraps(kd11, bus);
			if(kd11->state == STATE_HALT) {
				kd11->odt.state = ODT_STATE_INIT;
			}
			break;
		case STATE_WAIT:
			KD11HandleTraps(kd11, bus);
			break;
	}
}

void KD11Trap(KD11* kd11, int n)
{
	if(kd11->trap == 0 || kd11->trap > n) {
		kd11->trap = n;
	}
}
