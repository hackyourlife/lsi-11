#include <string.h>
#include <stdlib.h>

#include "lsi11.h"
#include "trace.h"

#define USE_FLOAT

/* ODT states */
#define	ODT_STATE_INIT		0
#define	ODT_STATE_WAIT		1
#define	ODT_STATE_ADDR		2
#define	ODT_STATE_REG		3
#define	ODT_STATE_REG_WAIT	4
#define	ODT_STATE_VAL		5
#define	ODT_STATE_REG_VAL	6
#define	ODT_STATE_WR		7
#define	ODT_STATE_SEMICOLON	8
#define	ODT_STATE_ADDR_SC	9

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

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
/* big endian host */
typedef struct {
	u16	opcode:10;
	u16	mode:3;
	u16	rn:3;
} KD11INSN1;

typedef struct {
	u16	opcode:4;
	u16	src_mode:3;
	u16	src_rn:3;
	u16	dst_mode:3;
	u16	dst_rn:3;
} KD11INSN2;

typedef struct {
	u16	opcode:8;
	u16	offset:8;
} KD11INSNBR;

typedef struct {
	u16	opcode:7;
	u16	r:3;
	u16	mode:3;
	u16	rn:3;
} KD11INSNJSR;

typedef struct {
	u16	opcode:13;
	u16	rn:3;
} KD11INSNRTS;

typedef struct {
	u16	opcode:10;
	u16	nn:6;
} KD11INSNMARK;

typedef struct {
	u16	opcode:7;
	u16	rn:3;
	u16	offset:6;
} KD11INSNSOB;

#else
/* little endian host */
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
#endif

static int KD11HasPendingIRQ(KD11* kd11, QBUS* bus);

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
#define	READ8(addr)		(bus->read8(bus->user, (addr)))
#define	WRITE8(addr, val)	(bus->write8(bus->user, (addr), (val)))
#define	CHECK()			{ \
	if((kd11->trap && kd11->trap <= 010) || bus->nxm) \
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
					case ';':
						odt->state = ODT_STATE_SEMICOLON;
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
					if(bus->nxm) {
						bus->nxm = 0;
					} else {
						KD11ODTWriteOctal(odt, val);
						KD11ODTWrite(odt, ' ');
						odt->next = ODT_STATE_VAL;
						odt->state = ODT_STATE_WR;
						odt->input = 0;
						odt->val = 0;
					}
				} else if(c >= '0' && c <= '7') {
					odt->addr <<= 3;
					odt->addr |= c - '0';
				} else if(c == 0177) {
					WRITE(0177566, '\\');
					odt->addr >>= 3;
				} else if(c == 'G') {
					odt->state = ODT_STATE_INIT;
					kd11->r[7] = odt->addr;
					kd11->state = STATE_RUN;
					TRCCPUEvent(TRC_CPU_ODT_G, odt->addr);
					bus->reset(bus);
				} else if(c == 'L') {
					TRCCPUEvent(TRC_CPU_ODT_L, odt->addr);
					/* TODO: size RAM instead of assuming 28K */
					WRITE(0157744, 0016701);
					WRITE(0157746, 0000026);
					WRITE(0157750, 0012702);
					WRITE(0157752, 0000352);
					WRITE(0157754, 0005211);
					WRITE(0157756, 0105711);
					WRITE(0157760, 0100376);
					WRITE(0157762, 0116162);
					WRITE(0157764, 0000002);
					WRITE(0157766, 0157400);
					WRITE(0157770, 0005267);
					WRITE(0157772, 0177756);
					WRITE(0157774, 0000765);
					WRITE(0157776, odt->addr);
					odt->state = ODT_STATE_INIT;
					kd11->r[7] = 0157744;
					kd11->state = STATE_RUN;
					bus->reset(bus);
				} else if(c == ';') {
					odt->state = ODT_STATE_ADDR_SC;
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
				if(c == '\r' || c == '\n' || c == '^'
						|| c == '@' || c == '_') {
					if(odt->input) {
						WRITE(odt->addr, odt->val);
					}
				} else if(c >= '0' && c <= '7') {
					odt->val <<= 3;
					odt->val |= c - '0';
					odt->input = 1;
				} else if(c == 0177) {
					WRITE(0177566, '\\');
					odt->val >>= 3;
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
					if(bus->nxm) {
						bus->nxm = 0;
						KD11ODTInputError(odt);
					} else {
						KD11ODTWrite(odt, '\r');
						KD11ODTWriteOctal(odt, odt->addr);
						KD11ODTWrite(odt, '/');
						KD11ODTWriteOctal(odt, val);
						KD11ODTWrite(odt, ' ');

						odt->next = ODT_STATE_VAL;
						odt->state = ODT_STATE_WR;
						odt->input = 0;
					}
				} else if(c == '^') {
					u16 val;

					odt->addr -= 2;
					odt->val = 0;
					val = READ(odt->addr);

					KD11ODTClear(odt);
					if(bus->nxm) {
						bus->nxm = 0;
						KD11ODTInputError(odt);
					} else {
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
				} else if(c == '@') {
					u16 val;

					odt->addr = READ(odt->addr) & 0xFFFE;
					odt->val = 0;
					val = READ(odt->addr);

					KD11ODTClear(odt);
					if(bus->nxm) {
						bus->nxm = 0;
						KD11ODTInputError(odt);
					} else {
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
				if(c == '\r' || c == '\n' || c == '^'
						|| c == '@' || c == '_') {
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
				} else if(c == 0177) {
					WRITE(0177566, '\\');
					odt->val >>= 3;
				} else {
					KD11ODTInputError(odt);
				}
				if(c == '\r' || c == '_' || ((c == '\n' || c == '^')
							&& odt->addr == 8)) {
					KD11ODTClear(odt);
					if(c != '\r') {
						KD11ODTWrite(odt, '\r');
					}
					if(c != '\n') {
						KD11ODTWrite(odt, '\n');
					}
					KD11ODTWrite(odt, '@');
					odt->state = ODT_STATE_WR;
					odt->next = ODT_STATE_WAIT;
				} else if(c == '\n') {
					u16 val;

					odt->addr++;
					odt->addr &= 0x07;
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
				} else if(c == '^') {
					u16 val;

					odt->addr--;
					odt->addr &= 0x07;
					odt->val = 0;
					val = kd11->r[odt->addr];

					KD11ODTClear(odt);
					KD11ODTWrite(odt, '\r');
					KD11ODTWrite(odt, '\n');
					KD11ODTWrite(odt, 'R');
					KD11ODTWrite(odt, odt->addr + '0');
					KD11ODTWrite(odt, '/');
					KD11ODTWriteOctal(odt, val);
					KD11ODTWrite(odt, ' ');

					odt->next = ODT_STATE_REG_VAL;
					odt->state = ODT_STATE_WR;
					odt->input = 0;
				} else if(c == '@') {
					u16 val;

					odt->val = 0;
					odt->addr = kd11->r[odt->addr] & 0xFFFE;
					val = READ(odt->addr);

					KD11ODTClear(odt);
					if(bus->nxm) {
						bus->nxm = 0;
						KD11ODTInputError(odt);
					} else {
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
			}
			break;
		case ODT_STATE_SEMICOLON:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 c = READ(0177562);
				WRITE(0177566, (u8) c);
				switch((u8) c) {
					case 'P':
						odt->state = ODT_STATE_INIT;
						kd11->state = STATE_RUN;
						TRCCPUEvent(TRC_CPU_ODT_P, odt->addr);
						break;
					default:
						KD11ODTInputError(odt);
						break;
				}
			}
			break;
		case ODT_STATE_ADDR_SC:
			if(READ(0177560) & 0x80) { /* ch available */
				u16 c = READ(0177562);
				WRITE(0177566, (u8) c);
				switch((u8) c) {
					case 'G':
						odt->state = ODT_STATE_INIT;
						kd11->r[7] = odt->addr;
						kd11->state = STATE_RUN;
						TRCCPUEvent(TRC_CPU_ODT_G, odt->addr);
						bus->reset(bus);
						break;
					default:
						KD11ODTInputError(odt);
						break;
				}
			}
			break;
	}
}

#define	CHECKREAD()		{ \
	if(bus->nxm) \
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

#ifdef USE_FLOAT
#define	FP_V_EXP		23
#define	FP_V_HB			23
#define	FP_M_EXP		0377
#define	FP_SIGN			(1 << 31)
#define	FP_EXP			(FP_M_EXP << FP_V_EXP)
#define	FP_HB			(1 << FP_V_HB)
#define	FP_FRAC			((1 << FP_V_HB) - 1)
#define	FP_BIAS			0200
#define	FP_GUARD		3

#define	FP_ROUND_GUARD		(1 << (FP_GUARD - 1))

#define	F_LT(x, y)		((x) < (y))
#define	F_LT_ABS(x, y)		(((x) & ~FP_SIGN) < ((y) & ~FP_SIGN))
#define	F_SIGN(x)		((x) & FP_SIGN)
#define	F_ADD(s1, s2, d)	d = ((s1) + (s2))
#define	F_SUB(s1, s2, d)	d = ((s2) - (s1))
#define	F_LSH_1(x)		x = ((x) << 1);
#define	F_RSH_1(x)		x = ((x) >> 1) & 0x7FFFFFFF
#define F_LSH_V(s, n, d)	d = ((n) >= 32) ? 0 : (s << (n));
#define F_RSH_V(s, n, d)	d = ((n) >= 32) ? 0 : ((s >> (n)) & fp_and_mask[32 - (n)]) & 0xFFFFFFFF
#define	F_LSH_K(s, n, d)	d = (s) << (n)
#define	F_RSH_K(s, n, d)	d = (((s) >> (n)) & fp_and_mask[32 - (n)]) & 0xFFFFFFFF
#define	F_LSH_GUARD(x)		F_LSH_K(x, FP_GUARD, x)
#define	F_RSH_GUARD(x)		F_RSH_K(x, FP_GUARD, x)
#define	F_EXP(x)		(((x) >> FP_V_EXP) & FP_M_EXP)
#define	F_FRAC(x)		(((x) & FP_FRAC) | FP_HB)

static const u32 fp_and_mask[33] = {
	0,
	0x1, 0x3, 0x7, 0xF,
	0x1F, 0x3F, 0x7F, 0xFF,
	0x1FF, 0x3FF, 0x7FF, 0xFFF,
	0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF,
	0x1FFFF, 0x3FFFF, 0x7FFFF, 0xFFFFF,
	0x1FFFFF, 0x3FFFFF, 0x7FFFFF, 0xFFFFFF,
	0x1FFFFFF, 0x3FFFFFF, 0x7FFFFFF, 0xFFFFFFF,
	0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
};

static s32 KD11FPURoundPack(KD11* kd11, s32 sign, s32 exp, s32 frac, unsigned int* trap)
{
	/* round */
	frac = frac + FP_ROUND_GUARD;
	if(frac & _BV(FP_V_HB + FP_GUARD + 1)) {
		F_RSH_1(frac);
		exp++;
	}

	/* pack */
	F_RSH_GUARD(frac);

	if(exp > 0377) {
		/* overflow */
		TRCTrap(0244, TRC_TRAP);
		TRAP(0244);
		PSW_SET(PSW_V);
		PSW_CLR(PSW_N);
		PSW_CLR(PSW_C);
		PSW_CLR(PSW_Z);
		*trap = 1;
		return 0;
	} else if(exp <= 0) {
		/* underflow */
		TRCTrap(0244, TRC_TRAP);
		TRAP(0244);
		PSW_SET(PSW_V);
		PSW_SET(PSW_N);
		PSW_CLR(PSW_C);
		PSW_CLR(PSW_Z);
		*trap = 1;
		return 0;
	} else {
		*trap = 0;
		return (sign & FP_SIGN) | ((exp & FP_M_EXP) << FP_V_EXP)
			| (frac & FP_FRAC);
	}
}

static s32 KD11FPUADD(KD11* kd11, s32 ac, s32 src, unsigned int* trap)
{
	*trap = 0;

	if(F_LT_ABS(ac, src)) {
		/* swap operands */
		s32 tmp = ac;
		ac = src;
		src = tmp;
	}

	s32 acexp = F_EXP(ac);
	s32 srcexp = F_EXP(src);

	if(acexp == 0) {
		return src;
	} else if(srcexp == 0) {
		return ac;
	}

	s32 ediff = acexp - srcexp;

	if(ediff >= 60) {
		return ac;
	}

	s32 acfrac = F_FRAC(ac);
	s32 srcfrac = F_FRAC(src);

	F_LSH_GUARD(acfrac);
	F_LSH_GUARD(srcfrac);

	if(F_SIGN(ac) != F_SIGN(src)) {
		if(ediff) {
			F_RSH_V(srcfrac, ediff, srcfrac);
		}
		F_SUB(srcfrac, acfrac, acfrac);
		if(!acfrac) {
			return 0;
		}
		if(ediff <= 1) {
			if((acfrac & (0x00FFFFFF << FP_GUARD)) == 0) {
				F_LSH_K(acfrac, 24, acfrac);
				acexp -= 24;
			}
			if((acfrac & (0x00FFF000 << FP_GUARD)) == 0) {
				F_LSH_K(acfrac, 12, acfrac);
				acexp -= 12;
			}
			if((acfrac & (0x00FC0000 << FP_GUARD)) == 0) {
				F_LSH_K(acfrac, 6, acfrac);
				acexp -= 6;
			}
		}
		while((acfrac & _BV(FP_V_HB + FP_GUARD)) == 0) {
			F_LSH_1(acfrac);
			acexp--;
		}
	} else {
		if(ediff) {
			F_RSH_V(srcfrac, ediff, srcfrac);
		}
		F_ADD(srcfrac, acfrac, acfrac);
		if(acfrac & _BV(FP_V_HB + FP_GUARD + 1)) {
			F_RSH_1(acfrac);
			acexp++;
		}
	}

	return KD11FPURoundPack(kd11, ac, acexp, acfrac, trap);
}

static s32 KD11FPUMUL(KD11* kd11, s32 ac, s32 src, unsigned int* trap)
{
	*trap = 0;

	s32 acexp = F_EXP(ac);
	s32 srcexp = F_EXP(src);

	if(!acexp || !srcexp) {
		return 0;
	}

	s32 acfrac = F_FRAC(ac);
	s32 srcfrac = F_FRAC(src);

	acexp += srcexp - FP_BIAS;
	ac ^= src;

	/* frac_mulfp11 */
	acfrac = ((u64) acfrac * (u64) srcfrac) >> 21;

	if((acfrac & _BV(FP_V_HB + FP_GUARD)) == 0) {
		F_LSH_1(acfrac);
		acexp--;
	}

	return KD11FPURoundPack(kd11, ac, acexp, acfrac, trap);
}

static s32 KD11FPUDIV(KD11* kd11, s32 ac, s32 src, unsigned int* trap)
{
	*trap = 0;

	s32 acexp = F_EXP(ac);
	s32 srcexp = F_EXP(src);

	if(srcexp == 0) {
		/* divide by zero */
		TRCTrap(0244, TRC_TRAP);
		TRAP(0244);
		PSW_SET(PSW_V);
		PSW_SET(PSW_N);
		PSW_SET(PSW_C);
		PSW_CLR(PSW_Z);
		*trap = 1;
		return 0;
	}

	if(acexp == 0) {
		return 0;
	}

	s32 acfrac = F_FRAC(ac);
	s32 srcfrac = F_FRAC(src);
	F_LSH_GUARD(acfrac);
	F_LSH_GUARD(srcfrac);

	acexp = acexp - srcexp + FP_BIAS + 1;
	ac ^= src;

	int count = FP_V_HB + FP_GUARD + 1;

	s32 quo = 0;
	int i;
	for(i = count; (i > 0) && acfrac; i--) {
		F_LSH_1(quo);
		if(!F_LT(acfrac, srcfrac)) {
			F_SUB(srcfrac, acfrac, acfrac);
			quo |= 1;
		}
		F_LSH_1(acfrac);
	}
	if(i > 0) {
		F_LSH_V(quo, i, quo);
	}

	if((quo & _BV(FP_V_HB + FP_GUARD)) == 0) {
		F_LSH_1(quo);
		acexp--;
	}

	return KD11FPURoundPack(kd11, ac, acexp, quo, trap);
}
#endif

void KD11CPUStep(KD11* kd11, QBUS* bus)
{
	u16 tmp, tmp2;
	u16 src, dst;
	s32 tmps32;
#ifdef USE_FLOAT
	s32 f1, f2, f3;
	unsigned int trap;
#endif

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

	switch(insn >> 12) {
		case 000: /* 00 xx xx group */
			switch(insn >> 6) {
				case 00000: /* 00 00 xx group */
					switch(insn) {
						case 0000000: /* HALT */
							TRCCPUEvent(TRC_CPU_HALT, kd11->r[7]);
							kd11->state = STATE_HALT;
							kd11->odt.state = ODT_STATE_INIT;
							break;
						case 0000001: /* WAIT */
							TRCCPUEvent(TRC_CPU_WAIT, kd11->r[7]);
							kd11->state = STATE_WAIT;
							break;
						case 0000002: /* RTI */
							kd11->r[7] = READ(kd11->r[6]);
							kd11->r[6] += 2;
							CHECK();
							kd11->psw = READ(kd11->r[6]);
							kd11->r[6] += 2;
							CHECK();
							break;
						case 0000003: /* BPT */
							TRCTrap(014, TRC_TRAP);
							TRAP(014);
							break;
						case 0000004: /* IOT */
							TRCTrap(020, TRC_TRAP);
							TRAP(020);
							break;
						case 0000005: /* RESET */
							bus->reset(bus);
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
						default: /* 00 00 07 - 00 00 77 */
							/* unused opcodes */
							TRCTrap(010, TRC_TRAP_ILL);
							TRAP(010);
							break;
					}
					break;
				case 00001: /* JMP */
					tmp = KD11CPUGetAddr(kd11, bus, insn1->rn, insn1->mode);
					CHECK();
					kd11->r[7] = tmp;
					break;
				case 00002: /* 00 02 xx group */
					/* mask=177740: CLN/CLZ/CLV/CLC/CCC/SEN/SEZ/SEV/SEC/SCC */
					if((insn & 0177770) == 0000200) {
						/* RTS */
						kd11->r[7] = kd11->r[insnrts->rn];
						kd11->r[insnrts->rn] = READ(kd11->r[6]);
						kd11->r[6] += 2;
					} else if((insn & 0177740) == 0000240) {
						tmp = insn & 017;
						if(insn & 020) {
							kd11->psw |= tmp;
						} else {
							kd11->psw &= ~tmp;
						}
					} else {
						/* 00 02 10 - 00 02 27: unused */
						TRCTrap(010, TRC_TRAP_ILL);
						TRAP(010);
					}
					break;
				case 00003: /* SWAB */
					tmp = CPUREADNW(insn1->rn, insn1->mode);
					tmp = ((tmp & 0x00FF) << 8) | ((tmp >> 8) & 0xFF);
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !((u8) tmp));
					PSW_CLR(PSW_V);
					PSW_CLR(PSW_C);
					break;
				case 00004: /* BR */
				case 00005:
				case 00006:
				case 00007:
					kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					break;
				case 00010: /* BNE */
				case 00011:
				case 00012:
				case 00013:
					if(!PSW_GET(PSW_Z)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00014: /* BEQ */
				case 00015:
				case 00016:
				case 00017:
					if(PSW_GET(PSW_Z)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00020: /* BGE */
				case 00021:
				case 00022:
				case 00023:
					if((PSW_GET(PSW_N) ^ PSW_GET(PSW_V)) == 0) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00024: /* BLT */
				case 00025:
				case 00026:
				case 00027:
					if(PSW_GET(PSW_N) ^ PSW_GET(PSW_V)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00030: /* BGT */
				case 00031:
				case 00032:
				case 00033:
					if((PSW_GET(PSW_Z) || (PSW_GET(PSW_N) ^ PSW_GET(PSW_V))) == 0) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00034: /* BLE */
				case 00035:
				case 00036:
				case 00037:
					if(PSW_GET(PSW_Z) || (PSW_GET(PSW_N) ^ PSW_GET(PSW_V))) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 00040: /* JSR */
				case 00041:
				case 00042:
				case 00043:
				case 00044:
				case 00045:
				case 00046:
				case 00047:
					dst = KD11CPUGetAddr(kd11, bus, insnjsr->rn, insnjsr->mode);
					src = kd11->r[insnjsr->r];
					CHECK();
					kd11->r[6] -= 2;
					WRITE(kd11->r[6], src);
					kd11->r[insnjsr->r] = kd11->r[7];
					kd11->r[7] = dst;
					break;
				case 00050: /* CLR */
					CPUWRITEW(insn1->rn, insn1->mode, 0);
					PSW_CLR(PSW_N | PSW_V | PSW_C);
					PSW_SET(PSW_Z);
					break;
				case 00051: /* COM */
					tmp = CPUREADNW(insn1->rn, insn1->mode);
					tmp = ~tmp;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_CLR(PSW_V);
					PSW_SET(PSW_C);
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00052: /* INC */
					src = CPUREADNW(insn1->rn, insn1->mode);
					tmp = src + 1;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 077777)
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00053: /* DEC */
					src = CPUREADNW(insn1->rn, insn1->mode);
					tmp = src - 1;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0100000)
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00054: /* NEG */
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
				case 00055: /* ADC */
					src = CPUREADNW(insn1->rn, insn1->mode);
					tmp2 = PSW_GET(PSW_C) ? 1 : 0;
					tmp = src + tmp2;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0077777 && PSW_GET(PSW_C));
					PSW_EQ(PSW_C, src == 0177777 && PSW_GET(PSW_C));
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00056: /* SBC */
					src = CPUREADNW(insn1->rn, insn1->mode);
					tmp2 = PSW_GET(PSW_C) ? 1 : 0;
					tmp = src - tmp2;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0100000);
					PSW_EQ(PSW_C, !src && PSW_GET(PSW_C));
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00057: /* TST */
					tmp = CPUREADW(insn1->rn, insn1->mode);
					PSW_CLR(PSW_V);
					PSW_CLR(PSW_C);
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 00060: /* ROR */
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
				case 00061: /* ROL */
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
				case 00062: /* ASR */
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
				case 00063: /* ASL */
					src = CPUREADNW(insn1->rn, insn1->mode);
					tmp = src << 1;
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_C, src & 0x8000);
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
					break;
				case 00064: /* MARK */
					kd11->r[6] = kd11->r[7] + 2 * insnmark->nn;
					kd11->r[7] = kd11->r[5];
					kd11->r[5] = READ(kd11->r[6]);
					kd11->r[6] += 2;
					break;
				case 00067: /* SXT */
					if(PSW_GET(PSW_N)) {
						tmp = 0xFFFF;
					} else {
						tmp = 0;
					}
					CPUWRITEW(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_Z, !PSW_GET(PSW_N));
					PSW_CLR(PSW_V);
					break;
				default: /* 006500-006677, 007000-007777: unused */
					TRCTrap(010, TRC_TRAP_ILL);
					TRAP(010);
					break;
			}
			break;
		case 001: /* MOV */
			tmp = CPUREADW(insn2->src_rn, insn2->src_mode);
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 002: /* CMP */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADW(insn2->dst_rn, insn2->dst_mode);
			tmp = src - dst;
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x8000) != (dst & 0x8000)) \
					&& ((dst & 0x8000) == (tmp & 0x8000)));
			PSW_EQ(PSW_C, ((u32) src - (u32) dst) & 0x10000);
			break;
		case 003: /* BIT */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADW(insn2->dst_rn, insn2->dst_mode);
			tmp = src & dst;
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 004: /* BIC */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = ~src & dst;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 005: /* BIS */
			src = CPUREADW(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNW(insn2->dst_rn, insn2->dst_mode);
			tmp = src | dst;
			CPUWRITEW(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x8000);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 006: /* ADD */
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
		case 007: /* 07 xx xx group */
			switch(insn >> 9) {
				case 0070: /* MUL */
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
				case 0071: /* DIV */
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
				case 0072: /* ASH */
					dst = kd11->r[insnjsr->r];
					src = CPUREADW(insnjsr->rn, insnjsr->mode);
					if(src & 0x20) { /* negative; right */
						s16 stmp, stmp2;
						src = (~src & 0x1F) + 1;
						stmp = (s16) dst;
						stmp2 = stmp >> (src - 1);
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
				case 0073: /* ASHC */
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
				case 0074: /* XOR */
					src = kd11->r[insnjsr->r];
					dst = CPUREADNW(insnjsr->rn, insnjsr->mode);
					tmp = src ^ dst;
					CPUWRITEW(insnjsr->rn, insnjsr->mode, tmp);
					PSW_EQ(PSW_N, tmp & 0x8000);
					PSW_EQ(PSW_Z, !tmp);
					PSW_CLR(PSW_V);
					break;
				case 0075: /* FIS instructions */
					switch(insn >> 3) {
#ifdef USE_FLOAT
						case 007500: /* FADD */
							f1 = (READ(kd11->r[insnrts->rn] + 4) << 16)
								| READ(kd11->r[insnrts->rn] + 6);
							f2 = (READ(kd11->r[insnrts->rn]) << 16)
								| READ(kd11->r[insnrts->rn] + 2);
							if(bus->nxm) {
								trap = 1;
							} else {
								f3 = KD11FPUADD(kd11, f1, f2, &trap);
							}
							if(!trap) {
								WRITE(kd11->r[insnrts->rn] + 4, (u16) ((u32) f3 >> 16));
								WRITE(kd11->r[insnrts->rn] + 6, (u16) f3);
								kd11->r[insnrts->rn] += 4;
								PSW_EQ(PSW_N, F_SIGN(f3));
								PSW_EQ(PSW_Z, f3 == 0);
								PSW_CLR(PSW_V);
								PSW_CLR(PSW_C);
							}
							if(!bus->nxm) {
								PSW_CLR(0140);
							}
							break;
						case 007501: /* FSUB */
							f1 = (READ(kd11->r[insnrts->rn] + 4) << 16)
								| READ(kd11->r[insnrts->rn] + 6);
							f2 = (READ(kd11->r[insnrts->rn]) << 16)
								| READ(kd11->r[insnrts->rn] + 2);
							if(f2) {
								f2 ^= FP_SIGN;
							}
							if(bus->nxm) {
								trap = 1;
							} else {
								f3 = KD11FPUADD(kd11, f1, f2, &trap);
							}
							if(!trap) {
								WRITE(kd11->r[insnrts->rn] + 4, (u16) ((u32) f3 >> 16));
								WRITE(kd11->r[insnrts->rn] + 6, (u16) f3);
								kd11->r[insnrts->rn] += 4;
								PSW_EQ(PSW_N, F_SIGN(f3));
								PSW_EQ(PSW_Z, f3 == 0);
								PSW_CLR(PSW_V);
								PSW_CLR(PSW_C);
							}
							if(!bus->nxm) {
								PSW_CLR(0140);
							}
							break;
						case 007502: /* FMUL */
							f1 = (READ(kd11->r[insnrts->rn] + 4) << 16)
								| READ(kd11->r[insnrts->rn] + 6);
							f2 = (READ(kd11->r[insnrts->rn]) << 16)
								| READ(kd11->r[insnrts->rn] + 2);
							if(bus->nxm) {
								trap = 1;
							} else {
								f3 = KD11FPUMUL(kd11, f1, f2, &trap);
							}
							if(!trap) {
								WRITE(kd11->r[insnrts->rn] + 4, (u16) ((u32) f3 >> 16));
								WRITE(kd11->r[insnrts->rn] + 6, (u16) f3);
								kd11->r[insnrts->rn] += 4;
								PSW_EQ(PSW_N, F_SIGN(f3));
								PSW_EQ(PSW_Z, f3 == 0);
								PSW_CLR(PSW_V);
								PSW_CLR(PSW_C);
							}
							if(!bus->nxm) {
								PSW_CLR(0140);
							}
							break;
						case 007503: /* FDIV */
							f1 = (READ(kd11->r[insnrts->rn] + 4) << 16)
								| READ(kd11->r[insnrts->rn] + 6);
							f2 = (READ(kd11->r[insnrts->rn]) << 16)
								| READ(kd11->r[insnrts->rn] + 2);
							if(f2 != 0 && !bus->nxm) {
								if(bus->nxm) {
									trap = 1;
								} else {
									f3 = KD11FPUDIV(kd11, f1, f2, &trap);
								}
								if(!trap) {
									WRITE(kd11->r[insnrts->rn] + 4, (u16) ((u32) f3 >> 16));
									WRITE(kd11->r[insnrts->rn] + 6, (u16) (u32) f3);
									kd11->r[insnrts->rn] += 4;
									PSW_EQ(PSW_N, F_SIGN(f3));
									PSW_EQ(PSW_Z, f3 == 0);
									PSW_CLR(PSW_V);
									PSW_CLR(PSW_C);
								}
							} else if(!bus->nxm) {
								PSW_SET(PSW_V);
								PSW_SET(PSW_N);
								PSW_SET(PSW_C);
								PSW_CLR(PSW_Z);
								TRCTrap(0244, TRC_TRAP);
								TRAP(0244);
							}
							if(!bus->nxm) {
								PSW_CLR(0140);
							}
							break;
#endif
						default:
							/* 075040-076777: unused */
							TRCTrap(010, TRC_TRAP_ILL);
							TRAP(010);
							break;
					}
					break;
				case 0077: /* SOB */
					kd11->r[insnsob->rn]--;
					if(kd11->r[insnsob->rn]) {
						kd11->r[7] -= 2 * insnsob->offset;
					}
					break;
				default:
					TRCTrap(010, TRC_TRAP_ILL);
					TRAP(010);
					break;
			}
			break;
		case 010: /* 10 xx xx group */
			switch(insn >> 6) {
				case 01000: /* BPL */
				case 01001:
				case 01002:
				case 01003:
					if(!PSW_GET(PSW_N)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01004: /* BMI */
				case 01005:
				case 01006:
				case 01007:
					if(PSW_GET(PSW_N)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01010: /* BHI */
				case 01011:
				case 01012:
				case 01013:
					if(!PSW_GET(PSW_C) && !PSW_GET(PSW_Z)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01014: /* BLOS */
				case 01015:
				case 01016:
				case 01017:
					if(PSW_GET(PSW_C) || PSW_GET(PSW_Z)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01020: /* BVC */
				case 01021:
				case 01022:
				case 01023:
					if(!PSW_GET(PSW_V)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01024: /* BVS */
				case 01025:
				case 01026:
				case 01027:
					if(PSW_GET(PSW_V)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01030: /* BCC */
				case 01031:
				case 01032:
				case 01033:
					if(!PSW_GET(PSW_C)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01034: /* BCS */
				case 01035:
				case 01036:
				case 01037:
					if(PSW_GET(PSW_C)) {
						kd11->r[7] += (s16) ((s8) insnbr->offset) * 2;
					}
					break;
				case 01040: /* EMT */
				case 01041:
				case 01042:
				case 01043:
					TRCTrap(030, TRC_TRAP);
					TRAP(030);
					break;
				case 01044: /* TRAP */
				case 01045:
				case 01046:
				case 01047:
					TRCTrap(034, TRC_TRAP);
					TRAP(034);
					break;
				case 01050: /* CLRB */
					CPUWRITEB(insn1->rn, insn1->mode, 0);
					PSW_CLR(PSW_N | PSW_V | PSW_C);
					PSW_SET(PSW_Z);
					break;
				case 01051: /* COMB */
					tmp = CPUREADNB(insn1->rn, insn1->mode);
					tmp = ~tmp;
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_CLR(PSW_V);
					PSW_SET(PSW_C);
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !((u8) tmp));
					break;
				case 01052: /* INCB */
					src = CPUREADNB(insn1->rn, insn1->mode);
					tmp = (u8) (src + 1);
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 000177)
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 01053: /* DECB */
					src = CPUREADNB(insn1->rn, insn1->mode);
					tmp = (u8) (src - 1);
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0000200)
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 01054: /* NEGB */
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
				case 01055: /* ADCB */
					src = CPUREADNB(insn1->rn, insn1->mode);
					tmp2 = PSW_GET(PSW_C) ? 1 : 0;
					tmp = (u8) (src + tmp2);
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0177 && PSW_GET(PSW_C));
					PSW_EQ(PSW_C, src == 0377 && PSW_GET(PSW_C));
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 01056: /* SBCB */
					src = CPUREADNB(insn1->rn, insn1->mode);
					tmp2 = PSW_GET(PSW_C) ? 1 : 0;
					tmp = (u8) (src - tmp2);
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_V, src == 0200);
					PSW_EQ(PSW_C, !src && PSW_GET(PSW_C));
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 01057: /* TSTB */
					tmp = CPUREADB(insn1->rn, insn1->mode);
					PSW_CLR(PSW_V);
					PSW_CLR(PSW_C);
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					break;
				case 01060: /* RORB */
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
				case 01061: /* ROLB */
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
				case 01062: /* ASRB */
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
				case 01063: /* ASLB */
					src = CPUREADNB(insn1->rn, insn1->mode);
					tmp = (u8) (src << 1);
					CPUWRITEB(insn1->rn, insn1->mode, tmp);
					PSW_EQ(PSW_C, src & 0x80);
					PSW_EQ(PSW_N, tmp & 0x80);
					PSW_EQ(PSW_Z, !tmp);
					PSW_EQ(PSW_V, PSW_GET(PSW_N) ^ PSW_GET(PSW_C));
					break;
				case 01064: /* MTPS */
					tmp = CPUREADB(insn1->rn, insn1->mode);
					kd11->psw = (kd11->psw & PSW_T) | (tmp & ~PSW_T);
					break;
				case 01067: /* MFPS */
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
				default:
					/* unused */
					TRCTrap(010, TRC_TRAP_ILL);
					TRAP(010);
					break;
			}
			break;
		case 011: /* MOVB */
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
		case 012: /* CMPB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADB(insn2->dst_rn, insn2->dst_mode);
			tmp = (u8) (src - dst);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_EQ(PSW_V, ((src & 0x80) != (dst & 0x80)) \
					&& ((dst & 0x80) == (tmp & 0x80)));
			PSW_EQ(PSW_C, (src - dst) & 0x100);
			break;
		case 013: /* BITB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADB(insn2->dst_rn, insn2->dst_mode);
			tmp = src & dst;
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 014: /* BICB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNB(insn2->dst_rn, insn2->dst_mode);
			tmp = (u8) (~src & dst);
			CPUWRITEB(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 015: /* BISB */
			src = CPUREADB(insn2->src_rn, insn2->src_mode);
			dst = CPUREADNB(insn2->dst_rn, insn2->dst_mode);
			tmp = src | dst;
			CPUWRITEB(insn2->dst_rn, insn2->dst_mode, tmp);
			PSW_EQ(PSW_N, tmp & 0x80);
			PSW_EQ(PSW_Z, !tmp);
			PSW_CLR(PSW_V);
			break;
		case 016: /* SUB */
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
		default: /* unused */
			TRCTrap(010, TRC_TRAP_ILL);
			TRAP(010);
			break;
	}
}

static int KD11HasPendingIRQ(KD11* kd11, QBUS* bus)
{
	return  (kd11->trap || bus->nxm || (bus->trap && !PSW_GET(PSW_PRIO)))
		&& (kd11->state != STATE_HALT);
}

void KD11HandleTraps(KD11* kd11, QBUS* bus)
{
	u16 trap = kd11->trap;

	/* ignore traps if in HALT mode */
	if(kd11->state == STATE_HALT) {
		return;
	}

	if(bus->nxm) {
		bus->nxm = 0;
		trap = 004;
	} else if(!PSW_GET(PSW_PRIO)) {
		/* internal traps have priority */
		if(kd11->trap) {
			trap = kd11->trap;
			kd11->trap = 0;
		} else {
			trap = bus->trap;
			bus->trap = 0;
		}
	} else {
		trap = kd11->trap;
		kd11->trap = 0;
	}

	if(!trap) {
		return;
	}

	TRCCPUEvent(TRC_CPU_TRAP, trap);

	kd11->r[6] -= 2;
	WRITE(kd11->r[6], kd11->psw);
	if(bus->nxm) {
		TRCCPUEvent(TRC_CPU_DBLBUS, kd11->r[6]);
		bus->nxm = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->r[6] -= 2;
	WRITE(kd11->r[6], kd11->r[7]);
	if(bus->nxm) {
		TRCCPUEvent(TRC_CPU_DBLBUS, kd11->r[6]);
		bus->nxm = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->r[7] = READ(trap);
	if(bus->nxm) {
		TRCCPUEvent(TRC_CPU_DBLBUS, trap);
		bus->nxm = 0;
		kd11->state = STATE_HALT;
		return;
	}

	kd11->psw = READ(trap + 2);
	if(bus->nxm) {
		TRCCPUEvent(TRC_CPU_DBLBUS, trap + 2);
		bus->nxm = 0;
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
			if(KD11HasPendingIRQ(kd11, bus)) {
				KD11HandleTraps(kd11, bus);
				if(kd11->state == STATE_HALT) {
					kd11->odt.state = ODT_STATE_INIT;
				}
				return;
			}

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
