#include <string.h>

#include "lsi11.h"

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
	u16	offset:6;
	u16	rn:3;
	u16	opcode:7;
} KD11INSNSOB;

#define	WRITE(s)	pos = LSI11Write(buf, s, pos)
#define	WRITEN(n)	pos = LSI11WriteN(buf, n, pos)
#define	WRITEC(c)	buf[pos++] = c, buf[pos] = 0

int LSI11Write(char* buf, char* str, int pos)
{
	int len = strlen(str);
	memcpy(&buf[pos], str, len);
	buf[pos + len] = 0;
	return pos + len;
}

int LSI11WriteN(char* buf, u16 val, int pos)
{
	int i;
	int start = pos;
	buf[pos] = ((val >> 15) & 0x7) + '0';
	if(buf[pos] != '0') {
		pos++;
	}
	for(i = 0; i < 5; i++) {
		buf[pos] = ((val >> 12) & 7) + '0';
		val <<= 3;
		if(pos != start || buf[pos] != '0') {
			pos++;
		}
	}
	if(pos == start) {
		buf[pos++] = '0';
	}
	buf[pos] = 0;
	return pos;
}

const u16* LSI11DisassemblePCOperand(u8 rn, u8 mode, const u16* x, u16* pc, char* buf, int* p)
{
	int pos = *p;
	switch(mode) {
		case 2:
			*pc += 2;
			WRITEC('#');
			WRITEN(*(x++));
			break;
		case 3:
			*pc += 2;
			WRITEC('@');
			WRITEC('#');
			WRITEN(*(x++));
			break;
		case 6:
			*pc += 2;
			WRITEN(*pc + *(x++));
			break;
		case 7:
			*pc += 2;
			WRITEC('@');
			WRITEN(*pc + *(x++));
			break;
	}
	*p = pos;
	return x;
}

#define	WRITERN(rn) { \
	if(rn == 6) { \
		WRITE("SP"); \
	} else if(rn == 7) { \
		WRITE("PC"); \
	} else { \
		WRITEC('R'); \
		WRITEC(rn + '0'); \
	} \
}

const u16* LSI11DisassembleOperand(u8 rn, u8 mode, const u16* x, u16* pc, char* buf, int* p)
{
	int pos = *p;
	if(rn == 7 && ((mode & 6) == 2 || (mode & 6) == 6)) {
		return LSI11DisassemblePCOperand(rn, mode, x, pc, buf, p);
	}
	switch(mode) {
		case 0:
			WRITERN(rn);
			break;
		case 1:
			WRITEC('(');
			WRITERN(rn);
			WRITEC(')');
			break;
		case 3:
			WRITEC('@');
		case 2:
			WRITEC('(');
			WRITERN(rn);
			WRITEC(')');
			WRITEC('+');
			break;
		case 5:
			WRITEC('@');
		case 4:
			WRITEC('-');
			WRITEC('(');
			WRITERN(rn);
			WRITEC(')');
			break;
		case 7:
			WRITEC('@');
		case 6:
			*pc += 2;
			WRITEN(*(x++));
			WRITEC('(');
			WRITERN(rn);
			WRITEC(')');
			break;
	}
	*p = pos;
	return x;
}

int LSI11DisassembleBranch(s8 offset, u16 pc, char* buf, int pos)
{
	s16 off = offset * 2;
	if(pc == 0xFFFF) {
		WRITEC('.');
		if(offset >= 0) {
			WRITEC('+');
			WRITEN(off);
		} else {
			WRITEC('-');
			WRITEN(-off);
		}
	} else {
		WRITEC('L');
		WRITEN(pc + off);
	}
	return pos;
}

#define OP1()	insn = LSI11DisassembleOperand(insn1->rn, insn1->mode, \
		insn, &pc, buf, &pos)
#define OP2()	insn = LSI11DisassembleOperand(insn2->src_rn, insn2->src_mode, \
		insn, &pc, buf, &pos), \
		WRITEC(','), \
		insn = LSI11DisassembleOperand(insn2->dst_rn, insn2->dst_mode, \
		insn, &pc, buf, &pos)
#define BR()	pos = LSI11DisassembleBranch((s8) insnbr->offset, pc, buf, pos)
#define	RET0()	return (int) (insn - start);
#define	RET1()	return (int) (insn - start);
#define	RET2()	return (int) (insn - start);

int LSI11Disassemble(const u16* insn, u16 pc, char* buf)
{
	int pos = 0;
	u16 opcd = *insn;
	const u16* start = insn;
	KD11INSN1* insn1 = (KD11INSN1*) insn;
	KD11INSN2* insn2 = (KD11INSN2*) insn;
	KD11INSNBR* insnbr = (KD11INSNBR*) insn;
	KD11INSNJSR* insnjsr = (KD11INSNJSR*) insn;
	KD11INSNRTS* insnrts = (KD11INSNRTS*) insn;
	KD11INSNSOB* insnsob = (KD11INSNSOB*) insn;
	insn++;

	pc += 2;

	switch(opcd & 0177700) {
		case 0005000: /* CLR */
			WRITE("CLR\t");
			OP1();
			RET1();
		case 0105000: /* CLRB */
			WRITE("CLRB\t");
			OP1();
			RET1();
		case 0005100: /* COM */
			WRITE("COM\t");
			OP1();
			RET1();
		case 0105100: /* COMB */
			WRITE("COMB\t");
			OP1();
			RET1();
		case 0005200: /* INC */
			WRITE("INC\t");
			OP1();
			RET1();
		case 0105200: /* INCB */
			WRITE("INCB\t");
			OP1();
			RET1();
		case 0005300: /* DEC */
			WRITE("DEC\t");
			OP1();
			RET1();
		case 0105300: /* DECB */
			WRITE("DECB\t");
			OP1();
			RET1();
		case 0005400: /* NEG */
			WRITE("NEG\t");
			OP1();
			RET1();
		case 0105400: /* NEGB */
			WRITE("NEGB\t");
			OP1();
			RET1();
		case 0005700: /* TST */
			WRITE("TST\t");
			OP1();
			RET1();
		case 0105700: /* TSTB */
			WRITE("TSTB\t");
			OP1();
			RET1();
		case 0006200: /* ASR */
			WRITE("ASR\t");
			OP1();
			RET1();
		case 0106200: /* ASRB */
			WRITE("ASRB\t");
			OP1();
			RET1();
		case 0006300: /* ASL */
			WRITE("ASL\t");
			OP1();
			RET1();
		case 0106300: /* ASLB */
			WRITE("ASLB\t");
			OP1();
			RET1();
		case 0006000: /* ROR */
			WRITE("ROR\t");
			OP1();
			RET1();
		case 0106000: /* RORB */
			WRITE("RORB\t");
			OP1();
			RET1();
		case 0006100: /* ROL */
			WRITE("ROL\t");
			OP1();
			RET1();
		case 0106100: /* ROLB */
			WRITE("ROLB\t");
			OP1();
			RET1();
		case 0000300: /* SWAB */
			WRITE("SWAB\t");
			OP1();
			RET1();
		case 0005500: /* ADC */
			WRITE("ADC\t");
			OP1();
			RET1();
		case 0105500: /* ADCB */
			WRITE("ADCB\t");
			OP1();
			RET1();
		case 0005600: /* SBC */
			WRITE("SBC\t");
			OP1();
			RET1();
		case 0105600: /* SBCB */
			WRITE("SBCB\t");
			OP1();
			RET1();
		case 0006700: /* SXT */
			WRITE("SXT\t");
			OP1();
			RET1();
		case 0106700: /* MFPS */
			WRITE("MFPS\t");
			OP1();
			RET1();
		case 0106400: /* MTPS */
			WRITE("MTPS\t");
			OP1();
			RET1();
		case 0000100: /* JMP */
			WRITE("JMP\t");
			OP1();
			RET1();
		case 0006400: /* MARK */
			WRITE("MARK\t");
			WRITEN(opcd & 077);
			RET0();
	}

	switch(opcd & 0170000) {
		case 0010000: /* MOV */
			WRITE("MOV\t");
			OP2();
			RET2();
		case 0110000: /* MOVB */
			WRITE("MOVB\t");
			OP2();
			RET2();
		case 0020000: /* CMP */
			WRITE("CMP\t");
			OP2();
			RET2();
		case 0120000: /* CMPB */
			WRITE("CMPB\t");
			OP2();
			RET2();
		case 0060000: /* ADD */
			WRITE("ADD\t");
			OP2();
			RET2();
		case 0160000: /* SUB */
			WRITE("SUB\t");
			OP2();
			RET2();
		case 0030000: /* BIT */
			WRITE("BIT\t");
			OP2();
			RET2();
		case 0130000: /* BITB */
			WRITE("BITB\t");
			OP2();
			RET2();
		case 0040000: /* BIC */
			WRITE("BIC\t");
			OP2();
			RET2();
		case 0140000: /* BICB */
			WRITE("BICB\t");
			OP2();
			RET2();
		case 0050000: /* BIS */
			WRITE("BIS\t");
			OP2();
			RET2();
		case 0150000: /* BISB */
			WRITE("BISB\t");
			OP2();
			RET2();
	}

	switch(opcd & 0177000) {
		case 0074000: /* XOR */
			WRITE("XOR\t");
			WRITERN(insnjsr->r);
			WRITEC(',');
			OP1();
			RET1();
		case 0004000: /* JSR */
			WRITE("JSR\t");
			WRITERN(insnjsr->r);
			WRITEC(',');
			OP1();
			RET1();
		case 0077000: /* SOB */
			WRITE("SOB\t");
			WRITERN(insnjsr->r);
			WRITEC(',');
			WRITEC('L');
			WRITEN(pc - insnsob->offset * 2);
			RET1();
		case 0070000: /* MUL */
			WRITE("MUL\t");
			OP1();
			WRITEC(',');
			WRITERN(insnjsr->r);
			RET1();
		case 0071000: /* DIV */
			WRITE("DIV\t");
			OP1();
			WRITEC(',');
			WRITERN(insnjsr->r);
			RET1();
		case 0072000: /* ASH */
			WRITE("ASH\t");
			OP1();
			WRITEC(',');
			WRITERN(insnjsr->r);
			RET1();
		case 0073000: /* ASHC */
			WRITE("ASHC\t");
			OP1();
			WRITEC(',');
			WRITERN(insnjsr->r);
			RET1();
	}

	switch(opcd & 0177770) {
		case 0000200: /* RTS */
			WRITE("RTS\t");
			WRITERN(insnrts->rn);
			RET0();
		case 0075000: /* FADD */
			WRITE("FADD\t");
			WRITERN(insnrts->rn);
			RET0();
		case 0075010: /* FSUB */
			WRITE("FSUB\t");
			WRITERN(insnrts->rn);
			RET0();
		case 0075020: /* FMUL */
			WRITE("FMUL\t");
			WRITERN(insnrts->rn);
			RET0();
		case 0075030: /* FDIV */
			WRITE("FDIV\t");
			WRITERN(insnrts->rn);
			RET0();
	}

	switch(opcd & 0177400) {
		case 0000400: /* BR */
			WRITE("BR\t");
			BR();
			RET0();
		case 0001000: /* BNE */
			WRITE("BNE\t");
			BR();
			RET0();
		case 0001400: /* BEQ */
			WRITE("BEQ\t");
			BR();
			RET0();
		case 0100000: /* BPL */
			WRITE("BPL\t");
			BR();
			RET0();
		case 0100400: /* BMI */
			WRITE("BMI\t");
			BR();
			RET0();
		case 0102000: /* BVC */
			WRITE("BVC\t");
			BR();
			RET0();
		case 0102400: /* BVS */
			WRITE("BVS\t");
			BR();
			RET0();
		case 0103000: /* BCC */
			WRITE("BCC\t");
			BR();
			RET0();
		case 0103400: /* BCS */
			WRITE("BCS\t");
			BR();
			RET0();
		case 0002000: /* BGE */
			WRITE("BGE\t");
			BR();
			RET0();
		case 0002400: /* BLT */
			WRITE("BLT\t");
			BR();
			RET0();
		case 0003000: /* BGT */
			WRITE("BGT\t");
			BR();
			RET0();
		case 0003400: /* BLE */
			WRITE("BLE\t");
			BR();
			RET0();
		case 0101000: /* BHI */
			WRITE("BHI\t");
			BR();
			RET0();
		case 0101400: /* BLOS */
			WRITE("BLOS\t");
			BR();
			RET0();
		case 0104000: /* EMT */
			if(opcd & 0377) {
				WRITE("EMT\t");
				WRITEN(opcd & 0377);
			} else {
				WRITE("EMT");
			}
			RET0();
		case 0104400: /* TRAP */
			if(opcd & 0377) {
				WRITE("TRAP\t");
				WRITEN(opcd & 0377);
			} else {
				WRITE("TRAP");
			}
			RET0();
	}

	switch(opcd) {
		case 0000003: /* BPT */
			WRITE("BPT");
			RET0();
		case 0000004: /* IOT */
			WRITE("IOT");
			RET0();
		case 0000002: /* RTI */
			WRITE("RTI");
			RET0();
		case 0000006: /* RTT */
			WRITE("RTT");
			RET0();
		case 0000000: /* HALT */
			WRITE("HALT");
			RET0();
		case 0000001: /* WAIT */
			WRITE("WAIT");
			RET0();
		case 0000005: /* RESET */
			WRITE("RESET");
			RET0();
		case 0000240: /* NOP */
			WRITE("NOP");
			RET0();
		case 0000241: /* CLC */
			WRITE("CLC");
			RET0();
		case 0000242: /* CLV */
			WRITE("CLV");
			RET0();
		case 0000243: /* CLVC */
			WRITE("CLVC");
			RET0();
		case 0000244: /* CLZ */
			WRITE("CLZ");
			RET0();
		case 0000250: /* CLN */
			WRITE("CLN");
			RET0();
		case 0000257: /* CCC */
			WRITE("CCC");
			RET0();
		case 0000260: /* NOP1 */
			WRITE("NOP1");
			RET0();
		case 0000261: /* SEC */
			WRITE("SEC");
			RET0();
		case 0000262: /* SEV */
			WRITE("SEV");
			RET0();
		case 0000263: /* SEVC */
			WRITE("SEVC");
			RET0();
		case 0000264: /* SEZ */
			WRITE("SEZ");
			RET0();
		case 0000270: /* SEN */
			WRITE("SEN");
			RET0();
		case 0000277: /* SCC */
			WRITE("SCC");
			RET0();
	}

	WRITE("; unknown [");
	WRITEN(opcd);
	WRITEC(']');
	return 1;
}

#define OP1LEN()	LSI11OperandLength(insn1->rn, insn1->mode) + 1
#define OP2LEN()	LSI11OperandLength(insn2->src_rn, insn2->src_mode) + \
			LSI11OperandLength(insn2->dst_rn, insn2->dst_mode) + 1

int LSI11OperandLength(const u8 rn, const u8 mode)
{
	if(rn == 7 && ((mode & 6) == 2 || (mode & 6) == 6)) {
		return 1;
	}
	switch(mode) {
		default:
		case 0:
		case 1:
		case 3:
		case 2:
		case 5:
		case 4:
			return 0;
		case 6:
		case 7:
			return 1;
	}
}

int LSI11InstructionLength(const u16* insn)
{
	u16 opcd = *insn;
	KD11INSN1* insn1 = (KD11INSN1*) insn;
	KD11INSN2* insn2 = (KD11INSN2*) insn;

	switch(opcd & 0177700) {
		case 0005000: /* CLR */
		case 0105000: /* CLRB */
		case 0005100: /* COM */
		case 0105100: /* COMB */
		case 0005200: /* INC */
		case 0105200: /* INCB */
		case 0005300: /* DEC */
		case 0105300: /* DECB */
		case 0005400: /* NEG */
		case 0105400: /* NEGB */
		case 0005700: /* TST */
		case 0105700: /* TSTB */
		case 0006200: /* ASR */
		case 0106200: /* ASRB */
		case 0006300: /* ASL */
		case 0106300: /* ASLB */
		case 0006000: /* ROR */
		case 0106000: /* RORB */
		case 0006100: /* ROL */
		case 0106100: /* ROLB */
		case 0000300: /* SWAB */
		case 0005500: /* ADC */
		case 0105500: /* ADCB */
		case 0005600: /* SBC */
		case 0105600: /* SBCB */
		case 0006700: /* SXT */
		case 0106700: /* MFPS */
		case 0106400: /* MTPS */
		case 0000100: /* JMP */
			return OP1LEN();
		case 0006400: /* MARK */
			return 1;
	}

	switch(opcd & 0170000) {
		case 0010000: /* MOV */
		case 0110000: /* MOVB */
		case 0020000: /* CMP */
		case 0120000: /* CMPB */
		case 0060000: /* ADD */
		case 0160000: /* SUB */
		case 0030000: /* BIT */
		case 0130000: /* BITB */
		case 0040000: /* BIC */
		case 0140000: /* BICB */
		case 0050000: /* BIS */
		case 0150000: /* BISB */
			return OP2LEN();
	}

	switch(opcd & 0177000) {
		case 0074000: /* XOR */
		case 0004000: /* JSR */
			return OP1LEN();
		case 0077000: /* SOB */
			return 1;
		case 0070000: /* MUL */
		case 0071000: /* DIV */
		case 0072000: /* ASH */
		case 0073000: /* ASHC */
			return OP1LEN();
	}

	switch(opcd & 0177770) {
		case 0000200: /* RTS */
		case 0075000: /* FADD */
		case 0075010: /* FSUB */
		case 0075020: /* FMUL */
		case 0075030: /* FDIV */
			return 1;
	}

	switch(opcd & 0177400) {
		case 0000400: /* BR */
		case 0001000: /* BNE */
		case 0001400: /* BEQ */
		case 0100000: /* BPL */
		case 0100400: /* BMI */
		case 0102000: /* BVC */
		case 0102400: /* BVS */
		case 0103000: /* BCC */
		case 0103400: /* BCS */
		case 0002000: /* BGE */
		case 0002400: /* BLT */
		case 0003000: /* BGT */
		case 0003400: /* BLE */
		case 0101000: /* BHI */
		case 0101400: /* BLOS */
		case 0104000: /* EMT */
		case 0104400: /* TRAP */
			return 1;
	}

	switch(opcd) {
		case 0000003: /* BPT */
		case 0000004: /* IOT */
		case 0000002: /* RTI */
		case 0000006: /* RTT */
		case 0000000: /* HALT */
		case 0000001: /* WAIT */
		case 0000005: /* RESET */
		case 0000240: /* NOP */
		case 0000241: /* CLC */
		case 0000242: /* CLV */
		case 0000243: /* CLVC */
		case 0000244: /* CLZ */
		case 0000250: /* CLN */
		case 0000257: /* CCC */
		case 0000260: /* NOP1 */
		case 0000261: /* SEC */
		case 0000262: /* SEV */
		case 0000263: /* SEVC */
		case 0000264: /* SEZ */
		case 0000270: /* SEN */
		case 0000277: /* SCC */
			return 1;
	}

	return 1;
}
