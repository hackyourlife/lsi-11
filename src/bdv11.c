#include <string.h>
#include <stdlib.h>

#include "lsi11.h"
#include "trace.h"

#define	BDV11_SWITCH	(BDV11_CPU_TEST | BDV11_MEM_TEST \
		| BDV11_DIALOG | BDV11_RX02)

extern const u16 bdv11_e53[2048];

u16 BDV11GetWordLow(BDV11* bdv, u16 word)
{
	u16 page = bdv->pcr & 0xFF;
	if(page < 0x10) {
		u16 romword = page * 0200 + word;
		return bdv11_e53[romword];
	} else {
		return 0177777;
	}
}

u16 BDV11GetWordHigh(BDV11* bdv, u16 word)
{
	u16 page = (bdv->pcr >> 8) & 0xFF;
	if(page < 0x10) {
		u16 romword = page * 0200 + word;
		return bdv11_e53[romword];
	} else {
		return 0233;
	}
}

void BDV11MemoryDump(BDV11* bdv, u16 pcr, int hi)
{
	const u16* data;
	u16 addr;
	u16 len = 0400;
	if(hi) {
		addr = 0173400;
		u16 page = (pcr >> 8) & 0xFF;
		if(page < 0x10) {
			u16 romword = page * 0200;
			data = &bdv11_e53[romword];
		} else {
			data = NULL;
		}
	} else {
		addr = 0173000;
		u16 page = pcr & 0xFF;
		if(page < 0x10) {
			u16 romword = page * 0200;
			data = &bdv11_e53[romword];
		} else {
			data = NULL;
		}
	}

	if(data) {
		TRCMemoryDump((u8*) data, addr, len);
	} else {
		u8 buf[0400];
		memset(buf, 0xFF, sizeof(buf));
		TRCMemoryDump((u8*) data, addr, len);
	}
}

u16 BDV11Read(void* self, u16 address)
{
	BDV11* bdv = (BDV11*) self;

	switch(address) {
		case 0177520:
			return bdv->pcr;
		case 0177522:
			return bdv->scratch;
		case 0177524:
			return bdv->sw;
		case 0177546:
			return bdv->ltc;
		default:
			if(address >= 0173000 && address < 0173400) {
				return BDV11GetWordLow(bdv,
						(address - 0173000) / 2);
			} else if(address >= 0173400 && address < 0173776) {
				return BDV11GetWordHigh(bdv,
						(address - 0173400) / 2);
			}
			return 0;
	}
}

u8 BDV11Read8(void* self, u16 address)
{
	if(address & 1) {
		return (u8) (BDV11Read(self, address & 0xFFFE) >> 8);
	} else {
		return (u8) BDV11Read(self, address & 0xFFFE);
	}
}

void BDV11Write(void* self, u16 address, u16 value)
{
	BDV11* bdv = (BDV11*) self;

	switch(address) {
		case 0177520:
			/* record new memory content in trace */
			if(value != bdv->pcr) {
				if((value & 0xFF) == (bdv->pcr & 0xFF)) {
					BDV11MemoryDump(bdv, value, 1);
				} else if((value & 0xFF00) == (bdv->pcr & 0xFF00)) {
					BDV11MemoryDump(bdv, value, 0);
				} else {
					BDV11MemoryDump(bdv, value, 0);
					BDV11MemoryDump(bdv, value, 1);
				}
			}
			bdv->pcr = value;
			break;
		case 0177522:
			bdv->scratch = value;
			break;
		case 0177524:
			bdv->display = value;
			break;
		case 0177546:
			bdv->ltc = value & 0100;
			break;
	}
}

void BDV11Write8(void* self, u16 address, u8 value)
{
	BDV11* bdv = (BDV11*) self;

	u16 tmp;

	switch(address) {
		case 0177520:
			tmp = bdv->pcr;
			bdv->pcr = (bdv->pcr & 0xFF00) | value;
			if(tmp != bdv->pcr) {
				BDV11MemoryDump(bdv, bdv->pcr, 0);
			}
			break;
		case 0177521:
			tmp = bdv->pcr;
			bdv->pcr = (bdv->pcr & 0xFF) | (value << 8);
			if(tmp != bdv->pcr) {
				BDV11MemoryDump(bdv, bdv->pcr, 1);
			}
			break;
		case 0177522:
			bdv->scratch = (bdv->scratch & 0xFF00) | value;
			break;
		case 0177523:
			bdv->scratch = (bdv->scratch & 0xFF) | (value << 8);
			break;
		case 0177524:
			bdv->display = (bdv->display & 0xFF00) | value;
			break;
		case 0177525:
			bdv->display = (bdv->display & 0xFF) | (value << 8);
			break;
		case 0177546:
			bdv->ltc = value & 0100;
			break;
	}
}

u8 BDV11Responsible(void* self, u16 address)
{
	switch(address) {
		case 0177520:
		case 0177521:
		case 0177522:
		case 0177523:
		case 0177524:
		case 0177525:
		case 0177546:
		case 0177547:
			return 1;
		default:
			return address >= 0173000 && address <= 0173776;
	}
}

void BDV11Reset(void* self)
{
	BDV11* bdv = (BDV11*) self;

	bdv->pcr = 0;
	bdv->scratch = 0;
	bdv->display = 0;
	bdv->ltc = 0;

	BDV11MemoryDump(bdv, bdv->pcr, 0);
	BDV11MemoryDump(bdv, bdv->pcr, 1);
}

void BDV11Init(BDV11* bdv)
{
	bdv->module.self = (void*) bdv;
	bdv->module.read = BDV11Read;
	bdv->module.write = BDV11Write;
	bdv->module.read8 = BDV11Read8;
	bdv->module.write8 = BDV11Write8;
	bdv->module.responsible = BDV11Responsible;
	bdv->module.reset = BDV11Reset;

	bdv->sw = BDV11_SWITCH;
}

void BDV11Destroy(BDV11* bdv)
{
	/* nothing */
}

void BDV11Step(BDV11* bdv, float dt)
{
	if(bdv->ltc & 0100) {
		bdv->time += dt;
		if(bdv->time >= LTC_TIME) {
			QBUS* bus = bdv->module.bus;
			bus->interrupt(bus, 0100);
			bdv->time -= LTC_TIME;
			if(bdv->time >= LTC_TIME) {
				bdv->time = 0;
			}
		}
	} else {
		bdv->time = 0;
	}
}

void BDV11SetSwitch(BDV11* bdv, u16 sw)
{
	bdv->sw = sw;
}
