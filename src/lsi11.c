#include <stdlib.h>
#include <string.h>

#include "lsi11.h"
#include "trace.h"

static u16 LSI11ReadDMA(void* user, u16 address, BOOL* nxm)
{
	u8 i;
	u16 addr = address;
	LSI11* lsi = (LSI11*) user;

	address &= 0xFFFE;

	*nxm = FALSE;

	for(i = 0; i < LSI11_SIZE; i++) {
		QBUSMod* module = lsi->backplane[i];
		if(!module || !module->responsible) {
			continue;
		}
		if(module->responsible(module->self, address)) {
			u16 value = module->read(module->self, address);
			TRCBus(TRC_BUS_RD, addr, value);
			return value;
		}
	}

	*nxm = TRUE;
	TRCBus(TRC_BUS_RDFAIL, addr, 0);
	return 0;
}

static u8 LSI11Read8DMA(void* user, u16 address, BOOL* nxm)
{
	u8 i;
	u16 addr = address;
	LSI11* lsi = (LSI11*) user;

	*nxm = FALSE;

	for(i = 0; i < LSI11_SIZE; i++) {
		QBUSMod* module = lsi->backplane[i];
		if(!module || !module->responsible) {
			continue;
		}
		if(module->responsible(module->self, address)) {
			u8 value = module->read8(module->self, address);
			TRCBus(TRC_BUS_RD8, addr, value);
			return value;
		}
	}

	*nxm = TRUE;
	TRCBus(TRC_BUS_RD8FAIL, addr, 0);
	return 0;
}

static BOOL LSI11WriteDMA(void* user, u16 address, u16 value)
{
	u8 i;
	LSI11* lsi = (LSI11*) user;

	address &= 0xFFFE;

	for(i = 0; i < LSI11_SIZE; i++) {
		QBUSMod* module = lsi->backplane[i];
		if(!module || !module->responsible) {
			continue;
		}
		if(module->responsible(module->self, address)) {
			TRCBus(TRC_BUS_WR, address, value);
			module->write(module->self, address, value);
			return FALSE;
		}
	}

	TRCBus(TRC_BUS_WRFAIL, address, value);
	return TRUE;
}

static BOOL LSI11Write8DMA(void* user, u16 address, u8 value)
{
	u8 i;
	LSI11* lsi = (LSI11*) user;

	for(i = 0; i < LSI11_SIZE; i++) {
		QBUSMod* module = lsi->backplane[i];
		if(!module || !module->responsible) {
			continue;
		}
		if(module->responsible(module->self, address)) {
			TRCBus(TRC_BUS_WR8, address, value);
			module->write8(module->self, address, value);
			return FALSE;
		}
	}

	TRCBus(TRC_BUS_WR8FAIL, address, value);
	return TRUE;
}

static u16 LSI11Read(void* user, u16 address)
{
	BOOL nxm;
	LSI11* lsi = (LSI11*) user;

	u16 data = LSI11ReadDMA(user, address, &nxm);

	if(nxm) {
		lsi->bus.nxm = 1;
	}

	return data;
}

static u8 LSI11Read8(void* user, u16 address)
{
	BOOL nxm;
	LSI11* lsi = (LSI11*) user;

	u8 data = LSI11Read8DMA(user, address, &nxm);

	if(nxm) {
		lsi->bus.nxm = 1;
	}

	return data;
}

static void LSI11Write(void* user, u16 address, u16 value)
{
	LSI11* lsi = (LSI11*) user;

	if(LSI11WriteDMA(user, address, value)) {
		lsi->bus.nxm = 1;
	}
}

static void LSI11Write8(void* user, u16 address, u8 value)
{
	LSI11* lsi = (LSI11*) user;

	if(LSI11Write8DMA(user, address, value)) {
		lsi->bus.nxm = 1;
	}
}

#define	IRCJITTER()	(rand() % QBUS_DELAY_JITTER)

static int LSI11QBUSInterrupt(QBUS* bus, int n)
{
	if(n != 004) {
		if(bus->trap || bus->irq) {
			TRCIRQ(n, TRC_IRQ_FAIL);
			return 0;
		} else {
			TRCIRQ(n, TRC_IRQ_OK);
			bus->irq = n;
			bus->delay = IRCJITTER();
			return 1;
		}
	} else {
		TRCIRQ(n, TRC_IRQ_OK);
		bus->trap = n;
		return 1;
	}
}

void LSI11QBUSCancelInterrupt(QBUS* bus, int n)
{
	if(bus->irq == n) {
		bus->irq = 0;
	}

	if(bus->trap == n) {
		bus->trap = 0;
	}
}

static void LSI11QBUSReset(QBUS* bus)
{
	u8 i;
	LSI11* lsi = (LSI11*) bus->user;

	TRCBus(TRC_BUS_RESET, 0, 0);

	/* clear pending interrupts */
	bus->trap = 0;
	bus->irq = 0;
	bus->delay = 0;
	bus->nxm = 0;

	for(i = 0; i < LSI11_SIZE; i++) {
		QBUSMod* module = lsi->backplane[i];
		if(!module || !module->responsible) {
			continue;
		}
		module->reset(module->self);
	}
}

static void LSI11QBUSStep(QBUS* bus)
{
	if(bus->delay >= QBUS_DELAY) {
		/* wait until last trap was serviced */
		if(!bus->trap) {
			TRCIRQ(bus->irq, TRC_IRQ_SIG);
			bus->trap = bus->irq;
			bus->irq = 0;
			bus->delay = 0;
		}
	} else if(bus->irq) {
		bus->delay++;
	}
}

void LSI11Init(LSI11* lsi)
{
	KD11Init(&lsi->cpu);
	lsi->bus.trap = 0;
	lsi->bus.nxm = 0;
	lsi->bus.user = (void*) lsi;
	lsi->bus.read = LSI11Read;
	lsi->bus.write = LSI11Write;
	lsi->bus.read8 = LSI11Read8;
	lsi->bus.write8 = LSI11Write8;
	lsi->bus.readDMA = LSI11ReadDMA;
	lsi->bus.writeDMA = LSI11WriteDMA;
	lsi->bus.read8DMA = LSI11Read8DMA;
	lsi->bus.write8DMA = LSI11Write8DMA;
	lsi->bus.interrupt = LSI11QBUSInterrupt;
	lsi->bus.reset = LSI11QBUSReset;

	memset(lsi->backplane, 0, sizeof(lsi->backplane));
}

void LSI11Destroy(LSI11* lsi)
{
	/* nothing */
}

void LSI11InstallModule(LSI11* lsi, int slot, QBUSMod* module)
{
	lsi->backplane[slot] = module;
	module->bus = &lsi->bus;
}

void LSI11Reset(LSI11* lsi)
{
	KD11Reset(&lsi->cpu);
	lsi->bus.reset(&lsi->bus);
}

void LSI11Step(LSI11* lsi)
{
	LSI11QBUSStep(&lsi->bus);
	KD11Step(&lsi->cpu, &lsi->bus);
}
