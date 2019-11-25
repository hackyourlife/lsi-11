#include <stdlib.h>
#include <string.h>

#include "lsi11.h"
#include "trace.h"

static u16 LSI11Read(void* user, u16 address)
{
	u8 i;
	u16 addr = address;
	LSI11* lsi = (LSI11*) user;

	address &= 0xFFFE;

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

	TRCBus(TRC_BUS_RDFAIL, addr, 0);
	lsi->bus.interrupt(&lsi->bus, 004);
	return 0;
}

static void LSI11Write(void* user, u16 address, u16 value)
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
			return;
		}
	}

	TRCBus(TRC_BUS_WRFAIL, address, value);
	lsi->bus.interrupt(&lsi->bus, 004);
}

#define	IRCJITTER()	(rand() % QBUS_DELAY_JITTER)

static int LSI11QBUSInterrupt(QBUS* bus, int n)
{
	if(n != 004) {
		if(bus->trap) {
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

static void LSI11QBUSReset(QBUS* bus)
{
	u8 i;
	LSI11* lsi = (LSI11*) bus->user;

	TRCBus(TRC_BUS_RESET, 0, 0);

	/* clear pending interrupts */
	bus->trap = 0;
	bus->irq = 0;
	bus->delay = 0;

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
	lsi->bus.user = (void*) lsi;
	lsi->bus.read = LSI11Read;
	lsi->bus.write = LSI11Write;
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
