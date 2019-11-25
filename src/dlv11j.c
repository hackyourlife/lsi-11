#include <string.h>
#include <stdlib.h>

#include "lsi11.h"
#include "trace.h"

#define	RCSR_READER_ENABLE	_BV(0)
#define	RCSR_RCVR_INT		_BV(6)
#define	RCSR_RCVR_DONE		_BV(7)

#define	RBUF_ERROR		_BV(15)
#define	RBUF_OVERRUN		_BV(14)
#define	RBUF_FRAMING_ERROR	_BV(13)
#define	RBUF_PARITY_ERROR	_BV(12)
#define	RBUF_ERROR_MASK		(RBUF_OVERRUN | RBUF_FRAMING_ERROR \
		| RBUF_PARITY_ERROR)

#define	XCSR_TRANSMIT_READY	_BV(7)
#define	XCSR_TRANSMIT_INT	_BV(6)
#define	XCSR_TRANSMIT_BREAK	_BV(0)

#define	RCSR_WR_MASK		(RCSR_RCVR_INT | RCSR_READER_ENABLE)
#define	XCSR_WR_MASK		(XCSR_TRANSMIT_INT | XCSR_TRANSMIT_BREAK)

void DLV11JReadChannel(DLV11J* dlv, int channel)
{
	DLV11Ch* ch = &dlv->channel[channel];
	if(ch->buf_size > 0) {
		ch->rbuf = (u8) ch->buf[ch->buf_r++];
		ch->buf_r %= DLV11J_BUF;
		ch->buf_size--;

		if(ch->buf_size) {
			/* more date in the RX buffer... */
			ch->rcsr |= RCSR_RCVR_DONE;
			if(ch->rcsr & RCSR_RCVR_INT) {
				QBUS* bus = dlv->module.bus;
				bus->interrupt(bus, ch->vector);
			}
		} else {
			ch->rcsr &= ~RCSR_RCVR_DONE;
		}
	} else {
		ch->rbuf = RBUF_OVERRUN;
		if(ch->rbuf & RBUF_ERROR_MASK) {
			ch->rbuf |= RBUF_ERROR;
		}
	}
}

void DLV11JWriteChannel(DLV11J* dlv, int channel)
{
	DLV11Ch* ch = &dlv->channel[channel];
	TRCDLV11(TRC_DLV11_TX, channel, ch->xbuf);
	if(ch->receive) {
		ch->receive((unsigned char) ch->xbuf);
	}
	ch->xcsr |= XCSR_TRANSMIT_READY;
	if(ch->xcsr & XCSR_TRANSMIT_INT) {
		QBUS* bus = dlv->module.bus;
		bus->interrupt(bus, ch->vector + 4);
	}
}

u16 DLV11JRead(void* self, u16 address)
{
	DLV11J* dlv = (DLV11J*) self;

	switch(address) {
		case 0177560:
			return dlv->channel[3].rcsr;
		case 0177562:
			DLV11JReadChannel(dlv, 3);
			return dlv->channel[3].rbuf;
		case 0177564:
			return dlv->channel[3].xcsr;
		case 0177566:
			return dlv->channel[3].xbuf;
		default:
			return 0;
	}
}

void DLV11JWriteRCSR(DLV11J* dlv, int n, u16 value)
{
	DLV11Ch* ch = &dlv->channel[n];
	u16 old = ch->rcsr;
	ch->rcsr = (ch->rcsr & ~RCSR_WR_MASK) | (value & RCSR_WR_MASK);
	if((value & RCSR_RCVR_INT) && !(old & RCSR_RCVR_INT)
			&& (ch->rcsr & RCSR_RCVR_DONE)) {
		QBUS* bus = dlv->module.bus;
		bus->interrupt(bus, ch->vector);
	}
}

void DLV11JWriteXCSR(DLV11J* dlv, int n, u16 value)
{
	DLV11Ch* ch = &dlv->channel[n];
	u16 old = ch->xcsr;
	ch->xcsr = (ch->xcsr & ~XCSR_WR_MASK) | (value & XCSR_WR_MASK);
	if((value & XCSR_TRANSMIT_INT) && !(old & XCSR_TRANSMIT_INT)
			&& (ch->xcsr & XCSR_TRANSMIT_READY)) {
		QBUS* bus = dlv->module.bus;
		bus->interrupt(bus, ch->vector + 4);
	}
}

void DLV11JWrite(void* self, u16 address, u16 value)
{
	DLV11J* dlv = (DLV11J*) self;

	switch(address) {
		case 0177560:
			DLV11JWriteRCSR(dlv, 3, value);
			break;
		case 0177562:
			/* ignored */
			break;
		case 0177564:
			DLV11JWriteXCSR(dlv, 3, value);
			break;
		case 0177566:
			dlv->channel[3].xbuf = value;
			DLV11JWriteChannel(dlv, 3);
			break;
	}
}

u8 DLV11JResponsible(void* self, u16 address)
{
	DLV11J* dlv = (DLV11J*) self;

	if(address >= dlv->base && address <= dlv->base + (3 * 8)) {
		return 1;
	}

	/* console device */
	if(address >= 0177560 && address <= 0177566) {
		return 1;
	}

	return 0;
}

void DLV11JReset(void* self)
{
	u8 i;

	DLV11J* dlv = (DLV11J*) self;

	for(i = 0; i < 4; i++) {
		dlv->channel[i].rcsr &= ~RCSR_RCVR_INT;
		dlv->channel[i].xcsr = XCSR_TRANSMIT_READY;
	}
}

void DLV11JInit(DLV11J* dlv)
{
	int i;

	dlv->module.self = (void*) dlv;
	dlv->module.read = DLV11JRead;
	dlv->module.write = DLV11JWrite;
	dlv->module.responsible = DLV11JResponsible;
	dlv->module.reset = DLV11JReset;

	/* factory configuration */
	dlv->base = 0176500;

	memset(dlv->channel, 0, sizeof(dlv->channel));

	for(i = 0; i < 4; i++) {
		dlv->channel[i].buf = (u8*) malloc(DLV11J_BUF);
		dlv->channel[i].buf_r = 0;
		dlv->channel[i].buf_w = 0;
		dlv->channel[i].buf_size = 0;
		dlv->channel[i].base = dlv->base + 8 * i;
		dlv->channel[i].vector = 300 + 8 * i;
	}

	dlv->channel[3].base = 0177560;
	dlv->channel[3].vector = 060;

	DLV11JReset(dlv);
}

void DLV11JDestroy(DLV11J* dlv)
{
	u8 i;

	for(i = 0; i < 4; i++) {
		free(dlv->channel[i].buf);
	}
}

void DLV11JSend(DLV11J* dlv, int channel, unsigned char c)
{
	DLV11Ch* ch = &dlv->channel[channel];
	if(ch->buf_size < DLV11J_BUF) {
		TRCDLV11(TRC_DLV11_RX, channel, c);
		ch->buf[ch->buf_w++] = c;
		ch->buf_w %= DLV11J_BUF;
		ch->buf_size++;
		ch->rcsr |= RCSR_RCVR_DONE;
		if(ch->rcsr & RCSR_RCVR_INT) {
			QBUS* bus = dlv->module.bus;
			bus->interrupt(bus, ch->vector);
		}
	}
}
