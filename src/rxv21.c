#include <stdlib.h>
#include <string.h>

#include "lsi11.h"
#include "trace.h"

/* RX2CS bits */
#define	RX_GO			_BV(0)
#define	RX_FUNCTION_MASK	(_BV(1) | _BV(2) | _BV(3))
#define	RX_UNIT_SEL		_BV(4)
#define	RX_DONE			_BV(5)
#define	RX_INTR_ENB		_BV(6)
#define	RX_TR			_BV(7)
#define	RX_DEN			_BV(8)
#define	RX_RX02			_BV(11)
#define	RX_EXT_ADDR_MASK	(_BV(12) | _BV(13))
#define	RX_INIT			_BV(14)
#define	RX_ERROR		_BV(15)

#define	RX_RMASK		(RX_UNIT_SEL | RX_DONE | RX_INTR_ENB \
		| RX_TR | RX_DEN | RX_RX02 | RX_ERROR)
#define	RX_WMASK		(RX_GO | RX_FUNCTION_MASK | RX_UNIT_SEL \
		| RX_INTR_ENB | RX_DEN | RX_EXT_ADDR_MASK | RX_INIT)

/* RX2CS function codes */
#define	RX_FILL_BUFFER		(00 << 1)
#define	RX_EMPTY_BUFFER		(01 << 1)
#define	RX_WRITE_SECTOR		(02 << 1)
#define	RX_READ_SECTOR		(03 << 1)
#define	RX_SET_MEDIA_DENSITY	(04 << 1)
#define	RX_READ_STATUS		(05 << 1)
#define	RX_WRITE_DELETED_DATA	(06 << 1)
#define	RX_READ_ERROR_CODE	(07 << 1)

/* RX2ES bits */
#define	RX2ES_CRC		_BV(0)
#define	RX2ES_ID		_BV(2)
#define	RX2ES_RX_AC_LO		_BV(3)
#define	RX2ES_DEN_ERR		_BV(4)
#define	RX2ES_DRV_DEN		_BV(5)
#define	RX2ES_DD		_BV(6)
#define	RX2ES_DRV_RDY		_BV(7)
#define	RX2ES_UNIT_SEL		_BV(8)
#define	RX2ES_WC_OVFL		_BV(10)
#define	RX2ES_NXM		_BV(11)

#define	RX2ES_DEFAULT		(RX2ES_ID | RX2ES_DRV_DEN | RX2ES_DRV_RDY)
#define	RX2ES_ERRORS		(RX2ES_CRC | RX2ES_RX_AC_LO | RX2ES_DEN_ERR \
		| RX2ES_WC_OVFL | RX2ES_NXM)

#define	READ(addr)		(bus->read(bus->user, (addr)))
#define	WRITE(addr, val)	(bus->write(bus->user, (addr), (val)))
#define	CHECK()			{ if(bus->trap) return; }

void RXV21Reset(void* self);

void RXV21ClearErrors(RXV21* rx)
{
	rx->rx2cs &= ~RX_ERROR;
	rx->rx2es &= ~RX2ES_ERRORS;
}

void RXV21Done(RXV21* rx)
{
	QBUS* bus = rx->module.bus;

	rx->state = 0;
	rx->rx2cs |= RX_DONE;
	rx->rx2es |= RX2ES_DRV_DEN;
	rx->rx2db = rx->rx2es;
	if(rx->rx2cs & RX_INTR_ENB) {
		bus->interrupt(bus, rx->vector);
	}
}

extern int trace;
void RXV21FillBuffer(RXV21* rx)
{
	QBUS* bus = rx->module.bus;
	u16 limit = (rx->rx2cs & RX_DEN) ? 128 : 64;
	u16 wc;
	u16 ptr;

	TRCRXV21DMA(TRC_RXV21_FILL, rx->rx2wc, rx->rx2ba);
	if(rx->rx2wc > limit) {
		TRCRXV21Error(TRC_RXV21_WC_OVFL, rx->rx2wc);
		rx->error = 0230; /* Word count overflow */
		rx->rx2es |= RX2ES_WC_OVFL;
		rx->rx2cs |= RX_ERROR;
		RXV21Done(rx);
		return;
	}

	memset(rx->buffer, 0, 256);

	for(wc = rx->rx2wc, ptr = 0; wc > 0; wc--, ptr++) {
		/* transfer words */
		rx->buffer[ptr] = READ(rx->rx2ba);
		rx->rx2ba += 2;
	}

	RXV21Done(rx);
}

void RXV21EmptyBuffer(RXV21* rx)
{
	QBUS* bus = rx->module.bus;
	u16 limit = (rx->rx2cs & RX_DEN) ? 128 : 64;
	u16 wc;
	u16 ptr;

	TRCRXV21DMA(TRC_RXV21_EMPTY, rx->rx2wc, rx->rx2ba);
	if(rx->rx2wc > limit) {
		TRCRXV21Error(TRC_RXV21_WC_OVFL, rx->rx2wc);
		rx->error = 0230; /* Word count overflow */
		rx->rx2es |= RX2ES_WC_OVFL;
		rx->rx2cs |= RX_ERROR;
		RXV21Done(rx);
		return;
	}

	for(wc = rx->rx2wc, ptr = 0; wc > 0; wc--, ptr++) {
		/* transfer words */
		WRITE(rx->rx2ba, rx->buffer[ptr]);
		rx->rx2ba += 2;
	}
	RXV21Done(rx);
}

void RXV21WriteSector(RXV21* rx)
{
	u32 offset;

	rx->rx2sa &= 0037;
	rx->rx2ta &= 0177;

	offset = (rx->rx2sa - 1) * 256 + rx->rx2ta * (26 * 256);

	TRCRXV21CMDCommit((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->rx2cs);
	TRCRXV21Disk(TRC_RXV21_WRITE, (rx->rx2cs & RX_UNIT_SEL) ? 1 : 0,
			(rx->rx2cs & RX_DEN) ? 1 : 0, rx->rx2sa, rx->rx2ta);

	if(!(rx->rx2cs & RX_DEN)) {
		TRCRXV21Error(TRC_RXV21_DEN_ERR, 0);
		rx->error = 0240; /* Density Error */
		rx->rx2cs |= RX_ERROR;
		rx->rx2es |= RX2ES_DEN_ERR;
		RXV21Done(rx);
		return;
	}

	if(rx->rx2sa == 0 || rx->rx2sa > 26 || rx->rx2ta > 76) {
		if(rx->rx2ta > 76) {
			rx->error = 0040; /* Tried to access a track greater than 76 */
			TRCRXV21Error(TRC_RXV21_TRACK_NO, rx->rx2ta);
		} else {
			rx->error = 0070; /* Desired sector could not be found after looking at 52 headers (2 revolutions) */
			TRCRXV21Error(TRC_RXV21_SECT_NO, rx->rx2sa);
		}
		rx->rx2cs |= RX_ERROR;
	} else {
		u16 i;
		u16* sec = (u16*) &rx->data[offset];
		memcpy(&rx->data[offset], rx->buffer, 256);
		for(i = 0; i < 128; i++) {
			sec[i] = U16L(sec[i]);
		}
	}

	RXV21Done(rx);
}

void RXV21ReadSector(RXV21* rx)
{
	u32 offset;

	rx->rx2sa &= 0037;
	rx->rx2ta &= 0177;

	offset = (rx->rx2sa - 1) * 256 + rx->rx2ta * (26 * 256);

	TRCRXV21CMDCommit((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->rx2cs);
	TRCRXV21Disk(TRC_RXV21_READ, (rx->rx2cs & RX_UNIT_SEL) ? 1 : 0,
			(rx->rx2cs & RX_DEN) ? 1 : 0, rx->rx2sa, rx->rx2ta);

	if(!(rx->rx2cs & RX_DEN)) {
		TRCRXV21Error(TRC_RXV21_DEN_ERR, 0);
		rx->error = 0240; /* Density Error */
		rx->rx2cs |= RX_ERROR;
		rx->rx2es |= RX2ES_DEN_ERR;
		RXV21Done(rx);
		return;
	}

	if(rx->rx2sa == 0 || rx->rx2sa > 26 || rx->rx2ta > 76) {
		if(rx->rx2ta > 76) {
			rx->error = 0040; /* Tried to access a track greater than 76 */
			TRCRXV21Error(TRC_RXV21_TRACK_NO, rx->rx2ta);
		} else {
			rx->error = 0070; /* Desired sector could not be found after looking at 52 headers (2 revolutions) */
			TRCRXV21Error(TRC_RXV21_SECT_NO, rx->rx2ta);
		}
		rx->rx2cs |= RX_ERROR;
	} else {
		u16 i;
		memcpy(rx->buffer, &rx->data[offset], 256);
		for(i = 0; i < 128; i++) {
			rx->buffer[i] = U16L(rx->buffer[i]);
		}
	}

	RXV21Done(rx);
}

void RXV21ReadStatus(RXV21* rx)
{
	TRCRXV21CMDCommit((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->rx2cs);

	rx->rx2es |= RX2ES_DRV_RDY | RX2ES_DRV_DEN;
	RXV21Done(rx);
}

void RXV21ReadErrorCode(RXV21* rx)
{
	QBUS* bus = rx->module.bus;

	TRCRXV21CMDCommit((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->rx2cs);

	/* < 7:0> Definitive Error Codes */
	/* <15:8> Word Count Register */
	WRITE(rx->rx2ba, rx->error | (rx->rx2wc << 8));

	/* < 7:0> Current Track Address of Drive 0 */
	/* <15:8> Current Track Address of Drive 1 */
	WRITE(rx->rx2ba + 2, rx->rx2ta | (rx->rx2ta << 8));

	/* < 7:0> Target Track of Current Disk Access */
	/* <15:8> Target Sector of Current Disk Access */
	WRITE(rx->rx2ba + 4, rx->rx2ta | (rx->rx2sa << 8));

	/* <7> Unit Select Bit */
	/* <5> Head Load Bit */
	/* <6> <4> Drive Density Bit of Both Drives */
	/* <0> Density of Read Error Register Command */
	/* <15:8> Track Address of Selected Drive */
	WRITE(rx->rx2ba + 6, _BV(5) | _BV(6) | _BV(4) | _BV(0) | (rx->rx2ta << 8));

	RXV21Done(rx);
}

void RXV21ExecuteCommand(RXV21* rx)
{
	TRCRXV21CMD((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->rx2cs);
	rx->rx2cs &= ~RX_GO;
	rx->state = 1;
	switch(rx->rx2cs & RX_FUNCTION_MASK) {
		case RX_FILL_BUFFER:
			RXV21ClearErrors(rx);
			rx->rx2cs &= ~RX_DONE;
			rx->rx2cs |= RX_TR;
			break;
		case RX_EMPTY_BUFFER:
			RXV21ClearErrors(rx);
			rx->rx2cs &= ~RX_DONE;
			rx->rx2cs |= RX_TR;
			break;
		case RX_WRITE_SECTOR:
			RXV21ClearErrors(rx);
			rx->rx2cs &= ~RX_DONE;
			rx->rx2cs |= RX_TR;
			rx->rx2es = RX2ES_DRV_RDY | RX2ES_DRV_DEN;
			break;
		case RX_READ_SECTOR:
			RXV21ClearErrors(rx);
			rx->rx2cs &= ~RX_DONE;
			rx->rx2cs |= RX_TR;
			rx->rx2es = RX2ES_DRV_RDY | RX2ES_DRV_DEN;
			break;
		case RX_SET_MEDIA_DENSITY:
			RXV21ClearErrors(rx);
			printf("[RXV21] NOT IMPLEMENTED: Set Media Density\n");
			fflush(stdout);
			exit(1);
			break;
		case RX_READ_STATUS:
			RXV21ReadStatus(rx);
			break;
		case RX_WRITE_DELETED_DATA:
			RXV21ClearErrors(rx);
			printf("[RXV21] NOT IMPLEMENTED: Write Deleted Data\n");
			fflush(stdout);
			exit(1);
			break;
		case RX_READ_ERROR_CODE:
			rx->rx2cs &= ~RX_DONE;
			rx->rx2cs |= RX_TR;
			rx->rx2es = RX2ES_DRV_RDY | RX2ES_DRV_DEN;
			break;
	}
}

void RXV21Process(RXV21* rx)
{
	TRCRXV21Step((rx->rx2cs & RX_FUNCTION_MASK) >> 1, rx->state, rx->rx2db);

	if(rx->state == 0) {
		return;
	}

	switch(rx->rx2cs & RX_FUNCTION_MASK) {
		case RX_FILL_BUFFER:
			switch(rx->state) {
				case 1: /* read RX2WC */
					rx->rx2wc = rx->rx2db;
					rx->state++;
					break;
				case 2: /* read RX2BA */
					rx->rx2ba = rx->rx2db;
					rx->rx2cs &= ~RX_TR;
					RXV21FillBuffer(rx);
					break;
			}
			break;
		case RX_EMPTY_BUFFER:
			switch(rx->state) {
				case 1: /* read RX2WC */
					rx->rx2wc = rx->rx2db;
					rx->state++;
					break;
				case 2: /* read RX2BA */
					rx->rx2ba = rx->rx2db;
					rx->rx2cs &= ~RX_TR;
					RXV21EmptyBuffer(rx);
					break;
			}
			break;
		case RX_WRITE_SECTOR:
			switch(rx->state) {
				case 1: /* read RX2SA */
					rx->rx2sa = rx->rx2db;
					rx->state++;
					break;
				case 2: /* read RX2TA */
					rx->rx2ta = rx->rx2db;
					rx->rx2cs &= ~RX_TR;
					RXV21WriteSector(rx);
					break;
			}
			break;
		case RX_READ_SECTOR:
			switch(rx->state) {
				case 1: /* read RX2SA */
					rx->rx2sa = rx->rx2db;
					rx->state++;
					break;
				case 2: /* read RX2TA */
					rx->rx2ta = rx->rx2db;
					rx->rx2cs &= ~RX_TR;
					RXV21ReadSector(rx);
					break;
			}
			break;
		case RX_READ_ERROR_CODE:
			rx->rx2ba = rx->rx2db;
			rx->rx2cs &= ~RX_TR;
			RXV21ReadErrorCode(rx);
			break;
	}
}

u16 RXV21Read(void* self, u16 address)
{
	RXV21* rx = (RXV21*) self;

	if(address == rx->base) { /* RX2CS */
		return rx->rx2cs & RX_RMASK;
	} else if(address == rx->base + 2) { /* RX2DB */
		return rx->rx2db;
	}

	return 0;
}

void RXV21Write(void* self, u16 address, u16 value)
{
	RXV21* rx = (RXV21*) self;

	if(address == rx->base) { /* RX2CS */
		int intr = rx->rx2cs & RX_INTR_ENB;
		rx->rx2cs = (rx->rx2cs & ~RX_WMASK) | (value & RX_WMASK);
		rx->rx2cs &= ~(RX_TR | RX_INIT | RX_ERROR);
		rx->error = 0;
		if(value & RX_INIT) {
			RXV21Reset(rx);
			return;
		}
		if(rx->rx2cs & RX_GO) {
			/* initiate command */
			RXV21ExecuteCommand(rx);
		}
		if(!intr && (value & RX_INTR_ENB) && (rx->rx2cs & RX_DONE)) {
			QBUS* bus = rx->module.bus;
			bus->interrupt(bus, rx->vector);
		}
	} else if(address == rx->base + 2) { /* RX2DB */
		rx->rx2db = value;
		RXV21Process(rx);
	}
}

u8 RXV21Responsible(void* self, u16 address)
{
	RXV21* rx = (RXV21*) self;

	return ((address >= rx->base) && (address <= (rx->base + 2)));
}

void RXV21Reset(void* self)
{
	RXV21* rx = (RXV21*) self;

	rx->state = 0;
	rx->rx2cs = RX_RX02 | RX_DONE;
	rx->rx2es = RX2ES_DEFAULT;
	rx->rx2db = rx->rx2es;
}

void RXV21Init(RXV21* rx)
{
	rx->module.self = (void*) rx;
	rx->module.read = RXV21Read;
	rx->module.write = RXV21Write;
	rx->module.responsible = RXV21Responsible;
	rx->module.reset = RXV21Reset;

	/* factory configuration */
	rx->base = 0177170;
	rx->vector = 0264;

	RXV21Reset(rx);
}

void RXV21Destroy(RXV21* rx)
{
	/* nothing */
}

void RXV21SetData(RXV21* rx, u8* data)
{
	rx->data = data;
}
