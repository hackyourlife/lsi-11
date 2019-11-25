#include <string.h>
#include <stdlib.h>

#include "lsi11.h"

u16 MSV11DRead(void* self, u16 address)
{
	MSV11D* msv = (MSV11D*) self;
	u16* mem = (u16*) &msv->data[address];

	return *mem;
}

void MSV11DWrite(void* self, u16 address, u16 value)
{
	MSV11D* msv = (MSV11D*) self;
	u16* mem = (u16*) &msv->data[address];
	*mem = value;
}

u8 MSV11DResponsible(void* self, u16 address)
{
	return address < MSV11D_SIZE;
}

void MSV11DReset(void* self)
{
	/* nothing */
}

void MSV11DInit(MSV11D* msv)
{
	msv->module.self = (void*) msv;
	msv->module.read = MSV11DRead;
	msv->module.write = MSV11DWrite;
	msv->module.responsible = MSV11DResponsible;
	msv->module.reset = MSV11DReset;

	msv->data = (u8*) malloc(MSV11D_SIZE);
	memset(msv->data, 0, MSV11D_SIZE);
}

void MSV11DDestroy(MSV11D* msv)
{
	free(msv->data);
}
