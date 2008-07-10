// this is the address of the user application
#define VECT_TABLE_LOC 0x0800

// lets define a more compact way to use interrupts
// RETI (return from interrupt) is called from within user app, so it is not needed here
#define INT_REDIRECT(intName)						\
void intName ## _HANDLER(void) __interrupt intName _naked		\
{									\
	_asm								\
		ljmp (VECT_TABLE_LOC+0x03+ intName *0x08)		\
	_endasm;							\
}

INT_REDIRECT(RFERR_VECTOR)
INT_REDIRECT(ADC_VECTOR)
INT_REDIRECT(URX0_VECTOR)
INT_REDIRECT(URX1_VECTOR)
INT_REDIRECT(ENC_VECTOR)
INT_REDIRECT(ST_VECTOR)
INT_REDIRECT(P2INT_VECTOR)
INT_REDIRECT(UTX0_VECTOR)
INT_REDIRECT(DMA_VECTOR)
INT_REDIRECT(T1_VECTOR)
INT_REDIRECT(T2_VECTOR)
INT_REDIRECT(T3_VECTOR)
INT_REDIRECT(T4_VECTOR)
INT_REDIRECT(P0INT_VECTOR)
INT_REDIRECT(UTX1_VECTOR)
INT_REDIRECT(P1INT_VECTOR)
INT_REDIRECT(RF_VECTOR)
INT_REDIRECT(WDT_VECTOR)
