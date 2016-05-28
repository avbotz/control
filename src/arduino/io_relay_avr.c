#include <avr/io.h>

#include "io_relay.h"
#include "macrodef.h"
#include "dbg.h"

#define PORT_DROPPER B
#define P_DROPPER 1 // arduino pin #52

#define PORT_GRABBER_L B
#define P_GRABBER_L 3 // arduino pin #50

#define PORT_GRABBER_R L
#define P_GRABBER_R 1 // arduino pin #48

#define PORT_TORPEDO_L L
#define P_TORPEDO_L 3 // arduino pin #46

#define PORT_TORPEDO_R L
#define P_TORPEDO_R 5 // arduino pin #44

/**
 * Sets pins to output.
 */
void io_relay_init()
{
	// Set all pins high initially. The relays on the 8 channel Sainsmart board
	// are active low on, active high or high impedance off.
	CC_XXX(PORT, PORT_DROPPER, ) |= 1 << CC_XXX(P, PORT_DROPPER, P_DROPPER);
	CC_XXX(PORT, PORT_GRABBER_L, ) |= 1 << CC_XXX(P, PORT_GRABBER_L, P_GRABBER_L);
	CC_XXX(PORT, PORT_GRABBER_R, ) |= 1 << CC_XXX(P, PORT_GRABBER_R, P_GRABBER_R);
	CC_XXX(PORT, PORT_TORPEDO_L, ) |= 1 << CC_XXX(P, PORT_TORPEDO_L, P_TORPEDO_L);
	CC_XXX(PORT, PORT_TORPEDO_R, ) |= 1 << CC_XXX(P, PORT_TORPEDO_R, P_TORPEDO_R);

	CC_XXX(DDR, PORT_DROPPER, ) |= 1U << CC_XXX(DD, PORT_DROPPER, P_DROPPER);
	CC_XXX(DDR, PORT_GRABBER_L, ) |= 1U << CC_XXX(DD, PORT_GRABBER_L, P_GRABBER_L);
	CC_XXX(DDR, PORT_GRABBER_R, ) |= 1U << CC_XXX(DD, PORT_GRABBER_R, P_GRABBER_R);
	CC_XXX(DDR, PORT_TORPEDO_L, ) |= 1U << CC_XXX(DD, PORT_TORPEDO_L, P_TORPEDO_L);
	CC_XXX(DDR, PORT_TORPEDO_R, ) |= 1U << CC_XXX(DD, PORT_TORPEDO_R, P_TORPEDO_R);
	return;
}

void io_relay_off(enum relay r)
{
	switch (r)
	{
		case DROPPER:
		{
			CC_XXX(PORT, PORT_DROPPER, ) |= 1 << CC_XXX(P, PORT_DROPPER, P_DROPPER);
			break;
		}
		case GRABBER_L:
		{
			CC_XXX(PORT, PORT_GRABBER_L, ) |= 1 << CC_XXX(P, PORT_GRABBER_L, P_GRABBER_L);
			break;
		}
		case GRABBER_R:
		{
			CC_XXX(PORT, PORT_GRABBER_R, ) |= 1 << CC_XXX(P, PORT_GRABBER_R, P_GRABBER_R);
			break;
		}
		case TORPEDO_L:
		{
			CC_XXX(PORT, PORT_TORPEDO_L, ) |= 1 << CC_XXX(P, PORT_TORPEDO_L, P_TORPEDO_L);
			break;
		}
		case TORPEDO_R:
		{
			CC_XXX(PORT, PORT_TORPEDO_R, ) |= 1 << CC_XXX(P, PORT_TORPEDO_R, P_TORPEDO_R);
			break;
		}
		default:
		{
			DEBUG("Invalid relay number:Relay number out of range: %u", r);
		}
	}
	return;
}

void io_relay_on(enum relay r)
{
	switch (r)
	{
		case DROPPER:
		{
			CC_XXX(PORT, PORT_DROPPER, ) &= ~(1 << CC_XXX(P, PORT_DROPPER, P_DROPPER));
			break;
		}
		case GRABBER_L:
		{
			CC_XXX(PORT, PORT_GRABBER_L, ) &= ~(1 << CC_XXX(P, PORT_GRABBER_L, P_GRABBER_L));
			break;
		}
		case GRABBER_R:
		{
			CC_XXX(PORT, PORT_GRABBER_R, ) &= ~(1 << CC_XXX(P, PORT_GRABBER_R, P_GRABBER_R));
			break;
		}
		case TORPEDO_L:
		{
			CC_XXX(PORT, PORT_TORPEDO_L, ) &= ~(1 << CC_XXX(P, PORT_TORPEDO_L, P_TORPEDO_L));
			break;
		}
		case TORPEDO_R:
		{
			CC_XXX(PORT, PORT_TORPEDO_R, ) &= ~(1 << CC_XXX(P, PORT_TORPEDO_R, P_TORPEDO_R));
			break;
		}
		default:
		{
			DEBUG("Relay number out of range: %u", r);
		}
	}
	return;
}
