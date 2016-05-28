#ifndef RELAYS_H
#define RELAYS_H

#ifdef __cplusplus
extern "C" {
#endif

enum relay
{
	DROPPER,
	GRABBER_L,
	GRABBER_R,
	TORPEDO_L,
	TORPEDO_R
};

void io_relay_init();

void io_relay_on(enum relay r);

void io_relay_off(enum relay r);

#ifdef __cplusplus
}
#endif

#endif
