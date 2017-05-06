/**
 * This file is mostly glue code between the C interfaces of the ahrs and motor
 * code and the cpp interfaces the PID code uses.
 */
#include <math.h>
#include <stdio.h>

#include "matrix.cpp"
#include "io.hpp"
#include "io_kill.hpp"
#include "io_ahrs.h"
#include "ahrs.h"
#include "io_depth.hpp"
#include "io_cpu.hpp"
#include "io_m5.h"
#include "m5.h"
#include "io_relay.h"
#include "io_millis.hpp"
#include "macrodef.h"
#include "rotation.h"

#define PI 3.14159f

#define EARTH_ACCEL 9.807f // TODO: add centrifugal acceleration

static unsigned long timestep;

static float leveloffset[3] = {0.f, -.001694f, .000908f};

void setLevelRef(uint_fast8_t dir, float offset)
{
	if (dir > 2)
	{
		// Out of bounds dir.
		return;
	}
	leveloffset[dir] = offset;
	return;
}

/**
 * Sets new depth in state based on past depth and sensor data.
 */
void updateDepth(State &state, float const depth_prev)
{
	// Acceleration vector is rotated according to sub's orientation
	float angles[3];
	for (uint_fast8_t i = 3; i--;)
	{
		angles[i] = 2.f * PI * state.property[S_YAW + i];
	}
    float matrix[3][3];
    rotation(angles, matrix);
	// All accelerations are negated because ahrs uses opposite directions as
	// we do.
	float accels[3] = {-ahrs_accel(SURGE) * EARTH_ACCEL,
		-ahrs_accel(SWAY) * EARTH_ACCEL, -ahrs_accel(HEAVE) * EARTH_ACCEL};
	float depth_accel = 0.f;
	for (uint_fast8_t i = 3; --i;)
	{
		// Multiplication is transposed to go from sub space to world space
		depth_accel += accels[i] * matrix[i][2];
	}
	depth_accel += EARTH_ACCEL; // subtract gravitational acceleration (+Z is down)

	

	unsigned long timeprev = timestep;
	timestep = milliseconds();
	double dt = (timestep - timeprev) / 1000;
	static double velocity = 0, accel = 0, sensordepth = 0, sensorvelocity = 0, pzz = 0, pzo = 0, poz = 0, poo = 0; // initial assumption should matter little
	int s;
	static int i = 0;
	sensorvelocity = sensorvelocity + depth_accel * dt;
	sensordepth = sensordepth + sensorvelocity * dt + depth_accel * 0.5 * dt * dt;

	/**
	variable names are cancerous, but I tried to match them with those used in: www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
	*/
	double identity1[] = {1};
	double identity2[] = {1, 0,
		              0, 1 }; // the two identity matrices
	double xk[] = { depth_prev, 
	                velocity }; // depth-velocity vector
	double fk[] = { 1, dt, 
	                0, 1 }; // multiplied to depth-velocity vector to update depth-velocity
	double bk[] = {dt*dt*accel*0.5,
	                dt*accel }; // added to depth-velocity (acceleration)
	double qk[] = {0.000000001016, 0,
                   0,   0.000001016   }; // noise from environment
	double pk[] = {pzz, pzo,
                       poz, poo}; // covariance of depth-velocity
	double rk[] = {1, 0,
	               0, 1}; // noise from sensor readings; they may be somewhat inaccurate
	double zk[] = {sensordepth, 
	               sensorvelocity }; // depth-velocity from the sensors
	double hk[] = {0.5, 0,
	               0,   1 }; // mapping of depth-velocity to sensordepth-sensorvelocity
	double xkk[2] = {0, 0};
	double Xk[2] = {0, 0};
	double pkk[4] = {0, 0, 0, 0};
	double Pk[4] = {0, 0, 0, 0};
	double Kk[4] = {0, 0, 0, 0};
	double intermediate[2] = {0, 0};
	double intermediate2[2] = {0, 0};
	double intermediate3[4] = {0, 0, 0, 0};
	double intermediate4[4] = {0, 0, 0, 0};
	double inverse[4] = {0, 0, 0, 0};

	//Kalman filter calculations
	MatProd2(fk, xk, intermediate);
	MatSum2(bk, intermediate, xkk);
	MatProd(fk, pk, intermediate3);
	MatTrans(fk);
	MatProd(intermediate3, fk, intermediate4);
	MatSum(qk, intermediate4, pkk);
	//Predict step should include:
	//xkk=Fk*Xk+Bk
	//pkk=Fk*Pk*transpose(Fk)+Qk


	MatProd(hk, pkk, intermediate3);
	MatProd(intermediate3, hk, intermediate4);
	MatSum(intermediate4, rk, inverse);
	MatInv(inverse);
	MatProd(pkk, hk, intermediate3);
	MatProd(intermediate3, inverse, Kk);
	MatProd2(hk, xkk, intermediate2);
	MatSub2(zk, intermediate2, intermediate);
	MatProd(Kk, intermediate, intermediate2);
	MatSum2(xkk, intermediate2, Xk);
	MatProd(hk, pkk, intermediate3);
	MatProd(Kk, intermediate3, intermediate4);
	MatSub(pkk, intermediate4, Pk);
	//Update step should include:
	//Xkk=xkk+Kk(Zk-Hk*xkk)
	//Pkk=pkk-Kk*Hk*pkk
	//Kk=Pk*trans(Hk)*(Hk*pkk*trans(Hk)+Rk)^-1

	float depth = Xk[0]; //new depth
	velocity = Xk[1]; //new velocity
	pzz = Pk[0];
	pzo = Pk[1];
	poz = Pk[2];
	poo = Pk[3]; //new elements of the matrix Pk
	i++;
/**
	unsigned long timeprev = timestep;
	timestep = milliseconds();
	float dt = (timestep - timeprev) / 1000.f;
	static float velocity = 0.f; // initial assumption should matter little
	float depth = .02f * ((io_depth() - 230.f) / 50.f) +
		.98f * (depth_prev + dt * velocity + .5f * dt * dt * depth_accel);
	velocity = .02 * ((depth - depth_prev) / dt) +
		.98f * (velocity + dt * depth_accel);
*/
	state.property[S_DEPTH] = depth;
}

bool alive()
{
	return io_kill();
}

// State.property[S_YAW, S_PITCH, S_ROLL] are non-modular angle values with
// delta physical degrees == delta 1.0 of the value. eg if S_YAW == 0 and there
// is a rotation of 720 degrees, S_YAW will become 2 rather than 0.
State getState(const State &current)
{
	ahrs_att_update();
	State newstate = current;
	// order of enum must be S_YAW, S_PITCH, S_ROLL
	for (uint_fast8_t dir = S_YAW; dir <= S_ROLL; ++dir)
	{
		// Scale one rotation from 360 degrees to 1 unit
		newstate.property[dir] = truncf(newstate.property[dir]) +
			ahrs_att((enum att_axis)(dir - S_YAW)) / 360.f;

		// Heuristic to allow non-modular angles. If the angle has changed by
		// more than .5, we assume that is due to angle overflow because it is
		// not likely to rotate that much between calls. Note that Pitch from
		// the ahrs can never overflow anyway because it only varies -90 - 90.
		if (newstate.property[dir] - current.property[dir] < -.5f)
		{
			newstate.property[dir] += 1.f;
		}
		else if (newstate.property[dir] - current.property[dir] > .5f)
		{
			newstate.property[dir] -= 1.f;
		}
		// Adjust for level offset.
		newstate.property[dir] -= leveloffset[dir - S_YAW];
	}
	updateDepth(newstate, current.property[S_DEPTH]);

	return newstate;
}

// The actual power values transmitted to the thrusters are truncated to the
// range [-1, 1].
void setMotor(const Motor& motor)
{
	for (uint_fast8_t t = numMotors; t--;)
	{
		// Assumes VERT_FR is the thruster with the lowest Motor ID and all of
		// the following motor IDs are consecutive. This is done so we can skip
		// beginning Motor IDs (ie Motor ID 0 since it caused a bug with our
		// thrusters).
		m5_power((enum thruster)(t + VERT_FR),
				TRUNC(-1.f, motor.thrust[t], 1.f));
	}
	m5_power_offer_resume();

	for (uint8_t i = 0; i < numMotors; i++)
	{
//		cprintf("motor; %i: %f\n", i, TRUNC(-1.f, motor.thrust[i], 1.f));
	}
}

static void setpowers(float vals[NUM_THRUSTERS])
{
	for (uint_fast8_t t = NUM_THRUSTERS; t--;)
	{
		// zero powers
		m5_power((enum thruster)t, vals[t]);
	}
}

void activateRelay(enum Relay r)
{
	io_relay_on((relay)r);
}

void deactivateRelay(enum Relay r)
{
	io_relay_off((relay)r);
}

// It will start again when setMotor is called
void pauseMotorComm()
{
	io_m5_trans_stop();
}

unsigned long milliseconds()
{
	return io_millis();
}

void init_io()
{
	io_cpu_init();
	io_kill_init();
	io_depth_init("depth_in");
	io_m5_init("/dev/ttyUSB1");
	float powers[NUM_THRUSTERS] = {0.f};
	setpowers(powers); // zero powers
	io_m5_trans_set(m5_power_trans);
	io_ahrs_init("/dev/ttyUSB0");
	// May not be strictly necessary, since the data components can be saved to
	// non-volatile memory.
	ahrs_set_datacomp();
	ahrs_cont_start();
	io_ahrs_recv_start(ahrs_att_recv); // Start receiving data asynchronously
	io_relay_init();
	timestep = milliseconds();
	return;
}
