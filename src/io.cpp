/**
 * This file is mostly glue code between the C interfaces of the ahrs and motor
 * code and the cpp interfaces the PID code uses.
 */
#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <stdio.h>
#include <gsl/gsl_linalg.h>

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
	static double velocity, accel, sensordepth, sensorvelocity, pzz, pzo, poz, poo = 0; // initial assumption should matter little
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
	double qk[] = {0.000001016, 0.00001016,
                   0.00001016,   0.000001016   }; // noise from environment
	double pk[] = {pzz, pzo,
                       poz, poo}; // covariance of depth-velocity
	double rk[] = {1e7 * (i%2==0 ? 1: -1), 0,
	               0, 1e7* (i%2==0 ? 1: -1)}; // noise from sensor readings; they may be somewhat inaccurate
	double zk[] = {sensordepth, 
	               sensorvelocity }; // depth-velocity from the sensors
	double hk[] = {0.5, 0,
	               0,   1 }; // mapping of depth-velocity to sensordepth-sensorvelocity
	double inva[4]; // for inverse


	//these just write all the arrays as matrices
	gsl_matrix_view Fk = gsl_matrix_view_array(fk, 2, 2); //helps crudely update depth-velocity
	gsl_matrix_view Xk = gsl_matrix_view_array(xk, 2, 1); //depth-velocity
	gsl_matrix_view Bk = gsl_matrix_view_array(bk, 2, 1); //acceleration
	gsl_matrix_view Iz = gsl_matrix_view_array(identity1, 1, 1); //1x1 identity matrix
	gsl_matrix_view Io = gsl_matrix_view_array(identity2, 2, 2); //2x2 identity matrix
	gsl_matrix_view Qk = gsl_matrix_view_array(qk, 2, 2); //noise from environment (known)
	gsl_matrix_view Pk = gsl_matrix_view_array(pk, 2, 2); //covariance of depth-velocity
	gsl_matrix_view Rk = gsl_matrix_view_array(rk, 2, 2); //noise from sensors
	gsl_matrix_view Zk = gsl_matrix_view_array(zk, 2, 1); //sensor depth-velocity
	gsl_matrix_view Hk = gsl_matrix_view_array(hk, 2, 2); //mapping between calculated values and sensor readings
	gsl_matrix_view inv = gsl_matrix_view_array(inva, 2, 2); // for computing inverse of matrix later 
	gsl_matrix * xkk = gsl_matrix_calloc (2,1);
	gsl_matrix * qkk = gsl_matrix_calloc (2,2);
	gsl_matrix * qkkk = gsl_matrix_calloc (2,1);
	gsl_matrix * rkk = gsl_matrix_calloc (2,2);
	gsl_matrix * rkkk = gsl_matrix_calloc (2,1);
	gsl_matrix * pkk = gsl_matrix_calloc (2,2); //all of these are for intermediate steps
	gsl_matrix * Kk = gsl_matrix_calloc (2,2); //Kalman gain
	gsl_matrix * Xkk = gsl_matrix_calloc (2,1); //new depth-velocity
	gsl_matrix * Pkk = gsl_matrix_calloc (2,2); //new covariance
	gsl_permutation * p = gsl_permutation_alloc (2); //for inverse

	//gsl_blas_dgemm (TransA, TransB, alpha, A, B, beta, C) computes C=alpha*op(A)*op(B)+beta*C where op(A)=A or transpose(A) for TransA = CblasNoTrans or CblasTrans respectively and similarly for TransB. (taken from gsl manual)
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Fk.matrix, &Xk.matrix, 0.0, xkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Bk.matrix, &Iz.matrix, 1.0, xkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Fk.matrix, &Pk.matrix, 0.0, qkk);
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, qkk, &Fk.matrix, 0.0, pkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Qk.matrix, &Io.matrix, 1.0, pkk); 
	//Predict step should include:
	//xkk=Fk*Xk+Bk
	//pkk=Fk*Pk*transpose(Fk)+Qk


	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Hk.matrix, pkk, 0.0, qkk);
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, qkk, &Hk.matrix, 0.0, rkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Rk.matrix, &Io.matrix, 1.0, rkk);
	gsl_linalg_LU_decomp (rkk, p, &s);
	gsl_linalg_LU_invert (rkk, p, &inv.matrix); //these two lines compute the inverse of a matrix (Hk*pkk*transpose(Hk)+Rk in this case)
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, pkk, &Hk.matrix, 0.0, qkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, qkk, &inv.matrix, 0.0, Kk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Hk.matrix, xkk, 0.0, rkkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, &Zk.matrix, &Iz.matrix, -1.0, rkkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, Kk, rkkk, 0.0, qkkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, xkk, &Iz.matrix, 1.0, qkkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, qkkk, &Iz.matrix, 0.0, Xkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, Kk, &Hk.matrix, 0.0, qkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, qkk, pkk, 0.0, rkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, pkk, &Io.matrix, -1.0, rkk);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rkk, &Io.matrix, 0.0, Pkk);
	//Update step should include:
	//Xkk=xkk+Kk(Zk-Hk*xkk)
	//Pkk=pkk-Kk*Hk*pkk
	//Kk=Pk*trans(Hk)*(Hk*pkk*trans(Hk)+Rk)^-1

	float depth = gsl_matrix_get (Xkk, 0, 0); //new depth
	velocity = gsl_matrix_get (Xkk, 1, 0); //new velocity
	pzz = gsl_matrix_get (Pkk, 0, 0);
	pzo = gsl_matrix_get (Pkk, 0, 1);
	poz = gsl_matrix_get (Pkk, 1, 0);
	poo = gsl_matrix_get (Pkk, 1, 1); //new elements of the matrix Pk
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
