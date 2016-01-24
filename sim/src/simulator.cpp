#include "simulator.hpp"
#include <iostream>
#include <string.h>
#include <vector>

FILE* sensor_in, *sensor_out, *motor_in, *motor_out;

/*
 * Update position, anglular orientation, velocity, and angular velocity of object
 * Accepts current state of variables above, plus forces acting on object, mass of object, inertia tensor, and time difference from last update
 * Angle orientation is input as 3x3 rotation matrix
 * Forces are a pair of (force, relative location), location (relative to center of mass) is used to calculate torque
 * Inertia tensor is 3x3 matrix(google it; different from moment of inertia usually learned about in early physics)
 */
void updatePos(arma::vec& pos, arma::mat& ang, arma::vec& vel, arma::vec& avel, std::vector<std::pair<arma::vec, arma::vec> > forces, double mass, arma::mat inert, long long td)
{
	arma::vec oldV(vel);
	arma::vec oldAV(avel);

	// add fluid resistance
	// TODO: make resistance more realistic
	double factor = pow(0.9996, td);
	vel = factor*vel;
	avel = factor*avel;
	arma::vec torque = {0, 0, 0};
	double timeDiff = td/1000.0;
	// update linear velocity and calculate torque
	for (int i = 0; i < forces.size(); i++)
	{
		// get this from equations v = v0+at and F=ma
		vel += forces[i].first*timeDiff / mass;
		// get this from equation T = r x F
		torque += arma::cross(forces[i].second, forces[i].first);
	}

	// inertia tensor is changed when an object rotates relative to the position axes, so it must be updated
	// transform axes to find relative axes of rotated sub
	arma::mat axisD = ang.i();
	arma::mat inertia = axisD*inert*axisD.t();

	// get new angular velocity based on old
	// equation is euler's second law of motion
	avel += inertia.i()*(torque*timeDiff);

	// get average angular velocity tensor
	arma::mat avelTens = {{0, -(avel[2]+oldAV[2])/2, (avel[1]+oldAV[1])/2},
				{(avel[2]+oldAV[2])/2, 0, -(avel[0]+oldAV[0])/2},
				{-(avel[1]+oldAV[1])/2, (avel[0]+oldAV[0])/2, 0}};
	// this is from solving the diff equation dR/dt = W*R
	// it is more accurate over long time differences than individually rotating about each axis
	ang = arma::expmat(avelTens*timeDiff)*ang;

	pos += (oldV + vel)/2.0;
}

void output()
{
	// find pitch roll yaw from rotation matrix
	arma::vec ypr = {atan2(angle.at(1,0), angle.at(0,0)), atan2(-angle.at(2,0),sqrt(angle.at(1,0)*angle.at(1,0)+angle.at(0,0)*angle.at(0,0))), atan2(angle.at(2,1), angle.at(2,2))};
	fprintf(sensor_in, "%f ", ypr[0]); 
	fprintf(sensor_in, "%f ", ypr[1]); 
	fprintf(sensor_in, "%f ", ypr[2]); 
	fprintf(sensor_in, "%f ", posit[2]); 
	fflush(sensor_in);
	
	//fprintf(log, "%f %f %f %f ", ypr[0], ypr[1], ypr[2], posit[2]);
	std::cout << ypr[0] << " " << ypr[1] << " " << ypr[2] << " " << posit[2] << "\n";
	/*
	std::cout<<"Pos\n"<<posit<<"\n\n";
	std::cout<<"Ang\n"<<ypr<<"\n"<<angle<<"\n\n";
	std::cout<<"Vel\n"<<velo<<"\n\n";
	std::cout<<"Ang Vel\n"<<avelo<<"\n\n\n";
	*/
	
}

int main()
{
	// initializes motor position and directions
	// order is VERT_FL, VERT_FR, VERT_BL, VERT_BR, DIAG_L, DIAG_R, SRGE_L, SRGE_R, STRAFE
	motorPos[0] = {1, -1, 0};
	motorPos[1] = {1, 1, 0};
	motorPos[2] = {-1, -1, 0};
	motorPos[3] = {-1, 1, 0};
	motorPos[4] = {1, -1, 0};
	motorPos[5] = {1, 1, 0};
	motorPos[6] = {-1, -1, 0};
	motorPos[7] = {-1, 1, 0};

	// vector with direction of motor and magnitude of motor force
	motorDir[0] = {0, 0, 1};
	motorDir[1] = {0, 0, 1};
	motorDir[2] = {0, 0, 1};
	motorDir[3] = {0, 0, 1};
	motorDir[4] = {0.966, 0.259, 0};
	motorDir[5] = {0.966, -0.259, 0};
	motorDir[6] = {0.966, -0.259, 0};
	motorDir[7] = {0.966, 0.259, 0};

	sensor_in = fopen("sensor_in", "w");
	sensor_out = fopen("sensor_out", "r");
	motor_in = fopen("motor_in", "w");
	motor_out = fopen("motor_out", "r");
	//FILE *log = fopen("log", "w");
	output();

	while(true)
	{
		// get forces/positions based on motor powers
		std::vector<std::pair<arma::vec, arma::vec> > fList;
		for (int i = 0; i < 8; i++)
		{
			fscanf(motor_out, " %i", &motorPower[i]);
			std::cout << motorPower[i] << " ";
			// TODO: simulate counter spinning from the spin of motors
			fList.push_back(std::pair<arma::vec, arma::vec>((motorPower[i])*angle*motorDir[i], angle*motorPos[i]));
		}
		std::cout << std::endl;
		// add force of gravity/buoyancy
		// TODO: simulate lift & other fluid forces
		fList.push_back(std::pair<arma::vec, arma::vec>(axisK*subVol*fluidDen*gravity, angle*centOfVol));
		fList.push_back(std::pair<arma::vec, arma::vec>(-(axisK*subMass*gravity), zeroV));
		int time = 10;

		// break up steps into smaller time differences to make linear acceleration while rotating more accurate
		int tDiff = 1;
		for (int i = 0; i < time; i+= tDiff)
		{
			// calculate physics
			updatePos(posit, angle, velo, avelo, fList, subMass, inertTens, tDiff);
		}

		// print values
		output();	
	}
}
