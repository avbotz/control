#include "motor.hpp"

// axis vectors
arma::vec axisI = {1, 0, 0};
arma::vec axisJ = {0, 1, 0};
arma::vec axisK = {0, 0, 1};
arma::vec zeroV = {0, 0, 0};

// initial state of sub variables
arma::vec posit = {0, 0, 0};
arma::mat angle = {{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}};
arma::vec velo = {0, 0, 0};
arma::vec avelo = {0, 0, 0};
double subMass = 80;
double subVol = 80;
double gravity = 10;
double fluidDen = 1;

// center of volume/bouyancy to see where to apply force
arma::vec centOfVol = {0, 0, 0};
arma::mat inertTens = {{50, 8, -1},
			{8, 100, 0},
			{-1, 0, 100}};

/*
 *
 * Waiting to put in accurate mass data until I have accurate motor location/force data
 *
 * 	mass(kg): 16.06628
 *
 * 	inertia(kg*m^2):
 *	Lxx = 0.98654262798	Lxy = -0.07551808038	Lxz = -0.00011512682
 *	Lyx = -0.07551808038	Lyy = 0.53351962979	Lyz = -0.00602791053
 *	Lzx = -0.00011512682	Lzy = -0.00602791053	Lzz = 1.02723341669
 *
 */
