#include <armadillo>
#include <math.h>

// I never actually use this
enum motors {VERT_FL, VERT_FR, VERT_BR, VERT_BL, DIAG_L, DIAG_R, SRGE_L, SRGE_R, STRAFE};

// Power of each motor
float motorPower[9];

// Position of each motor
arma::vec motorPos[9];
// Direction of each motor
arma::vec motorDir[9];
