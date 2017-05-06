#include <math.h>
#include <stdio.h>



double MatProd(double a[4], double b[4], double c[4]) {
	for (int i=0; i<4; i++) {
		c[i] = a[i-(i%2)]*b[i%2]+b[(i%2)+2]*a[i-(i%2)+1];
	}
	return c[4];
}

double MatProd2(double a[4], double b[2], double c[2]) {
	c[0] = a[0]*b[0] + a[1]*b[1];
	c[1] = a[2]*b[0] + a[3]*b[1];
	return c[2];
}

double MatSum(double a[4], double b[4], double c[4]) {
	for (int i=0; i<4; i++) {
		c[i] = a[i]+b[i];
	}
	return c[4];
}

double MatSum2(double a[2], double b[2], double c[2]) {
	c[0] = a[0]+b[0];
	c[1] = a[1]+b[1];
	return c[2];
}

double MatSub(double a[4], double b[4], double c[4]) {
	for (int i=0; i<4; i++) {
		c[i] = a[i]-b[i];
	}
	return c[4];
}

double MatSub2(double a[2], double b[2], double c[2]) {
	c[0] = a[0]-b[0];
	c[1] = a[1]-b[1];
	return c[2];
}

double MatTrans(double a[4]) {
	double t = a[1];
	a[1] = a[2];
	a[2] = t;
	return a[4];
}

double MatInv(double a[4]) {
	double detinv = 1/(a[0]*a[3]-a[1]*a[2]);
	double t = a[0];
	a[0] = detinv * a[3];
	a[3] = detinv * t;
	a[1] = -detinv * a[1];
	a[2] = -detinv * a[2];
	return a[0];
}
