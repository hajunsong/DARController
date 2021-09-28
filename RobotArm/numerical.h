/*
 * Numerical.h
 *
 *  Created on: 2019. 5. 27.
 *      Author: keti-hajun
 */

#ifndef NUMERICAL_H_
#define NUMERICAL_H_

#include <iostream>
#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <cmath>

using namespace std;
typedef unsigned int uint;
const double TINY = 1.0e-20;

#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)>(y)?(y):(x))

class Numerical {
public:
    Numerical();
    ~Numerical();
    // LU solver
public:
    void ludcmp(double *a, int n, int* indx, double d, double *fac);
    void lubksb(double *a, int n, int* indx, double *b, double *x);

    // Singular Value Decomposition
public:
    double pythag(double a, double b);
    void svdcmp(double *a, int m, int n, double *U, double *w, double *v);

    // Integrator - Adams Bashforth 3rd order formulation
public:
    void absh3Initialize(double h, uint array_size);
    double absh3(double *Y, double *Yp, double t_current);
    void getY_next(double *Y);
    bool absh3_flag;
private:
    double step_size, t_next;
    uint n, intcount;
    double *Y_next, *AW, *AW1;
};

#endif /* NUMERICAL_H_ */
