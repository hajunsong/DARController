#include "numerical.h"

static double *vv, *a;
static int i, imax=0, j, k;
static double big, temp;

static int ii = 0, ip;
static double sum;

Numerical::Numerical() {
    absh3_flag = false;
}

Numerical::~Numerical()
{
    if (absh3_flag) {
        delete[] Y_next;
        delete[] AW;
        delete[] AW1;
    }
}

void Numerical::ludcmp(double *A, int n, int* indx, double d, double *fac)
{

    vv = new double[n];
    a = new double[n*n];
    memcpy(a, A, sizeof(double) * 6 * 6);
    for (i = 0; i < n; i++) {
        big = 0.0;
        for (j = 0; j < n; j++)
            if ((temp = fabs(a[i*n + j])) > big) big = temp;
        if (big == 0.0) {
//            printf("Singular matrix in LUdcmp");
        }
        vv[i] = 1.0 / big;
    }
    for (k = 0; k < n; k++) {
        big = 0.0;
        for (i = k; i < n; i++) {
            temp = vv[i] * fabs(a[i*n + k]);
            if (temp > big) {
                big = temp;
                imax = i;
            }
        }
        if (k != imax) {
            for (j = 0; j < n; j++) {
                temp = a[imax*n + j];
                a[imax*n + j] = a[k*n + j];
                a[k*n + j] = temp;
            }
            d = -d;
            vv[imax] = vv[k];
        }
        indx[k] = imax;
        if (a[k*n + k] == 0.0) a[k*n + k] = TINY;
        for (i = k + 1; i < n; i++) {
            temp = a[i*n + k] /= a[k*n + k];
            for (j = k + 1; j < n; j++)
                a[i*n + j] -= temp * a[k*n + j];
        }
    }
    //////////////////
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            fac[i*n + j] = a[i*n + j];
        }
    }

    delete[] vv;
    delete[] a;
}

void Numerical::lubksb(double *a, int n, int* indx, double *b, double *x)
{
    for (i = 0; i < n; i++) x[i] = b[i];
    for (i = 0; i < n; i++) {
        ip = indx[i];
        sum = x[ip];
        x[ip] = x[i];
        if (ii != 0)
            for (j = ii - 1; j < i; j++) sum -= a[i*n + j] * x[j];
        else if (sum != 0.0)
            ii = i + 1;
        x[i] = sum;
    }
    for (i = n - 1; i >= 0; i--) {
        sum = x[i];
        for (j = i + 1; j < n; j++) sum -= a[i*n + j] * x[j];
        x[i] = sum / a[i*n + i];
    }
}

double Numerical::pythag(double a, double b)
{
    double absa, absb;
    absa = fabs(a);
    absb = fabs(b);
    if (absa > absb) return absa * sqrt(1.0 + sqrt(absb / absa));
    else return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + sqrt(absa / absb)));
}
/*
void Numerical::svdcmp(double *a, int m, int n, double *U, double *w, double *v)
{
    bool flag;
    int i, its, j, jj, k, l = 0, nm;
    double anorm, c, f, g, h, s, scale, x, y, z, *rv1;
    rv1 = new double[n];
    memcpy(U, a, sizeof(double)*static_cast<uint>(m*n));
    g = scale = anorm = 0.0;
    for (i = 0; i < n; i++) {
        l = i + 2;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m) {
            for (k = i; k < m; k++) scale += fabs(U[k*n+i]);
            if (scale != 0.0) {
                for (k = i; k < m; k++) {
                    U[k*n + i] /= scale;
                    s += U[k*n + i] * U[k*n + i];
                }
                f = U[i*n + i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U[i*n+i] = f - g;
                for (j = l - 1; j < n; j++) {
                    for (s = 0.0, k = i; k < m; k++) s += U[k*n+i] * U[k*n+j];
                    f = s / h;
                    for (k = i; k < m; k++) U[k*n+j] += f * U[k*n+i];
                }
                for (k = i; k < m; k++) U[k*n+i] *= scale;
            }
        }
        w[i] = scale * g;
        g = s = scale = 0.0;
        if (i + 1 <= m && i + 1 != n) {
            for (k = l - 1; k < n; k++) scale += fabs(U[i*n+k]);
            if (scale != 0.0) {
                for (k = l - 1; k < n; k++) {
                    U[i*n+k] /= scale;
                    s += U[i*n+k] * U[i*n+k];
                }
                f = U[i*n+l - 1];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                U[i*n+l - 1] = f - g;
                for (k = l - 1; k < n; k++) rv1[k] = U[i*n+k] / h;
                for (j = l - 1; j < m; j++) {
                    for (s = 0.0, k = l - 1; k < n; k++) s += U[j*n+k] * U[i*n+k];
                    for (k = l - 1; k < n; k++) U[j*n+k] += s * rv1[k];
                }
                for (k = l - 1; k < n; k++) U[i*n+k] *= scale;
            }
        }
        anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
    }
    for (i = n - 1; i >= 0; i--) {
        if (i < n - 1) {
            if (g != 0.0) {
                for (j = l; j < n; j++)
                    v[j*n+i] = (U[i*n+j] / U[i*n+l]) / g;
                for (j = l; j < n; j++) {
                    for (s = 0.0, k = l; k < n; k++) s += U[i*n+k] * v[k*n+j];
                    for (k = l; k < n; k++) v[k*n+j] += s * v[k*n+i];
                }
            }
            for (j = l; j < n; j++) v[i*n+j] = v[j*n+i] = 0.0;
        }
        v[i*n+i] = 1.0;
        g = rv1[i];
        l = i;
    }
    for (i = MIN(m, n) - 1; i >= 0; i--) {
        l = i + 1;
        g = w[i];
        for (j = l; j < n; j++) U[i*n+j] = 0.0;
        if (g != 0.0) {
            g = 1.0 / g;
            for (j = l; j < n; j++) {
                for (s = 0.0, k = l; k < m; k++) s += U[k*n+i] * U[k*n+j];
                f = (s / U[i*n+i])*g;
                for (k = i; k < m; k++) U[k*n+j] += f * U[k*n+i];
            }
            for (j = i; j < m; j++) U[j*n+i] *= g;
        }
        else for (j = i; j < m; j++) U[j*n+i] = 0.0;
        ++U[i*n+i];
    }
    for (k = n - 1; k >= 0; k--) {
        for (its = 0; its < 30; its++) {
            flag = true;
            for (l = k; l >= 0; l--) {
                nm = l - 1;
                if (abs(rv1[l]) + anorm == anorm) {
                    flag = false;
                    break;
                }
                if (abs(w[nm]) + anorm == anorm) break;
            }
            if (flag) {
                c = 0.0;
                s = 1.0;
                for (i = l; i < k + 1; i++) {
                    f = s * rv1[i];
                    rv1[i] = c * rv1[i];
                    if (abs(f) + anorm == anorm) break;
                    g = w[i];
                    h = pythag(f, g);
                    w[i] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    for (j = 0; j < m; j++) {
                        y = U[j*n+nm];
                        z = U[j*n+i];
                        U[j*n+nm] = y * c + z * s;
                        U[j*n+i] = z * c - y * s;
                    }
                }
            }
            z = w[k];
            if (l == k) {
                if (z < 0.0) {
                    w[k] = -z;
                    for (j = 0; j < n; j++) v[j*n+k] = -v[j*n+k];
                }
                break;
            }
            if (its == 29) {
                cout << "no convergence in 30 svdcmp iterations" << endl;
                break;
            }

            x = w[l];
            nm = k - 1;
            y = w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z)*(y + z) + (g - h)*(g + h)) / (2.0*h*y);
            g = pythag(f, 1.0);
            f = ((x - z)*(x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
            c = s = 1.0;
            for (j = l; j <= nm; j++) {
                i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g = c * g;
                z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for (jj = 0; jj < n; jj++) {
                    x = v[jj*n+j];
                    z = v[jj*n+i];
                    v[jj*n+j] = x * c + z * s;
                    v[jj*n+i] = z * c - x * s;
                }
                z = pythag(f, h);
                w[j] = z;
                if (z) {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                for (jj = 0; jj < m; jj++) {
                    y = U[jj*n+j];
                    z = U[jj*n+i];
                    U[jj*n+j] = y * c + z * s;
                    U[jj*n+i] = z * c - y * s;
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
    delete[] rv1;
}
*/
void Numerical::absh3Initialize(double h, uint array_size)
{
    step_size = h;
    n = array_size;
    Y_next = new double[n];
    AW = new double[n * 2];
    AW1 = new double[n * 2];

    intcount = 1;
    absh3_flag = true;
}

double Numerical::absh3(double *Y, double *Yp, double t_current) {
    /* ABSH3 : constant step Adams Bashforth 3rd order formulation.
        written by Sung-Soo Kim
        Date: Oct. 19, 1998
        copyright reserved by Sung-Soo Kim

        input variables
        t_current: current time
        Y : current state
        Yp : current derivative of state
        step_size: integration step_size

        output variables
        Y_next : state at next time step
        t_next : Next time

        STARTER: upto 2h, i.e., derivatives are stored for the initial  time steps at 0, h, 2h, to form
        3rd order Adams Bashforth formula */

    switch (intcount)
    {
    case 1:
        // Forward Euler method with 0.25 step_size for initial step
        // use derivative information at 0 step
        // y=y+step_size*Yp/4.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * Yp[i] / 4.0;
        }
        // w(:,2) = Yp;
        for (uint i = 0; i < n; i++) {
            AW[i * 2 + 1] = Yp[i];
        }
        // w1(:,2) = Yp;
        for (uint i = 0; i < n; i++) {
            AW1[i * 2 + 1] = Yp[i];
        }
        t_next = t_current + step_size / 4.0;
        break;
    case 2:
        // Adams Bashforth 2nd order method with 0.25 step_size for 2nd step
        // use derivative information at 0, h/4
        // y = y + step_size_h * ( 3.0*Yp - w1(:,2))/8.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (3 * Yp[i] - AW1[i * 2 + 1]) / 8.0;
        }
        // w1(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW1[i * 2] = Yp[i];
        }
        t_next = t_current + step_size / 4.0;
        break;
    case 3:
        // Adams Bashforth 3rd order method with 0.25 step_size for 3rd step
        // use derivative information at 0, h/4, h/2
        // y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/48.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i * 2] + 5.0*AW1[i * 2 + 1]) / 48.0;
        }
        // w1(:,2) = w1(:,1);
        for (uint i = 0; i < n; i++) {
            AW1[i * 2 + 1] = AW1[i * 2];
        }
        // w1(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW1[i * 2] = Yp[i];
        }
        t_next = t_current + step_size / 4.0;
        break;
    case 4:
        // Adams Bashforth 3rd order method with 0.25 step_size for 4th step
        // use derivative information at h/4, h/2, 3h/4
        // y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/48.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i * 2] + 5.0*AW1[i * 2 + 1]) / 48.0;
        }
        // w1(:,2) = w(:,2);
        for (uint i = 0; i < n; i++) {
            AW1[i * 2 + 1] = AW[i * 2 + 1];
        }
        t_next = t_current + step_size / 4.0;
        break;
    case 5:
        // Adams Bashforth 3rd order method with 0.5 step_size for 5th step
        // use derivative information at 0, h/2, h
        // y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/24.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i * 2] + 5.0*AW1[i * 2 + 1]) / 24.0;
        }
        // w(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW[i * 2] = Yp[i];
        }
        // w1(:,2) = w1(:,1);
        for (uint i = 0; i < n; i++) {
            AW1[i * 2 + 1] = AW1[i * 2];
        }
        // w1(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW1[i * 2] = Yp[i];
        }
        t_next = t_current + step_size / 2.0;
        break;
    case 6:
        // Adams Bashforth 3rd order method with 0.5 step_size for 6th step
        // use derivative information at h/2, h,  3h/2
        // y = y + step_size * ( 23.0*Yp - 16.0*w1(:,1) + 5.0*w1(:,2))/24.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW1[i * 2] + 5.0*AW1[i * 2 + 1]) / 24.0;
        }
        // w1(:,2) = w1(:,1);
        for (uint i = 0; i < n; i++) {
            AW1[i * 2 + 1] = AW1[i * 2];
        }
        // w1(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW1[i * 2] = Yp[i];
        }
        t_next = t_current + step_size / 2.0;
        break;
    case 7:
        // Adams Bashforth 3rd order method with step_size for 7th step
        // use derivative information at 0,  h,  2h
        // y = y + step_size * ( 23.0*Yp - 16.0*w(:,1) + 5.0*w(:,2))/12.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW[i * 2] + 5.0*AW[i * 2 + 1]) / 12.0;
        }
        // w(:,2) = w(:,1);
        for (uint i = 0; i < n; i++) {
            AW[i * 2 + 1] = AW[i * 2];
        }
        // w(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW[i * 2] = Yp[i];
        }
        t_next = t_current + step_size;
        break;
    default:
        // Adams Bashforth 3rd order method with step_size for more than 8th step
        // use derivative information t_current-2h, t_current-h, t_current
        // y = y + step_size * ( 23.0*Yp - 16.0*w(:,1) + 5.0*w(:,2))/12.0;
        for (uint i = 0; i < n; i++) {
            Y_next[i] = Y[i] + step_size * (23.0*Yp[i] - 16.0*AW[i * 2] + 5.0*AW[i * 2 + 1]) / 12.0;
        }
        // w(:,2) = w(:,1);
        for (uint i = 0; i < n; i++) {
            AW[i * 2 + 1] = AW[i * 2];
        }
        // w(:,1) = Yp;
        for (uint i = 0; i < n; i++) {
            AW[i * 2] = Yp[i];
        }
        t_next = t_current + step_size;
        break;
    }
    intcount++;

    return t_next;
}

void Numerical::getY_next(double *Y) {
    memcpy(Y, Y_next, sizeof(double)*n);
}
