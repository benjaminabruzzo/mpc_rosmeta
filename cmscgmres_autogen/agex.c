#include "rhfuncu.c"

/*#define HDIR_EQ_HT */ 
/*#define RESET_DU */ 
/*#define ADAMS */
/*#define TRACE_ON */


/*-------------- Dimensions -------------- */
#define DIMX   2
#define DIMUC  3
#define DIMP   0

/*-------------- Global Variables -------------- */
double tsim0 = 0;
double tsim = 20;
double ht = 0.01;
double tf = 1;
double alpha = 0.5;
double zeta = 100; 
double x0[DIMX] = {2, 0};
double u0[DIMUC] = {0.01, 0.02, 0.03};
double hdir = 0.002;
double rtol = 1.e-6;
int kmax = 2;
int dv = 10;
int dstep =   5;
#define FNMHD  "agex00"

/*-------------- Global Variables Defined by User -------------- */
double  a0 = 1; // initial position of example object?
double  u1max = 0.5; // max value of input u1 

double  q[2] = {1, 1}; // diagonal elements of Q stage cost matrix
double  r[2] = {1, 0.01}; // diagonal elements of R input matrix
double  sf[2] = {1, 1}; // Diagonal elements of S terminal cost matrix

/*------------------------------------------------------------*/
/*-------------- dPhi/dx -------------- */
// S is the square matrix for the temrinal state cost
void phix(double t, double x[], double phx1[])
{
    phx1[0] = sf[0]*x[0];
    phx1[1] = sf[1]*x[1]; 
}

/*-------------- State Equation -------------- */
void xpfunc(double t, double x[], double u[], double xprime[])
{
    xprime[0] = x[1];
    xprime[1] = u[0] - 1.*x[0] + (1. - 1.*a0*(pow(x[0],2.) + pow(x[1],2.)))*x[1];
}

/*-------------- Costate Equation -------------- */
void lpfunc(double t, double lmd[], double linp[], double lprime[])
{
    double x[DIMX], u[DIMUC];
    mmov(1,DIMX, linp, x);
    mmov(1,DIMUC, linp+DIMX, u);
    lprime[0] = -1.*q[0]*x[0] + lmd[1]*(1. + 2.*a0*x[0]*x[1]);
    lprime[1] = -1.*lmd[0] + lmd[1]*(-1. + a0*pow(x[0],2.) +\
 
    3.*a0*pow(x[1],2.)) - 1.*q[1]*x[1];
}

/*-------------- Error in Optimality Condition, Hu -------------- */
void hufunc(double t, double x[], double lmd[], double u[], double hui[])
{
    double o[12];

    o[0] = pow(u[1],2.);
    o[1] = o[0]*r[0];
    o[2] = pow(u[0],2.);
    o[3] = o[0] + o[2];
    o[4] = o[3]*u[2];
    o[5] = o[1] + o[4];
    o[6] = 1/o[5];
    o[7] = pow(u1max,2.);
    o[8] = lmd[1]*u[0];
    o[9] = o[2]*u[2];
    o[10] = r[0] + u[2];
    o[11] = o[7]*o[10];
    hui[0] = 0.5*o[6]*(2.*lmd[1]*o[0] + 2.*r[1]*u[0]*u[1] + u[0]*(2.*o[1] + (o[0] + o[2] - 1.*o[7])*u[2]));
    hui[1] = 0.5*o[6]*(-2.*o[2]*r[1] + u[1]*(o[1] - 2.*o[8] + o[9] - 1.*o[11] - 1.*o[2]*r[0] + o[0]*u[2]));
    hui[2] = (0.5*(2.*o[8] + (-1.*o[0] + o[2])*o[10] + o[11])*u[2] + o[10]*u[1]*(-1.*r[1] + u[1]*u[2]))/(o[9] + o[0]*o[10]);
}

/*-------------- Save Simulation Conditions -------------- */
void final(FILE *fp, float t_cpu, float t_s2e)
{
    int i;
    fprintf(fp, "%% Simulation Result by agex.c \n");
    fprintf(fp, "Precondition = 1 \n");
    fprintf(fp, "tsim = %g\n", tsim);
    fprintf(fp, "ht = %g, dstep = %d\n", ht, dstep);
    fprintf(fp, "tf = %g, dv = %d, alpha = %g, zeta = %g \n", (float)tf, dv, (float)alpha, (float)zeta);
    fprintf(fp, "hdir = %g, rtol = %g, kmax = %d \n", (float)hdir, (float)rtol, kmax);
    fprintf(fp, "u0 = [%g", (float)u0[0] );
    for(i=1; i<DIMUC; i++)
        fprintf(fp, ",%g", (float)u0[i] );
    fprintf(fp, "]\n" );
    fprintf(fp, "t_cpu = %g, t_s2e = %g  %% [sec] \n", t_cpu, t_s2e);

    fprintf(fp, "a0 = %g \n", (float)a0 );
    fprintf(fp, "u1max = %g \n", (float)u1max );


    fprintf(fp, "q = [%g", (float)q[0] );
    for(i=1; i<2; i++)
        fprintf(fp, ",%g", (float)q[i] );
    fprintf(fp, "]\n" );

    fprintf(fp, "r = [%g", (float)r[0] );
    for(i=1; i<2; i++)
        fprintf(fp, ",%g", (float)r[i] );
    fprintf(fp, "]\n" );

    fprintf(fp, "sf = [%g", (float)sf[0] );
    for(i=1; i<2; i++)
        fprintf(fp, ",%g", (float)sf[i] );
    fprintf(fp, "]\n" );

}

#include "rhmainu.c"