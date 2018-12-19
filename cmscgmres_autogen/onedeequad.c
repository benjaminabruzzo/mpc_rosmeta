#include "rhfuncu.c"

/*#define HDIR_EQ_HT */ 
/*#define RESET_DU */ 
/*#define ADAMS */
/*#define TRACE_ON */

/*-------------- Dimensions -------------- */
// dimx : Dimension of the State Vector
#define DIMX   2
// dimu : Dimension of the Control Input Vector
// dimc : Number of Constraints
#define DIMUC  3
// dimp : Dimension of Time-Variant Parameters
#define DIMP   0

/*-------------- Global Variables -------------- */
// tsim0 : Initial Time of Simulation
double tsim0 = 0;
// tsim : Final Time of Simulation
double tsim = 20;
// ht : Time Step in Simulation
double ht = 0.01;
// tf : Final Horizon Length
double tf = 1;
// alpha : Parameter for Variable Horizon, T = tf*(1-exp(-alpha*t)) 
double alpha = 0.5;
// zeta : Parameter for Stabilization of Continuation Method
double zeta = 100; 
// x0 : Initial State
double x0[DIMX] = {2, 0};
// u0 : Initial Guess for Initial Control Input and Multipliers 
double u0[DIMUC] = {0.01, 0.02, 0.03};
// *Notice: Dimension of u0 = dimu+dimc
// hdir : Step in the Forward Difference Approximation
double hdir = 0.002;
// rtol=10^(-6) : Tolerance of Error in Initial Control Input and Multipliers, u0
double rtol = 1.e-6;
// kmax : Number of Iteration in GMRES
int kmax = 2;
// dv : Number of Grids on the Horizon
int dv = 10;
// dstep : Step for Saving Data
int dstep =   5;


// fndat : Header of Data Filenames
#define FNMHD  "agex00"

/*-------------- Global Variables Defined by User -------------- */
double  a0 = 1;
double  u1max = 0.5;

double  q[2] = {1, 1};
double  r[2] = {1, 0.01};
double  sf[2] = {1, 1};


/*-------------------- ~ ~ ~ ~ ~ ~ ~ ~ --------------------------*/
// fxu : State Equation, dx/dt = f(x,u,p)
// Cxu : Constraints, C(x,u,p) = 0
// pt : Time-Variant Parameters, p(t)
// L : Integrand in the Performance Index
// phi : Terminal Penalty


/*-------------- dPhi/dx -------------- */
void phix(double t, double x[], double phx1[])
{
    phx1[0] = sf[0]*x[0];
    phx1[1] = sf[1]*x[1]; 
}

/*-------------- State Equation -------------- */
void xpfunc(double t, double x[], double u[], double xprime[])
{
    xprime[0] = x[1];
    xprime[1] = u[0] - 1.*x[0] + (1. - 1.*a0*(pow(x[0],2.) +\
    pow(x[1],2.)))*x[1];
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