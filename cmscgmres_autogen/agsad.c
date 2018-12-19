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
double ht = 0.001;
double tf = 1;
double alpha = 0.5;
double zeta = 1000; 
double x0[DIMX] = {2, 0};
double u0[DIMUC] = {0.01, 0.02, 0.03};
double hdir = 0.002;
double rtol = 1.e-6;
int kmax = 5;
int dv = 50;
int dstep =   5;
#define FNMHD  "agsad00"

/*-------------- Global Variables Defined by User -------------- */
double  a = -1;
double  b = -1;
double  umax = 1;

double  q[2] = {1, 10};
double  r[2] = {1, 0.01};
double  sf[2] = {1, 10};


/*------------------------------------------------------------*/



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
    xprime[1] = a*x[0] + b*u[0]*x[1];
}

/*-------------- Costate Equation -------------- */
void lpfunc(double t, double lmd[], double linp[], double lprime[])
{
    double x[DIMX], u[DIMUC];

    mmov(1,DIMX, linp, x);
    mmov(1,DIMUC, linp+DIMX, u);
    lprime[0] = -1.*a*lmd[1] - 1.*q[0]*x[0];
    lprime[1] = -1.*lmd[0] - 1.*b*lmd[1]*u[0] - 1.*q[1]*x[1];
}

/*-------------- Error in Optimality Condition, Hu -------------- */
void hufunc(double t, double x[], double lmd[], double u[], double hui[])
{
    double o[19];

    o[0] = pow(u[1],2.);
    o[1] = o[0]*r[0];
    o[2] = 2.*o[1];
    o[3] = pow(umax,2.);
    o[4] = umax*u[0];
    o[5] = -4.*o[4];
    o[6] = pow(u[0],2.);
    o[7] = o[0] + o[6];
    o[8] = 4.*o[7];
    o[9] = o[3] + o[5] + o[8];
    o[10] = o[9]*u[2];
    o[11] = o[2] + o[10];
    o[12] = 1/o[11];
    o[13] = r[1]*u[1];
    o[14] = o[6]*u[2];
    o[15] = 2.*o[14];
    o[16] = o[0]*u[2];
    o[17] = 2.*o[16];
    o[18] = b*lmd[1]*u[0]*x[1];
    hui[0] = o[12]*(o[3]*u[0]*u[2] - 1.*umax*(o[13] + (o[0] + 3.*o[6])*u[2])\
 
  \
+ 2.*(o[0]*r[0]*u[0] + r[1]*u[0]*u[1] + pow(u[0],3.)*u[2] + o[0]*u[0]*u[2] +\
 b*lmd[1]*o[0]*x[1]));
    hui[1] = (-1.*o[3]*r[1] - 4.*o[6]*r[1] + 2.*(o[1] + o[15] + o[17] -\
 
  2.*o[\
18] - 1.*o[6]*r[0])*u[1] + 2.*umax*(2.*r[1]*u[0] + u[1]*(-2.*u[0]*u[2] + b*l\
md[1]*x[1])))/(4.*o[1] + 2.*o[10]);
    hui[2] = o[12]*(r[0]*(-1.*o[13] + o[7]*u[2]) + u[2]*(-2.*o[13] + o[15] +\
 
  \
o[17] + 2.*o[18] + o[3]*u[2] - 2.*umax*u[0]*u[2] - 1.*b*umax*lmd[1]*x[1]));
}

/*-------------- Save Simulation Conditions -------------- */
void final(FILE *fp, float t_cpu, float t_s2e)
{
    int i;
    fprintf(fp, "%% Simulation Result by agsad.c \n");
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

    fprintf(fp, "a = %g \n", (float)a );
    fprintf(fp, "b = %g \n", (float)b );
    fprintf(fp, "umax = %g \n", (float)umax );


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