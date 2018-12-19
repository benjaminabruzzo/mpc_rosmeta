#include "rhfuncu.c"

/*#define HDIR_EQ_HT */ 
/*#define RESET_DU */ 
/*#define ADAMS */
/*#define TRACE_ON */
<* If[dimp>0,"#define P_IS_USED\n",""]*>

/*-------------- Dimensions -------------- */
#define DIMX   <*dimx*>
#define DIMUC  <*dimuc*>
#define DIMP   <*dimp*>

/*-------------- Global Variables -------------- */
double tsim0 = <*CForm[tsim0]*>;
double tsim = <*CForm[tsim]*>;
double ht = <*CForm[ht]*>;
double tf = <*CForm[tf]*>;
double alpha = <*CForm[alpha]*>;
double zeta = <*CForm[zeta]*>; 
double x0[DIMX] = <*Map[CForm,x0]*>;
double u0[DIMUC] = <*Map[CForm,u0]*>;
double hdir = <*CForm[hdir]*>;
double rtol = <*CForm[rtol]*>;
int kmax = <*kmax*>;
int dv = <*dv*>;
int dstep =   <*dstep*>;
#define FNMHD  <*CForm[fndat]*>

/*-------------- Global Variables Defined by User -------------- */
<*st="";
Do[st=st<>"double  "<>MyVarNames[[i]]<>" = "<>
      ToString[ Map[CForm,MyVarValues[[i]]] ] <>";\n",{i,Length[MyVarNames]}];
st *>
<* st="";
Do[st=st<>"double  "<>MyArrNames[[i]]<>"["<>ToString[MyArrDims[[i]]]<>
      "] = "<>ToString[ Map[CForm,MyArrValues[[i]]] ] <>";\n",{i,Length[MyArrNames]}];
st *>

/*------------------------------------------------------------*/

<* If[dimp > 0, 
  ou = "/*-------------- Time-Variant Parameters --------------*/\n";
  ou = ou <> "void pfunc(double t, double p1[])\n{\n";
  If[lept != 0, ou = ou <> "    double o[" <> ToString[lept] <> "];\n\n"]; 
  ou = ou <> ToString[CAssign[p1, pt]] <> "\n}", ou=""]; ou *>

/*-------------- dPhi/dx -------------- */
void phix(double t, double x[], double phx1[])
{
<*If[leph!=0,ou="    double o["<>ToString[leph]<>"];\n",ou=""];
  If[ !(FreeQ[phix,p[_]]),
      ou=ou<>"    double p[DIMP];\n\n    pfunc(t, p);\n"];ou *>
<*CAssign[phx1,phix]*> 
}

/*-------------- State Equation -------------- */
void xpfunc(double t, double x[], double u[], double xprime[])
{
<*If[lexp!=0,ou="    double o["<>ToString[lexp]<>"];\n",ou=""];
  If[!(FreeQ[fxu,p[_]]),
      ou=ou<>"    double p[DIMP];\n\n    pfunc(t, p);\n"];ou*>
<*CAssign[xprime,fxu]*>
}

/*-------------- Costate Equation -------------- */
void lpfunc(double t, double lmd[], double linp[], double lprime[])
{
    double x[DIMX], u[DIMUC];
<*If[lelp!=0,ou="    double o["<>ToString[lelp]<>"];\n",ou=""];
  If[!(FreeQ[dlmddt,p[_]]),
      ou=ou<>"    double p[DIMP];\n\n    pfunc(t, p);\n"];ou*>
    mmov(1,DIMX, linp, x);
    mmov(1,DIMUC, linp+DIMX, u);
<*CAssign[lprime,dlmddt]*>
}

/*-------------- Error in Optimality Condition, Hu -------------- */
void hufunc(double t, double x[], double lmd[], double u[], double hui[])
{
<*If[lehu!=0,ou="    double o["<>ToString[lehu]<>"];\n",ou=""];
  If[!(FreeQ[Hu,p[_]]),
      ou=ou<>"    double p[DIMP];\n\n    pfunc(t, p);\n"];ou*>
<*CAssign[hui,Hu]*>
}

/*-------------- Save Simulation Conditions -------------- */
void final(FILE *fp, float t_cpu, float t_s2e)
{
    int i;
    fprintf(fp, "%% Simulation Result by <*outfile*> \n");
    fprintf(fp, "Precondition = <*Precondition*> \n");
    fprintf(fp, "tsim = %g\n", tsim);
    fprintf(fp, "ht = %g, dstep = %d\n", ht, dstep);
    fprintf(fp, "tf = %g, dv = %d, alpha = %g, zeta = %g \n", (float)tf, dv, (float)alpha, (float)zeta);
    fprintf(fp, "hdir = %g, rtol = %g, kmax = %d \n", (float)hdir, (float)rtol, kmax);
    fprintf(fp, "u0 = [%g", (float)u0[0] );
    for(i=1; i<DIMUC; i++)
        fprintf(fp, ",%g", (float)u0[i] );
    fprintf(fp, "]\n" );
    fprintf(fp, "t_cpu = %g, t_s2e = %g  %% [sec] \n", t_cpu, t_s2e);
<*st="\n";
      Do[st=st<>"    fprintf(fp, \""<>MyVarNames[[i]]<>" = %g \\n\", (float)"<>MyVarNames[[i]]<>" );\n",{i,Length[MyVarNames]}]
st*>
<*st="";
      Do[st=st<>"\n    fprintf(fp, \""<>MyArrNames[[i]]<>" = [%g\", (float)"<>MyArrNames[[i]]<>"[0] );\n"<>"    for(i=1; i<"<>ToString[MyArrDims[[i]]]<>"; i++)\n"<>"        fprintf(fp, \",%g\", (float)"<>MyArrNames[[i]]<>"[i] );\n"<>"    fprintf(fp, \"]\\n\" );\n",{i,Length[MyArrNames]}]
st*>
}

#include "rhmainu.c"