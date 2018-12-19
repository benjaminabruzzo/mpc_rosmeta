(* ::Package:: *)

(*-------- Define dimensions of x, u, C(x,u), and p(t). --------*)
dimx=2;
dimu=2;
dimc=1;
dimp=0;

(*-------- Do not touch the following difinition of vectors. --------*)
xv=Array[x,dimx];
lmdv=Array[lmd,dimx];
uv=Array[u,dimu];
muv=Array[u,dimc,dimu+1];
pv=Array[p,dimp];

(*-------- Define f(x,u,p), C(x,u,p), p(t), L(x,u,p) and phi(x,p). --------*)
(* f(x,u,p) : non-linear state equations *)
fxu = {x[2], (1 - a0 * (x[1]^2+x[2]^2) ) * x[2] - x[1] + u[1]};
(* C(x,u,p) : equality contraint vector *)
Cxu = {uv.uv/2 - u1max^2/2};

pt={};

qv = Array[q,dimx];
rv = Array[r,dimu];
sfv = Array[sf,dimx];
Q = DiagonalMatrix[qv];
Sf = DiagonalMatrix[sfv];

L = xv.Q.xv/2 + r[1]*u[1]^2/2 - r[2]*u[2];
phi = xv.Sf.xv/2;

(*-------- Define user's variables and arrays. --------*)
(*--- Numbers must be in Mathematica format. ---*)
MyVarNames={"a0", "u1max"};
MyVarValues={1, 0.5};
MyArrNames={"q", "r", "sf"};
MyArrDims={dimx, dimu, dimx};
MyArrValues={{1,1}, {1,0.01}, {1,1}};

(*-------- Define simulation conditions. --------*)
(*--- Real Numbers ---*)
tsim0=0;
tsim=20;
tf=1;
ht=0.01;
alpha=0.5;
zeta=100;
x0={2,0};
u0={0.01, 0.02, 0.03};
hdir=0.002;
rtol=10^(-6);
(*--- Integers ---*)
kmax = 2;
dv=10;
dstep=5;
(*--- Strings ---*)
outfn="agex";
fndat="agex00";

(*------------------------------------------------------------------ 
Define SimplifyLevel. 
If SimplifyLevel=0, then Simplify[] is not used. 
If SimplifyLevel>0, then Simplify[] is used. 
The larger SimplifyLevel is, the more expressions are simplified. 
------------------------------------------------------------------*)
SimplifyLevel=3;

(*------------------------------------------------------------------ 
Define Precondition.
If Precondition=0, no preconditioning for a linear equation in the algorithm.
If Precondition=1, preconditioning by the Hessian of the Hamiltonian. 
------------------------------------------------------------------*)
Precondition = 1;
