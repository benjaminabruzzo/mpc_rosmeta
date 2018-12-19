AutoGenU : An Automatic Code Generation System for Nonlinear Receding Horizon Control

by Toshiyuki Ohtsuka



---------- Introduction ----------

AutoGenU is a Mathematica program to generate automatically simulation programs for nonlinear receding horizon control. Once the state equation, the performance index and some other simulation conditions are given by a user as an input file in Mathematica Format, then AutoGenU.nb loads the input file, executes such necessary operations as partial differentiation, and generates a C source file. The generated source file is ready for compilation and execution. The simulation program employs a fast optimization algorithm, C/GMRES (Continuation/Generalized Minimum Residual). 



---------- Files ----------

AutoGenU.zip consists of the following files.

inputEX.m : Example of Input File written by User (in Mathematica Format)
inputSAD.m : Another Example of Input File for a Semi-Active Damper  AutoGenU.nb : Mathematica Notebook to Generate C Code
AutoGenU.mc : Template of C Source File
rhfuncu.c : C Source File (Common Functions)
rhmainu.c : C Source File (Main Function)
agex.c : Example of C Source File Generated from inputEX.m by AutoGenU
agsad.c : Example of C Source File Generated from inputSAD.m by AutoGenU
plotsim.m : Matlab M-File to Plot a Simulation Result
Format2.m : Modified Format Package (for Mathematica ver. 8 or earlier)
Format3.m : Modified Format Package (for Mathematica ver. 9)
Optimize2.m : Modified Optimize Package
Readme.txt : This file


AutoGenU.zip is available at:
http://www.symlab.sys.i.kyoto-u.ac.jp/~ohtsuka/code/index.htm

The packages Format and Optimize are originally developed by M. Sofroniou and are available, e.g., at the following URL. 
Format: 
http://library.wolfram.com/infocenter/MathSource/60/

Optimize: 
http://library.wolfram.com/infocenter/MathSource/3947/

*Notice: In the modified packages, Format2.m and Optimize2.m, a command "NProtectedAll" in the original packages, Format.m and Optimize.m, are replaced with "NHoldAll". Save Format2.m and Optimize2.m into a directory, "AddOns/Applications" under the Mathematica directory. They may work when placed in the same directory as AutoGenU.nb. Format3.m is a modified version of Format2.m for Mathematica ver. 9, in which "FilterOptions" command is replaced with "FilterRules" command. 



---------- Usage ----------

First, edit the input file, e.g, inputEX.m, with a text editor. Note that numbers in the input file must be in such Mathematica format as "2*10^(-4)" rather than "2e-4". In AutoGenU.nb, specify the directory and filename of the input file. Then, execute all cells in AutoGenU.nb. If AutoGenU.nb generates a C source file with a name specified in the input file, then, compile it with a C compiler. For example, if the generated file is "agex.c" and you use gcc, then "gcc agex.c" will generate an executable file. You do not have to compile rhfuncu.c and rhmainu.c, since they are included  automatically as source files. Simulation program generates data files with a header specified in the input file. A file "*x.m" saves the state vector, "*u.m" the control input, "*e.m" the error, "*p.m" the time-variant parameters if they are given, and "*c.m" simulation conditions. They can be plotted with the Matlab M-file, plotsim.m. 

If you have any problem in generating the C code, try a step-by-step (i.e., cell-by-cell) execution of AutoGenU.nb and modify the input file (or AutoGenU.nb). If you have any problem in compiling the generated source file, browse the source file and debug the input file (or AutoGenU.nb). You may debug the generated source file itself. In that case, the same problem may occur again when you generate a new source file. 

The present algorithm, C/GMRES, for nonlinear receding horizon control is more numerically robust than the algorithm used in the previous version, AutoGen. However, it is a numerical algorithm anyway and can fail depending on the simulation conditions. You should first try a small horizon, small weights in the performance index, and an initial state near the objective state. A large value of kmax (number of iterations in GMRES) may also be necessary for an accurate solution. An appropriate value of the parameter for stabilization of the solution, zeta, would be about 1/ht. The simulation conditions are defined at the beginning of the generated source file. Good luck! 



---------- Variables in the Input File ----------

dimx : Dimension of the State Vector
dimu : Dimension of the Control Input Vector
dimc : Number of Constraints
dimp : Dimension of Time-Variant Parameters

xv : State Vector {x[1], ..., x[dimx]}
lmdv : Costate Vector {lmd[1], ..., lmd[dimx]}
uv : Control Input Vector {u[1], ..., u[dimu]}
muv : Multiplier Vector for Constraints {u[dimu+1], ..., u[dimu+dimc]}
pv : Vector of Time-Variant Parameters {p[1], ..., p[dimp]}

*Notice: Multipliers are also denoted by u[] and are saved in *u.m. 

fxu : State Equation, dx/dt = f(x,u,p)
Cxu : Constraints, C(x,u,p) = 0
pt : Time-Variant Parameters, p(t)
L : Integrand in the Performance Index
phi : Terminal Penalty

*Notice: User's variables and arrays may be used in fxu, Cxu, pt, L, and phi. 

MyVarNames : List of Variable Names Defined by User
MyVarValues : List of Values for User's Variables
MyArrNames : List of Array Names Defined by User
MyArrDims : List of Dimensions of User's Arrays
MyArrValues : List of Values for User's Arrays

tsim0 : Initial Time of Simulation
tsim : Final Time of Simulation
ht : Time Step in Simulation
tf : Final Horizon Length
alpha : Parameter for Variable Horizon, T = tf*(1-exp(-alpha*t)) 
zeta : Parameter for Stabilization of Continuation Method
x0 : Initial State
u0 : Initial Guess for Initial Control Input and Multipliers 
*Notice: Dimension of u0 = dimu+dimc
hdir : Step in the Forward Difference Approximation
rtol=10^(-6) : Tolerance of Error in Initial Control Input and Multipliers, u0

kmax : Number of Iteration in GMRES
dv : Number of Grids on the Horizon
dstep : Step for Saving Data

outfn : Filename of C Source File
fndat : Header of Data Filenames

SimplifyLevel : Specification of Simplify[]
SimplifyLevel = 0 : Simplify[] is not used in AutoGenU.
SimplifyLevel > 0 : H_x, H_u (H is the Hamiltonian), and phi_x are simplified.
*Notice: If H or phi are too complicated, Simplify may take too much time or fail. In such a case, SimplifyLevel should be set zero. 

Precondition : Specification of Preconditioning
Precondition=0 : no preconditioning for a linear equation in the algorithm.
Precondition=1 : preconditioning by the Hessian of the Hamiltonian. 
*Notice: In most problems, GMRES converges faster with preconditioning. However, if the Hessian of the Hamiltonian is singular or nearly singular, 



---------- References ----------

Ohtsuka, T., "Continuation/GMRES Method for Fast Algorithm of Nonlinear Receding Horizon Control," Proceedings of the 39th Conference on Decision and Control, Sydney, Australia, Dec. 2000, pp. 766-771. 

Ohtsuka, T., ÅgA Continuation/GMRES Method for Fast Computation of Nonlinear Receding Horizon Control,Åh Automatica, Vol. 40, No. 4, Apr. 2004, pp. 563-574. DOI:10.1016/j.automatica.2003.11.005 

Ohtsuka, T., and Kodama, A., "Automatic Code Generation System for Nonlinear Receding Horizon Control," Transactions of the Society of Instrument and Control Engineers, Vol. 38, No. 7, July 2002, pp. 617-623. 

Ohtsuka, T., "On a Computational Method for Nonlinear Receding Horizon Control," Journal of the Society of Instrument and Control Engineers, Vol. 41, No. 5, 2002, pp. 366-371 (in Japanese).

Ohtsuka, T. Introduction to Nonlinear Optimal Control, Corona Publishing, 2011 (in Japanese). 

Ohtsuka, T. (Ed.), Practical Applications of Control by Real-Time Optimizaion, Corona Publishing, 2015 (in Japanese).

*Notice: A Maple version of AutoGenU is also available at: 
http://www.maplesoft.com/applications/view.aspx?SID=153555



---------- Acknowledgments ----------

This research is partially supported by Saneyoshi Scholarship Foundation, Kurata Foundation, and a Grant-in-Aid for Scientific Research from Ministry of Education, Japan (No. 11750178). 



---------- History ----------

Oct. 1997 : Prototype of AutoGen by T. Ohtsuka
Feb. 2000 : Modified Version of AutoGen by A. Kodama (Format and Optimize are utilized.) 
March 2000 : AutoGenU by T. Ohtsuka (on Mathematica 3.0)
Aug. 2000 : Minor Modification (Directory of the packages are changed.) (on Mathematica 4)
April 2013 : Minor Update for Mathematica ver. 9
Nov. 2014 : Additional Example of a Semi-Active Damper



-------------------------------------------
Toshiyuki Ohtsuka
Department of Systems Science
Graduate School of Informatics
Kyoto University
Email ohtsuka@i.kyoto-u.ac.jp 
Web http://www.symlab.sys.i.kyoto-u.ac.jp/~ohtsuka/
-------------------------------------------
