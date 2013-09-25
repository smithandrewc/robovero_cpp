robovero_cpp
============

C++ class files that allow access to Robovero resources through the Overo using C++ code

I've included some of the Robovero firmware header files that may be needed to compile.  The files were obtained from https://github.com/robovero/firmware

There is a Robovero_SP.cpp file which is a MATLAB Simulink S-Function.  It will need to be compiled ("mex Robovero_SP.cpp") before you can use it in the supplied RoboveroMDL.mdl Simulink model (create in R2011b).
