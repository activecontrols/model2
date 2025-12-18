cd cpp_mex
% disable warning about float literals being converted to doubles
mex COMPFLAGS="$COMPFLAGS /wd4305" cpp_controller_mex_wrap.cpp ASTRAv2_Controller.cpp controller.cpp DigitalNF.cpp EstimateStateFCN.cpp matlab_helpers.cpp -I"eigen"
cd ..