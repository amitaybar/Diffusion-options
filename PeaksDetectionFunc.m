function [ListPeaks] = PeaksDetectionFunc(Mat4PeaksDetect)

Mat4PeaksDetect(isnan(Mat4PeaksDetect)) = -10;
TF1                                     = islocalmax(Mat4PeaksDetect,1);
TF2                                     = islocalmax(Mat4PeaksDetect,2);
TFMul                                   = double((TF1.*TF2) > 0);
TFMul(isnan(Mat4PeaksDetect))           = 0.5;
KOptions2DAnd                           = sum(TFMul(:)==1);
% output
ListPeaks                               = find(TFMul(:)==1);   
