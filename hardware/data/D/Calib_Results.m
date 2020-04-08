% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 233.113413923451280 ; 232.848020471331380 ];

%-- Principal point:
cc = [ 159.834501673978850 ; 109.300375957580510 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.464675870854515 ; 0.446628781889631 ; 0.016197802817698 ; 0.003253516138597 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 7.438074317352637 ; 7.514504996194616 ];

%-- Principal point uncertainty:
cc_error = [ 6.567445251987023 ; 6.779816600262848 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.061608757622706 ; 0.165229860098092 ; 0.006127313077350 ; 0.003674508414723 ; 0.000000000000000 ];

%-- Image size:
nx = 320;
ny = 240;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 6;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.074048e+00 ; 2.034325e+00 ; -4.958153e-01 ];
Tc_1  = [ -9.424757e+01 ; -8.981128e+01 ; 4.119934e+02 ];
omc_error_1 = [ 2.306864e-02 ; 2.895357e-02 ; 5.359005e-02 ];
Tc_error_1  = [ 1.185046e+01 ; 1.199891e+01 ; 1.232320e+01 ];

%-- Image #2:
omc_2 = [ -1.828013e+00 ; -1.657505e+00 ; -1.535119e-01 ];
Tc_2  = [ -1.133774e+02 ; -9.941769e+01 ; 3.209449e+02 ];
omc_error_2 = [ 2.294270e-02 ; 2.167349e-02 ; 3.573035e-02 ];
Tc_error_2  = [ 9.297989e+00 ; 9.660679e+00 ; 1.070401e+01 ];

%-- Image #3:
omc_3 = [ -2.145807e+00 ; -2.141977e+00 ; -1.062901e-02 ];
Tc_3  = [ -1.199533e+02 ; -9.562540e+01 ; 3.236169e+02 ];
omc_error_3 = [ 2.563253e-02 ; 2.476697e-02 ; 5.386042e-02 ];
Tc_error_3  = [ 9.332758e+00 ; 9.726630e+00 ; 1.152609e+01 ];

%-- Image #4:
omc_4 = [ -1.929185e+00 ; -1.902720e+00 ; 3.396112e-01 ];
Tc_4  = [ -9.779211e+01 ; -1.030250e+02 ; 3.335427e+02 ];
omc_error_4 = [ 2.482680e-02 ; 1.870671e-02 ; 4.213906e-02 ];
Tc_error_4  = [ 9.664645e+00 ; 9.764878e+00 ; 1.011395e+01 ];

%-- Image #5:
omc_5 = [ 2.153370e+00 ; 2.064205e+00 ; 4.112915e-01 ];
Tc_5  = [ -1.413800e+02 ; -1.010770e+02 ; 2.600384e+02 ];
omc_error_5 = [ 2.292708e-02 ; 2.312933e-02 ; 4.395214e-02 ];
Tc_error_5  = [ 8.184594e+00 ; 8.155952e+00 ; 9.894180e+00 ];

%-- Image #6:
omc_6 = [ -1.919036e+00 ; -1.797502e+00 ; -5.734823e-01 ];
Tc_6  = [ -1.176578e+02 ; -7.186137e+01 ; 2.383411e+02 ];
omc_error_6 = [ 2.022537e-02 ; 2.473541e-02 ; 3.585365e-02 ];
Tc_error_6  = [ 7.030633e+00 ; 7.373139e+00 ; 9.287658e+00 ];

