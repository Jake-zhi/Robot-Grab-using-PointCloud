% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 295.748545256181100 ; 295.700963918975730 ];

%-- Principal point:
cc = [ 294.567332643417900 ; 263.307268858534600 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.045669022021119 ; -0.031601287527858 ; -0.001899316264961 ; -0.000537381026652 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.743539561018365 ; 1.725219454525061 ];

%-- Principal point uncertainty:
cc_error = [ 2.017090539678981 ; 1.591269790978058 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013362194234267 ; 0.034412445861803 ; 0.001850848836250 ; 0.002577993869433 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


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
omc_1 = [ -2.148703e+00 ; -2.155304e+00 ; 3.341866e-02 ];
Tc_1  = [ -1.249957e+02 ; -5.922740e+01 ; 3.161607e+02 ];
omc_error_1 = [ 4.667025e-03 ; 4.980683e-03 ; 1.010044e-02 ];
Tc_error_1  = [ 2.170397e+00 ; 1.740817e+00 ; 2.180336e+00 ];

%-- Image #2:
omc_2 = [ -1.917590e+00 ; -1.905213e+00 ; 3.470257e-01 ];
Tc_2  = [ -1.026959e+02 ; -6.655674e+01 ; 3.244415e+02 ];
omc_error_2 = [ 5.006319e-03 ; 4.888655e-03 ; 7.889491e-03 ];
Tc_error_2  = [ 2.208711e+00 ; 1.748094e+00 ; 1.876168e+00 ];

%-- Image #3:
omc_3 = [ 2.153551e+00 ; 2.063105e+00 ; 4.079400e-01 ];
Tc_3  = [ -1.472663e+02 ; -6.395837e+01 ; 2.468929e+02 ];
omc_error_3 = [ 4.856266e-03 ; 4.267239e-03 ; 9.681986e-03 ];
Tc_error_3  = [ 1.784232e+00 ; 1.439387e+00 ; 1.939042e+00 ];

%-- Image #4:
omc_4 = [ -1.923118e+00 ; -1.795307e+00 ; -5.498453e-01 ];
Tc_4  = [ -1.238265e+02 ; -3.569060e+01 ; 2.287719e+02 ];
omc_error_4 = [ 3.720137e-03 ; 5.487435e-03 ; 8.399548e-03 ];
Tc_error_4  = [ 1.573843e+00 ; 1.289155e+00 ; 1.803554e+00 ];

%-- Image #5:
omc_5 = [ 2.057200e+00 ; 2.045323e+00 ; -5.364490e-01 ];
Tc_5  = [ -9.640515e+01 ; -5.442888e+01 ; 4.035401e+02 ];
omc_error_5 = [ 4.482781e-03 ; 5.402298e-03 ; 1.065331e-02 ];
Tc_error_5  = [ 2.745493e+00 ; 2.155480e+00 ; 2.279549e+00 ];

%-- Image #6:
omc_6 = [ -1.815956e+00 ; -1.650137e+00 ; -1.490445e-01 ];
Tc_6  = [ -1.191066e+02 ; -6.198379e+01 ; 3.093928e+02 ];
omc_error_6 = [ 4.361912e-03 ; 5.380296e-03 ; 7.595708e-03 ];
Tc_error_6  = [ 2.119445e+00 ; 1.705730e+00 ; 2.014215e+00 ];

