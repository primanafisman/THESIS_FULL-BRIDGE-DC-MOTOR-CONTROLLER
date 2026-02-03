% =========================================================================
% SERVO CONTROL INITIALIZATION SCRIPT (PLECS)
% System: DC Motor Full Bridge Driver with Cascade Control
% =========================================================================

clear all; clc;

%% 1. PARAMETER FISIK (PLANT)
%  Sesuaikan nilai ini dengan Datasheet Motor & Komponen Anda
V_batt  = 48;          % Tegangan Suplai Baterai (Volt)
R       = 2.5;         % Resistansi Armature (Ohm)
L       = 1.2e-3;      % Induktansi Armature (Henry) -> 1.2 mH
Fbulk	= 1.0e-3;	   % Bulk Capacitor
Ke      = 0.05;        % Konstanta Back-EMF (V.s/rad)
Kt      = 0.05;        % Konstanta Torsi (Nm/A) -> Biasanya sama dengan Ke
J_load  = 4.5e-3;      % Inersia Beban (Nms^2)
I_max   = 5.0;         % Batas Arus Maksimum (Ampere) untuk proteksi

%% 2. SETTING PWM & DRIVER
f_sw    = 8000;        % Frekuensi Switching PWM (4 kHz)
%Ts      = 1/f_sw;      % Periode Sampling
Ts      = 0.005;
dt_dead = 0.5e-6;      % Deadtime MOSFET (0.5 mikrodetik)

%% 3. CURRENT LOOP TUNING (Inner Loop - PI Controller)
%  Metode: Pole-Zero Cancellation
%  Target Bandwidth: 1/10 dari Frekuensi Switching (Aturan jempol)

f_bw_curr = f_sw / 10;           % Bandwidth dalam Hz
w_bw_curr = 2 * pi * f_bw_curr;  % Bandwidth dalam Rad/s

% Hitung Gain PI Arus
Kp_curr = L * w_bw_curr;         % Proportional Gain
Ki_curr = R * w_bw_curr;         % Integral Gain
BackEmf_Gain = Ke;               % Feedforward Gain (Kompensasi Kecepatan)
Kd_curr = 0;           			 % (BARU) Derivative Current - Set 0
Kf_curr = 100;         			 % (BARU) Filter Coeff Current - Set 100

% Normalisasi Output (Volt ke Duty Cycle)
% Agar output PI (Volt) menjadi range -1 s.d 1
K_norm = 1 / V_batt;

%% 4. POSITION LOOP TUNING (Outer Loop - PID Controller)
%  Metode: IP/PD Standard Design
%  Target Bandwidth: 1/10 dari Bandwidth Current Loop (Cascade Rule)

f_bw_pos = f_bw_curr / 10;       % Bandwidth Posisi lebih lambat dari Arus
w_bw_pos = 2 * pi * f_bw_pos;    % Rad/s

% Hitung Gain PID Posisi
Kp_pos = J_load * (w_bw_pos^2); % Stiffness (Proportional)
Kd_pos = J_load * 2 * w_bw_pos; % Damping (Derivative - Anti Overshoot)
Ki_pos = 0;                      % Integral (Mulai 0 dulu agar stabil)

% Koefisien Filter Derivative (Low Pass Filter pada D)
% Nilai tipikal: 100 s.d 500. Semakin besar = semakin cepat tapi noisy.
Kf_pos = 200;                    

%% 5. SIMULATION TIMING (STEP TIMES)
StepTime_Ref1 = 0.1;    % Waktu step pertama (detik)
Theta_Ref1    = 30;     % Target pertama: 30 derajat
StepTime_Ref2 = 2.5;    % Waktu step kedua (detik)
Theta_Ref2    = -70;    % Delta perubahan: 30 + (-70) = -40 derajat

StepTime_Load1 = 0.5;    % Waktu gangguan beban dimulai
Torque_Load1   = 6.5;      % Besar gangguan torsi (Nm)
StepTime_Load2 = 0.7;    % Waktu gangguan beban dimulai
Torque_Load2   = -6.5;      % Besar gangguan torsi (Nm)

fprintf('Initialization Complete.\n');
fprintf('Kp Current: %.4f | Ki Current: %.4f\n', Kp_curr, Ki_curr);
fprintf('Kp Pos: %.4f | Kd Pos: %.4f\n', Kp_pos, Kd_pos);