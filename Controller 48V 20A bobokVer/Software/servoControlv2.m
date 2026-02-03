% =========================================================================
% ROBUST SERVO CONTROL (FIXED FOR 24V 350W MOTOR)
% Supply: 48V | Motor: 24V | Switching: 8 kHz
% =========================================================================

clear all; clc;

%% 1. PARAMETER FISIK (REALISTIS UNTUK 350W)
V_batt   = 48;          % Sumber Tegangan (Volt)
V_motor  = 24;          % Rating Motor (Volt)

% Motor 350W 24V (Data MY1016 Typical):
% Rated Current ~19A. Stall Current ~80A.
Ra        = 0.35;        % Resistansi (Ohm) - Jauh lebih kecil dari 2.5!
La        = 1.0e-3;      % Induktansi (Henry) - 1mH
Kt       = 0.09;        % Torque Constant (Nm/A)
Ke       = 0.09;        % Back-EMF Constant
J_motor  = 2.0e-4;      % Inersia Rotor
J_load   = 4.5e-3;      % Inersia Beban
J_total  = J_motor + J_load; 

% LIMITASI (PENTING AGAR TIDAK OSILASI)
% Arus dinaikkan agar PID tidak cepat jenuh (saturation)
I_max    = 25.0;        % Limit Arus (Ampere)
Duty_Lim = V_motor / V_batt; % 24/48 = 0.5 (Limit Duty Cycle)

%% 2. SETTING PWM
f_sw    = 8000;         % Frekuensi Switching (8 kHz) sesuai request
Ts      = 1/f_sw;       
dt_dead = 1.0e-6;       % Deadtime sedikit dilonggarkan (1us)

%% 3. CURRENT LOOP TUNING (Inner Loop - Bandwidth 800Hz)
%  Target: Respon Cepat untuk menstabilkan torsi
f_bw_curr = 800;        % 1/10 dari f_sw
w_bw_curr = 2 * pi * f_bw_curr;

Kp_curr = La * w_bw_curr * 1.5;        
Ki_curr = Ra * w_bw_curr * 5.0;        
Kd_curr = 0;            
Kf_curr = 1000;         
BackEmf_Gain = Ke;      
K_norm = 1 / V_batt;    % Scaling Volt ke Duty Cycle

%% 4. SPEED LOOP TUNING (Middle Loop - Bandwidth 40Hz)
%  Target: Cukup lambat agar tidak bertabrakan dengan loop arus
%  Diturunkan dari 80Hz ke 40Hz untuk stabilitas beban berat
f_bw_speed = 20;       
w_bw_speed = 2 * pi * f_bw_speed;

Kp_speed = J_total * w_bw_speed * 1.8;
Ki_speed = J_total * (w_bw_speed^2) * 0.1;
Kd_speed = 0;
Kf_speed = 100;

%% 5. POSITION LOOP TUNING (Outer Loop - Bandwidth 5Hz)
%  Target: Smooth approach, tanpa overshoot.
Kp_pos = 2;             % Mulai dari 8. Bisa dinaikkan ke 10-12 nanti.

%% 6. SKENARIO SIMULASI
StepTime_Ref1 = 0.1;
Theta_Ref1    = 30;     % Target 30 derajat
StepTime_Ref2 = 3.0;    % Beri waktu cukup untuk settling
Theta_Ref2    = -70;    % Target balik arah

StepTime_Load1 = 1.0;
Torque_Load1   = 2.0;   % Beban 2Nm (Cukup berat untuk start)
StepTime_Load2 = 2.0;
Torque_Load2   = -2.0;

%% 7. KONVERSI
Rad2Deg = 180/pi;
Deg2Rad = pi/180;

fprintf('=== TUNING UPDATED (8kHz) ===\n');
fprintf('Kp Curr: %.4f | Ki Curr: %.4f\n', Kp_curr, Ki_curr);
fprintf('Kp Spd : %.4f | Ki Spd : %.4f\n', Kp_speed, Ki_speed);
fprintf('Kp Pos : %.4f\n', Kp_pos);