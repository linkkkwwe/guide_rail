%% =====================================================================
%  simulate_guide_rail.m — Guide_rail 工程的 MATLAB 仿真
%  ---------------------------------------------------------------------
%  复刻固件控制逻辑（D:\mxproject\Guide_rail）：
%    [水平轴] trajectory.c 三角波 → Motor.c 单环角度 PID → 电机模型
%    [Yaw]    匀速模式：速度环直接跟踪 60rpm（Motor.c 的 spin 分支）
%  仿真步长 = 固件控制周期 1ms（TIM6 节拍），PID 写法与 pid.c 一致。
%
%  用法：MATLAB 里 cd 到 simulation 文件夹，命令行输入：
%        >> simulate_guide_rail
%  只需要基础 MATLAB，不需要任何工具箱。
%  =====================================================================
clear; clc; close all;

%% ---------- 1. 公共参数 ----------
dt    = 1e-3;             % 控制周期 1ms（对应 TIM6 中断节拍）
T_end = 6;                % 仿真总时长（秒）
N     = round(T_end/dt);
t     = (0:N-1)' * dt;

% ---- 电机被控对象（仿真假设）----
% 把电机近似成一阶惯性环节：tau·d(speed)/dt + speed = Kv·u
% Kv ：稳态增益（满指令 30000 → 320rpm，按 GM6020 空载估计）
% tau：时间常数（电气+机械综合，估计值；实物需实测修正）
Kv  = 320/30000;          % (rpm) / (指令)
tau = 0.08;               % 秒

%% ---------- 2. 水平轴：三角波 + 单环角度 PID ----------
% 参数与 Motor.h 的 HORIZONTAL_* 宏一一对应
Kp_a    = 20.0;           % HORIZONTAL_ANGLE_KP
Ki_a    = 0.0;            % HORIZONTAL_ANGLE_KI
Kd_a    = 0.0;            % HORIZONTAL_ANGLE_KD
maxo_a  = 3000.0;         % HORIZONTAL_ANGLE_MAX_OUT（输出当电压指令）
maxio_a = 5000.0;         % HORIZONTAL_ANGLE_MAX_IOUT

A     = 30.0;             % TRAJECTORY_AMPLITUDE  ±30°
T_per = 2.0;              % TRAJECTORY_PERIOD_MS  2000ms → 2s

% ---- 三角波目标轨迹（复刻 trajectory.c 的 triangle_angle）----
target = zeros(N,1);
for k = 1:N
    ph = mod(t(k), T_per)/T_per;
    if ph < 0.25
        target(k) = 4*A*ph;                    % 0°  → +30°
    elseif ph < 0.75
        target(k) = 2*A - 4*A*ph;              % +30° → -30°
    else
        target(k) = -4*A + 4*A*ph;             % -30° → 0°
    end
end

% ---- 逐拍闭环（复刻 pid.c 的位置式 PID + 一阶电机）----
angle = 0; speed = 0; Iout = 0; e_prev = 0;
act = zeros(N,1); spd = zeros(N,1); err = zeros(N,1);
for k = 1:N
    e    = target(k) - angle;                  % error[0] = set - ref
    Pout = Kp_a * e;
    Iout = Iout + Ki_a * e;                    % Iout += Ki*e
    Iout = max(min(Iout, maxio_a), -maxio_a);   % 积分限幅
    Dout = Kd_a * (e - e_prev);                % 无滤波微分
    e_prev = e;
    u    = Pout + Iout + Dout;
    u    = max(min(u, maxo_a), -maxo_a);        % 输出限幅 → 电压指令

    speed = speed + dt/tau*(Kv*u - speed);      % 一阶电机（欧拉离散）
    angle = angle + speed*6*dt;                 % rpm → deg/s → deg（×360/60=×6）

    act(k) = angle; spd(k) = speed; err(k) = e;
end

figure('Color','w','Name','水平轴仿真');
subplot(3,1,1);
plot(t, target, 'b--', t, act, 'r-', 'LineWidth', 1.2); grid on;
legend('目标角度','实际角度','Location','best');
ylabel('角度 / °'); title('水平轴：三角波轨迹跟踪');
subplot(3,1,2);
plot(t, err, 'LineWidth', 1.0); grid on;
ylabel('误差 / °'); title('跟踪误差（目标 - 实际）');
subplot(3,1,3);
plot(t, spd, 'LineWidth', 1.0); grid on;
xlabel('t / s'); ylabel('转速 / rpm'); title('电机转速');

fprintf('水平轴：最大跟踪误差 %.2f°，后半程平均误差 %.2f°\n', ...
        max(abs(err)), mean(abs(err(round(N/2):end))));

%% ---------- 3. Yaw：匀速模式（速度环跟踪 60rpm） ----------
Kp_s    = 1000.0;         % YAW_SPEED_KP
Ki_s    = 1.0;            % YAW_SPEED_KI
maxo_s  = 30000.0;        % YAW_SPEED_MAX_OUT
maxio_s = 5000.0;         % YAW_SPEED_MAX_IOUT
spin    = 60.0;            % YAW_SPIN_SPEED_RPM

sy = 0; Iout_s = 0;
speed_y = zeros(N,1);
for k = 1:N
    e      = spin - sy;                        % 速度环误差
    Iout_s = Iout_s + Ki_s*e;
    Iout_s = max(min(Iout_s, maxio_s), -maxio_s);
    u      = max(min(Kp_s*e + Iout_s, maxo_s), -maxo_s);

    sy = sy + dt/tau*(Kv*u - sy);
    speed_y(k) = sy;
end

figure('Color','w','Name','Yaw 匀速模式仿真');
plot(t, spin*ones(N,1), 'b--', t, speed_y, 'r-', 'LineWidth', 1.2); grid on;
legend('目标转速','实际转速','Location','best');
xlabel('t / s'); ylabel('rpm'); title('Yaw 匀速模式：60rpm 阶跃响应');

%% ---------- 4. 怎么用这个仿真调参数 ----------
% 1) 水平轴 Kp=20 偏小：三角波约需 20rpm 扫摆速度，Kp=20 产生的电压
%    指令远不够，实际角度只能跟到目标的一小部分（幅值严重衰减）。
%    这正是仿真的价值——不上机就能发现参数偏小。
%    试着把 Kp_a 改成 100 / 300 / 800，对比"实际角度"曲线的变化。
% 2) 电机模型 Kv / tau 是估计值：换电机或实测后改这两个数即可。
% 3) 改 A / T_per 可模拟不同摆幅、不同扫摆速度。