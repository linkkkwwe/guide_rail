function tune_yaw_speed()
% =====================================================================
%  tune_yaw_speed.m — Yaw 电机速度环图形化调参工具
%  ---------------------------------------------------------------------
%  Yaw 匀速模式（Motor.c 第 132-146 行）跳过角度环，速度环直接跟踪
%  spin_speed_rpm。所以本工具只调速度环 PID（YAW_SPEED_KP/KI/KD）。
%
%  用法：MATLAB 中 cd 到 simulation 文件夹，运行：
%        >> tune_yaw_speed
%  阶跃输入：0 → 60 rpm，观察速度响应曲线。
%  调好后把 Kp/Ki/Kd 写回 Motor.h 的 YAW_SPEED_* 宏。
%
%  注意：仿真复刻固件 pid.c（仅 Iout 限幅，无条件积分抗饱和）。
%  若超调大且 Ki 引起 windup，可考虑给 pid.c 加条件积分。
%  =====================================================================

% ---------- 默认参数（与固件一致） ----------
S.Kp    = 1000;     % YAW_SPEED_KP
S.Ki    = 1.0;      % YAW_SPEED_KI
S.Kd    = 0;        % YAW_SPEED_KD
S.tau   = 0.08;     % 电机时间常数 s（实测后修正）
S.KvMul = 1.0;      % 电机增益倍率
S.Spin  = 60;       % YAW_SPIN_SPEED_RPM（目标转速）

MAXOUT  = 30000;         % YAW_SPEED_MAX_OUT
MAXIOUT = 5000;          % YAW_SPEED_MAX_IOUT
KV0     = 320/30000;     % 满指令→320rpm
dt = 1e-3; N = 3000; t = (0:N-1)'*dt;   % 仿真 3s，1ms 步长

% ---------- 界面 ----------
fig = figure('Color','w','Position',[60 60 1080 680], ...
    'Name','Guide_rail Yaw 速度环调参','NumberTitle','off', ...
    'MenuBar','none','ToolBar','none');

ax1 = axes('Parent',fig,'Units','pixels','Position',[50 400 640 230]);
ax2 = axes('Parent',fig,'Units','pixels','Position',[50 105 640 230]);

uicontrol(fig,'Style','text','String','拖滑块调参，曲线实时刷新；调完写回 Motor.h 的 YAW_SPEED_*', ...
    'Position',[50 645 640 20],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');

uicontrol(fig,'Style','text','String','速度环 PID（写回 Motor.h）', ...
    'Position',[740 640 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');

S.hval = struct();
mkrow('Kp','Kp',[0 140],33.33*log10(1000),612);
mkrow('Ki','Ki',[0 50],1.0,568);
mkrow('Kd','Kd',[0 10000],0,524);

uicontrol(fig,'Style','text','String','电机模型（实测标定后改）', ...
    'Position',[740 484 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');
mkrow('tau','时间常数 tau',[0.02 0.30],0.08,452);
mkrow('Kv','增益倍率',[0.5 3.0],1.0,408);

uicontrol(fig,'Style','text','String','目标', ...
    'Position',[740 368 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');
mkrow('Spin','目标转速',[10 300],60,336);

S.metr = uicontrol(fig,'Style','text','String','', ...
    'Position',[740 30 320 225],'HorizontalAlignment','left', ...
    'BackgroundColor',[0.97 0.97 0.98],'FontName','FixedWidth', ...
    'FontSize',10);

runSim();

    function mkrow(tag,label,rng,initV,y)
        uicontrol(fig,'Style','text','String',label, ...
            'Position',[740 y+2 95 18],'HorizontalAlignment','left', ...
            'BackgroundColor','w');
        uicontrol(fig,'Style','slider','Tag',tag,'Min',rng(1),'Max',rng(2), ...
            'Value',initV,'Position',[840 y 140 22],'Callback',@onSlider);
        S.hval.(tag) = uicontrol(fig,'Style','text','String','', ...
            'Position',[985 y+2 75 18],'HorizontalAlignment','left', ...
            'BackgroundColor','w');
    end

    function onSlider(hObject,~)
        v = get(hObject,'Value');
        switch get(hObject,'Tag')
            case 'Kp',   S.Kp = round(10^(v/33.33));
            case 'Ki',   S.Ki = v;
            case 'Kd',   S.Kd = v;
            case 'tau',  S.tau = v;
            case 'Kv',   S.KvMul = v;
            case 'Spin', S.Spin = v;
        end
        runSim();
    end

    function runSim()
        set(S.hval.Kp,  'String',sprintf('%.0f',S.Kp));
        set(S.hval.Ki,  'String',sprintf('%.2f',S.Ki));
        set(S.hval.Kd,  'String',sprintf('%.0f',S.Kd));
        set(S.hval.tau, 'String',sprintf('%.3f s',S.tau));
        set(S.hval.Kv,  'String',sprintf('%.2f x',S.KvMul));
        set(S.hval.Spin,'String',sprintf('%.0f rpm',S.Spin));

        Kv = KV0*S.KvMul;
        target = S.Spin * ones(N,1);   % 阶跃：0 → Spin rpm

        % 速度环闭环（复刻 pid.c：误差=set-ref，Iout 仅限幅，D 无 /dt）
        speed = 0; Iout = 0; e0 = 0; e1 = 0;
        spd = zeros(N,1); volt = zeros(N,1);
        for k = 1:N
            e2 = e1; e1 = e0;
            e0 = target(k) - speed;
            Pout = S.Kp * e0;
            Iout = Iout + S.Ki * e0;
            Iout = max(min(Iout,MAXIOUT),-MAXIOUT);
            Dout = S.Kd * (e0 - e1);
            raw  = Pout + Iout + Dout;
            u    = max(min(raw,MAXOUT),-MAXOUT);

            speed = speed + dt/S.tau*(Kv*u - speed);
            spd(k) = speed; volt(k) = u;
        end

        % 指标
        idx10 = find(spd >= 0.1*S.Spin, 1, 'first');
        idx90 = find(spd >= 0.9*S.Spin, 1, 'first');
        if isempty(idx90)
            rise_t = NaN; over = NaN;
        else
            rise_t = (idx90 - idx10)*dt;
            peak = max(spd(idx90:end));
            over = 100*(peak - S.Spin)/S.Spin;
        end
        ss_err = S.Spin - mean(spd(round(N*0.7):end));
        band = 0.02*S.Spin;
        settle_idx = find(abs(spd - S.Spin) > band, 1, 'last');
        if isempty(settle_idx), settle_t = 0; else, settle_t = settle_idx*dt; end

        set(S.metr,'String',sprintf([ ...
            ' 阶跃响应指标（目标 %.0f rpm）\n' ...
            ' --------------------------------\n' ...
            ' 上升时间(10~90%%): %6.2f s\n' ...
            ' 超调量          : %6.1f %%\n' ...
            ' 稳态误差        : %6.2f rpm\n' ...
            ' 调节时间(±2%%)   : %6.2f s\n' ...
            ' --------------------------------\n' ...
            ' 调参说明：\n' ...
            ' 上升慢 → 加大 Kp\n' ...
            ' 超调大 → 减 Kp 或加 Kd(0-10000)\n' ...
            ' 稳态有差 → 加 Ki\n' ...
            ' Ki 大易 windup → 考虑加抗饱和\n'], ...
            S.Spin, rise_t, over, ss_err, settle_t));

        plot(ax1,t,target,'b--',t,spd,'r-','LineWidth',1.3); grid(ax1,'on');
        legend(ax1,{'目标转速','实际转速'},'Location','southeast');
        ylabel(ax1,'转速 / rpm'); title(ax1,'Yaw 速度环阶跃响应');

        plot(ax2,t,volt,'LineWidth',1.0); grid(ax2,'on');
        ylabel(ax2,'电压指令'); xlabel(ax2,'t / s'); title(ax2,'速度环输出（饱和上限 ±30000）');
        yline(ax2, MAXOUT,'k:'); yline(ax2,-MAXOUT,'k:');
    end
end