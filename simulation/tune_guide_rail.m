function tune_guide_rail()
% =====================================================================
%  tune_guide_rail.m — 水平轴 PID 图形化调参工具（无需任何工具箱）
%  ---------------------------------------------------------------------
%  用法：MATLAB 中 cd 到 simulation 文件夹，运行：
%        >> tune_guide_rail
%  拖动右侧滑块，左侧曲线实时刷新：
%    上图  目标角度(蓝虚线) / 实际角度(红线) / 灰线=保存的对比曲线
%    下图  跟踪误差
%    右下  指标：最大误差、RMS误差、摆幅比、输出饱和占比
%  调好后把 Kp / Ki / Kd 写回 Motor.h 的 HORIZONTAL_ANGLE_* 宏。
%  =====================================================================

% ---------- 默认参数（与固件一致） ----------
S.Kp    = 20;       % HORIZONTAL_ANGLE_KP
S.Ki    = 0;        % HORIZONTAL_ANGLE_KI
S.Kd    = 0;        % HORIZONTAL_ANGLE_KD
S.tau   = 0.08;     % 电机时间常数 s（实测后修正）
S.KvMul = 1.0;      % 电机增益倍率（1 = GM6020 估计值）
S.Tper  = 2.0;      % 轨迹周期 s（TRAJECTORY_PERIOD_MS/1000）
S.ghostAct = [];    % 对比曲线
S.lastAct  = [];

MAXOUT  = 3000;         % HORIZONTAL_ANGLE_MAX_OUT
MAXIOUT = 5000;         % HORIZONTAL_ANGLE_MAX_IOUT
KV0     = 320/30000;    % 满指令→320rpm
dt = 1e-3; N = 6000; t = (0:N-1)'*dt;   % 仿真 6s，1ms 步长

% ---------- 界面 ----------
fig = figure('Color','w','Position',[60 60 1080 680], ...
    'Name','Guide_rail 水平轴图形化调参','NumberTitle','off', ...
    'MenuBar','none','ToolBar','none');

ax1 = axes('Parent',fig,'Units','pixels','Position',[50 400 640 230]);
ax2 = axes('Parent',fig,'Units','pixels','Position',[50 105 640 230]);

uicontrol(fig,'Style','text','String','拖滑块调参，曲线实时刷新；调完写回 Motor.h', ...
    'Position',[50 645 640 20],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');

uicontrol(fig,'Style','text','String','角度环 PID（写回 Motor.h）', ...
    'Position',[740 640 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');

S.hval = struct();
mkrow('Kp','Kp',[0 100],33.33*log10(20),612);
mkrow('Ki','Ki',[0 50],0,568);
mkrow('Kd','Kd',[0 10000],0,524);

uicontrol(fig,'Style','text','String','电机模型（实测标定后改）', ...
    'Position',[740 484 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');
mkrow('tau','时间常数 tau',[0.02 0.30],0.08,452);
mkrow('Kv','增益倍率',[0.5 3.0],1.0,408);

uicontrol(fig,'Style','text','String','轨迹', ...
    'Position',[740 368 320 18],'HorizontalAlignment','left', ...
    'BackgroundColor','w','FontWeight','bold');
mkrow('Tper','周期',[0.5 5.0],2.0,336);

uicontrol(fig,'Style','pushbutton','String','保存当前曲线作对比', ...
    'Position',[740 272 150 30],'Callback',@onSave);
uicontrol(fig,'Style','pushbutton','String','清除对比', ...
    'Position',[905 272 155 30],'Callback',@onClear);

S.metr = uicontrol(fig,'Style','text','String','', ...
    'Position',[740 30 320 225],'HorizontalAlignment','left', ...
    'BackgroundColor',[0.97 0.97 0.98],'FontName','FixedWidth', ...
    'FontSize',10);

runSim();

    % ---------- 控件行：标签 + 滑块 + 数值 ----------
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
            case 'Kp',   S.Kp = round(10^(v/33.33));   % 对数刻度 1~1000
            case 'Ki',   S.Ki = v;
            case 'Kd',   S.Kd = v;
            case 'tau',  S.tau = v;
            case 'Kv',   S.KvMul = v;
            case 'Tper', S.Tper = v;
        end
        runSim();
    end

    function onSave(~,~)
        S.ghostAct = S.lastAct;
        runSim();
    end

    function onClear(~,~)
        S.ghostAct = [];
        runSim();
    end

    % ---------- 仿真 + 刷新 ----------
    function runSim()
        set(S.hval.Kp,  'String',sprintf('%.0f',S.Kp));
        set(S.hval.Ki,  'String',sprintf('%.1f',S.Ki));
        set(S.hval.Kd,  'String',sprintf('%.1f',S.Kd));
        set(S.hval.tau, 'String',sprintf('%.3f s',S.tau));
        set(S.hval.Kv,  'String',sprintf('%.2f x',S.KvMul));
        set(S.hval.Tper,'String',sprintf('%.1f s',S.Tper));

        Kv = KV0*S.KvMul;

        % 三角波目标（复刻 trajectory.c 的 triangle_angle）
        A  = 30;
        ph = mod(t,S.Tper)/S.Tper;
        target = 4*A*ph;
        target(ph>=0.25 & ph<0.75) = 2*A - 4*A*ph(ph>=0.25 & ph<0.75);
        target(ph>=0.75) = -4*A + 4*A*ph(ph>=0.75);

        % 闭环（复刻 pid.c 位置式 PID + 一阶电机）
        angle = 0; speed = 0; Iout = 0; eprev = 0;
        act = zeros(N,1); sat = false(N,1);
        for k = 1:N
            e  = target(k) - angle;
            PD = S.Kp*e + S.Kd*(e - eprev);        % 比例+微分
            raw = PD + Iout;
            % 条件积分抗饱和：输出已饱和且积分会加剧饱和时，停止积分
            if ~((raw >= MAXOUT && e > 0) || (raw <= -MAXOUT && e < 0))
                Iout = Iout + S.Ki*e;
                Iout = max(min(Iout,MAXIOUT),-MAXIOUT);
                raw  = PD + Iout;
            end
            eprev = e;
            u    = max(min(raw,MAXOUT),-MAXOUT);
            sat(k) = abs(raw) > MAXOUT;
            speed = speed + dt/S.tau*(Kv*u - speed);
            angle = angle + speed*6*dt;      % rpm→deg/s：×360/60=6
            act(k) = angle;
        end
        S.lastAct = act;

        % 指标（跳过首个周期，去除启动瞬态）
        idx = t >= S.Tper;
        if ~any(idx), idx = t > 0; end
        err    = target(idx) - act(idx);
        maxe   = max(abs(err));
        rms    = sqrt(mean(err.^2));
        ratio  = 100*std(act(idx))/std(target(idx));
        satpct = 100*mean(sat(idx));
        set(S.metr,'String',sprintf([ ...
            ' 调参指标（跳过首周期）\n' ...
            ' --------------------------------\n' ...
            ' 最大误差       : %6.2f 度\n' ...
            ' RMS 误差       : %6.2f 度\n' ...
            ' 摆幅比(实/目标): %6.0f %%\n' ...
            ' 输出饱和占比   : %6.1f %%\n' ...
            ' --------------------------------\n' ...
            ' 调参说明：\n' ...
            ' 摆幅比<90%%   → 加大 Kp\n' ...
            ' 拐点过冲大    → 加大 Kd(0-10000)\n' ...
            ' 已加抗饱和：Ki 小量消除静差\n' ...
            ' 饱和占比高    → 正常（三角波需要）\n'], ...
            maxe, rms, ratio, satpct));

        % 绘图
        plot(ax1,t,target,'b--',t,act,'r-','LineWidth',1.3);
        hold(ax1,'on');
        if ~isempty(S.ghostAct)
            plot(ax1,t,S.ghostAct,'-','Color',[.62 .62 .66],'LineWidth',1.0);
            legend(ax1,{'目标角度','实际角度','对比(旧)'},'Location','northeast');
        else
            legend(ax1,{'目标角度','实际角度'},'Location','northeast');
        end
        hold(ax1,'off'); grid(ax1,'on');
        ylabel(ax1,'角度 / 度');
        title(ax1,'水平轴：三角波跟踪');

        plot(ax2,t,target-act,'LineWidth',1.0);
        grid(ax2,'on'); xlabel(ax2,'t / s');
        ylabel(ax2,'误差 / 度'); title(ax2,'跟踪误差（目标-实际）');
    end
end