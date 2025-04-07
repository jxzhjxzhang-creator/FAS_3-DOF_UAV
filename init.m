% 假设 out.ref 和 out.state 已经定义并包含相关数据
% out.ref.x, out.ref.z 是参考信号
% out.state.x, out.state.z 是实际状态
close all;
% 获取数据长度（假设两者的长度相同）
sim("quadrotor_3dof_fas1.slx")
t = out.tout;
Z_1 = [1 1 1 1];
Z_2 = [1 1];
cPole_1 = [-1 -1.5 -2.0 -2.5];
cPole_2 = [-1.0 -1.5];
F_1 = diag(cPole_1);
F_2 = diag(cPole_2);
V_1 = [Z_1; Z_1*F_1; Z_1*F_1^2;Z_1*F_1^3;];
V_2 = [Z_2; Z_2*F_2];
A1 = -Z_1*F_1^4/V_1;
A2 = -Z_2*F_2^2/V_2;


% Calculate positive P in quadratic Lyapunov function V = x^T P x
setlmis([])
P_1 = lmivar(1,[4,1]);
P_2 = lmivar(1,[2,1]);
B=max(cPole_1);
lmiterm([1 1 1 P_1],1,[0 1 0 0; 0 0 1 0; 0 0 0 1; -A1],'s')
lmiterm([1 1 1 P_1],-2*max(cPole_1),1)
lmiterm([-2 1 1 P_1],1,1)
lmiterm([3 1 1 P_2],1,[0 1 ; -A2],'s')
lmiterm([3 1 1 P_2],-2*max(cPole_2),1)
lmiterm([-4 1 1 P_2],1,1)
lmisys = getlmis;
[tmin,xfeas] = feasp(lmisys);
P_1 = dec2mat(lmisys,xfeas,P_1);
P_2 = dec2mat(lmisys,xfeas,P_2);
% 提取矩阵 P1 的最后一列并转置为行向量
P1_L = P_1(:, end).';

% 提取矩阵 P2 的最后一列并转置为行向量
P2_L = P_2(:, end).';

disp(P1_L);

disp(P2_L);
% 设置默认字体样式
set(0, 'DefaultAxesFontSize', 10);
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultLegendFontSize', 12);
set(0, 'DefaultTextFontSize', 10);
set(0, 'DefaultAxesTitleFontSize', 1.1);        % 设置标题字体大小
% 全局颜色定义
color_ref = [0.8 0 0];        % 参考信号 - 深红色
color_with = [0 0.4 0.8];     % 有鲁棒补偿 - 深蓝色
color_without = [0.9 0.5 0];  % 无鲁棒补偿 - 橙色（替换所有绿色）
color_traj = [0 0.6 0.6];     % 轨迹图专用青蓝色

%% 状态跟踪图 (原第一个figure) ------------------------------------------
figure;
% x方向
subplot(3,1,1);
hold on;
plot(t, out.ref(:,1), '--', 'Color', color_ref, 'LineWidth', 1.5);
plot(t, out.state(:,1), '-', 'Color', color_with, 'LineWidth', 1.5);
plot(t, out.state1(:,1), '-', 'Color', color_without, 'LineWidth', 1.5); % 橙色实线
xlabel('t(s)');
ylabel('x(m)');
legend('Ref x', 'Actual x (with robust)', 'Actual x (without robust)');
title('x(m) vs time(s)', 'FontSize', 13);
grid on; box on;

% z方向
subplot(3,1,2);
hold on;
plot(t, out.ref(:,2), '--', 'Color', color_ref, 'LineWidth', 1.5);
plot(t, out.state(:,2), '-', 'Color', color_with, 'LineWidth', 1.5);
plot(t, out.state1(:,2), '-', 'Color', color_without, 'LineWidth', 1.5); % 橙色实线
xlabel('t(s)');
ylabel('z(m)');
legend('Ref z', 'Actual z (with robust)', 'Actual z (without robust)');
title('z(m) vs time(s)','FontSize', 13);
grid on; box on;

% theta角度
subplot(3,1,3);
hold on;
plot(t, out.state(:,3), '-', 'Color', color_with, 'LineWidth', 1.5);
plot(t, out.state1(:,3), '-', 'Color', color_without, 'LineWidth', 1.5); % 橙色实线
xlabel('t(s)');
ylabel('theta(rad)');
legend('Theta (with robust)', 'Theta (without robust)');
title('theta vs time(s)','FontSize', 13);
grid on; box on;

%% 误差分析图 (原第二个figure) ------------------------------------------
% 计算误差（保持原计算不变）
error_x = out.state(:,1) - out.ref(:,1);
error_z = out.state(:,2) - out.ref(:,2);
error_x_no_robust = out.state1(:,1) - out.ref(:,1);
error_z_no_robust = out.state1(:,2) - out.ref(:,2);

figure;
% x误差（保持不变）
subplot(2,1,1);
hold on;
plot(t, error_x, '-', 'Color', color_with, 'LineWidth', 1.5);
plot(t, error_x_no_robust, '--', 'Color', color_without, 'LineWidth', 1.5);
xlabel('t(s)');
ylabel('error (m)');
title('Error in x vs t');
legend('Error x (with robust)', 'Error x (without robust)');
grid on; box on;
axis([-inf inf -1 0.5]);

% z误差（优化后的局部放大）
subplot(2,1,2);
hold on;
% 主绘图
main_ax = gca;
h1 = plot(t, error_z, '-', 'Color', [0 0.6 0], 'LineWidth', 1.5);
h2 = plot(t, error_z_no_robust, '--', 'Color', [0.7 0.3 0], 'LineWidth', 1.5);

% 标记放大区域（更细的虚线）
rectangle('Position', [6,-0.1,6,0.2], 'EdgeColor', [0.4 0.4 0.4],...
          'LineWidth', 1.2, 'LineStyle',':');

% 主图格式设置
xlabel('t(s)');
ylabel('error (m)');
title('Error in z vs t');
legend([h1 h2], 'Error z (with robust)', 'Error z (without robust)',...
       'Location','northeast');
grid on; box on;
axis([0 max(t) -1 0.5]);

% 创建紧凑型放大坐标系（尺寸缩小30%）
inset_ax = axes('Position', [0.60 0.15 0.24 0.12]); % 调整后的位置参数
hold on;
plot(t, error_z, '-', 'Color', [0 0.6 0], 'LineWidth', 1);
plot(t, error_z_no_robust, '--', 'Color', [0.7 0.3 0], 'LineWidth', 1);

% 放大图精细设置
set(inset_ax,...
    'FontSize', 7,...                    % 更小字号
    'XAxisLocation', 'bottom',...
    'YAxisLocation', 'right',...
    'Box', 'on',...
    'XTick', 6:2:12,...                  % 优化刻度间隔
    'YTick', -0.1:0.05:0.1,...           % 新y轴范围
    'TickLength', [0.03 0.025]);         % 缩短刻度线
xlim([6 12]);
ylim([-0.1 0.1]);                        % 调整后的y轴范围

% 隐藏坐标轴标签（避免与主图重叠）
set(inset_ax, 'XLabel', [], 'YLabel', []);

% 连接线优化（更细的虚线）
annotation('line', [0.5 0.6], [0.31 0.2], 'Color', [0.4 0.4 0.4],...
           'LineStyle',':', 'LineWidth',1.2);

%% 轨迹对比图 (原第三个figure) ------------------------------------------
figure;
% 左图：无鲁棒轨迹（使用青蓝色）
subplot(1,2,1);
hold on;
plot(out.state1(:,1), out.state1(:,2), '-', 'Color', color_traj, 'LineWidth', 2);
plot(out.ref(:,1), out.ref(:,2), '--', 'Color', color_ref, 'LineWidth', 1.5);
xlabel('x(m)');
ylabel('z(m)');
legend('Actual Trajectory', 'Ref Trajectory');
title('No Robust Compensator');
axis([-1.5 1.5 -1.5 1.5]);
grid on; axis equal; box on;

% 右图：有鲁棒轨迹
subplot(1,2,2);
hold on;
plot(out.state(:,1), out.state(:,2), '-', 'Color', color_with, 'LineWidth', 2);
plot(out.ref(:,1), out.ref(:,2), '--', 'Color', color_ref, 'LineWidth', 1.5);
xlabel('x(m)');
ylabel('z(m)');
legend('Actual Trajectory', 'Ref Trajectory');
title('With Robust Compensator');
axis([-1.5 1.5 -1.5 1.5]);
grid on; axis equal; box on;

%% 推力曲线图 (原第四个figure) ------------------------------------------
figure('Position', [100 100 600 800]);
% 推力f1
subplot(3,1,1);
plot(t, out.u(:,1), '-', 'Color', [0 0.4 0.8], 'LineWidth', 1.5); % 深蓝色
xlabel('Time (s)');
ylabel('$f_1$ (N)', 'Interpreter', 'latex');
title('Rotor Thrust $f_1$', 'Interpreter', 'latex');
grid on; box on;
ylim([2 13]);

% 推力f2
subplot(3,1,2);
plot(t, out.u(:,2), '-', 'Color', [0.8 0 0], 'LineWidth', 1.5); % 深红色
xlabel('Time (s)');
ylabel('$f_2$ (N)', 'Interpreter', 'latex');
title('Rotor Thrust $f_2$', 'Interpreter', 'latex');
grid on; box on;
ylim([2 13]);

% 推力差
subplot(3,1,3);
delta_f = out.u(:,1) - out.u(:,2);
plot(t, delta_f, '-', 'Color', [0.5 0 0.5], 'LineWidth', 1.5); % 紫色
xlabel('Time (s)');
ylabel('$\Delta f$ (N)', 'Interpreter', 'latex');
title('Thrust Differential $f_1 - f_2$', 'Interpreter', 'latex');
grid on; box on;
ylim([-2.5 2.5]);
