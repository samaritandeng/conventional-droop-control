function run_noripple(stopTime)
% RUN_NORIPPLE  Open the no-ripple model in Simulink, simulate it, and pop up the
% 4-panel data figure (output power / voltage set-point / reactive power / output
% voltage). Reactive power shows no 100 Hz ripple — quadrature (alpha-beta) power.
if nargin<1, stopTime=8; end
here=fileparts(mfilename('fullpath')); oldc=cd(here); rcd=onCleanup(@()cd(oldc));
mdl='conv_droop_2Rinv_noripple';
open_system(mdl);                               % open the block diagram in Simulink
out = sim(mdl,'StopTime',num2str(stopTime));
t=out.P1.Time; tc=3; wv=[tc-0.04 tc+0.06];
P1=out.P1.Data; P2=out.P2.Data; Q1=out.Q1.Data; Q2=out.Q2.Data;
E1=out.E1.Data; E2=out.E2.Data; Vo=out.Vo.Data;
f=figure('Visible','on','Position',[80 50 1000 840],'Name','no-ripple droop');
subplot(4,1,1); plot(t,P1,t,P2,'LineWidth',1.2); grid on; xline(tc,'r--','inv1 connects');
legend('P_1','P_2','Location','east'); ylabel('P [W]'); title('output power');
subplot(4,1,2); plot(t,E1,t,E2,'LineWidth',1.2); grid on; xline(tc,'r--');
legend('E_1','E_2','Location','east'); ylabel('E_i [V]'); title('voltage set-point  E_i = E* - n_i P_i');
subplot(4,1,3); plot(t,Q1,t,Q2,'LineWidth',1.2); grid on; xline(tc,'r--');
legend('Q_1','Q_2','Location','east'); ylabel('Q [var]'); title('reactive power  (no 100 Hz ripple)');
subplot(4,1,4); plot(t,Vo,'LineWidth',1.0); grid on; xlim(wv); xline(tc,'r--');
ylabel('v_o [V]'); xlabel('t [s]'); title('output voltage  (5 cycles around t=3 s)');
saveas(f,'noripple_run_results.png');
fprintf('RUN_NORIPPLE done\n');
end
