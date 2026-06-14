function compare_ripple()
% Run the original (LPF) and the no-ripple (quadrature) models and compare.
m1='conv_droop_2Rinv'; m2='conv_droop_2Rinv_noripple';
here=fileparts(mfilename('fullpath')); oldc=cd(here); rcd=onCleanup(@()cd(oldc));
addpath(fullfile(here,'..','original'));   % locate the original conv_droop_2Rinv
o1=simrun(m1); o2=simrun(m2);
w=[6 8];                                   % steady window, both inverters on
fprintf('--- steady window t=[6,8] s ---\n');
report('orig    (LPF)       ',o1,w);
report('noripple(quadrature)',o2,w);

f=figure('Visible','on','Position',[60 40 1100 820]);
subplot(3,1,1);
plot(o1.t,o1.Q1,'Color',[.88 .55 .55]); hold on; plot(o2.t,o2.Q1,'b','LineWidth',1.1);
grid on; xline(3,'r--'); ylabel('Q_1 [var]');
legend('orig (LPF, \omega_c=30)','no-ripple (quadrature)','Location','northeast');
title('Reactive power Q_1 — the 100 Hz ripple band collapses to a line');
subplot(3,1,2);
plot(o1.t,o1.P1,'Color',[.88 .55 .55]); hold on; plot(o2.t,o2.P1,'b','LineWidth',1.1);
grid on; xline(3,'r--'); ylabel('P_1 [W]');
legend('orig','no-ripple','Location','east'); title('Active power P_1 — same steady value, no ripple');
subplot(3,1,3);
plot(o2.t,o2.P1,'b',o2.t,o2.P2,'r','LineWidth',1.1); grid on; xline(3,'r--');
ylabel('P [W]'); xlabel('t [s]');
legend('P_1','P_2','Location','east'); title('no-ripple model: load sharing P_1/P_2 unchanged (\approx 1.78)');
saveas(f,'noripple_compare.png'); fprintf('saved noripple_compare.png\n');
end

function o=simrun(m)
if ~bdIsLoaded(m), load_system(m); end
s=sim(m,'StopTime','8');
o.t=s.P1.Time; o.P1=s.P1.Data; o.P2=s.P2.Data;
o.Q1=s.Q1.Data; o.Q2=s.Q2.Data; o.E1=s.E1.Data; o.E2=s.E2.Data; o.Vo=s.Vo.Data;
end

function report(tag,o,w)
idx=o.t>=w(1)&o.t<=w(2);
pp=@(x) max(x(idx))-min(x(idx)); mn=@(x) mean(x(idx));
fprintf('%s  P1=%.3f(pp %.4f)  P2=%.3f  Q1=%.3f(pp %.4f)  Q2=%.3f(pp %.4f)  P1/P2=%.3f\n',...
  tag, mn(o.P1),pp(o.P1), mn(o.P2), mn(o.Q1),pp(o.Q1), mn(o.Q2),pp(o.Q2), mn(o.P1)/mn(o.P2));
end
