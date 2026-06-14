function metrics_noripple()
% Extract real metrics from a no-ripple run for the sim log.
m='conv_droop_2Rinv_noripple';
here=fileparts(mfilename('fullpath')); oldc=cd(here); rcd=onCleanup(@()cd(oldc));
if ~bdIsLoaded(m), load_system(m); end
s=sim(m,'StopTime','8');
t=s.P1.Time; P1=s.P1.Data; P2=s.P2.Data; Q1=s.Q1.Data; Q2=s.Q2.Data; Vo=s.Vo.Data;
w=[6 8]; idx=t>=w(1)&t<=w(2);
P1m=mean(P1(idx)); P2m=mean(P2(idx)); ratio=P1m/P2m;
des=2.0; err=100*(des-ratio)/des;
ppQ1=max(Q1(idx))-min(Q1(idx)); ppQ2=max(Q2(idx))-min(Q2(idx));
Vrms=sqrt(mean(Vo(idx).^2)); Estar=12; vdev=100*(Estar-Vrms)/Estar;
% P1 settling after connect (t>3): first instant after which P1 stays within +-2% of P1m
band=0.02*P1m; tc=3; ii=find(t>tc); ts=NaN;
for k=1:numel(ii)
  if all(abs(P1(ii(k:end))-P1m)<=band), ts=t(ii(k))-tc; break; end
end
fprintf('P1m=%.3f W  P2m=%.3f W\n',P1m,P2m);
fprintf('P1/P2=%.4f  (design 2.0)  sharing_error=%.2f %%\n',ratio,err);
fprintf('Vo_rms=%.3f V  (design E*=12)  voltage_deviation=%.2f %%\n',Vrms,vdev);
fprintf('Q1_ripple_pp=%.4f var  Q2_ripple_pp=%.4f var\n',ppQ1,ppQ2);
fprintf('P1_settling_after_connect(+-2%%)=%.3f s\n',ts);
end
