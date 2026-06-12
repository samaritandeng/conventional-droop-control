function build_noripple()
% BUILD_NORIPPLE  Create conv_droop_2Rinv_noripple from conv_droop_2Rinv by
% replacing the (product + LPF) power calc with QUADRATURE (alpha-beta) power,
% whose 2-omega ripple cancels mathematically. Original model file is untouched.
%
%   P = 0.5*( v_a*i_a + v_b*i_b )      Q = 0.5*( v_b*i_a - v_a*i_b )
%   v_a=vo, v_b=delay90(vo), i_a=i, i_b=delay90(i)
%
% P path keeps a HIGH-BANDWIDTH LPF_P (wc=300) ONLY to break the algebraic loop
% that appears because E=E*-nP is algebraic (no integrator). It does not remove
% ripple -- the quadrature cancellation does. Q path needs no filter: the
% omega->theta integrator (int_theta) already breaks its loop.
src='conv_droop_2Rinv'; dst='conv_droop_2Rinv_noripple';
bdclose('all');
if isfile([dst '.slx']), delete([dst '.slx']); end
load_system(src);
save_system(src,dst);          % Save-As: in-memory model renamed to dst; src.slx unchanged
for c={'Ctrl1','Ctrl2'}, surgery([dst '/' c{1}]); end
save_system(dst);

% ---- self-check ----
sub=[dst '/Ctrl1'];
has=@(n) ~isempty(find_system(sub,'SearchDepth',1,'Name',n));
fprintf('=== %s / Ctrl1 blocks after surgery ===\n',dst);
b=find_system(sub,'SearchDepth',1,'Type','Block');
fprintf('  %s\n',strjoin(sort(cellfun(@(x) get_param(x,'Name'),b,'uni',0)),', '));
assert(~has('LPF_Q'),'LPF_Q should be removed (Q path needs no filter)');
assert(has('LPF_P'),'LPF_P should remain as loop-breaker');
assert(strcmp(get_param([sub '/LPF_P'],'Denominator'),'[1 300]'),'LPF_P should be high-bandwidth');
assert(has('gain_halfP')&&has('gain_halfQ')&&has('delay90_i')&&has('m_ab')&&has('m_bb'),'quadrature blocks missing');
fprintf('OK: Q path pure-quadrature (no LPF); P path quadrature + high-bw LPF_P (loop breaker).\n');
bdclose(dst);
end

function surgery(sub)
% ===== shared: current quadrature component i_b = delay90(i) =====
add_block('simulink/Continuous/Transport Delay',[sub '/delay90_i'],'DelayTime','0.005');
add_line(sub,'i/1','delay90_i/1','autorouting','on');

% ===== P path: quadrature power, REUSE LPF_P as a high-bandwidth loop breaker =====
% old: p_mult -> LPF_P -> gain_n,Pout.  new: (p_mult,m_bb)->sum_P->gain_halfP->LPF_P->gain_n,Pout
delete_line(sub,'p_mult/1','LPF_P/1');
set_param([sub '/LPF_P'],'Numerator','300','Denominator','[1 300]');   % wc=300: breaks loop, not ripple
add_block('simulink/Math Operations/Product',[sub '/m_bb']);
add_block('simulink/Math Operations/Sum',[sub '/sum_P'],'Inputs','++');
add_block('simulink/Math Operations/Gain',[sub '/gain_halfP'],'Gain','0.5');
add_line(sub,'delay90/1','m_bb/1','autorouting','on');        % v_b
add_line(sub,'delay90_i/1','m_bb/2','autorouting','on');      % i_b  => m_bb = v_b*i_b
add_line(sub,'p_mult/1','sum_P/1','autorouting','on');        % v_a*i_a (existing p_mult)
add_line(sub,'m_bb/1','sum_P/2','autorouting','on');
add_line(sub,'sum_P/1','gain_halfP/1','autorouting','on');
add_line(sub,'gain_halfP/1','LPF_P/1','autorouting','on');    % LPF_P still -> gain_n, Pout

% ===== Q path: pure quadrature, DELETE LPF_Q (int_theta already breaks the loop) =====
delete_line(sub,'q_mult/1','LPF_Q/1');
delete_line(sub,'LPF_Q/1','gain_m/1');
delete_line(sub,'LPF_Q/1','Qout/1');
delete_block([sub '/LPF_Q']);
add_block('simulink/Math Operations/Product',[sub '/m_ab']);
add_block('simulink/Math Operations/Sum',[sub '/sum_Q'],'Inputs','+-');
add_block('simulink/Math Operations/Gain',[sub '/gain_halfQ'],'Gain','0.5');
add_line(sub,'vo/1','m_ab/1','autorouting','on');            % v_a
add_line(sub,'delay90_i/1','m_ab/2','autorouting','on');     % i_b  => m_ab = v_a*i_b
add_line(sub,'q_mult/1','sum_Q/1','autorouting','on');       % v_b*i_a (existing q_mult)
add_line(sub,'m_ab/1','sum_Q/2','autorouting','on');         % minus on input 2
add_line(sub,'sum_Q/1','gain_halfQ/1','autorouting','on');
add_line(sub,'gain_halfQ/1','gain_m/1','autorouting','on');
add_line(sub,'gain_halfQ/1','Qout/1','autorouting','on');

try, Simulink.BlockDiagram.arrangeSystem(sub); catch, end
end
