%--------------------------------------------------------------------------
% running Butterworth -- 
%--------------------------------------------------------------------------
    clear all;
    close all;
% load data
	load file_kalman.txt;
	t = file_kalman(:,2);
	sax = file_kalman(:,3);
	say = file_kalman(:,4);
	saz = file_kalman(:,5);
    p = file_kalman(:,6);
	q = file_kalman(:,7);
	r = file_kalman(:,8);
    say_filtered = file_kalman(:,9);
    phi_kalman = file_kalman(:,10);
    fs = length(t)*1000000/(t(length(t))-t(1));
	n = length(t);

    % say_filtered is filtered in the micro
    figure(1);
    plot(t,say,t,say_filtered);
    legend('say','say\_filtered');
    
    figure(2);
    plot(t,say_filtered,t,p,t,phi_kalman);
    legend('say\_filtered','p','phi\_kalman');
    
    % compare phi from kalman with phi from p
    phi_p(1:n) = 0;
    phi_p(1) = 0;
    for i=(2:n) 
        phi_p(i) = phi_p(i-1) + p(i-1) * 0.01; 
    end;
    figure(3);
    plot(t,phi_p,t,phi_kalman);
    legend('phi\_p','phi\_kalman');
