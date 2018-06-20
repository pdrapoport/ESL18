%--------------------------------------------------------------------------
% running Butterworth -- 
%--------------------------------------------------------------------------
    clear all;
    close all;
% load data
	load file_butterworth.txt;
	t = file_butterworth(:,2);
	sax = file_butterworth(:,3);
	say = file_butterworth(:,4);
	saz = file_butterworth(:,5);
    p = file_butterworth(:,6);
	q = file_butterworth(:,7);
	r = file_butterworth(:,8);
    p2 = file_butterworth(:,9);
    phi_p2 = file_butterworth(:,10);
    fs = length(t)*1000000/(t(length(t))-t(1));
	n = length(t);

    % p2 is p filtered in the micro
    figure(1);
    plot(t,p,t,p2);
    legend('p','p\_butterworth');
    
    % phi_p2 is p2 integrated in the micro
    figure(2);
    plot(t,p2,t,phi_p2);
    legend('p\_butterworth','phi');

    
    % compute phi from p
    phi_p(1:n) = 0;
    phi_p(1) = 0;
    for i=(2:n) phi_p(i) = phi_p(i-1) + p(i-1) * 0.01; end;
    
    % show that phi_p2 equals phi_p (so no need to filter p)
    figure(3);
    plot(t,phi_p,t,phi_p2);
    legend('phi\_p','phi\_p2');
