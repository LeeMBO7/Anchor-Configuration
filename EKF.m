function [xe ye] = TrackKalman(xm, ym)
%
%
persistent A H Q R
persistent x P
persistent firstRun

if isempty(firstRun)
    dt = 0.01; % Timestamp

    % system model
    A = [1 dt 0 0
         0 1 0 0 
         0 0 1 dt
         0 0 0 1];

    % measurement model
    H = [1 0 0 0
         0 0 1 0];

    Q = 1.0*eye(4)
    R = [3 0
         0 3];

    x = [0,0,0,0]';

    P = 100*eye(4);
 
    firstRun = 1;
end

xp = A*x;
Pp = A*P*A'+Q;

K = Pp*H'/(H*Pp*H' + R);

z = [xm ym]';
x = xp + K*(z-H*xp);
P = Pp - K*H*Pp;

xe = x(1);
ye = x(3);

