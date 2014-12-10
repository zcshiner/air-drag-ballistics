function [T,motionTotal] = ballistic(p0, theta, phi, velocity) % angles are in radians
% BALLISTIC(ICS, azimuth, elevation, v0)
% Calculate the path of a ballistic object with air resistance.
% aizimuth is in degrees along xy plane from the x-axis, elevation is in
% degrees from the xy plane to the initial velocity vector.
%
% Equations via: http://www.cs.cornell.edu/~bindel/class/cs3220-s12/notes/lec27.pdf

g = [0,0,9.81]'; % m/s % xyz
c = 0.45; %1/m; % scaled drag coefficient

% decompose heading & pitch into xyz magnitudes times velocity scalar
v0 = velocity.*[cos(theta)*cos(phi), sin(theta)*cos(phi), sin(phi)]';
ICS = [p0';v0];
tFinal = v0(3)/g(3)+sqrt((v0(3)/g(3)).^2+2*p0(3)/g(3));
% for testing: v = v0

    % Ballistic model, considering air drag
    function rhs = dragModel(t,motion)
        v = motion(4:6); % last 3 terms are velocity
        rhs = [v; (-g-c.*abs(v).*v)]; %[vx,vy,vz,ax,ay,az]'
    end

    % ODE options: stop when we hit zero
    function [value,isterminal, direction ] = hitground(t,y)
        value = y(3); % Check for zero crossings of z position
        isterminal = 1; % Terminate on a zero crossing
        direction = -1; % We only care about crossing y > 0 to y < 0
    end

odeopt = odeset('Events', @hitground);
[T,XV] = ode45(@dragModel,[0,tFinal.^2],ICS,odeopt);

% add in acceleraton data.. because I can.
motionTotal = horzcat(XV,[-c.*abs(XV(:,4:5)).*XV(:,4:5),-g(3)-c.*abs(XV(:,6)).*XV(:,6)]);

end
