% InertialPower.m
% Computes the total inertial power of a link
% m = mass of link
% I = moment of inertia of link
% v = velocity of CM of link
%Force Analysis on Linkages 423
% a = acceleration of CM of link
% omega = angular velocity of link
% alpha = angular acceleration of link
function P = InertialPower(m, I, v, a, omega, alpha)
P = m*dot(v,a) + I*omega*alpha; % inertial power of link
end