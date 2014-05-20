function [PRx0, PRy0] = computeinitposrobot(VRx, VRy, mpd, PAxf, PAyf, t, tgammaMax)

% Solve the equation of second order
ax = 1;
bx = 2*(VRx * 2* tgammaMax / 2 + (t-2*tgammaMax)*VRx);
cx = (VRx * 2* tgammaMax / 2 + (t-2*tgammaMax)*VRx)^2 - mpd^2 - PAxf^2;

ay = 1;
by = 2*(VRy * 2* tgammaMax / 2 + (t-2*tgammaMax)*VRy);
cy = (VRy * 2* tgammaMax / 2 + (t-2*tgammaMax)*VRy)^2 - mpd^2 - PAyf^2;

PRx0 = (-bx + sqrt(bx^2 - 4*ax*cx))/2*ax;
%(-bx - sqrt(bx^2 - 4*ax*cx))/2*ax

PRy0 = (-by + sqrt(by^2 - 4*ay*cy))/2*ay;
%(-by - sqrt(by^2 - 4*ay*cy))/2*ay

end