clc
hf = clf(figure(1));
ax = gca(hf); hold all, axis equal

ang = 45+25;
targPos = 50*[cosd(ang), sind(ang)];
turretAng = 45;
turretRadius = 5;
camAngle = turretAng+0;
camPos = [-turretRadius, 0];


plot(targPos(2), targPos(1), 'sk', 'MarkerFaceColor', 'k');

% Plot the turret
ang = linspace(0, 2*pi, 20);
plot(ax, turretRadius*sin(ang), turretRadius*cos(ang), 'k', turretRadius*[0 sind(turretAng)], turretRadius*[0 cosd(turretAng)], 'k')

camXY = camPos * [cosd(turretAng) sind(turretAng);
                -sind(turretAng) cosd(turretAng)];
quiver(camXY(2), camXY(1), turretRadius*sind(camAngle), turretRadius*cosd(camAngle), 'LineWidth', 3, 'Color', 'b')

dX = targPos(1) - camXY(1);
dY = targPos(2) - camXY(2);
rng2Targ = hypot(dX,dY);
ang2Targ = atan2(dY, dX)*180/pi;
quiver(camXY(2), camXY(1), turretRadius*sind(ang2Targ), turretRadius*cosd(ang2Targ), 'LineWidth', 3, 'Color', 'r')

fprintf('CamRngToTarg: %0.2f\n', rng2Targ);
fprintf('CamAngleToTarg: %0.2f\n', ang2Targ);

% Use the law of Cosines to find the range from the center of the turret to
% the target
% a^2 = b^2 + c^2 - 2*b*c*cos(A)
A = turretAng - ang2Targ;
b = norm(camPos);
c = rng2Targ;
a = sqrt( b^2 + c^2 - 2*b*c*cos(A));

% Now use the law of sines to find the smaller angle (i.e. angle between
% camera and turret center as seen from target position
% sin(B/b) = sin(A/a)
B = asind( sind(abs(A)/a) * b);


A2 = (atan2(0-camXY(2), 0-camXY(1)) - atan2(targPos(2)-camXY(2), targPos(1)-camXY(1)))*180/pi;
B2 = (atan2(camXY(2)-targPos(2), camXY(1)-targPos(1)) - atan2(-targPos(2), -targPos(1)))*180/pi;
% Sum of all angles inside a triange is 180;
finalAngle = 180 - A - B;

% Because camera is behind turret center, we want the adjacent angle
DesiredTurretAngle = turretAng - (180 - finalAngle);

str = {
       sprintf('A: %0.2f', A), ...
       sprintf('B: %0.2f', B), ...
       sprintf('FinalAngle: %0.2f', finalAngle), ...
       sprintf('DesiredTurretAngle: %0.2f (deg)', DesiredTurretAngle)};


title(ax, str);