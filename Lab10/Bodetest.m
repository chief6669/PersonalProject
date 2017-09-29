R=9.824*10^6;
L=10^(-3);
c=270*10^(-12);
a=.9;
H=tf([1 0 1/((R*c)^2)],[1 4*(1-a)/(R*c) 1/((R*c)^2)]);
P = bodeoptions;
P.FreqUnits = 'Hz';
figure(1)
bodeplot(H,P);

R5=11368;
R6=11368;
C5=.1*10^-6;
C6=.1*10^-6;

B=tf([0 0 1/(R6*C6*C5*R5)],[1 (R5+R6)/(R6*C6*R5) 1/(R6*C6*C5*R5)]);
figure(2)
bodeplot(B,P);

% w=0:100000000;
% s=1i.*w;
% z2c=1./(s.*2.*c);
% zc=1./(s.*c);
% vout = R*(z2c./(2.*z2c + R)) + 2.*z2c.*(R./(2.*(R + zc)));
% vin = R.*a - R*((R.*a + z2c)./(2.*z2c + R)) + 2.*z2c.*a - 2.*z2c.*((2.*zc.*a + R)./(2.*(R + zc)));
% F=vout./vin;
% P=(pi./2)-atan2(w,(1./(R.*C)-(w.^2)..*(L./R)));
% figure(2)
% semilogx(w,mag2db(F));
% subplot(2,1,2)
% semilogx(w,(180./pi)..*P);