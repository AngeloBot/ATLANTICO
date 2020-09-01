
V=0.3;
d=0.37;
dV=d/V;

atan_5= @(dtheta,dV) (dtheta*dV).^5/5 - (dtheta*dV).^3/3 + (dtheta*dV);

atan_3=@(dtheta,dV) - (dtheta*dV).^3/3 + (dtheta*dV);

atan_1=@(dtheta,dV) (dtheta*dV);

atan_ref=@(dtheta,dV) atan(dtheta*dV);

vect_dV=d*linspace(0.01,1,1000);

vect_dtheta=transpose(linspace(-pi,pi,1000));

vect_atans={atan_ref(vect_dtheta,vect_dV)
            atan_1(vect_dtheta,vect_dV)
            atan_3(vect_dtheta,vect_dV)
            atan_5(vect_dtheta,vect_dV)};

figure(1);

mesh(radtodeg(vect_dtheta),vect_dV,abs(vect_atans{4}-vect_atans{1}))
title({'atan 5º grau';'com V=[0.01,1](m/s) e d=0.37m'})
xlabel('velocidade angular (rad/s)')
ylabel('valor da razão d/V ')
%set(get(gca,'ylabel'),'rotation',-30)

figure(2);

mesh(radtodeg(vect_dtheta),vect_dV,abs(vect_atans{3}-vect_atans{1}))
title({'atan 3º grau';'com V=[0.01,1](m/s) e d=0.37m'})
xlabel('velocidade angular (rad/s)')
ylabel('valor da razão d/V')
%set(get(gca,'ylabel'),'rotation',-30)

figure(3);

mesh(radtodeg(vect_dtheta),vect_dV,abs(vect_atans{2}-vect_atans{1}))
title({'atan 1º grau';'com V=[0.01,1](m/s) e d=0.37m'})
xlabel('velocidade angular (rad/s)')
ylabel('valor da razão d/V')
%set(get(gca,'ylabel'),'rotation',-30)

        