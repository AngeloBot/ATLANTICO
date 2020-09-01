%nomenclatura:
%variavel x => x
%expressao da variavel x => eq_x

%equacao geral do equilibrio de momentos
eq_sym=M_res - M_amort + M_vela == M_leme;

%definindo momentos e substituindo

eq_M_res=J_zz*ddtheta;
eq_M_leme=F_leme*d;
eq_M_amort=-dtheta*b;

eq_sym=subs(eq_sym,M_res,eq_M_res);
eq_sym=subs(eq_sym,M_leme,eq_M_leme);
eq_sym=subs(eq_sym,M_amort,eq_M_amort);

%equacionamento de forcas de lift...

eq_a=a_0/(1+a_0/(P_i*AR));
eq_C_L=a*alpha_res;
eq_L=0.5*ro*C_L*V_res^2*Ap;

%substuindo

eq_C_L=subs(eq_C_L,a,eq_a);
eq_L=subs(eq_L,C_L,eq_C_L);

%... e drag

eq_C_L_inf=a_0*alpha_res;
eq_C_D_inf=c0 ...
    +c1*alpha_res ...
    +c2*alpha_res^2 ...
    +c3*alpha_res^3 ...
    +c4*alpha_res^4 ...
    +c5*alpha_res^5 ...
    +c6*alpha_res^6;
eq_C_D=C_D_inf+C_L_inf/(P_i*AR);
eq_D=0.5*ro*C_D*V_res^2*Ap;

%substituindo
eq_C_D=subs(eq_C_D,C_D_inf,eq_C_D_inf);
eq_C_D=subs(eq_C_D,C_L_inf,eq_C_L_inf);

eq_D=subs(eq_D,C_D,eq_C_D);

%equacionamento para a forca do leme
eq_F_leme=L*cos(alpha_v)+D*sin(alpha_v);

%equacionamento para a forca do leme (usando taylor para cos e sin)
%eq_F_leme=L*taylor(cos(alpha_v))+D*taylor(sin(alpha_v));

%substituindo

eq_F_leme=subs(eq_F_leme,L,eq_L);
eq_F_leme=subs(eq_F_leme,D,eq_D);

%forca do leme na equacao geral

eq_sym=subs(eq_sym,F_leme,eq_F_leme);

%equacionamento V_rot
eq_V_rot=dtheta*d;

%equacionamento alpha_v (usando taylor)
%declarando simbolica place holder para a razão V_rot/V
syms V_V
eq_V_V=V_rot/V;

%considerando que taylor(atan(V_V))=V_V^5/5 - V_V^3/3 + V_V
%para pequenos desvios de V_rot, V_V será pequeno, e portanto pode-se
%aproximar alpha_v para V_rot/V
eq_alpha_v=V_rot/V;

%equacionamento V_res
eq_V_res=sqrt(V_rot^2+V^2);

%equacionamento alpha_res
eq_alpha_res=alpha_v+alpha_leme;

%forma final

eq_sym=subs(eq_sym,alpha_res,eq_alpha_res);
eq_sym=subs(eq_sym,alpha_v,eq_alpha_v);
eq_sym=subs(eq_sym,V_res,eq_V_res);
eq_sym=subs(eq_sym,V_rot,eq_V_rot);

eq_sym=expand(eq_sym)