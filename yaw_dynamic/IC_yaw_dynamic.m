clear all;
close all;
clc;

diary 

%script de declaracao simbolica
IC_yaw_syms_declaration;

%script de equacionamento
IC_yaw_equation;

%valores dos parametros
V= 0.3;    %m/s
J_zz=0.14;  %kg*m²
P_i=3.1415; %adim
M_vela=0.02;%N*m (max=0.04 N*m)
b=0.013;    %kg*m/s
d=0.37;     %m
AR=3.33;     %adim
Ap=0.0092;  %m²
ro=997;     %kg/m³
a_0=0.104;   %adim

%paramentros de C_L_inf
c0=5e-3;
c1=3.55e-3;
c2=-1.2e-3;
c3=1.46e-4;
c4=-3.56e-6;
c5=-3.53e-7;
c6=1.73e-8;

diary IC_yaw.out
eq_num=subs(eq_sym)
eq_num=simplify(eq_num)
diary off

%{
eq_lin=subs(eq_num,cos((37*theta)/30)),1)
eq_lin=subs(eq_num,
%}
%eq_collect=collect(eq_num,alpha_leme)
