%densidade da agua
syms ro

%variaveis de estado
syms ddtheta dtheta theta

%velocidade de avanco do barco
syms V

%velocidade tangencial
syms V_rot

%velocidade resultante em módulo
syms V_res

%angulo entre vetor velocidade resultante e rumo do barco (negativo se vel
%angular é para  boreste)
syms alpha_v

%angulo do leme (positivo se leme para boreste)
syms alpha_leme

%angulo de ataque do escoamento que chega ao leme
syms alpha_res

%inclinacao da reta de aproximacao
syms a_0

%razao de aspecto do leme e constante pi
syms AR P_i

%inclinacao corrigida
syms a

%area projetada do leme
syms Ap

% forca de lift e coeficientes de lift corrigido e para asa infinita
syms L C_L C_L_inf

%forca de drag coeficientes de drag corrigido e para asa infinita
syms D C_D C_D_inf
%coeficientes do polinomio aproximador para C_D_inf
syms c0 c1 c2 c3 c4 c5 c6

%momento da vela
syms M_vela

%momento do leme, forca do leme, braco do leme
syms M_leme F_leme d

%momento devido ao amortecimento, constante de amortecimento
syms M_amort b

%momento resultante, momento de inercia em yaw
syms M_res J_zz
