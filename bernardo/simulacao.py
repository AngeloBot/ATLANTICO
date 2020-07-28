import numpy as np
import matplotlib.pyplot as plt
import math
import random
## LEME > 0 Ã‰ LEME PARA BE
## LEME < 0 Ã‰ LEME PARA BB
## RUMO AUMENTANDO Ã‰ PROA INDO PARA BE; v > 0
## RUMO DIMINUINDO Ã‰ PROA INDO PARA BB; v < 0
## E < 0 Ã‰ RUMO IDEAL PARA BB
## E > 0 Ã‰ RUMO IDEAL PARA BE
#MOMENTO > 0 É MOMENTO PARA VIRAR PARA BE
#MOMENTO < 0 É MOMENTO PARA VIRAR PARA BB

#criar seguranÃ§a para que o leme nao fique fixo em valor maior que 10 graus
#o zero do leme nÃ£o pode ser tÃ£o grande
#voltar o leme pro centro nesse caso

def erro_rumo(rumo_real, rumo_ideal):
    E = rumo_ideal - rumo_real
    if abs(E) <= 180:
        pass
    elif abs(E) > 180 and E > 0:
        E -= 360
    elif abs(E) > 180 and E < 0:
        E += 360
    return E

def biruta(rumo_real, dir_vento):
    ## VENTO POR BE É ALPHA > 0
    ## VENTO POR BB É ALPHA < 0
    alpha = dir_vento - rumo_real
    if abs(alpha) <= 180:
        pass
    elif abs(alpha) > 180 and alpha > 0:
        alpha -= 360
    elif abs(alpha) > 180 and alpha < 0:
        alpha += 360
        
    if alpha >= 70 or alpha <= -70:
        hall = '0000'
    elif alpha < 40 and alpha > -40:
        hall = '0110'
        
    elif alpha >= 40 and alpha < 50:
        hall = '0010'
    elif alpha >= 50 and alpha < 60:
        hall = '0011'
    elif alpha >= 60 and alpha < 70:
        hall = '0001'
        
    elif alpha <= -40 and alpha > -50:
        hall = '0100'
    elif alpha <= -50 and alpha > -60:
        hall = '1100'
    elif alpha <= -60 and alpha > -70:
        hall = '1000'
        
    return hall
    
def atan3(y,x):
    if x == 0 and y > 0:
        v = 0
    elif x == 0 and y < 0:
        v = math.pi
    elif y == 0 and x > 0:
        v = math.pi/2
    elif y == 0 and x < 0:
        v = 3*math.pi/2   
    
    elif y > 0 and x > 0:
        v = -math.atan(y/x) + math.pi/2
    elif y > 0 and x < 0:
        v = 3*math.pi/2 - math.atan(y/x)
    elif y < 0 and x < 0:
        v = 3*math.pi/2 - math.atan(y/x)
    elif y < 0 and x > 0:
        v = math.pi/2 - math.atan(y/x)
    return v

def rumoideal(xbarco, ybarco, xwaypoint, ywaypoint):
    rumo_ideal = atan3((ywaypoint - ybarco), (xwaypoint - xbarco))
    rumo_ideal = (180/np.pi)*rumo_ideal
    return(rumo_ideal)
    
def lemeok(leme):
    if leme > 30:    # condiÃƒÂ§oes impossÃ­Â­veis de leme
        leme = 30
    elif leme < -30:
        leme = -30
    return leme
    

def main():
    waypoints = [[40,0],[40,40],[0,40],[0,80], [40,80], [0,0]]
    plt.figure(figsize=(5,5))
    plt.axes().set_aspect('equal', 'datalim')
    for i in range(len(waypoints)):
        plt.scatter(waypoints[i][0], waypoints[i][1])
    dir_vento = 0
    estado = 'estado'
    lamb = 2
    lamb2 = 10
    dist = 4
    j = 0  
    chegou = False
    SOMAE = 0
    leme = 0
    passo = 0.01   #em segundos
    posx = []
    posy = []
    t = 0
    i = 0
    k = 0
    s = 0
    momento_oscilante = 0
    while (not chegou):     #TEM QUE SER WHILE

        t += passo
        if estado == 'estado':
            estado = 0    #estado inicial
            estado_anterior = 0
            v = 0  #velocidade angular inicial
            rumo_real = 180  #rumo inicial
            xbarco = 0
            ybarco = 0
            
        #TROCA DE WAYPOINT
        if ((xbarco - waypoints[j][0])**2 + (ybarco - waypoints[j][1])**2)**0.5 < dist:
            j += 1
        if j >= len(waypoints):
            chegou = True
            print('CHEGOU')
            break
        
        rumo_ideal = rumoideal(xbarco, ybarco, waypoints[j][0], waypoints[j][1])
        hall = biruta(rumo_real, dir_vento)
        
        #CONTROLADOR DE CADA ESTADO
        
        if SOMAE > 30:
            SOMAE = 0
            print('zerou SOMAE')
        Kp = 2
        Ki = 0.5
        Kd = 0.5
        
        if estado == 0:
            E = erro_rumo(rumo_real, rumo_ideal)
            SOMAE += E*passo
            leme = Kp*E + Ki*SOMAE - Kd*v
            leme = lemeok(leme)
            
        elif estado == 3:
            E = erro_rumo(rumo_real, rumo_ideal)
            if estado_anterior != 3: #acabou de entrar no estado
                if estado_anterior == 0 or estado_anterior == 9:
                    cte = -5   #5 graus orcado
                    if abs(E) < lamb:
                        cte = 0
                elif estado_anterior == 7 or estado_anterior == 4:
                    cte = 5   #5 graus arribado
                elif estado_anterior == 5:
                    cte = 0
                rumo_ideal2 = rumo_real + cte
            E = erro_rumo(rumo_real, rumo_ideal2)
            SOMAE += E*passo
            leme = Kp*E + Ki*SOMAE - Kd*v
            leme = lemeok(leme)
            if estado_anterior == 12:
                leme = 0
                    
            
        elif estado == 5:
            E = erro_rumo(rumo_real, rumo_ideal)
            SOMAE += E*passo
            leme = Kp*E + Ki*SOMAE - Kd*v
            leme = lemeok(leme)
            
        elif estado == 7:
            if hall == '0100':
                E = 15
            elif hall == '0110':
                E = 40
            SOMAE = 0
            leme = Kp*E - Kd*v
            leme = lemeok(leme)
            
        elif estado == 9:
            E = -10
            SOMAE = 0
            leme = Kp*E - Kd*v
            leme = lemeok(leme)
            
        elif estado == 11:
            leme = 30
            SOMAE = 0
            
        elif estado == 4:
            E = erro_rumo(rumo_real, rumo_ideal)
            if estado_anterior != 4: #acabou de entrar no estado
                if estado_anterior == 0 or estado_anterior == 10:
                    cte = 5
                    if abs(E) < lamb:
                        cte = 0
                elif estado_anterior == 8 or estado_anterior == 3:
                    cte = -5
                elif estado_anterior == 6:
                    cte = 0
                rumo_ideal2 = rumo_real + cte
            E = erro_rumo(rumo_real, rumo_ideal2)
            SOMAE += E*passo
            leme = Kp*E + Ki*SOMAE - Kd*v
            leme = lemeok(leme)
            if estado_anterior == 11:
                leme = 0
            
        elif estado == 6:
            E = erro_rumo(rumo_real, rumo_ideal)
            SOMAE += E*passo
            leme = Kp*E + Ki*SOMAE - Kd*v
            leme = lemeok(leme)
            
        elif estado == 8:
            if hall == '0010':
                E = -15
            elif hall == '0110':
                E = -40
            SOMAE = 0
            leme = Kp*E - Kd*v
            leme = lemeok(leme)
            
        elif estado == 10:
            E = 10
            SOMAE = 0
            leme = Kp*E - Kd*v
            leme = lemeok(leme)
            
        elif estado == 12:
            leme = -30
            SOMAE = 0
    

        #MOMENTO > 0 É MOMENTO PARA VIRAR PARA BE
        V = 0.5   #velocidade do barco
        ro = 997  #kg/m3
        AR = 3.33  #razao de aspecto do leme
        d = 0.37   #braço da força do leme
        Ap = 0.0092  #área projetada do leme
        Vrot = ((np.pi)/180)*v*d  #velocidade linear induzida pela velocidade angular
        Vres = (Vrot**2 + V**2)**0.5  #velocidade resultante no leme
        if V == 0:
            angulo_V = 90
        else:
            angulo_V = abs((180/np.pi)*(np.arctan(abs(Vrot)/abs(V))))  #angulo entre V e Vrot em modulo
        if v > 0:
            angulo_V = -1*angulo_V    #angulo_V com sinal negativo quando o leme no centro faria momento para BB
        angulo_res1 = leme + angulo_V  #angulo de ataque no leme
        angulo_res = abs(angulo_res1)   #angulo de ataque no leme em modulo
        #ANGULO_RES > 0 É MOMENTO DO LEME PARA VIRAR À BE
        a0 = 0.104   #inclinacao do diagrama CL x angulo_res para NACA0012
        CL = a0*angulo_res   # CL para AR infinita  
        CD = 5*10**-3 + 3.55*10**-3*angulo_res -1.2*10**-3*angulo_res**2 + 1.46*10**-4*angulo_res**3 + -3.56*10**-6*angulo_res**4 - 3.53*10**-7*angulo_res**5 + 1.73*10**-8*angulo_res**6  #CD para AR infinita
        CD += CL**2/(np.pi*AR)   #CD para AR finita
        if CD > 0.15:
            CD = 0.15    #correcao de CD no estol
        a = a0/(1 + a0/(np.pi*AR))  #correcao de a0 para AR finita
        CL = a*angulo_res   #CL para AR finita
        if CL > 1.4:
            CL = 1.4   #correcao de CL no estol
            if angulo_res > 30:    #leme estolado
                CL = 0.5
        L = 0.5*CL*ro*(Vres**2)*Ap   #calculo do lift
        D = 0.5*CD*ro*(Vres**2)*Ap   #calculo do drag
        
        Fleme = abs(abs(L*np.cos((np.pi/180)*angulo_V)) - abs(D*np.sin((np.pi/180)*angulo_V)))  #calculo da força longitudinal do leme
        if angulo_res1 < 0:
            Fleme = -1*Fleme
        momento_leme = Fleme*d   #calculo do momento do leme
        
        momento_vela = 0.1
        i += passo
        k += passo
        s += passo
        if i > 1:
            momento_oscilante = random.randrange(-40, 40)*0.01
            i = 0
        if k > 5:
            dir_vento = random.randrange(0, 30)
            if t > 450:
                dir_vento = random.randrange(160, 180)
            k = 0
        if s > 5.1:
            plt.text(xbarco, ybarco, estado)
            #origin = [xbarco], [ybarco]
            #plt.quiver(*origin, -math.sin(dir_vento*math.pi/180), -math.cos(dir_vento*math.pi/180), scale = 10)
            s = 0
        
        b = 0.013    #constante de amortecimento hidrodinamico
        momento_amort = -v*b    #momento gerado pelo amortecimento hidrodinamico
        momento =  momento_amort + momento_leme + momento_vela + momento_oscilante  #momento resultante
        inercia = 0.14    #momento de inercia do barco. FALTA INERCIA ADICIONAL
        delta_v = (180/np.pi)*passo*momento/inercia  #variaÃ§ao de velocidade em grau/s naquele periodo de tempo sujeito Ã quele momento
        v += delta_v
        delta_rumo = v*passo  #variacao de rumo naquele periodo de tempo
        rumo_real += delta_rumo
        if rumo_real >= 360:
            rumo_real -= 360
        elif rumo_real < 0:
            rumo_real += 360
        delta_s = V*passo
        delta_x = delta_s*math.sin((np.pi/180)*rumo_real)
        xbarco += delta_x
        delta_y = delta_s*math.cos((np.pi/180)*rumo_real)
        ybarco += delta_y
        posx.append(xbarco)
        posy.append(ybarco)
        
        #print('leme = ', leme)
        print('estado = ', estado)
        #print(hall, end = ' ')
        #print(estado, end = '; ')
        #print('rumo_real = ', rumo_real)
        #print('rumo_ideal = ', rumo_ideal)
        #print('xbarco = ', xbarco)
        #print('ybarco = ', ybarco)
        #print('E = ', erro_rumo(rumo_real, rumo_ideal))
        #print('hall = ', hall)
        #print('v = ', v)
        #print('momento_oscilante = ', momento_oscilante)
        #print('dir_vento = ', dir_vento)
        #print('tempo = ', t)
        #print('')
        
        #TROCA DE ESTADO
        
        estado_anterior = estado
        E = erro_rumo(rumo_real, rumo_ideal)
        
        if estado == 0:
            if hall == '1000' or hall == '1100' or hall == '0100' or hall == '0110':
                estado = 3
            elif hall == '0001' or hall == '0011' or hall == '0010':
                estado = 4
        
        elif estado == 3:
            if hall == '0000':
                estado = 0
            elif hall == '0110' or hall == '0100':
                estado = 7
            elif E > 0 and abs(E) > lamb:
                estado = 5
            elif E < 0 and abs(E) > lamb and hall == '1000':
                estado = 9
            elif hall == '1100' and E < -110:
                estado = 11
            elif hall == '0001' or hall == '0011' or hall == '0010':
                estado = 4
        
        elif estado == 5:
            if abs(E) < lamb or E < 0 or (hall != '1000' and hall != '1100'):  #notar que conjunto de != entra com and
                estado = 3
                
        elif estado == 7:
            if hall != '0110' and hall != '0100':
                estado = 3
                
        elif estado == 9:
            if abs(E) < lamb or E > 0 or hall != '1000':
                estado = 3
                
        elif estado == 11:
            if abs(E) < lamb2 or (hall != '0100' and hall != '1100' and hall != '1000' and hall != '0000'):
                estado = 4
                
        elif estado == 4:
            if hall == '0000':
                estado = 0
            elif hall == '0110' or hall == '0010':
                estado = 8
            elif E < 0 and abs(E) > lamb:
                estado = 6
            elif E > 0 and abs(E) > lamb and hall == '0001':
                estado = 10
            elif hall == '0011' and E > 110:
                estado = 12
            elif hall == '1000' or hall == '1100' or hall == '0100':
                estado = 3
        
        elif estado == 6:
            if abs(E) < lamb or E > 0 or (hall != '0001' and hall != '0011'):
                estado = 4
                
        elif estado == 8:
            if hall != '0110' and hall != '0010':
                estado = 4
                
        elif estado == 10:
            if abs(E) < lamb or E < 0 or hall != '0001':
                estado = 4
                
        elif estado == 12:
            if abs(E) < lamb2 or (hall != '0010' and hall != '0011' and hall != '0001' and hall != '0000'):
                estado = 3
        
    
    plt.plot(posx, posy)
    

main()