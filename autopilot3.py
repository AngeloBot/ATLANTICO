
## LEME > 0 Ã‰ LEME PARA BE
## LEME < 0 Ã‰ LEME PARA BB
## RUMO AUMENTANDO Ã‰ PROA INDO PARA BE; v > 0
## RUMO DIMINUINDO Ã‰ PROA INDO PARA BB; v < 0
## E < 0 Ã‰ RUMO IDEAL PARA BB
## E > 0 Ã‰ RUMO IDEAL PARA BE

#criar seguranÃ§a para que o leme nao fique fixo em valor maior que 10 graus
#o zero do leme nÃ£o pode ser tÃ£o grande
#voltar o leme pro centro nesse caso

def main():
    rumo_ideal = 180
    rumo_ant = 180  #no 1o segundo colocar rumo_real = rumo_ideal
    SOMAE = 0
    leme = 0
    for i in range(50):     #a cada segundo, entra um novo rumo_real
        #if i > 0:
         #   rumo_real = int(input('rumo_real: '))
        if i == 0:
            rumo_real = 180
        #if rumo_real == 999:  #parar o loop
         #   break
        if i == 1:
            rumo_real = 220    #mudanÃ§a repentina de rumo
        if i == 30:
            rumo_real = 200    #2a mudança repentina de rumo
        E = rumo_ideal - rumo_real
        print('E: ', end = '')
        print(E)
        v = rumo_real - rumo_ant   #velocidade angular
        #print('v: ', end = '')
        #print(v)

        SOMAE += E	       ## integraÃ§Ã£o do erro
        Kp = 2.5
        Ki = 0.04
        Kd = 3.8
        
        leme = Kp*E + Ki*SOMAE - Kd*v           ## leme em direÃ§ao ao rumo ideal; resposta proporcional
        
        rumo_ant = rumo_real
        #momento = inÃ©rcia*aceleraÃ§Ã£o
        #momento = inÃ©rcia*delta_v
        #delta_v = momento/inÃ©rcia
        #delta_rumo = v*1segundo = v
        #MOMENTO > 0 Ã‰ MOMENTO PARA VIRAR PARA BE
        momento_vela = 10  #momento gerado pela vela
        
        if leme > 90:    # condiÃ§oes impossÃ­veis de leme
            leme = 90
        elif leme < -90:
            leme = -90
        print('leme: ', end = '')
        print(leme)
        momento_leme = leme  #momento gerado pelo leme
        b = 0.3    #constante de amortecimento
        momento_amort = -v*b    #momento gerado pelo amortecimento hidrodinamico
        momento = momento_vela + momento_leme + momento_amort   #momento resultante
        #print('momento: ', end = '')
        #print(momento)
        #o barco tem que estabilizar o leme em leme = -momento_vela
        inercia = 5    #inercia inventada do barco
        delta_v = momento/inercia  #variaÃ§ao de velocidade naquele segundo sujeito Ã quele momento
        v += delta_v
        delta_rumo = v
        rumo_real += delta_rumo
    print("Kp*E = ", end = '')
    print(Kp*E)
    print("Ki*SOMAE = ", end = '')
    print(Ki*SOMAE)
    print("Kd*v = ", end = '')
    print(-Kd*v)

main()