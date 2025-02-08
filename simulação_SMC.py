####################################################
#                                                  #
#  Código para simular o comportamento do controle #
#      SMC implementado no Seletor de remédios     #
#                                                  #
####################################################

import numpy as np
import math as math
import matplotlib.pyplot as plt

################# Representação das equações #################

def calculateBeta(gama, K, M):

    Beta = K / (gama * M)
    return Beta

def FrequenciaNaturalaoQuadrado(K, M, J):

    FreqNat2 = K * ((1 / J) + (1 / M))
    return FreqNat2

def acoeficient(setpoint, vel, pos, Kv, Kp):

    ac = setpoint - (Kv * vel + Kp * pos)
    return ac

def calculatephi(pos, R): 

    phi = pos / R
    return phi

def calculatew(phi, pos):

    w = phi - pos
    return w

def wder(w, prevW, tempo):

    wder = (w - prevW) / tempo
    return wder

def acelaration(vel, prevVel, tempo):

    acel = (vel - prevVel) / tempo
    return acel

def TorqueEquivalente(Beta, FreqNat2, J, M, ac, alpha, w, derw):

    Teq = (Beta / FreqNat2) * (J + M) * ac - J * (alpha * derw + (Beta - FreqNat2) * w)
    return Teq

def calculateSigma(setpoint, acel, vel, pos, Kv, Kp, gama, wder2, wder1, alpha):

    sigma = setpoint - (acel + Kv * vel + Kp * pos + gama * (wder2 + alpha * wder1)) 
    return sigma

def calculateDsigma(sigma, prevSigma, tempo):

    Dsigma = - ((sigma - prevSigma) / tempo)
    return Dsigma

def TorqueUtilizado(Teq, FreqNat2, Beta, J, M, Dsigma):

    Tu = Teq + (Beta / FreqNat2) * (J + M) * Dsigma
    return Tu

#################################

# Parâmetros do sistema
K = 2.5                             # coeficiente de elasticidade 
# Ke = 0.01642                      # Constante CEMF
Kt = 0.012569                       # Constante de torque
M = 0.62                            # Massa do Load
J = 0.0000605                       # Inercia do motor
# C = 0.5                             
Kv = 1.7       
Kp = 1        
gama =  0.000097571124108       
alpha = 20
setpoint = 0.6
R = 0.011
tempo_total = 10                    # Tempo total de simulação
dt = 0.01                           # Passo de tempo

# Inicialização das variáveis
tempo = np.arange(0, tempo_total, dt)
pos = np.zeros_like(tempo)    # Posição
vel = np.zeros_like(tempo)    # Velocidade
w = np.zeros_like(tempo)      # Belt Strech 
wder1 = np.zeros_like(tempo)  # Derivada de w
sigma = np.zeros_like(tempo)  # Sigma
Dsigma = np.zeros_like(tempo) # Derivada de Sigma
acel = np.zeros_like(tempo)   # Aceleração

# Loop de simulação
for i in range(1, len(tempo)):
    # Cálculo das variáveis dependentes
    phi = pos[i-1] / R                                # Posição angular
    w[i] = phi - pos[i-1]
    wder1[i] = (w[i] - w[i-1]) / dt                   # Derivada de w
    wder2 = (wder1[i] - wder1[i-1]) / dt             
    ac = setpoint - (Kv * vel[i-1] + Kp * pos[i-1])

    # Cálculo do torque equivalente
    Beta = K / (gama * M)
    FreqNat2 = K * ((1 / J) + (1 / M))
    Teq = ((Beta / FreqNat2) * (J + M) * ac) - (J * (alpha * wder1[i] + (Beta - FreqNat2) * w[i])) 

    # Fatores não linear
    sigma[i] = setpoint - (acel[i-1] + Kv * vel[i-1] + Kp * pos[i-1] + gama * (wder2 + alpha * wder1[i]))
    Dsigma[i] = - ((sigma[i] - sigma[i-1]) / dt)
    tansigmoid = np.tanh(sigma[i])    

    # Filtro inicial no Dsigma para evitar que a derivada exploda
    if (i == 1 or i == 2):          
        Dsigma[i] = 0
    
    Tu = Teq + (tansigmoid * ((Beta / FreqNat2) * (J + M) * Dsigma[i]))

    # Torque máximo
    if (Tu >= 0.181):   
        Tu = 0.181  
    if (Tu <= -0.181): 
        Tu = -0.181  

    Vel_angular = vel[i-1] / R
    Eb = Vel_angular * Kt
    V = ((Tu * 0.805) / Kt) + Eb  
    PWM = (V / 18) * 255

    acel[i] = Tu / M
    vel[i] = vel[i-1] + acel[i] * dt  # Integração da aceleração
    pos[i] = pos[i-1] + vel[i] * dt   # Integração da velocidade

# Plotando os resultados
plt.figure(figsize=(10, 6))
plt.plot(tempo, pos, label="Posição")
plt.plot(tempo, vel, label="Velocidade")
plt.xlabel("Tempo (s)")
plt.ylabel("Valores")
plt.legend()
plt.title("Simulação Discreta do Sistema Motor-Cinta")
plt.grid()
plt.show()