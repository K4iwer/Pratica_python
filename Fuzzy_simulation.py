####################################################
#                                                  #
#  Código para simular o comportamento do controle #
#     Fuzzy implementado no seletor de remédios    #
#                                                  #
####################################################

import numpy as np
import math as math
from scipy.integrate import solve_ivp
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def manipulator_dynamics(t, state, PWM):
    # Simula o comportamento do Motor DC 
    pos, vel = state
    Kt = 0.012569                                  # Constante de Torque do Motor
    M = 0.6                                        # Massa do Manipulador
    R = 5.18                                       # Resistência Motor
    C = 0.5                         
    Vel_angular = vel / 0.02
    V = (PWM / 255) * 12
    Eb = Vel_angular * Kt
    torque = (Kt * (V - Eb)) / R
    accel = ((torque / 0.02) - C * vel) / M        # Simulação Motor DC
    vel = vel + (accel * t)
    pos = pos + (vel * t)
    return [pos, vel]

# Inputs/Output
error = ctrl.Antecedent(np.arange(-600, 601, 1), 'error')
error_velocity = ctrl.Antecedent(np.arange(-800, 801, 1), 'error_velocity')
PWM = ctrl.Consequent(np.arange(-255, 256, 1), 'PWM')

# Member Functions
error['negative_high'] = fuzz.trapmf(error.universe, [-600, -600, -300, -250])
error['negative'] = fuzz.trimf(error.universe, [-300, -250, 0])
error['zero'] = fuzz.trimf(error.universe, [-300, 0, 300])
error['positive'] = fuzz.trimf(error.universe, [0, 250, 300])                         
error['positive_high'] = fuzz.trapmf(error.universe, [250, 300, 600, 600])

error_velocity['negative_high'] = fuzz.trapmf(error_velocity.universe, [-800, -800, -400, -135])
error_velocity['negative'] = fuzz.trimf(error_velocity.universe, [-135, -85, 0])
error_velocity['zero'] = fuzz.trimf(error_velocity.universe, [-85, 0, 85])
error_velocity['positive'] = fuzz.trimf(error_velocity.universe, [0, 85, 135])
error_velocity['positive_high'] = fuzz.trapmf(error_velocity.universe, [135, 400, 800, 800])

PWM['NVH'] = fuzz.trimf(PWM.universe, [-200, -200, -184])        # Negative Very High
PWM['NH'] = fuzz.trimf(PWM.universe, [-200, -184, -146])         # Negative High
PWM['NM'] = fuzz.trimf(PWM.universe, [-184, -146, -10])          # Negative Medium
PWM['NS'] = fuzz.trimf(PWM.universe, [-146, -10, 0])             # Negative Small
PWM['Z'] = fuzz.trimf(PWM.universe, [-120, 0, 120])              # Zero
PWM['PS'] = fuzz.trimf(PWM.universe, [0, 10, 146])               # Positive Small
PWM['PM'] = fuzz.trimf(PWM.universe, [10, 146, 184])             # Positive Medium
PWM['PH'] = fuzz.trimf(PWM.universe, [146, 184, 200])            # Positive High
PWM['PVH'] = fuzz.trimf(PWM.universe, [184, 200, 200])           # Positive Very High

# error.view()          # Mostra as funções de pertinência de 'pos'
# error_velocity.view() # Mostra as funções de pertinência de 'vel'
# PWM.view()            # Mostra as funções de pertinência de 'PWM'

# plt.show()            # Mostra os gráficos

# Regras de Controle
rule1  = ctrl.Rule(error['positive_high'] & error_velocity['positive_high'], PWM['PVH'])
rule2  = ctrl.Rule(error['positive_high'] & error_velocity['positive'], PWM['PVH'])
rule3  = ctrl.Rule(error['positive_high'] & error_velocity['zero'], PWM['PVH'])
rule4  = ctrl.Rule(error['positive_high'] & error_velocity['negative'], PWM['PH'])
rule5  = ctrl.Rule(error['positive_high'] & error_velocity['negative_high'], PWM['PS'])
rule6  = ctrl.Rule(error['positive'] & error_velocity['positive_high'], PWM['PVH'])
rule7  = ctrl.Rule(error['positive'] & error_velocity['positive'], PWM['PVH'])
rule8  = ctrl.Rule(error['positive'] & error_velocity['zero'], PWM['PH'])
rule9  = ctrl.Rule(error['positive'] & error_velocity['negative'], PWM['PM'])
rule10 = ctrl.Rule(error['positive'] & error_velocity['negative_high'], PWM['PS'])
rule11 = ctrl.Rule(error['zero'] & error_velocity['positive_high'], PWM['PM'])
rule12 = ctrl.Rule(error['zero'] & error_velocity['positive'], PWM['PS'])
rule13 = ctrl.Rule(error['zero'] & error_velocity['zero'], PWM['Z'])
rule14 = ctrl.Rule(error['zero'] & error_velocity['negative'], PWM['NS'])
rule15 = ctrl.Rule(error['zero'] & error_velocity['negative_high'], PWM['NM'])
rule16 = ctrl.Rule(error['negative'] & error_velocity['positive_high'], PWM['NS'])
rule17 = ctrl.Rule(error['negative'] & error_velocity['positive'], PWM['NM'])
rule18 = ctrl.Rule(error['negative'] & error_velocity['zero'], PWM['NH'])
rule19 = ctrl.Rule(error['negative'] & error_velocity['negative'], PWM['NVH'])
rule20 = ctrl.Rule(error['negative'] & error_velocity['negative_high'], PWM['NVH'])
rule21 = ctrl.Rule(error['negative_high'] & error_velocity['positive_high'], PWM['NS'])
rule22 = ctrl.Rule(error['negative_high'] & error_velocity['positive'], PWM['NH'])
rule23 = ctrl.Rule(error['negative_high'] & error_velocity['zero'], PWM['NVH'])
rule24 = ctrl.Rule(error['negative_high'] & error_velocity['negative'], PWM['NVH'])
rule25 = ctrl.Rule(error['negative_high'] & error_velocity['negative_high'], PWM['NVH'])

PWM_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25])
PWM_simulation = ctrl.ControlSystemSimulation(PWM_ctrl)

############ fazer surface plot ###############
 
# Gera valores para as entradas 'error' e 'error_velocity'
x = np.linspace(-800, 800, 100)  # 'error'
y = np.linspace(-50, 50, 10)  # 'error_velocity'
X, Y = np.meshgrid(x, y)  # Gera a malha para o gráfico
Z = np.zeros_like(X)  # Inicializa a matriz de saída 'PWM'

# Calcula o valor da saída 'PWM' para cada par de entradas
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        PWM_simulation.input['error'] = X[i, j]
        PWM_simulation.input['error_velocity'] = Y[i, j]
        PWM_simulation.compute()
        Z[i, j] = PWM_simulation.output['PWM']

# Cria o gráfico de superfície
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, cmap='viridis')

# Configurações dos eixos
ax.set_xlabel('error')
ax.set_ylabel('error Velocity')
ax.set_zlabel('PWM')
ax.set_title('Fuzzy Control Surface')

plt.show()

####################################

# Inicialização da simulação
t_total = 10               # Tempo total
dt = 0.1                   # Passo de tempo
steps = int(t_total / dt)  # Número de passos
state = [0, 0]             # Estado inicial: posição e velocidade
desired_position = 100     # Posição desejada

time = [0]                 # Lista de tempo
positions = [state[0]]     # Lista de posições

# Simulação do Sistema
for step in range(steps):
    current_time = step * dt
    current_error = desired_position - (state[0] * 1000)
    current_error_velocity = -(state[1] * 1000)

    # Controle Fuzzy
    PWM_simulation.input['error'] = current_error
    PWM_simulation.input['error_velocity'] = current_error_velocity
    PWM_simulation.compute()
    PWM_output = PWM_simulation.output['PWM']

    state = manipulator_dynamics(dt, state, PWM_output)

    # Salvar dados para gráfico
    time.append(current_time + dt)
    positions.append(state[0] * 1000)

# Plotar os resultados
plt.figure()
plt.plot(time, positions, label="Posição")
plt.axhline(desired_position, color='r', linestyle='--', label="Posição Desejada")
plt.xlabel("Tempo (s)")
plt.ylabel("Posição")
plt.legend()
plt.grid()
plt.show()



