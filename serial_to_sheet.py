####################################################
#                                                  #
#  Código para converter dados obtidos no Serial   #
#  de uma ESP32 em uma tabela CSV e em um gráfico  #
#    utilizado para análise do comportamento do    #
#        controle do braço do manipulador          #
#                                                  #
####################################################

import matplotlib.pyplot as plt
import numpy as np
import serial
import pandas as pd
from scipy.interpolate import make_interp_spline

baudrate = 9600 # define o baudrate padrao
port = "COM4"
# counter = 0

ser = serial.Serial(port, baudrate) # tenta conectar ao Arduino
print("ESP conectado na porta", port)

my_list = list()
serial_out = 0
first = True
counter = 0
while(counter <= 20):
    if (first == True):
        first = ser.readline().decode().strip()
        print(first)

    serial_out = ser.readline().decode().strip()

    if (serial_out == "STAND-BY") :
        break
    data = serial_out.split(", ")
    my_list.append(tuple(data))
    print(data)
    counter += 1
    print(counter)

# Repete o resultado final 5 vezes para melhor representação no gráfico e ajusta a posição inicial
my_list[0] = ('0', '0', first)
for i in range(5):
    my_list.append(my_list[-1])

# Converte a lista para um arquivo csv
df = pd.DataFrame(list(my_list))
df.to_csv('Output.csv', index=False, header=False)

# Separa os valores da lista em listas indiviudais para cada medida 
master_values = [x for x, y, z in my_list]
slave_values = [y for x, y, z in my_list]
time = [z for x, y, z in my_list]

# Converte os valores obtidos em arrays de floats
master_values = np.array(master_values, dtype=float)
slave_values = np.array(slave_values, dtype=float)
time = np.array(time, dtype=float)

# Tira o valor inicial do tempo de todos os valores de tempo e converte de ms para s
first = float(first)
time = (time - first) / 1000

# Ajusta os tempos dos valores repetidos para melhor vizualização
for i in range(6):
    time[i - 6] = time[i - 7] + 0.025


# Faz a interpolação dos dados para suavizar as linhas do gráfico
spl_master = make_interp_spline(time, master_values, k=3)  # k=3 para suavização cúbica
spl_slave = make_interp_spline(time, slave_values, k=2)

# Define um novo conjunto de dados interpolados
time = np.linspace(time.min(), time.max(), 500)
master_smooth = spl_master(time)
slave_smooth = spl_slave(time)

# Plota o gráfico
plt.figure(figsize=(10, 6))
plt.plot(time, master_smooth, label = "posição do master")
plt.plot(time, slave_smooth, label = "posição do slave")
plt.axhline(100, color = 'gray', linestyle = '--')
plt.xlabel("Tempo (s)")
plt.ylabel("Posição")
plt.legend()
plt.title("Resposta do PID_incremental")
plt.grid()
plt.show()

