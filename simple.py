import numpy as np
import pyswarm

deseada = 5
ie = 0

def pid(kp, kd, ki, e, e0):
    global ie
    de = (e-e0) * 100
    ie = ie + (e + e0) * 0.005
    pid_out = kp * e - kd * de + ki * ie
    return pid_out

# Definir la funcin objetivo para minimizar el error medio cuadrtico
def objective_function(x):
    global ie, deseada
    # Coeficientes a optimizar
    kp = x[0]
    kd = x[1]
    ki = x[2]

    # Restaurar los valores iniciales de las variables
    actual = 0
    error_acumulado = 0
    e = deseada - actual
    vector = [e]
    e0 = 0    
    ie = 0
    
    # Ejecutar el controlador PID y acumular el error cuadrtico
    for _ in range(100):  
        e = deseada - pid(kp, kd, ki, e, e0)
        e0 = vector[-1]
        vector.append(e) 
        
    for i in vector:
        error_acumulado += (i**2)/len(vector)
    
    #print(x, error_acumulado)        
    return error_acumulado

inferior = [0, 0, 0]
superior = [10, 10, 20]

# Ejecutar el algoritmo PSO para optimizar los coeficientes
xopt, fopt = pyswarm.pso(objective_function, inferior, superior)

# Imprimir los resultados
print("Los mejores coeficientes encontrados:")
print("kp =", xopt[0])
print("kd =", xopt[1])
print("ki =", xopt[2])
print("Valor mnimo del error medio cuadrtico:", fopt)

ex = deseada
ex0 = 0
ie = 0
vectorx = [deseada]
for _ in range(50):  
    ex = deseada - pid(xopt[0], xopt[1], xopt[2], ex, ex0)
    ex0 = vectorx[-1]
    vectorx.append(ex) 

print(vectorx)