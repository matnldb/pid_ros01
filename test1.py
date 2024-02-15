# import numpy as np
# import pyswarm

# deseada = 5
# actual = 0
# error_acumulado = 0

# def pid(kp, kd,ki,e,e0,ie):
    
#     de = np.subtract(e, e0) * 100
#     ie = ie+np.add(e, e0)*0.005
#     pid_out = kp * e + kd * de + ki*ie
#     return pid_out
     

# # Definir la funcin objetivo para minimizar el error medio cuadrtico
# def objective_function(x):
    
#     # Coeficientes a optimizar
#     kp = x[0]
#     kd = x[1]
#     ki = x[2]
    
#     # Restaurar los valores iniciales de las variables
#     actual = 0
#     error_acumulado = 0
#     e = deseada - actual
#     vector = [e]
#     e0 = 0    
#     ie = 0
    
#     # Ejecutar el controlador PID y acumular el error cuadrtico
#     for _ in range(100):  
#         e = deseada - pid(kp, kd,ki,e,e0,ie)
#         e0 = e
#         vector.append(e)
#         error_acumulado += e**2
#         if e == 0:
#             contador +=1
#         else: contador = 0
#         if contador == 5:
#             break
    
#     # Calcular el error medio cuadrtico
#     mse = error_acumulado / (len(vector)-1)    
#     return mse

# inferior = [0, 0, 0]
# superior = [10,10,20]

# # Ejecutar el algoritmo PSO para optimizar los coeficientes
# xopt, fopt = pyswarm.pso(objective_function, inferior, superior)

# # Imprimir los resultados
# print("Los mejores coeficientes encontrados:")
# print("kp =", xopt[0])
# print("kd =", xopt[1])
# print("ki =", xopt[2])
# print("Valor mnimo del error medio cuadrtico:", fopt)
import numpy as np
a = [1,2,3,4]
b= [3,5]

c = np.add(a[:2],b)
d = np.hstack((c, a[2:4]))
print(len(d))