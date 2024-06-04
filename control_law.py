""" 
Este módulo sirve para la implementación de leyes de control.
    La interfaz envia los datos obtenidos del Qube-Servo 2 como argumento a
        control_law(ti, pm, vm, pp, cm)
    o sólo el tiempo actual en cada periodo de muestreo a
        control_law_offline(ti)
    Ambas funciones operan dentro de un ciclo que se ejecuta al inicio de cada periodo de muestreo o paso de integración,
    por lo que no se deben incluir ciclos de espera activos dentro de estas funciones. 
    La información adquirida por las entradas ditales y analogicas está etiquetada de la siguiente manera
        ti -> instante de tiempo actual basado en el periodo de muestreo.
        pm -> posición del motor
        vm -> velocidad del motor
        pp -> posición del péndulo
        cm -> corriente del motor además .
    Para "control_law_offline" sólo proporciona "ti" como paso de integración. 

    La interfaz grafíca cuatro señales cuando se llama a "control_law", mientras que permite determinarlas numericamente
    cuando se emplea "control_law_offline". Adicionalmente, ambas funciones permiten la cración y vizualización de las siguientes señales:
        float array: {vp, u, r1, r2, r3, r4, a1, a2}
            etiquetadas como: velocidad del péndulo, señal de control, referencia de posición del motor, referencia de velocidad del motor.
            referencia de posición del péndulo, referencia de velocidad del péndulo, señal auxiliar 1 y señal auxiliar 2.
            Las señales auxiliares pueden emplearse para visualizar cualquier variable deseada, por ejemplo, la señal de error.
            En caso de no emplearse, deben asignarse en cero tanto en "control_law" como en "control_law_offline".               
"""
import numpy as np
import random

import matplotlib.pyplot as plt

from ReTimeSystems import TransferFunction, ZeroHolder

# Variables globales

t_span=[]
fi_array=[]
sim=[]
ep=[]

filtro_u0 = TransferFunction([20], [1, 20])
filtro_u1 = TransferFunction([20], [1, 20])

planta = TransferFunction([50], [1, 2])
integrador = TransferFunction([1], [1, 0])

filtro_i1 = TransferFunction([400, 0, 0], [1, 40, 400])
filtro_i2 = TransferFunction([-400, 0], [1, 40, 400])
filtro_i3 = TransferFunction([400], [1, 40, 400])

zH = ZeroHolder()
A = [[],[]]
B = []

#k_p, k_d = (4.000, 0.175) % official test values.
k_p, k_d = (400, 31.95)
bval = 171.7

# funciones adicionales

def DC_motor_PD(pm, vm, r):
    error = r - pm
    u = (k_p/bval) * error - (k_d/bval) * vm
    return u

#----------------> Funciones de vinculación

def control_law(ti, pm, vm, pp, cm, vp=0, u=0, r1=0, r2=0, r3=0, r4=0, a1=0, a2=0):
    r1 = square_wave(ti, 1, 0.1, 0) # Señal de onda cuadrada
    #r1 = ruido_blanco_de_banda_limitada(ti, 0.1, 0, 200) # Señal rica en frecuencias

    r2 = filtro_u0.aplicar_tf(r1)
    
    u = DC_motor_PD(pm, vm, r2) #sistema estable

    # B
    ypp_f = filtro_i1.aplicar_tf(pm)

    # A
    yp_f = filtro_i2.aplicar_tf(pm)
    uf = filtro_i3.aplicar_tf(u)

    a1 = yp_f
    a2 = ypp_f
    vp = uf

    # cada 50 ms almacenar un dato de salida de los filtros
    if zH.isTime(ti):
        A[0].append(yp_f)
        A[1].append(uf)
        B.append(ypp_f)

    return vp, u, r1, r2, r3, r4, a1, a2

def control_law_offline(ti, vm=0, pp=0, vp=0, u=0, cm=0, r1=0, r2=0, r3=0, r4=0, a1=0, a2=0):
    global t_span  # Declara t_span como global
    t_span.append(ti)  # Almacena el tiempo actual en t_span

    r1 = ruido_blanco_de_banda_limitada(ti, 0.1, 0, 100) # Señal rica en frecuencias
    r2 = filtro_u1.aplicar_tf(r1) # señal filtrada

    if len(t_span)>1:
        error = r2-sim[-1]
    else:
        error = r2
        ep.append(0)
    
    u = 25 * error - 0.8 * ep[-1]

    e_p = planta.aplicar_tf(u)
    ep.append(e_p)

    pm = integrador.aplicar_tf(ep[-1])
    sim.append(pm)
    
    #B
    ypp_f = filtro_i1.aplicar_tf(pm)

    #A
    yp_f = filtro_i2.aplicar_tf(pm)
    uf = filtro_i3.aplicar_tf(u)

    a1 = yp_f
    a2 = ypp_f
    vp = uf

    # cada 50 ms almacenar un dato de salida de los filtros
    if zH.isTime(ti):
        A[0].append(yp_f)
        A[1].append(uf)
        B.append(ypp_f)

    return pm, vm, pp, vp, u, cm, r1, r2, r3, r4, a1, a2

def funcion_adicional_fx():
    # Importante limpiar las variables almacenadas
    global t_span
    t_span = []
    outStr = ""

    if len(B) > 0:
        AnT = np.array(A)
        BnT = np.array(B)
        An = AnT.T
        Bn = BnT.T
        AnTA = np.matmul(AnT,An)
        eigs = np.linalg.eigvals(AnTA)
        cond = np.max(eigs)/np.min(eigs)
        A_inv = np.linalg.inv(AnTA)
        param = np.matmul(np.matmul(A_inv, AnT), Bn)
    
    outStr += "Identificación paramétrica" + "\n\n"
    outStr += "a = " + str(param[0]) + "\n"
    outStr += "b = " + str(param[1]) + "\n\n"
    #outStr += "condicionamiento = " + str(cond) + "\n\n"
    #outStr += "Buffer interno liberado: " + str(len(t_span)) + "\n"
    
    return outStr


#-----------------------------------------------------------------------------------------------------
def ruido_blanco_de_banda_limitada(ti, step, min_freq, max_freq, potencia=1, num_freq=1024, seed=120793):
    '''Ruido blanco de banda limitada.'''
    global t_span
    global frecuencias_del_espectro
    global fases
    if len(t_span) <= 1:
        random.seed(seed)  # Establece una semilla
        frecuencias_del_espectro = np.linspace(min_freq, max_freq, num_freq) # Mallado uniforme de frecuencias en la banda 
        fases = np.array([random.random() for _ in range(num_freq)]) * 2*np.pi # Inicialización aleatoria (con semilla) de la fase entre 0 y 2*pi
    if step > 0.001:
        ti = np.floor(ti/step)
    signal = np.array([np.sin(2 * np.pi * frecuencia* ti + fase) for frecuencia, fase in zip(frecuencias_del_espectro, fases)])
    rb = np.sum(signal)*np.sqrt(potencia/num_freq)
    return rb

def square_wave(t, amplitud=1, frec=1, fase=0):
    '''Señal de onda cuadrada: square( (2*pi*frec)*t + fase) \n\n
    pp = square_wave(ti, 1, 0.2)\n
    vp = filtro_u1.aplicar_tf(pp)'''
    square = amplitud * np.sign(np.sin(2 * np.pi * frec * t + fase))
    return square

def derivada(f, ti, *args):
    '''Derivada por diferencia finitas hacia a tras.\n Requiere almacenar t_span.\n fp=derivada(señal_de_ejemplo,ti)'''
    global t_span
    if len(t_span) > 1: 
        fp = ( f(ti, *args) - f(t_span[-2], *args) ) / ( ti-t_span[-2] )
    else:
        fp = 0 # np.nan # omitirá el primer elemento.
    return fp

def integral(f, ti, *args): 
    '''Integral mediante el método del trapecio.\n Requiere almacenar t_span y fi_array.\n fi=integral(señal_de_ejemplo,ti)'''
    global t_span
    if len(t_span) > 1: 
        fi = fi_array[-1] + (ti-t_span[-2])*(f(ti,*args)+f(t_span[-2],*args))/2
        fi_array.append(fi)
    else:
        fi_array.append(0) # inicializa en 0
        fi = 0 #np.nan # omitirá graficar el primer elemento.
    return fi

def señal_de_ejemplo(ti):
    '''
    r1 = señal_de_ejemplo(ti)\n
    r2 = derivada(señal_de_ejemplo, ti)\n
    r3 = integral(señal_de_ejemplo, ti)'''
    # Derivada: np.exp(-ti) + 0.5*np.cos(5*t) 
    # Integral: ti + np.exp(-ti) - 0.02*np.cos(5*ti) - 49/50
    return 1 - np.exp(-ti) + 0.1*np.sin(5*ti)



def debug_function():
    time_span = np.linspace(0, 10, 10000)
    out1 = []
    out2 = []

    for i in range(len(time_span)):
        ti = time_span[i]
        #pm = np.sin(ti)
        pm, vm, pp, vp, u, cm, r1, r2, r3, r4, a1, a2 = control_law_offline(ti)
        out1.append(pm)
        out2.append(r2)


    plt.figure(figsize=(10, 6))
    plt.plot(time_span, out1, label='f(t)')
    plt.plot(time_span, out2, label='ref')
    plt.title('Gráfico de f(t)')
    plt.xlabel('t')
    plt.ylabel('f(x)')
    plt.legend()
    plt.grid(True)
    plt.show()

    print(funcion_adicional_fx())

#debug_function()