import numpy as np
import math

class TransferFunction:
    def __init__(self, num, den, y0=None, u0=None):
        '''Funcion de transferencia de un sistema:\n
        an*y^(n) + ... + a1*y' + a0*y = bm*u^(m) + ... + b1*u' + b0*u\n\n
        den = [an, ..., a1, a0];    num = [bm, ..., b1, b0]; .\n\n
        y0 = [y'(0),..., y^(n)(0)]; u0 = [u'(0),..., u^(m)(0)]'''
        self.m = len(num)-1 
        self.n = len(den)-1
        self.a = den[::-1]
        self.b = num[::-1]

        if u0 is None:
            self.u0 = np.zeros(self.m + 1) # dimensión adicional para incluir a u(t).
        else:
            self.u0 = np.insert(u0,0,0) # dimensión adicional para incluir a u(t).

        if y0 is None:
            self.y0 = np.zeros(self.n + 1)
        else:
            self.y0 = np.array([y0])

    def aplicar_tf(self, ut, dt=0.001):
        '''Aplica la función de transferencia a la señal: ut\n\n
        Regresa la salida en el dominio del tiempo.'''
        self.u0[0] = ut

        u_term = 0
        for i in range(self.m + 1):
            for j in range(i, self.m + 1):
                u_term += (-1)**i*math.comb(j,i)*self.b[j]*self.u0[i]/(dt**j)
        
        y_term = 0
        for i in range(self.n):
            for j in range(i, self.n):
                y_term += (-1)**i*math.comb(j,i)*self.a[j]*self.y0[i]/(dt**j)
    
        y_dif = 0
        for i in range(1, self.n+1):
            y_dif += (-1)**i*math.comb(self.n,i)*self.y0[i-1]

        yt = dt**self.n*(u_term-y_term)/self.a[self.n] - y_dif

        self.u0 = np.roll(self.u0, 1)
        self.y0 = np.roll(self.y0, 1)
        #if len(self.y0) > 0: 
        self.y0[0] = yt
        #else:
        #    self.y0 = np.array([yt])

        return yt

class ZeroHolder:
    def __init__(self, sample_time=0.05):
        self.sample_time = sample_time
        self.last_st = 0.0

    def isTime(self, ti):
        st = ti - ti % self.sample_time
        if st != self.last_st:
            self.last_st = st
            return True
        return False

class Derivador:
    def __init__(self, y0=0, t0=0) -> None:
        self.y0 = y0
        self.t0 = t0

    def aplicar_derivada(self, ti, y):
        yp = (y-self.y0) / (ti-self.t0)
        self.t0=ti
        self.y0=y
        return yp

class Integrador:
    def __init__(self, t0=0, y0=0) -> None:
        self.t0=t0
        self.y0=y0

    def aplicar_integral(self, ti, y):
        iy = self.y0 + 0.5*(ti-self.t0)*(y+self.y0)
        self.t0=ti
        self.y0=y
        return iy

