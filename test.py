import ReTimeSystems
import time

if __name__ == "__main__":
    print("Iniciando")
    
    
    
    
    s_time = 2.0
    t_start = time.perf_counter() #round(time.perf_counter(), 6)
    elapsedTime = time.perf_counter()-t_start #round(time.perf_counter(), 6)-t_start
    
    while(elapsedTime <= s_time):
        # Here the computation.
        #
        # ...
        ant_elapsed = elapsedTime
        elapsedTime = time.perf_counter()-t_start
        step = elapsedTime-ant_elapsed
        print(elapsedTime)
        print(step)


