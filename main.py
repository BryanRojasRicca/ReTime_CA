from multiprocessing import Process, Queue, Value, freeze_support, set_start_method
import time

from PyQt5.QtGui import QCloseEvent
import numpy as np

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel, QPushButton ,QCheckBox, QProgressBar, QMessageBox, QSizePolicy
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from quanser.hardware import HIL, HILError, StringProperty
import importlib.util

from control_law import control_law, control_law_offline, funcion_adicional_fx

class QubeServo2():
    def __init__(self, id='0'):
        self.id = id

        #Region: define read/write channels and buffers. 
            #Encoders
        self.r_encoder_channels = np.array([0, 1], dtype=np.uint32)
        self.r_num_encoder_channels = len(self.r_encoder_channels)
        self.r_encoder_buffer = np.zeros(self.r_num_encoder_channels, dtype=np.int32)
            #Analog input:  channel 0: Motor Current
        self.r_analog_channels = np.array([0], dtype=np.uint32)
        self.r_num_analog_channels = len(self.r_analog_channels)
        self.r_analog_buffer = np.zeros(self.r_num_analog_channels, dtype=np.float64)
            #Other input: channel 0: Motor Thacometer
        self.r_other_channels = np.array([14000], dtype=np.uint32)
        self.r_num_other_channels = len(self.r_other_channels)
        self.r_other_buffer = np.zeros(self.r_num_other_channels, dtype=np.float64)
            #Digital outputs: channel 0: Motor Enable/Disable
        self.w_digital_channels = np.array([0], dtype=np.uint32)
        self.w_num_digital_channels = len(self.w_digital_channels)
        self.w_digital_buffer = np.array([1], dtype=np.int8)
            #Analog outputs: chanel 0: Motor Voltage
        self.w_analog_channels = np.array([0], dtype=np.uint32)
        self.w_num_analog_channels = len(self.w_analog_channels)
        self.w_analog_buffer = np.array([0], dtype=np.float64)
            #Others outputs: channels 11000,11001,11002: Led RGB 
        self.w_other_channels = np.array([11000, 11001, 11002], dtype=np.uint32)
        self.w_num_other_channels = len(self.w_other_channels)
        self.w_other_buffer = np.array([0, 0, 1], dtype=np.float64)

        # Inicializar un objeto Qube-Servo 2 HIL, y verificar si es válido
        self.qube = HIL("qube_servo2_usb", id)
        if self.qube.is_valid():
            print("Qube-Servo operativo")
            # Definir la cuenta en 0 de los encoders 0 y 1.
            counts = np.array([0, 0], dtype=np.int32)
            self.qube.set_encoder_counts(self.r_encoder_channels, self.r_num_encoder_channels, counts)

            # Indicar el encendido
            self.qube.write_other(self.w_other_channels, self.w_num_other_channels, self.w_other_buffer)

            # Habilitar Motor
            self.qube.write_digital(self.w_digital_channels, self.w_num_digital_channels, self.w_digital_buffer)

    def write_voltage(self, voltage):
        self.w_analog_buffer = np.array([-1*voltage], dtype=np.float64)
        self.qube.write_analog(self.w_analog_channels, self.w_num_analog_channels, self.w_analog_buffer)

    def write_led(self, color=np.array([1,0,0], dtype=np.float64)):
        self.w_other_buffer = color
        self.qube.write_other(self.w_other_channels, self.w_num_other_channels, self.w_other_buffer)

    def read_position_and_speed(self):
        """ read_position_and_speed -> {pos_m, vel_m, pos_p, amp_m} """
        self.qube.read(self.r_analog_channels,self.r_num_analog_channels,self.r_encoder_channels, self.r_num_encoder_channels, None,0,self.r_other_channels, self.r_num_other_channels, self.r_analog_buffer,self.r_encoder_buffer, None,self.r_other_buffer)
        return -2*np.pi*self.r_encoder_buffer[0]/2048, -2*np.pi*self.r_other_buffer[0]/2048, -2*np.pi*self.r_encoder_buffer[1]/2048, float(self.r_analog_buffer[0])
    
    def stop_all(self):
        self.write_voltage(0) #Off motor

        self.write_led(np.array([0,0,1], dtype=np.float64)) # --------------> Blue color
        
        # Deshabilitar Motor
        self.w_digital_buffer = np.array([0], dtype=np.int32)
        self.qube.write_digital(self.w_digital_channels, self.w_num_digital_channels, self.w_digital_buffer) 

    def terminate(self):
        self.stop_all()

        self.write_led(np.array([1,0,0], dtype=np.float64)) # -------------------> Red color
        self.qube.close()

def load_module_from_file(filepath):
    spec = importlib.util.spec_from_file_location("user_module", filepath)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def adquisicion_de_datos(q_and_f):
    print("Subproceso...")
    card = None
    try:
        pass
        #user_module = load_module_from_file('control_law.py')
        #if not hasattr(user_module, 'control_law'):
        #    raise Exception("Función 'control_law' no encontrada.")
        #if not hasattr(user_module, 'control_law_offline'):
        #    raise Exception("Función 'control_law_offline' no encontrada.")
        #if not hasattr(user_module, 'funcion_adicional_fx'):
        #    raise Exception("Función 'funcion_adicional_fx' no encontrada.")
    except:
        q_and_f.qQubeErrors.put("Error en la lectura del archivo de la ley de control.")
    
    while not q_and_f.fSalir.value:
        if q_and_f.fConectar.value:
            q_and_f.fConectar.value = False
            if isinstance(card, QubeServo2):
                card.terminate()
                time.sleep(0.5)
            try:
                card = QubeServo2()
                q_and_f.qQubeErrors.put("Conexión exitosa")
                
                q_and_f.qinfoQube.put(card.qube.get_string_property(StringProperty.MANUFACTURER, 64))
                q_and_f.qinfoQube.put(card.qube.get_string_property(StringProperty.PRODUCT_NAME, 64))
                q_and_f.qinfoQube.put(card.qube.get_string_property(StringProperty.MODEL_NAME, 64))
                q_and_f.qinfoQube.put(card.qube.get_string_property(StringProperty.FIRMWARE_VERSION, 64))
                q_and_f.qinfoQube.put(card.qube.get_string_property(StringProperty.SERIAL_NUMBER, 64))

            except HILError as e:
                card = None
                print(e.get_error_message())
                q_and_f.qQubeErrors.put(e.get_error_message())

        if q_and_f.fIniciar.value:
            q_and_f.fIniciar.value = False

            t_span =[]
            pm_t = []
            vm_t = []
            pp_t = []
            vp_t = []
            u_t = []
            cm_t = []
            r1_t = []
            r2_t = []
            r3_t = []
            r4_t = []
            a1_t = []
            a2_t = []
            
            timeover = False
            
            t_start = round(time.perf_counter(), 6)
            elapsedTime = round(time.perf_counter(), 6)-t_start

            periodo_de_muestreo_deseado = 0.001
            timpo_de_espera_al_pdm = periodo_de_muestreo_deseado*0.999

            if card is not None:
                card.write_led(np.array([0.0, 1.0, 0.0], dtype=np.float64))

            while not timeover and not q_and_f.fDetener.value:
                if q_and_f.fSalir.value:
                    break

                ti = elapsedTime
                if card is not None:
                    pm, vm, pp, cm = card.read_position_and_speed()
                    try:
                        vp, u, r1, r2, r3, r4, a1, a2 = control_law(ti, pm, vm, pp, cm)
                        #vp, u, r1, r2, r3, r4, a1, a2 = user_module.control_law(ti, pm, vm, pp, cm)
                    except:
                        card.stop_all()
                        q_and_f.qQubeErrors.put("Error en la ley de control: control_law(pm, vm, pp, cm) -> (vp, u, r1, r2, r3, r4, a1, a2)")
                        break
                    else:
                        card.write_voltage(u)
                else:
                    try:
                        pm, vm, pp, vp, u, cm, r1, r2, r3, r4, a1, a2 = control_law_offline(ti)
                        #pm, vm, pp, vp, u, cm, r1, r2, r3, r4, a1, a2 = user_module.control_law_offline(ti)
                    except:
                        q_and_f.qQubeErrors.put("Error en la ley de control: control_law_offline(ti) -> (pm, vm, pp, vp, u, cm, r1, r2, r3, r4, a1, a2)")
                        break

                t_span.append(ti)
                pm_t.append(pm)
                vm_t.append(vm)
                pp_t.append(pp)
                vp_t.append(vp)
                u_t.append(u)
                cm_t.append(cm)
                r1_t.append(r1)
                r2_t.append(r2)
                r3_t.append(r3)
                r4_t.append(r4)
                a1_t.append(a1)
                a2_t.append(a2)

                while round(time.perf_counter(),6)-t_start < ti + timpo_de_espera_al_pdm:
                    pass
                
                elapsedTime = round(time.perf_counter(),6)-t_start
                
                if elapsedTime > 10:
                    timeover = True
            #fin de la ejecución.
            if card is not None:
                card.write_voltage(0)
                card.write_led(np.array([0, 0, 1], dtype=np.float64))

            try:
                fx_info = funcion_adicional_fx()
                #fx_info = user_module.funcion_adicional_fx()
            except:
                q_and_f.qQubeErrors.put("Error en la función: funcion_adicional_fx() -> str")

            try:
                if isinstance(fx_info, str):
                    q_and_f.qfx_info.put(fx_info)
            except:
                q_and_f.qfx_info.put("Output error: funcion_adicional_fx() -> str")

            if len(t_span) > 2: 
                info = "Tiempo inicial: " + str(t_span[0]) + "\n"
                info += "Tiempo final: " + str(t_span[-1]) + "\n \n"
                
                info += "No. de iteraciones: " + str(np.size(t_span)) + "\n \n"
                pasos = np.diff(t_span)
                info += "Paso promedio: " + str(np.mean(pasos)) + "\n"
                info += "Desviación estandar: " + str(np.std(pasos)) + "\n \n"

                info += "Paso mínimo: " + str(np.min(pasos)) + "\n"
                info += "Paso máximo: " + str(np.max(pasos)) + "\n \n"

                error = np.abs(pasos - periodo_de_muestreo_deseado)
                info += "Superiores al 2'%' de error: " + str( np.sum(error > 0.000020) ) + "\n"
                info += "Superiores al 1'%' de error: " + str( np.sum(error > 0.000010) ) + "\n"
                info += "Inferiores al 0.2'%' de error: " + str(np.sum(error < 0.000002) ) + "\n"
                info += "Inferiores al 0.1'%' de error: " + str(np.sum(error < 0.000001) ) + "\n"
            else:
                info = "No hay información disponible."

            q_and_f.qinfo.put(info)
            
            q_and_f.qpm.put(pm_t)
            q_and_f.qvm.put(vm_t)
            q_and_f.qpp.put(pp_t)
            q_and_f.qvp.put(vp_t)
            q_and_f.qu.put(u_t)
            q_and_f.qcm.put(cm_t)
            q_and_f.qr1.put(r1_t)
            q_and_f.qr2.put(r2_t)
            q_and_f.qr3.put(r3_t)
            q_and_f.qr4.put(r4_t)
            q_and_f.qa1.put(a1_t)
            q_and_f.qa2.put(a2_t)

            q_and_f.qt.put(t_span) # al final para que ya se hayan puesto todaslas demas

            q_and_f.fTermino.value = True
        time.sleep(0.2)

    #Algo que hacer antes de salir?...
    if card is not None:
       print("Cerrando el Qube.")
       card.terminate()
    return
     
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class miVentana(QMainWindow):
    def __init__(self, q_and_f) -> None:
        super().__init__()

        self.q_and_f = q_and_f

        self.t_aux = []
        self.pm_aux = []
        self.vm_aux = []
        self.pp_aux = []
        self.vp_aux = []
        self.u_aux = []
        self.cm_aux = []
        self.r1_aux = []
        self.r2_aux = []
        self.r3_aux = []
        self.r4_aux = []
        self.a1_aux = []
        self.a2_aux = []

        self.setGeometry( 200,200, 800,450 )
        self.setWindowTitle("DCA - Leyes de control en QUBE-Servo 2")

        widget = QWidget()
        layout = QGridLayout()

        self.timerCheck = QTimer()
        self.timerCheck.timeout.connect(self.actualizar_grafica)

        self.timerErrorQube = QTimer()
        self.timerErrorQube.timeout.connect(self.verificar_error)

        self.pdm_msgBox = QMessageBox()
        self.pdm_msgBox.setWindowTitle("Periodo de muestreo")
        self.pdm_msgBox.setText("No hay información disponible.")

        self.fx_msgBox = QMessageBox()
        self.fx_msgBox.setWindowTitle("Función adicional")
        self.fx_msgBox.setText("No hay información disponible.")

        # Crear elementos
        lb_info1 = QLabel("Fabricante:")
        lb_info2 = QLabel("Producto:")
        lb_info3 = QLabel("Modelo:")
        lb_info4 = QLabel("Firmware:")
        lb_info5 = QLabel("No. de Serie:")

        self.info1 = QLabel("_________")
        self.info2 = QLabel("_________")
        self.info3 = QLabel("_________")
        self.info4 = QLabel("_________")
        self.info5 = QLabel("_________")

        lb_tiempo = QLabel("Tiempo de ejecución:")
        lb_pdm = QLabel("Periodo de muestreo:")

        in_tiempo = QLabel("10 s")
        in_pdm =    QLabel("1e-3 s")

        btn_conectar = QPushButton("Conectar")
        btn_conectar.clicked.connect(self.conectar_qube)

        btn_comenzar = QPushButton("►")
        btn_comenzar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        btn_comenzar.clicked.connect(self.comenzar_ejecucion)

        btn_detener = QPushButton("■")
        btn_detener.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        btn_detener.clicked.connect(self.detener_ejecucion)

        btn_pdm_info = QPushButton("!")
        btn_pdm_info.clicked.connect(self.show_pdm_info)

        btn_fx = QPushButton("f(x)")
        btn_fx.clicked.connect(self.show_fx_info)

        self.check_1 = QCheckBox("Posición del motor")
        self.check_2 = QCheckBox("Velocidad del motor")
        self.check_3 = QCheckBox("Posición del péndulo")
        self.check_4 = QCheckBox("Velocidad del péndulo")
        self.check_5 = QCheckBox("Señal de control")
        self.check_6 = QCheckBox("Corriente del motor")
        self.check_7 = QCheckBox("Referencia de posición del motor")
        self.check_8 = QCheckBox("Referencia de posición del motor")
        self.check_9 = QCheckBox("Referencia de posición del motor")
        self.check_10 = QCheckBox("Referencia de posición del motor")
        self.check_11 = QCheckBox("Señal auxiliar 1")
        self.check_12 = QCheckBox("Señal auxiliar 1")

        self.check_1.setChecked(True)

        self.check_1.stateChanged.connect(self.mostrar_grafica)
        self.check_2.stateChanged.connect(self.mostrar_grafica)
        self.check_3.stateChanged.connect(self.mostrar_grafica)
        self.check_4.stateChanged.connect(self.mostrar_grafica)
        self.check_5.stateChanged.connect(self.mostrar_grafica)
        self.check_6.stateChanged.connect(self.mostrar_grafica)
        self.check_7.stateChanged.connect(self.mostrar_grafica)
        self.check_8.stateChanged.connect(self.mostrar_grafica)
        self.check_9.stateChanged.connect(self.mostrar_grafica)
        self.check_10.stateChanged.connect(self.mostrar_grafica)
        self.check_11.stateChanged.connect(self.mostrar_grafica)
        self.check_12.stateChanged.connect(self.mostrar_grafica)

        self.sc = MplCanvas(self, width=5, height=4, dpi=100)
        toolbar = NavigationToolbar2QT(self.sc, self)

        self.line_pm, = self.sc.axes.plot([])
        self.line_vm, = self.sc.axes.plot([])
        self.line_pp, = self.sc.axes.plot([])
        self.line_vp, = self.sc.axes.plot([])
        self.line_u, = self.sc.axes.plot([])
        self.line_cm, = self.sc.axes.plot([])
        self.line_r1, = self.sc.axes.plot([])
        self.line_r2, = self.sc.axes.plot([])
        self.line_r3, = self.sc.axes.plot([])
        self.line_r4, = self.sc.axes.plot([])
        self.line_a1, = self.sc.axes.plot([])
        self.line_a2, = self.sc.axes.plot([])

        self.sc.axes.grid(True)
        
        self.msj = QLabel("Iniciado.")
        self.progressBar = QProgressBar()
        
        #Colocar elementos
        layout.addWidget(btn_conectar, 0, 0, 1, 4)
        
        layout.addWidget(lb_info1,  1, 0, 1, 1)
        layout.addWidget(lb_info2,  2, 0, 1, 1)
        layout.addWidget(lb_info3,  3, 0, 1, 1)
        layout.addWidget(lb_info4,  4, 0, 1, 1)
        layout.addWidget(lb_info5,  5, 0, 1, 1)

        layout.addWidget(self.info1,  1, 1, 1, 2)
        layout.addWidget(self.info2,  2, 1, 1, 2)
        layout.addWidget(self.info3,  3, 1, 1, 2)
        layout.addWidget(self.info4,  4, 1, 1, 2)
        layout.addWidget(self.info5,  5, 1, 1, 2)

        layout.addWidget(lb_tiempo,  0, 4, 1, 1)
        layout.addWidget(in_tiempo,  0, 5, 1, 1)

        layout.addWidget(lb_pdm,  1, 4, 1, 1)
        layout.addWidget(in_pdm,  1, 5, 1, 1)

        layout.addWidget(btn_comenzar, 0, 6, 2, 2)
        layout.addWidget(btn_detener, 0, 8, 2, 2)
        layout.addWidget(btn_pdm_info, 0, 10, 1, 2)
        layout.addWidget(btn_fx, 1, 10, 1, 2)

        layout.addWidget(self.check_1, 6, 0, 1, 3)
        layout.addWidget(self.check_2, 7, 0, 1, 3)
        layout.addWidget(self.check_3, 8, 0, 1, 3)
        layout.addWidget(self.check_4, 9, 0, 1, 3)
        layout.addWidget(self.check_5, 10, 0, 1, 3)
        layout.addWidget(self.check_6, 11, 0, 1, 3)
        layout.addWidget(self.check_7, 12, 0, 1, 3)
        layout.addWidget(self.check_8, 13, 0, 1, 3)
        layout.addWidget(self.check_9, 14, 0, 1, 3)
        layout.addWidget(self.check_10, 15, 0, 1, 3)
        layout.addWidget(self.check_11, 16, 0, 1, 3)
        layout.addWidget(self.check_12, 17, 0, 1, 3)

        layout.addWidget(self.sc, 3, 3, 15, 9)
        layout.addWidget(toolbar, 2, 3, 1, 9)

        layout.addWidget(self.msj, 18, 0, 1, 9)
        layout.addWidget(self.progressBar, 18, 9, 1, 3)

        widget.setLayout(layout)
        self.setCentralWidget(widget)   

    def comenzar_ejecucion(self):
        self.msj.setText("Ejecutando...")
        self.q_and_f.fIniciar.value = True

        self.line_pm.set_data([], [])  # Borra los datos de cada línea
        self.line_vm.set_data([], [])
        self.line_pp.set_data([], [])
        self.line_vp.set_data([], [])
        self.line_u.set_data([], [])
        self.line_cm.set_data([], [])
        self.line_r1.set_data([], [])
        self.line_r2.set_data([], [])
        self.line_r3.set_data([], [])
        self.line_r4.set_data([], [])
        self.line_a1.set_data([], [])
        self.line_a2.set_data([], [])
        
        self.sc.axes.relim() 
        self.sc.axes.autoscale_view(True,True,True)  
        self.sc.draw()
        
        self.progressValue = 0
        self.timerCheck.start(95)

    def actualizar_grafica(self):
        if self.progressValue < 100:
            self.progressValue += 1
        self.progressBar.setValue(self.progressValue)

        if not self.q_and_f.qt.empty() and not self.q_and_f.qpm.empty():
            if not self.q_and_f.fDetener.value:
                self.progressBar.setValue(100)
                self.msj.setText("Terminado.")
            else:
                self.q_and_f.fDetener.value = False
            
            self.t_aux = self.q_and_f.qt.get()
            self.pm_aux = self.q_and_f.qpm.get()
            self.vm_aux = self.q_and_f.qvm.get()
            self.pp_aux = self.q_and_f.qpp.get()
            self.vp_aux = self.q_and_f.qvp.get()
            self.u_aux = self.q_and_f.qu.get()
            self.cm_aux = self.q_and_f.qcm.get()
            self.r1_aux = self.q_and_f.qr1.get()
            self.r2_aux = self.q_and_f.qr2.get()
            self.r3_aux = self.q_and_f.qr3.get()
            self.r4_aux = self.q_and_f.qr4.get()
            self.a1_aux = self.q_and_f.qa1.get()
            self.a2_aux = self.q_and_f.qa2.get()

            self.mostrar_grafica()

            self.sc.axes.relim() 
            self.sc.axes.autoscale_view(True,True,True)  
            self.sc.draw()

            self.timerCheck.stop()

    def mostrar_grafica(self):
        if self.check_1.isChecked():
            self.line_pm.set_data(self.t_aux, self.pm_aux)
        else:
            self.line_pm.set_data([],[])
        if self.check_2.isChecked():
            self.line_vm.set_data(self.t_aux, self.vm_aux)
        else:
            self.line_vm.set_data([],[])
        if self.check_3.isChecked():
            self.line_pp.set_data(self.t_aux, self.pp_aux)
        else:
            self.line_pp.set_data([],[])
        if self.check_4.isChecked():
            self.line_vp.set_data(self.t_aux, self.vp_aux)
        else:
            self.line_vp.set_data([],[])
        if self.check_5.isChecked():
            self.line_u.set_data(self.t_aux, self.u_aux)
        else:
            self.line_u.set_data([],[])
        if self.check_6.isChecked():
            self.line_cm.set_data(self.t_aux, self.cm_aux)
        else:
            self.line_cm.set_data([],[])
        if self.check_7.isChecked():
            self.line_r1.set_data(self.t_aux, self.r1_aux)
        else:
            self.line_r1.set_data([],[])
        if self.check_8.isChecked():
            self.line_r2.set_data(self.t_aux, self.r2_aux)
        else:
            self.line_r2.set_data([],[])
        if self.check_9.isChecked():
            self.line_r3.set_data(self.t_aux, self.r3_aux)
        else:
            self.line_r3.set_data([],[])
        if self.check_10.isChecked():
            self.line_r4.set_data(self.t_aux, self.r4_aux)
        else:
            self.line_r4.set_data([],[])
        if self.check_11.isChecked():
            self.line_a1.set_data(self.t_aux, self.a1_aux)
        else:
            self.line_a1.set_data([],[])
        if self.check_12.isChecked():
            self.line_a2.set_data(self.t_aux, self.a2_aux)
        else:
            self.line_a2.set_data([],[])
        
        self.sc.axes.relim() 
        self.sc.axes.autoscale_view(True,True,True)  
        self.sc.draw()

    def detener_ejecucion(self):
        self.q_and_f.fDetener.value = True
        self.msj.setText("Detenido")
    
    def show_pdm_info(self):
        if not self.q_and_f.qinfo.empty():
            info = self.q_and_f.qinfo.get()
            self.pdm_msgBox.setText(info)

        self.pdm_msgBox.exec()

    def show_fx_info(self):
        if not self.q_and_f.qfx_info.empty():
            info = self.q_and_f.qfx_info.get()
            self.fx_msgBox.setText(info)

        self.fx_msgBox.exec()
    
    def conectar_qube(self):
        self.id_info = 0
        self.timerErrorQube.start(500)
        self.q_and_f.fConectar.value = True

    def verificar_error(self):
        if not self.q_and_f.qQubeErrors.empty():
            self.msj.setText(self.q_and_f.qQubeErrors.get())

        if not self.q_and_f.qinfoQube.empty():
            if self.id_info == 0:
                self.info1.setText(self.q_and_f.qinfoQube.get())
            if self.id_info == 1:
                self.info2.setText(self.q_and_f.qinfoQube.get())
            if self.id_info == 2:
                self.info3.setText(self.q_and_f.qinfoQube.get())
            if self.id_info == 3:
                self.info4.setText(self.q_and_f.qinfoQube.get())
            if self.id_info == 4:
                self.info5.setText(self.q_and_f.qinfoQube.get())
            self.id_info += 1

    def closeEvent(self, a0: QCloseEvent) -> None:
        self.detener_ejecucion()
        time.sleep(0.2)
        self.q_and_f.fSalir.value = True
        time.sleep(0.2)

        super().closeEvent(a0)       

class MemoriaCompartida():
    def __init__(self):
        self.qt = Queue()
        self.qpm = Queue()
        self.qvm = Queue()
        self.qpp = Queue()
        self.qvp = Queue()
        self.qu = Queue()
        self.qcm = Queue()
        self.qr1 = Queue()
        self.qr2 = Queue()
        self.qr3 = Queue()
        self.qr4 = Queue()
        self.qa1 = Queue()
        self.qa2 = Queue()

        self.qinfo = Queue()
        self.qfx_info = Queue()
        
        self.fSalir = Value('b', False)
        self.fIniciar = Value('b', False)
        self.fDetener = Value('b', False)
        self.fTermino = Value('b', False)

        self.fConectar = Value('b', False)
        self.qinfoQube = Queue()
        self.qQubeErrors = Queue()

if __name__ == "__main__":
    print("Abriendo programa...")
    freeze_support()
    set_start_method('spawn')

    q_and_f = MemoriaCompartida()

    processQube = Process(target=adquisicion_de_datos, args=(q_and_f,))
    processQube.start()

    # Aqui se ejecuta el hilo principal
        # Dibujar Interfaz

    app = QApplication([])
    win = miVentana(q_and_f)
    win.show()
    app.exec()

    print("Saliendo de la interfaz...")
    processQube.join()
    print("Programa Terminado\n")
