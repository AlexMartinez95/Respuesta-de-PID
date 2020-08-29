import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk
import control as control
import numpy as np


naranja_fuerte_codigo_de_colores='#EEA006'
naranja_claro_codigo_de_colores='#FCEAC7'

def graficado_de_las_curvas(y_respuesta_ante_perturbacion,y_respuesta_ante_cambio_de_referencia,t):
    
    if titulo_pestanya_actual=='modelo en paralelo':
        fig.clf()
        fig.add_subplot(111).plot(t, y_respuesta_ante_perturbacion)
        fig.add_subplot(111).plot(t, y_respuesta_ante_cambio_de_referencia)
        fig.add_subplot(111).plot(t, np.ones(len(y_respuesta_ante_cambio_de_referencia)))
        fig.add_subplot(111).plot(t, np.zeros(len(y_respuesta_ante_cambio_de_referencia)))
        canvas.draw()
        
    if titulo_pestanya_actual=='ISA':
        fig.clf()
        fig.add_subplot(111).plot(t, y_respuesta_ante_perturbacion)
        fig.add_subplot(111).plot(t, y_respuesta_ante_cambio_de_referencia)
        fig.add_subplot(111).plot(t, np.ones(len(y_respuesta_ante_cambio_de_referencia)))
        canvas2.draw()  
        
def calculo_PID(Kp,Ki,Kd,b,c,td,valor_ajustado_de_la_deslizadera_de_tiempo,variable_eleccion_de_controlador):          
       
        if variable_eleccion_de_controlador=='P':         
            if titulo_pestanya_actual=='modelo en paralelo':
                Ki_objeto.deslizadera.configure(state='disabled',bg='gray')
                Kd_objeto.deslizadera.configure(state='disabled',bg='gray')
                b_objeto.deslizadera.configure(state='disabled',bg='gray')
                c_objeto.deslizadera.configure(state='disabled',bg='gray')
                td_objeto.deslizadera.configure(state='disabled',bg='gray')
            if titulo_pestanya_actual=='ISA':                
                Ti_objeto.deslizadera.configure(state='disabled',bg='gray')
                Td_objeto.deslizadera.configure(state='disabled',bg='gray')
                N_objeto.deslizadera.configure(state='disabled',bg='gray')
                b_objeto2.deslizadera.configure(state='disabled',bg='gray')
                c_objeto2.deslizadera.configure(state='disabled',bg='gray')
            
            controlador=control.tf([Kp],[1])
            bucle_cerrado_respuesta_ante_cambio_de_referencia=controlador*G0/(1+controlador*G0)
            bucle_cerrado_respuesta_ante_perturbacion=G0/(1+controlador*G0)
            t,y_respuesta_ante_cambio_de_referencia=control.step_response(bucle_cerrado_respuesta_ante_cambio_de_referencia,valor_ajustado_de_la_deslizadera_de_tiempo)
            t,y_respuesta_ante_perturbacion=control.step_response(bucle_cerrado_respuesta_ante_perturbacion,valor_ajustado_de_la_deslizadera_de_tiempo)
            
        if variable_eleccion_de_controlador=='PI':
            
            if titulo_pestanya_actual=='modelo en paralelo':
                Ki_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)  
                Kd_objeto.deslizadera.configure(state='disabled',bg='gray') 
                b_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores) 
                c_objeto.deslizadera.configure(state='disabled',bg='gray')                
                td_objeto.deslizadera.configure(state='disabled',bg='gray')
                 
            if titulo_pestanya_actual=='ISA':        
                Td_objeto.deslizadera.configure(state='disabled',bg='gray') 
                Ti_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                b_objeto2.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                c_objeto2.deslizadera.configure(state='disabled',bg='gray')
                N_objeto.deslizadera.configure(state='disabled',bg='gray')
                    
            controlador=control.tf([Kp,Ki],[1,0])
            controlador_fast_forward=control.tf([Kp*b,Ki],[1,0])
            bucle_cerrado_respuesta_ante_cambio_de_referencia=controlador_fast_forward*G0/(1+controlador*G0)
            bucle_cerrado_respuesta_ante_perturbacion=G0/(1+controlador*G0)
            t,y_respuesta_ante_cambio_de_referencia=control.step_response(bucle_cerrado_respuesta_ante_cambio_de_referencia,valor_ajustado_de_la_deslizadera_de_tiempo)
            t,y_respuesta_ante_perturbacion=control.step_response(bucle_cerrado_respuesta_ante_perturbacion,valor_ajustado_de_la_deslizadera_de_tiempo)
            
        if variable_eleccion_de_controlador=='PD':
            
            if titulo_pestanya_actual=='modelo en paralelo':
                
                Ki_objeto.deslizadera.configure(state='disabled',bg='gray')   
                Kd_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores) 
                b_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                c_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                td_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)                      
                
            if titulo_pestanya_actual=='ISA':
                
                Ti_objeto.deslizadera.configure(state='disabled',bg='gray')    
                Td_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores) 
                b_objeto2.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                c_objeto2.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)   
                N_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                                     
            controlador_fast_forward=control.tf([Kp*b*(td)+c*Kd,Kp*b],[td,1])
            controlador=control.tf([Kd+Kp*(td),Kp],[td,1])
            bucle_cerrado_respuesta_ante_cambio_de_referencia=controlador_fast_forward*G0/(1+controlador*G0)
            bucle_cerrado_respuesta_ante_perturbacion=G0/(1+controlador*G0)
            t,y_respuesta_ante_cambio_de_referencia=control.step_response(bucle_cerrado_respuesta_ante_cambio_de_referencia,valor_ajustado_de_la_deslizadera_de_tiempo)
            t,y_respuesta_ante_perturbacion=control.step_response(bucle_cerrado_respuesta_ante_perturbacion,valor_ajustado_de_la_deslizadera_de_tiempo)        
            
        if variable_eleccion_de_controlador=='PID':
            
            if titulo_pestanya_actual=='modelo en paralelo':
                Ki_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)     
                Kd_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores) 
                b_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                c_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                td_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                        
            if titulo_pestanya_actual=='ISA':      
                Ti_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)     
                Td_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores) 
                b_objeto2.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                c_objeto2.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
                N_objeto.deslizadera.configure(state='normal',bg=naranja_fuerte_codigo_de_colores)
            
            controlador_fast_forward=control.tf([(Kp*(td)*b+Kd*c),(b*Kp+Ki*(td)),Ki],[td,1,0 ])
            controlador=control.tf([(Kp*(td)+Kd),(Kp+Ki*(td)),Ki],[td,1,0 ])
            bucle_cerrado_respuesta_ante_cambio_de_referencia=controlador_fast_forward*G0/(1+controlador*G0)
            bucle_cerrado_respuesta_ante_perturbacion=G0/(1+controlador*G0)
            t,y_respuesta_ante_cambio_de_referencia=control.step_response(bucle_cerrado_respuesta_ante_cambio_de_referencia,valor_ajustado_de_la_deslizadera_de_tiempo)
            t,y_respuesta_ante_perturbacion=control.step_response(bucle_cerrado_respuesta_ante_perturbacion,valor_ajustado_de_la_deslizadera_de_tiempo)
            
        ts98_encontrado=0 
        for i in range(len(y_respuesta_ante_cambio_de_referencia)):
                
            if (y_respuesta_ante_cambio_de_referencia[i]>=0.98 and y_respuesta_ante_cambio_de_referencia[i]<=1.02 and ts98_encontrado==0):
                y_en_ts98=i
                ts98_encontrado=1
            elif (y_respuesta_ante_cambio_de_referencia[i]<0.98 or y_respuesta_ante_cambio_de_referencia[i]>1.02):
                ts98_encontrado=0
                y_en_ts98=0        
        ts98=t[y_en_ts98]
        sobreoscilacion=(np.amax(y_respuesta_ante_cambio_de_referencia)-1)*100
        if sobreoscilacion<0:
            sobreoscilacion=0
        sobreoscilacion_string=str(round(sobreoscilacion,3))+"%"
        
 
        area_trapecio=0
        for i in range(len(y_respuesta_ante_perturbacion)):           

              if  y_respuesta_ante_perturbacion[i]>=0:
                  base_trapecio=y_respuesta_ante_perturbacion[i-1]
                  base_segunda_trapecio=y_respuesta_ante_perturbacion[i]
                  altura_trapecio=t[i]-t[i-1]
                  area_trapecio=((base_trapecio+base_segunda_trapecio)*altura_trapecio/2)+area_trapecio
                  
              if  y_respuesta_ante_perturbacion[i]<0:
                  base_trapecio=-y_respuesta_ante_perturbacion[i-1]
                  base_segunda_trapecio=-y_respuesta_ante_perturbacion[i]
                  altura_trapecio=t[i]-t[i-1]
                  print(t[i-1])
                  area_trapecio=((base_trapecio+base_segunda_trapecio)*altura_trapecio/2)+area_trapecio
                  
              IAE=area_trapecio        
        
         
        tspert_encontrado=0
        for i in range(len(y_respuesta_ante_perturbacion)):
            if (y_respuesta_ante_perturbacion[i]>=-0.994 and y_respuesta_ante_perturbacion[i]<=0.006 and tspert_encontrado==0): #preguntar
                y_en_tspert=i
                tspert_encontrado=1
            elif (y_respuesta_ante_perturbacion[i]<-0.994 or y_respuesta_ante_perturbacion[i]>0.006):
                tspert_encontrado=0
                y_en_tspert=0
        tspert=t[y_en_tspert]
        ypertmax=np.amax(y_respuesta_ante_perturbacion)  
       
        if titulo_pestanya_actual=='modelo en paralelo':
            valor_sobreoscilacion.configure(text=sobreoscilacion_string)
            valor_ts98.configure(text=round(ts98,3))
            valor_ypert.configure(text=round(ypertmax,3))
            valor_tspert.configure(text=round(tspert,3))
            valor_IAE.configure(text=round(IAE,3))
        if titulo_pestanya_actual=='ISA':
            valor_sobreoscilacion2.configure(text=sobreoscilacion_string)
            valor_ts98_2.configure(text=round(ts98,3))
            valor_ypert2.configure(text=round(ypertmax,3))
            valor_tspert2.configure(text=round(tspert,3))
            valor_IAE2.configure(text=round(IAE,3))

                
        graficado_de_las_curvas(y_respuesta_ante_perturbacion,y_respuesta_ante_cambio_de_referencia,t)

    
def leer_datos_de_la_ventana():
    
    global G0,s
    
    s=control.tf('s')
    G0=eval(input_del_Ys_en_la_ventana.get())   
      
    b=b_objeto.variable.get()
    c=c_objeto.variable.get()
    
    if titulo_pestanya_actual=='modelo en paralelo':
        Kp=Kp_objeto.variable.get()  
        b=b_objeto.variable.get()
        c=c_objeto.variable.get()
        Ki=Ki_objeto.variable.get()
        Kd=Kd_objeto.variable.get()
        td=td_objeto.variable.get()
        variable_eleccion_de_controlador=variable_eleccion_de_controlador_boton.get()
        valor_de_la_deslizadera_del_tiempo=deslizadera_tiempo_objeto.variable.get()
        
        print(valor_de_la_deslizadera_del_tiempo)
        
    if titulo_pestanya_actual=='ISA':
        Kp=Kp_objeto2.variable.get()
        b=b_objeto2.variable.get()
        c=c_objeto2.variable.get()
        Ti=Ti_objeto.variable.get()
        Td=Td_objeto.variable.get()
        N=N_objeto.variable.get()
        td=Td/N 
        Ki=Kp/Ti
        Kd=Td*Kp                
        variable_eleccion_de_controlador=variable_eleccion_de_controlador_boton_pestanya_ISA.get()
        valor_de_la_deslizadera_del_tiempo=deslizadera_tiempo_objeto2.variable.get()
    
   
    valor_ajustado_de_la_deslizadera_de_tiempo=np.linspace(0.0,valor_de_la_deslizadera_del_tiempo, num=1000)
    calculo_PID(Kp,Ki,Kd,b,c,td,valor_ajustado_de_la_deslizadera_de_tiempo,variable_eleccion_de_controlador)
    
def pestanya_seleccionada(event):
    
    global titulo_pestanya_actual
    
    pestanya_actual=event.widget.select()
    titulo_pestanya_actual=event.widget.tab(pestanya_actual,"text")
        
class Deslizadera:

    def __init__(self,master,texto,valor_inicial,valor_maximo,valor_minimo):
        
        self.dini=valor_inicial
        self.dmin=valor_minimo
        self.dmax=valor_maximo
        self.master=master
        self.variable=tk.DoubleVar()
        self.variable.set(valor_inicial)
        self.deslizadera=tk.Scale(master,variable=self.variable,from_=self.dmin,to=self.dmax,orient=tk.HORIZONTAL,label=texto,resolution=0.001,length=400,bg=naranja_fuerte_codigo_de_colores,troughcolor='black')
        self.deslizadera.pack(side=tk.BOTTOM,fill=tk.BOTH,expand=True)
        self.deslizadera.bind("<ButtonRelease-1>",self.update)
        
    def update(self,event):
        
       if self.dmax==self.dini: 
            leer_datos_de_la_ventana()
       else:     
            if self.variable.get()==0:
                
                self.dmax=self.dmax/2
                self.deslizadera.configure(to=self.dmax)
                
            if self.variable.get() >= (self.dmax-self.dmax/1000):
                
                self.dmax=self.dmax*2
                self.deslizadera.configure(to=self.dmax)
            
            if self.dmax < 10:
                
                self.deslizadera.configure(resolution=(self.dmax/1000))
            
            leer_datos_de_la_ventana()
  
titulo_pestanya_actual='modelo en paralelo'
ventana = tk.Tk()
ventana.wm_title("Ajuste de PID")
ventana.config(width=500, height=200)

pestanya_maestra=ttk.Notebook(ventana)
pestanya_paralelo=tk.Frame(pestanya_maestra,bg=naranja_claro_codigo_de_colores)
pestanya_ISA=tk.Frame(pestanya_maestra,bg=naranja_claro_codigo_de_colores)
pestanya_maestra.add(pestanya_paralelo, text="modelo en paralelo")
pestanya_maestra.add(pestanya_ISA, text="ISA")
pestanya_maestra.pack()
pestanya_maestra.bind("<<NotebookTabChanged>>",pestanya_seleccionada)

frame_canvas_paralelo=tk.Frame(pestanya_paralelo,bg=naranja_claro_codigo_de_colores)
frame_canvas_paralelo.grid(row=1, column=1)
fig = Figure(figsize=(7, 4), dpi=100)
canvas= FigureCanvasTkAgg(fig, master=frame_canvas_paralelo)  
toolbar = NavigationToolbar2Tk(canvas,frame_canvas_paralelo)
canvas.get_tk_widget().pack(side=tk.TOP,fill=tk.BOTH,expand=True)

toolbar.update()
 
frame_del_PID=tk.Frame(pestanya_paralelo,bg=naranja_claro_codigo_de_colores)
frame_del_PID.grid(row=1,column=0)

frame_del_PID_bc=tk.Frame(pestanya_paralelo,bg=naranja_claro_codigo_de_colores)
frame_del_PID_bc.grid(row=2,column=1)
c_objeto=Deslizadera(frame_del_PID_bc,"c",1,1,0)
b_objeto=Deslizadera(frame_del_PID_bc,"b",1,1,0)
td_objeto=Deslizadera(frame_del_PID,"td",0.1,1,0)
Kd_objeto=Deslizadera(frame_del_PID,"Kd",1,10,0)
Ki_objeto=Deslizadera(frame_del_PID,"Ki",1,10,0)
Kp_objeto=Deslizadera(frame_del_PID,"Kp",1,10,0)

deslizadera_tiempo_objeto=Deslizadera(frame_canvas_paralelo,"Tiempo de visualizacion",20,100,0.05)    

frame_Ys=tk.Frame(pestanya_paralelo,bg=naranja_claro_codigo_de_colores)
frame_Ys.grid(row=0,column=0)

input_del_Ys_en_la_ventana = ttk.Entry(frame_Ys, width=30)
input_del_Ys_en_la_ventana.insert(0,"(1)/((1+1*s)*(1+1*s))")
input_del_Ys_en_la_ventana.grid(row=1,column=0)

label_G0=tk.Label(frame_Ys,text="G(s)",bg=naranja_claro_codigo_de_colores)
label_G0.grid(row=0,column=0)

frame_resultados=tk.Frame(pestanya_paralelo,bg=naranja_claro_codigo_de_colores)
frame_resultados.grid(row=2,column=0)


valor_sobreoscilacion=ttk.Label(frame_resultados, width=20,relief='solid')
valor_sobreoscilacion_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores, width=1,text="δ")
valor_sobreoscilacion.grid(row=2,column=0)
valor_sobreoscilacion_titulo.grid(row=1,column=0)

valor_ts98=ttk.Label(frame_resultados, width=20,relief='solid')
valor_ts98_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores,width=10,text="ts98")
valor_ts98.grid(row=2,column=1)
valor_ts98_titulo.grid(row=1,column=1)


valor_ypert=ttk.Label(frame_resultados, width=20,relief='solid')
valor_ypert_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores, width=10,text="ypert_max")
valor_ypert.grid(row=4,column=0)
valor_ypert_titulo.grid(row=3,column=0)


valor_tspert=ttk.Label(frame_resultados, width=20,relief='solid')
valor_tspert_titulo=tk.Label(frame_resultados, bg=naranja_claro_codigo_de_colores, width=10,text="tspert")
valor_tspert.grid(row=4,column=1)
valor_tspert_titulo.grid(row=3,column=1)
valor_IAE_titulo=tk.Label(frame_resultados,width=10,text="IAE",bg=naranja_claro_codigo_de_colores)
valor_IAE_titulo.grid(row=3,column=2)
valor_IAE=ttk.Label(frame_resultados, width=20,relief='solid')
valor_IAE.grid(row=4,column=2)

boton_de_ok = tk.Button(frame_Ys, text="Ok",command=leer_datos_de_la_ventana,bg=naranja_fuerte_codigo_de_colores)
boton_de_ok.grid(row =2, column =0)

variable_eleccion_de_controlador_boton=tk.StringVar()
variable_eleccion_de_controlador_boton.set(0)

boton_P=tk.Radiobutton(frame_Ys, text="P", variable=variable_eleccion_de_controlador_boton,  value='P', command=leer_datos_de_la_ventana,bg=naranja_claro_codigo_de_colores)
boton_P.grid(row=1,column=1)
boton_PI=tk.Radiobutton(frame_Ys, text="PI", variable=variable_eleccion_de_controlador_boton,  value='PI', command=leer_datos_de_la_ventana,bg=naranja_claro_codigo_de_colores)
boton_PI.grid(row=1,column=2)
boton_PD=tk.Radiobutton(frame_Ys, text="PD", variable=variable_eleccion_de_controlador_boton,  value='PD', command=leer_datos_de_la_ventana,bg=naranja_claro_codigo_de_colores)
boton_PD.grid(row=1,column=3)
boton_PID=tk.Radiobutton(frame_Ys, text="PID", variable=variable_eleccion_de_controlador_boton,  value='PID', command=leer_datos_de_la_ventana,bg=naranja_claro_codigo_de_colores)
boton_PID.grid(row=1,column=4)

if 1:
    
    frame_canvas_ISA=tk.Frame(pestanya_ISA,bg=naranja_claro_codigo_de_colores)
    frame_canvas_ISA.grid(row=1, column=1)
    fig2 = Figure(figsize=(7, 4), dpi=100)
    canvas2= FigureCanvasTkAgg(fig, master=frame_canvas_ISA)  
    toolbar2 = NavigationToolbar2Tk(canvas2,frame_canvas_ISA)
    canvas2.get_tk_widget().pack(side=tk.TOP,fill=tk.BOTH,expand=True)
    toolbar2.update()
    
    frame_del_PID=tk.Frame(pestanya_ISA,bg=naranja_claro_codigo_de_colores)
    frame_del_PID.grid(row=1,column=0)
    frame_del_PID_bc=tk.Frame(pestanya_ISA,bg=naranja_claro_codigo_de_colores)
    frame_del_PID_bc.grid(row=2,column=1)
    
    c_objeto2=Deslizadera(frame_del_PID_bc,"c",1,1,0)
    b_objeto2=Deslizadera(frame_del_PID_bc,"b",1,1,0)
    N_objeto=Deslizadera(frame_del_PID,"N",10,20,0)
    Td_objeto=Deslizadera(frame_del_PID,"Td",1,10,0)
    Ti_objeto=Deslizadera(frame_del_PID,"Ti",1,10,0)
    Kp_objeto2=Deslizadera(frame_del_PID,"Kp",1,10,0)
    deslizadera_tiempo_objeto2=Deslizadera(frame_canvas_ISA,"Tiempo de visualizacion",20,100,0.05)    
    
    frame_Ys=tk.Frame(pestanya_ISA,bg=naranja_claro_codigo_de_colores)
    frame_Ys.grid(row=0,column=0)
    
    input_del_Ys_en_la_ventana = ttk.Entry(frame_Ys, width=30)
    input_del_Ys_en_la_ventana.insert(0,"(1)/((1+1*s)*(1+1*s))")
    input_del_Ys_en_la_ventana.grid(row=1,column=0)
    
    label_G0=tk.Label(frame_Ys,text="G(s)",bg=naranja_claro_codigo_de_colores)
    label_G0.grid(row=0,column=0)
    
    frame_resultados=tk.Frame(pestanya_ISA,bg=naranja_claro_codigo_de_colores)
    frame_resultados.grid(row=2,column=0)
    
    
    valor_sobreoscilacion2=ttk.Label(frame_resultados, width=20,relief='solid')
    valor_sobreoscilacion2.grid(row=2,column=0)
    valor_sobreoscilacion_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores, width=1,text="δ")
    valor_sobreoscilacion_titulo.grid(row=1,column=0)
    
    valor_ts98_2=ttk.Label(frame_resultados, width=20,relief='solid')
    valor_ts98_2.grid(row=2,column=1)
    valor_ts98_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores, width=10,text="ts98")
    valor_ts98_titulo.grid(row=1,column=1)
    
    
    valor_ypert2=ttk.Label(frame_resultados, width=20,relief='solid')
    valor_ypert_titulo=tk.Label(frame_resultados,bg=naranja_claro_codigo_de_colores, width=10,text="ypert_max")
    valor_ypert2.grid(row=4,column=0)
    valor_ypert_titulo.grid(row=3,column=0)
    
    
    valor_tspert2=ttk.Label(frame_resultados, width=20,relief='solid')
    valor_tspert_titulo=tk.Label(frame_resultados, bg=naranja_claro_codigo_de_colores,width=10,text="tspert")
    valor_tspert2.grid(row=4,column=1)
    valor_tspert_titulo.grid(row=3,column=1)
    
    valor_IAE_titulo=tk.Label(frame_resultados,width=10,text="IAE",bg=naranja_claro_codigo_de_colores)
    valor_IAE_titulo.grid(row=3,column=2)
    valor_IAE2=ttk.Label(frame_resultados,width=20,relief='solid')
    valor_IAE2.grid(row=4,column=2)
    
    boton_de_ok = tk.Button(frame_Ys, text="Ok",command=leer_datos_de_la_ventana,bg=naranja_fuerte_codigo_de_colores)
    boton_de_ok.grid(row =2, column = 0)
    
    variable_eleccion_de_controlador_boton_pestanya_ISA=tk.StringVar()
    variable_eleccion_de_controlador_boton_pestanya_ISA.set(0)
    
    boton_P=tk.Radiobutton(frame_Ys, text="P", variable=variable_eleccion_de_controlador_boton_pestanya_ISA,bg=naranja_claro_codigo_de_colores,  value='P', command=leer_datos_de_la_ventana)
    boton_P.grid(row=1,column=1)
    boton_PI=tk.Radiobutton(frame_Ys, text="PI", variable=variable_eleccion_de_controlador_boton_pestanya_ISA, bg=naranja_claro_codigo_de_colores, value='PI', command=leer_datos_de_la_ventana)
    boton_PI.grid(row=1,column=2)
    boton_PD=tk.Radiobutton(frame_Ys, text="PD", variable=variable_eleccion_de_controlador_boton_pestanya_ISA,bg=naranja_claro_codigo_de_colores,  value='PD', command=leer_datos_de_la_ventana)
    boton_PD.grid(row=1,column=3)
    boton_PID=tk.Radiobutton(frame_Ys, text="PID", variable=variable_eleccion_de_controlador_boton_pestanya_ISA, bg=naranja_claro_codigo_de_colores, value='PID', command=leer_datos_de_la_ventana)
    boton_PID.grid(row=1,column=4)


ventana.mainloop()

