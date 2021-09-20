# NOTA GENERAL: LOS BITS SE ENVIAN AL REVES


import os
import spidev
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal




WAKEUP= 0x02
STANDBY=0X04
RESET=0X06
START=0X08
STOP=0X0A
RDATAC=0X10
SDATAC=0X11
RDATA=0X12
RREG=0X20
WREG=0X40

ID=0X00
CONFIG1=0X01
CONFIG2=0X02
CH1SET=0X04
CH2SET=0X05
RLD_SENS=0X06



data_send = 0
ads1292_ready = 0
filename = input('Nombre de archivo: ')  # SE CREA ARCHIVO PARA GUARDAR DATOS
file = open('{}.txt'.format(filename), 'w')
spi = spidev.SpiDev()  # SE INICIA EL SPI
GPIO.setwarnings(False)


def rot(byte):
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4)
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2)
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1)
    return byte


def init_conf():  # CONFIGURACION INICIAL

    global CS, DRDY

    GPIO.setmode(GPIO.BCM)
    CS = 7
    DRDY = 24
    GPIO.setup(CS, GPIO.OUT)  # CS
    GPIO.setup(DRDY, GPIO.IN)  # DRDY
    spi.open(0, 0)
    spi.mode = 0b01
    spi.max_speed_hz = 4000000  # 4MHz, FRECUENCIA PARA COMUNICACION SPI
    sendcmd(RESET)
    time.sleep(0.0001)


def sendcmd(comando):
    GPIO.output(CS, 0)
    time.sleep(0.00001)
    spi.writebytes([comando])
    time.sleep(0.00003)
    GPIO.output(CS, 1)
    time.sleep(0.00002)


def writeReg(direccion, dato):
    direccion |= WREG
    time.sleep(0.00002)
    GPIO.output(CS, 0)
    time.sleep(0.00001)
    spi.writebytes([direccion])
    time.sleep(0.00002)
    spi.writebytes([0X00])
    time.sleep(0.00002)
    spi.writebytes([dato])
    time.sleep(0.00003)
    GPIO.output(CS, 1)
    time.sleep(0.00002)


def readreg(direccion):
    direccion |= RREG
    time.sleep(0.00002)
    GPIO.output(CS, 0)
    time.sleep(0.00001)
    spi.writebytes([direccion])
    time.sleep(0.00002)
    spi.writebytes([0x00])
    time.sleep(0.00002)
    dato = spi.xfer([0xFF])
    time.sleep(0.00003)
    GPIO.output(CS, 1)
    time.sleep(0.00002)
    return dato


def startconv():
    global data_send
    sendcmd(RDATAC)  # RDATAC 0X10
    sendcmd(START)  # START 0X08
    data_send = 1


def stopconv():
    global data_send
    sendcmd(SDATC)  # SDATAC 0X11
    sendcmd(STOP)  # STOP 0X0A
    data_send = 0


def save2(data):
    
    C1=[]
    C2=[]
    C3=[]
    C4=[]
    C5=[]
    C6=[]
    C7=[]
    C8=[]
    
    
    
    
    # ~ D2=0.01577059737
    # ~ D3=-2.031541195
    # ~ D4=0.01577059737
    # ~ KF=0.03007468895
    c1=0
    c2=0
    c3=0
    c4=0
    c5=0
    c6=0
    c7=0
    c8=0
    # ~ NL=3
    # ~ NUM=[0.9994348288,-0.9994348288,0]
    # ~ DL=3
    # ~ DEN=[1,-0.9988696575,0]
    # ~ BL=26
    # ~ B=[0.0001979996014,-0.0006330574397, 0.001568097738,-0.003304710612, 0.006254237611,   -0.01095288899,  0.01811391674, -0.02876197733,  0.04459231347, -0.06900795549,
     # ~ 0.1107223779,   -0.201850757,   0.6330996752,   0.6330996752,   -0.201850757,     0.1107223779, -0.06900795549,  0.04459231347, -0.02876197733,  0.01811391674,
   # ~ -0.01095288899, 0.006254237611,-0.003304710612, 0.001568097738,-0.0006330574397,  0.0001979996014]
    # ~ NL_N=3
    # ~ NUM_N=[0.9689791799,-0.1216854081,0.9689791799]
    # ~ DL_N=3
    # ~ DEN_N=[1,-0.1216854081,0.9379583001]
    # ~ v1=[0,0,0,0,0,0,0,0,0]
    # ~ hpcanal[0,0,0,0,0,0,0,0,0]   
    
    ## 0,3,6,9,12,15,18,21
    
    for entry in data:
        c1=(entry[6] << 16) + (entry[7] << 8)+ entry[8]
        
        if c1 >= 0x800000 and c1<= 0xFFFFFF:
            c1 |= 0XFF000000
            # ~ c1=c1-2*pow(2,23)
        c1=c1/0x7fffff
        
        
        
        c2=  (entry[3] << 16) + (entry[4] << 8)+ entry[5] 
        if c2 >= 0x800000 and c2<= 0xFFFFFF:
            c2 |= 0XFF000000
        c2=c2/0x7fffff
                        
        
        
        
        
    # ~ for entry in data:
        # ~ input_buffer[1,9]=input_buffer[1,8]
        # ~ input_buffer[1,8]=input_buffer[1,7]
        # ~ input_buffer[1,7]=input_buffer[1,6]
        # ~ input_buffer[1,6]=input_buffer[1,5]
        # ~ input_buffer[1,5]=input_buffer[1,4]
        # ~ input_buffer[1,4]=input_buffer[1,3]
        # ~ input_buffer[1,3]=input_buffer[1,2]
        # ~ input_buffer[1,2]=input_buffer[1,1]
        # ~ input_buffer[1,1]=entry[1]
        
        # ~ temp_inter_buffer[1,9]=temp_inter_buffer[1,8]
        # ~ temp_inter_buffer[1,8]=temp_inter_buffer[1,7]
        # ~ temp_inter_buffer[1,7]=temp_inter_buffer[1,6]
        # ~ temp_inter_buffer[1,6]=temp_inter_buffer[1,5]
        # ~ temp_inter_buffer[1,5]=temp_inter_buffer[1,4]
        # ~ temp_inter_buffer[1,4]=temp_inter_buffer[1,3]
        # ~ temp_inter_buffer[1,3]=temp_inter_buffer[1,2]
        # ~ temp_inter_buffer[1,2]=temp_inter_buffer[1,1]
        # ~ temp_inter_buffer[1,1]=temp_inter_buffer[1,9]
        
        # ~ if cr[1]<0:
            # ~ cr[1]=-cr[1]
        # ~ if cr[1]<= M[1]:
            # ~ y_i[1]=(input_buffer[1,4]+input_buffer[1,5]+input_buffer[1,6]+(input_buffer[1,3]+input_buffer[1,7])/2)/4
            # ~ output_buffer[1]=y_i[1]/(1-KF)-input_buffer[1,5]*KF/(1-KF)
            # ~ temp_inter_buffer[1,1]=input_buffer[1,5]-output_buffer[1]
        # ~ else:
            # ~ restore_inter_bufer[1]=8*KF*temp_inter_buffer[1,3]-temp_inter_buffer[1,1]-2*(temp_inter_buffer[1,2]+temp_inter_buffer[1,3]+temp_inter_buffer[1,4])
            # ~ output_buffer[1]=input_buffer[1,5]-restore_inter_buffer[1]
        # ~ if Mmax[1] < output_buffer[1]:
            # ~ Mmax[1]=output_buffer[1]
        # ~ else:
            # ~ Mmax[1]=Mmax[1]-0.0001*(Mmax[1]-output_buffer[1])
        # ~ if Mmin[1] > output_buffer[1]:
            # ~ Mmin[1]=output_buffer[1]
        # ~ else:
            # ~ Mmin[1]=Mmin[1]+0.0001*(output_buffer[1]-Mmin[1])
        # ~ Mpp[1]=Mmax[1]-Mmin[1]
        # ~ if Mu[1]>Mpp[1]:
            # ~ Mu[1]=Mpp[1]
        # ~ else:
            # ~ Mu[1]=Mu[1]+0.01*Mpp[1]
        # ~ M[1]=0.1*Mu[1]
        C1.append(c1)
        C2.append(c2)
        
        file.write('{} \n'.format(c2))
    return C1,C2
    
    
def sus(signal):
    D2=0.01577059737
    D3=-2.031541195
    D4=0.01577059737
    KF=0.03007468895
    cr=[0,0,0,0,0,0,0,0]
    input_buffer=[0,0,0,0,0,0,0,0,0,0]
    temp_inter_buffer=[0,0,0,0,0,0,0,0,0,0]
    M=[0,0,0,0,0,0,0,0]
    Mmax=[0,0,0,0,0,0,0,0]
    Mmin=[0,0,0,0,0,0,0,0]
    Mpp=[0,0,0,0,0,0,0,0]
    Mu=[0,0,0,0,0,0,0,0]
    y_i=[0,0,0,0,0,0,0,0]
    
    restore_inter_buffer=[0,0,0,0,0,0,0,0]
    output_buffer=0
    S=[]
    
    for entry in signal:
        input_buffer[9]=input_buffer[8]
        input_buffer[8]=input_buffer[7]
        input_buffer[7]=input_buffer[6]
        input_buffer[6]=input_buffer[5]
        input_buffer[5]=input_buffer[4]
        input_buffer[4]=input_buffer[3]
        input_buffer[3]=input_buffer[2]
        input_buffer[2]=input_buffer[1]
        input_buffer[1]=entry
        
        temp_inter_buffer[9]=temp_inter_buffer[8]
        temp_inter_buffer[8]=temp_inter_buffer[7]
        temp_inter_buffer[7]=temp_inter_buffer[6]
        temp_inter_buffer[6]=temp_inter_buffer[5]
        temp_inter_buffer[5]=temp_inter_buffer[4]
        temp_inter_buffer[4]=temp_inter_buffer[3]
        temp_inter_buffer[3]=temp_inter_buffer[2]
        temp_inter_buffer[2]=temp_inter_buffer[1]
        temp_inter_buffer[1]=temp_inter_buffer[9]
        
        cr[1]=((input_buffer[1])+(input_buffer[3]*D2)+(input_buffer[5]*D3)+(input_buffer[7]*D4)+(input_buffer[9]))
        
        if cr[1]<0:
            cr[1]=-cr[1]
        if cr[1]<= M[1]:
            y_i[1]=(input_buffer[4]+input_buffer[5]+input_buffer[6]+(input_buffer[3]+input_buffer[7])/2)/4
            output_buffer=y_i[1]/(1-KF)-input_buffer[5]*KF/(1-KF)
            temp_inter_buffer[1]=input_buffer[5]-output_buffer
        else:
            restore_inter_buffer[1]=8*KF*temp_inter_buffer[3]-temp_inter_buffer[1]-2*(temp_inter_buffer[2]+temp_inter_buffer[3]+temp_inter_buffer[4])
            output_buffer=input_buffer[5]-restore_inter_buffer[1]
        if Mmax[1] < output_buffer:
            Mmax[1]=output_buffer
        else:
            Mmax[1]=Mmax[1]-0.0001*(Mmax[1]-output_buffer)
        if Mmin[1] > output_buffer:
            Mmin[1]=output_buffer
        else:
            Mmin[1]=Mmin[1]+0.0001*(output_buffer-Mmin[1])
        Mpp[1]=Mmax[1]-Mmin[1]
        if Mu[1]>Mpp[1]:
            Mu[1]=Mpp[1]
        else:
            Mu[1]=Mu[1]+0.01*Mpp[1]
        M[1]=0.1*Mu[1]
        S.append(output_buffer)
    return S
        
def save(data):
    a1=[]
    a2=[]
    status = 0
    chan1 = 0
    chan2 = 0
    MSB = 2*pow(2, 23)
    for entry in data:

        status = format((entry[0] << 16) + (entry[1] << 8) + entry[2], '024b')

        chan1 = (entry[3] << 16) + (entry[4] << 8) + entry[5]
        if format(chan1, '024b')[0] == 1:
            chan1 = chan1 #- MSB
           
        chan2 = (entry[6] << 16) + (entry[7] << 8) + entry[8]
        if format(chan2, '024b')[0] == 1:
            chan2 = chan2 #- MSB
        # ~ if chan1 >= 0x800000 and chan1<= 0xFFFFFF:
                # ~ chan1 |= 0XFF000000
        a1.append(chan1)
        a2.append(chan2)
        file.write('{} , {} , {} \n'.format(status, chan1, chan2))
    return a2


def readinfo():
    sendcmd(SDATAC)
    sendcmd(STOP)
    info = readreg(ID)  # ID

    print('ID ES:', info)
    info = readreg(CONFIG1)  # CONFIG1
    print('CONFIG1 ES:', info)
    info=readreg(CH1SET)
    print('CH1SET ES :', info)
    
def readECG2():
    
    
    input_buffer=np.zeros((9,10))
    output_buffer=np.zeros(9)
    temp_inter_buffer=np.zeros((9,10))
    restore_inter_buffer=np.zeros(9)
    M=np.zeros(9)
    Mmax=np.zeros(9)
    Mmin=np.zeros(9)
    Mpp=np.zeros(9)
    Mu=np.zeros(9)
    y_i=np.zeros(9)
    
    Canal=[]
    fderiv=np.zeros(9)  
    cr=np.zeros(9)
    po=0
    D2=0.01577059737
    D3=-2.031541195
    D4=0.01577059737
    KF=0.03007468895
     
    GPIO.output(CS, 0)
    time.sleep(0.0002)
    
    
    
    for i in range(1,10):
      
      Canal.append(spi.xfer([0XFF]))
  
    Canal=np.array(Canal)
    GPIO.output(CS, 1)
    for i in range(1,9):
        if ((Canal[i]>= 0x800000 ) and(Canal[i]<= 0xFFFFFF ) ):
            Canal[i] |= 0xFF000000
    for i in range(1,9):
        Canal[i]=Canal[i]/0x7FFFFF
    fderiv[1]=2.4*Canal[2]
    fderiv[2]=2.4*Canal[3]
    fderiv[3]=2.4*Canal[8]
    fderiv[4]=2.4*Canal[4]
    fderiv[5]=2.4*Canal[5]
    fderiv[6]=2.4*Canal[6]
    fderiv[7]=2.4*Canal[7]
    fderiv[8]=2.4*Canal[1]
    
    for i in range(1,9):
        input_buffer[i,9]=input_buffer[i,8]
        input_buffer[i,8]=input_buffer[i,7]
        input_buffer[i,7]=input_buffer[i,6]
        input_buffer[i,6]=input_buffer[i,5]
        input_buffer[i,5]=input_buffer[i,4]
        input_buffer[i,4]=input_buffer[i,3]
        input_buffer[i,3]=input_buffer[i,2]
        input_buffer[i,2]=input_buffer[i,1]
        input_buffer[i,1]=fderiv[i]
        
        temp_inter_buffer[i,9]=temp_inter_buffer[i,8]
        temp_inter_buffer[i,8]=temp_inter_buffer[i,7]
        temp_inter_buffer[i,7]=temp_inter_buffer[i,6]
        temp_inter_buffer[i,6]=temp_inter_buffer[i,5]
        temp_inter_buffer[i,5]=temp_inter_buffer[i,4]
        temp_inter_buffer[i,4]=temp_inter_buffer[i,3]
        temp_inter_buffer[i,3]=temp_inter_buffer[i,2]
        temp_inter_buffer[i,2]=temp_inter_buffer[i,1]
        temp_inter_buffer[i,1]=temp_inter_buffer[i,9]
        
        if cr[i]<0:
            cr[i]=-cr[i]
        if cr[i]<= M[i]:
            y_i[i]=(input_buffer[i,4]+input_buffer[i,5]+input_buffer[i,6]+(input_buffer[i,3]+input_buffer[i,7])/2)/4
            output_buffer[i]=y_i[i]/(1-KF)-input_buffer[i,5]*KF/(1-KF)
            temp_inter_buffer[i,1]=input_buffer[i,5]-output_buffer[i]
        else:
            restore_inter_bufer[i]=8*KF*temp_inter_buffer[i,3]-temp_inter_buffer[i,1]-2*(temp_inter_buffer[i,2]+temp_inter_buffer[i,3]+temp_inter_buffer[i,4])
            output_buffer[i]=input_buffer[i,5]-restore_inter_buffer[i]
        if Mmax[i] < output_buffer[i]:
            Mmax[i]=output_buffer[i]
        else:
            Mmax[i]=Mmax[i]-0.0001*(Mmax[i]-output_buffer[i])
        if Mmin[i] > output_buffer[i]:
            Mmin[i]=output_buffer[i]
        else:
            Mmin[i]=Mmin[i]+0.0001*(output_buffer[i]-Mmin[i])
        Mpp[i]=Mmax[i]-Mmin[i]
        if Mu[i]>Mpp[i]:
            Mu[i]=Mpp[i]
        else:
            Mu[i]=Mu[i]+0.01*Mpp[i]
        M[i]=0.1*Mu[i]
            
    return output_buffer



def readECG():
    data = []
    GPIO.output(CS, 0)
    time.sleep(0.0002)
    x=0
    for i in range(9):
        
        x=spi.xfer([0xFF])[0]
        # ~ x= (x<<8)|spi.xfer([0xFF])[0]
        # ~ x=(x<<8)|spi.xfer([0xFF])[0]
        
        data.append(x)
        
        
    GPIO.output(CS, 1)
    return data


############################ MAIN ################################
init_conf()
sendcmd(SDATAC)
writeReg(CONFIG1, 0X01)  # 220
writeReg(CONFIG2, 0XA0) 
writeReg(RLD_SENS, 0XFF) 
writeReg(CH1SET, 0X10) 
writeReg(CH2SET, 0X10) 


cadena="Lectura de Registros".capitalize()
print(cadena.center(50,"="))
readinfo()

startconv()
record = []
ch1=[]
ch2=[]

start = time.time()
cadena="Lectura de ECG".capitalize()
print(cadena.center(50,"="))
cadena="Tiempo Estimado: 10s ".capitalize()
print(cadena.center(50,"="))

try:
    while time.time()-start <10:
        if GPIO.input(DRDY) == 0:
            read = readECG()
            record.append(read)
       

    ch1,ch2=save2(record)
    ch1=np.array(ch1)
    ch2=np.array(ch2)
    
    b, a = signal.butter(3, 0.1)
    
    CANAL1=sus(ch1)
    CANAL2=sus(ch2)
    CANAL1C=signal.filtfilt(b,a,ch1)
    CANAL2C=signal.filtfilt(b,a,ch2)
    
    
    # ~ CANAL1=CANAL1[3500:4500]
    # ~ CANAL2=CANAL2[3500:4500]
    # ~ CANAL1C=CANAL1C[3500:4500]
    # ~ CANAL2C=CANAL2C[3500:4500]
    
    
    
    
    
    plt.figure(figsize=(15,7))
    ax0 = plt.subplot(2,2,1) 
    ax0.set_xlabel('Canal 1 CON ALG')
    ax0.plot(CANAL1)
    
    
    ax1 = plt.subplot(2,2,2)    
    ax1.set_xlabel('Canal 2 CON ALG')         
    ax1.plot(CANAL2)
    ax2 = plt.subplot(2,2,3)    
    ax2.set_xlabel('Canal 1 SIN ALG')         
    ax2.plot(CANAL1C)
    ax3 = plt.subplot(2,2,4)    
    ax3.set_xlabel('Canal 2 SIN ALG')         
    ax3.plot(CANAL2C)
    
    
    plt.show()
    
    
   
    file.close()
    GPIO.cleanup()
    spi.close
except KeyboardInterrupt:
    save(record)
    file.close()
    GPIO.cleanup()
    spi.close
   
  

