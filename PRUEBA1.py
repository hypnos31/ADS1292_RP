

import os
import spidev
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as signal
fs = 250
ts = 1/fs

WAKEUP = 0x02
STANDBY = 0X04
RESET = 0X06
START = 0X08
STOP = 0X0A
RDATAC = 0X10
SDATAC = 0X11
RDATA = 0X12
RREG = 0X20
WREG = 0X40

ID = 0X00
CONFIG1 = 0X01
CONFIG2 = 0X02
CONFIG3 = 0X03
LOFF = 0X04
CH1SET = 0X05
CH2SET = 0X06
CH3SET = 0X07
CH4SET = 0X08
CH5SET = 0X09
CH6SET = 0X0A
CH7SET = 0X0B
CH8SET = 0X0C

RLD_SENSP = 0X0D
RLD_SENSN = 0X0E
LOFF_SENSP = 0X0F
LOFF_SENSN = 0X10
LOFF_FLIP = 0X11


WCT1 = 0X18
WCT2 = 0X19


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


def correc(deri):
    N=len(deri)
    for i in range(N):
        if deri[i] < 100:
           deri[i]=deri[i]*0x7fffff
        else:
           deri[i]=deri[i]
    return deri
    
    

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
   
    
    for entry in data:
        c1=(entry[24] << 16) + (entry[25] << 8)+ entry[26]
        
        if c1 < 0x800000:
            c1=c1+0xff000000
        c1=c1/0x7fffff
        # ~ if c1 >= 0x800000 and c1<= 0xFFFFFF:
            # ~ c1 |= 0XFF000000
           
        # ~ c1=c1/0x7fffff
        
        
        
        c2=  (entry[3] << 16) + (entry[4] << 8)+ entry[5] 
        if c2 >= 0x800000 and c2<= 0xFFFFFF:
            c2 |= 0XFF000000
        c2=c2/0x7fffff
        
        
                
        c3=(entry[6] << 16) + (entry[7] << 8)+ entry[8]
        if c3 >= 0x800000 and c3<= 0xFFFFFF:
            c3 |= 0XFF000000
        c3=c3/0x7fffff
        
        
        c4=(entry[12] << 16) + (entry[13] << 8)+ entry[14]  
        if c4 >= 0x800000 and c2<= 0xFFFFFF:
            c4 |= 0XFF000000
        c4=c4/0x7fffff
        
        
        c5=(entry[9] << 16) + (entry[10] << 8)+ entry[11]  
        if c5 >= 0x800000 and c5<= 0xFFFFFF:
            c5 |= 0XFF000000
        c5=c5/0x7fffff
        
        
        c6=(entry[15] << 16) + (entry[16] << 8)+ entry[17] 
        if c6 >= 0x800000 and c6<= 0xFFFFFF:
            c6 |= 0XFF000000
        c6=c6/0x7fffff
        
        
        c7=(entry[18] << 16) + (entry[19] << 8)+ entry[20]  
        if c7 >= 0x800000 and c7<= 0xFFFFFF:
            c7 |= 0XFF000000
        c7=c7/0x7fffff
        
        
        c8=(entry[21] << 16) + (entry[22] << 8)+ entry[23]
        if c8 >= 0x800000 and c8<= 0xFFFFFF:
            c8 |= 0XFF000000
        c8=c8/0x7fffff
        
        
  
        C1.append(c1)
        C2.append(c2)
        C3.append(c3)
        C4.append(c4)
        C5.append(c5)
        C6.append(c6)
        C7.append(c7)
        C8.append(c8)
        file.write('{} \n'.format(c1))
    return C1,C2,C3,C4,C5,C6,C7,C8



def readinfo():
    sendcmd(SDATAC)
    sendcmd(STOP)
    info = readreg(ID)  # ID

    print('ID ES:', info)
    info = readreg(CONFIG3)  # CONFIG1
    print('CONFIG3 ES:', info)
    info = readreg(CH1SET)  # CONFIG2
    print('CCH1SET ES:', info)
    info = readreg(WCT1)  # RLDSENS
    print('WCT1 ES', info)
    info = readreg(WCT2)  # RLDSENS
    print('WCT2 ES', info)


def readECG():
    data = []
    GPIO.output(CS, 0)
    time.sleep(0.0002)
    x=0
    for i in range(27):
        
        x=spi.xfer([0xFF])[0]
        # ~ x= (x<<8)|spi.xfer([0xFF])[0]
        # ~ x=(x<<8)|spi.xfer([0xFF])[0]
        
        data.append(x)
        
        
    GPIO.output(CS, 1)
    return data


############################ MAIN ################################
init_conf()
sendcmd(SDATAC)
writeReg(CONFIG3, 0XDC)  # 220
writeReg(WCT1, 0X0B)  # 11
writeReg(WCT2, 0XD4)  # 212
writeReg(RLD_SENSP, 0XFF)
writeReg(RLD_SENSN, 0XFF)
writeReg(CH1SET, 0X10)  # 16
writeReg(CH2SET, 0X10)
writeReg(CH3SET, 0X10)
writeReg(CH4SET, 0X10)
writeReg(CH5SET, 0X10)
writeReg(CH6SET, 0X10)
writeReg(CH7SET, 0X10)
writeReg(CH8SET, 0X10)
writeReg(CONFIG2, 0X10)


cadena="Lectura de Registros".capitalize()
print(cadena.center(50,"="))
readinfo()

startconv()
record = []
ch1=[]
ch2=[]
ch3=[]
ch4=[]
ch5=[]
ch6=[]
ch7=[]
ch8=[]
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
       

    ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8=save2(record)
    
    # ~ ch1=np.array(ch1)
    # ~ ch2=np.array(ch2)
    # ~ ch3=np.array(ch3)
    # ~ ch4=np.array(ch4)
    # ~ ch5=np.array(ch5)
    # ~ ch6=np.array(ch6)
    # ~ ch7=np.array(ch7)
    # ~ ch8=np.array(ch8)
    
    # ~ ch1=correc(ch1)
    # ~ ch2=correc(ch2)
    # ~ ch3=correc(ch3)
    # ~ ch4=correc(ch4)
    # ~ ch5=correc(ch5)
    # ~ ch6=correc(ch6)
    # ~ ch7=correc(ch7)
    # ~ ch8=correc(ch8)
    
    # ~ ch1=sus(ch1)
    # ~ ch2=sus(ch2)
    # ~ ch3=sus(ch3)
    # ~ ch4=sus(ch4)
    # ~ ch5=sus(ch5)
    # ~ ch6=sus(ch6)
    # ~ ch7=sus(ch7)
    # ~ ch8=sus(ch8)
    
    # ~ ch1=ch1[220:300]
    # ~ ch2=ch2[220:300]
    # ~ ch3=ch3[220:300]
    # ~ ch4=ch4[220:300]
    # ~ ch5=ch5[220:300]
    # ~ ch6=ch6[220:300]
    # ~ ch7=ch7[220:300]
    # ~ ch8=ch8[220:300]
   
    
    
    plt.figure(figsize=(13,8))
    ax0 = plt.subplot(4,2,1) 
    ax0.set_ylabel('canal 1')
    ax0.plot(ch1)
    
    
    ax1 = plt.subplot(4,2,2)    
    ax1.set_ylabel('canal 2')         
    ax1.plot(ch2)
    
    ax2=plt.subplot(4,2,3)
    ax2.set_ylabel('canal 3')  
    ax2.plot(ch3)
    
    ax3 = plt.subplot(4,2,4) 
    ax3.set_ylabel('canal 4')  
    ax3.plot(ch4)
    
    
    ax4 = plt.subplot(4,2,5)
    ax4.set_ylabel('canal 5')               
    ax4.plot(ch5)
    
    ax5=plt.subplot(4,2,6)
    ax5.set_ylabel('canal 6')  
    ax5.plot(ch6)
    
    ax6 = plt.subplot(4,2,7)
    ax6.set_ylabel('canal 7')               
    ax6.plot(ch7)
    
    ax7=plt.subplot(4,2,8)
    ax7.set_ylabel('canal 8')  
    ax7.plot(ch8)
    
    plt.show()
    
    
   
    file.close()
    GPIO.cleanup()
    spi.close
except KeyboardInterrupt:
    save(record)
    file.close()
    GPIO.cleanup()
    spi.close

  
