## NOTA GENERAL: LOS BITS SE ENVIAN AL REVES 



import spidev
import RPi.GPIO as GPIO
import time


data_send=0
ads1292_ready=0
filename = input('Enter file name: ')
file = open('{}.txt'.format(filename),'w') #Open a file now for saving data later
spi = spidev.SpiDev() #initialize the spi package


  

def trigger():
    global ads1292_ready
    ads1292_ready=1


def init_conf(): ####### CONFIGURACION INICIAL 
  
    global  CS_gpio, DRDY_gpio, ADS, data_send
    GPIO.setmode(GPIO.BCM)
    CS_gpio = 8
    DRDY_gpio = 12
    GPIO.setup(CS_gpio, GPIO.OUT)
    GPIO.output(CS_gpio, True)
    GPIO.setup(DRDY_gpio, GPIO.IN, pull_up_down = GPIO.PUD_UP)
      
    ADS = spidev.SpiDev()
    ADS.mode = 0b01
    ADS.max_speed_hz =4000000 #4MHz, FRECUENCIA PARA COMUNICACION SPI
    sendcmd(0X60)
    time.sleep(0.0001)
    DRDY_gpio.fall(trigger()) #### OJO AQUI


    
    
def sendcmd(comando):
    time.sleep(0.00002)
    GPIO.output(CS_gpio, False)
    time.sleep(0.00001)
    ADS.writebytes(comando)
    time.sleep(0.00003)
    GPIO.output(CS_gpio,True)
    time.sleep(0.00002)
    
def writeReg(direccion, dato):
    direccion |= 0x04
    time.sleep(0.00002)
    GPIO.output(CS_gpio, False)
    time.sleep(0.00001)
    spi.xfer([direccion])
    spi.xfer([0X00])
    time.sleep(0.00002)
    spi.xfer([dato])
    time.sleep(0.00002)
    GPIO.output(CS_gpio,True)
    time.sleep(0.00002) 
   
def readreg(direccion,dato):
    direccion |= 0x02
    time.sleep(0.00002)
    GPIO.output(CS_gpio, False)
    time.sleep(0.00001)
    spi.xfer([direccion])
    time.sleep(0.00002)
    spi.xfer([0])
    time.sleep(0.00002)
    dato=spi.xfer([0XFF])
    time.sleep(0.00003)
    GPIO.output(CS_gpio,True)
    time.sleep(0.00002) 
    return dato
   

def startconv():
    global data_send
    sendcmd(0x01) # RDATAC 0X10
    sendcmd(0x80) # START 0X08
    data_send=1 
    
def stopconv():
    global data_send
    sendcmd(0x11) # SDATAC 0X11
    sendcmd(0xA0) # STOP 0X0A
    data_send=0    
    
def save(data):
    
    status =0
    chan1 = 0
    chan2 = 0
    MSB= 2*pow(2,23);
    for entry in data:
        
        status = format((entry[0]<< 16) + (entry[1] << 8) + entry[2],'024b')
        
        chan1 = (entry[3] << 16) + (entry[4] << 8 )+ entry[5]
        if format(chan1,'024b')[0] == 1:
            chan1 = chan1 - MSB
        chan2 = (entry[6] << 16 )+ (entry[7] << 8 ) + entry[8]
        if format(chan2,'024b')[0] == 1:
            chan2 = chan2 - MSB
        
        file.write('{} , {} , {} \n'.format(status,chan1,chan2))
        
def readECG():#The ECG data consists of 72 bits of data, of which byte 4:6 is ch1
    data =[]
    if ads1292_ready==1:
        GPIO.output(CS_gpio, False)
        time.sleep(0.0002)
        for i in range(9):
            data.append(spi.xfer([0XFF])[0])#Hence, send 9 times a dummy byte to get the data in return from the ADS
        
        GPIO.output(CS_gpio,True)
    return data
    
############################ MAIN ################################
init_conf()
sendcmd(0X11)
writeReg(0X01, 0X01)####### CONFIG1 ---- 0X01
writeReg(0X02, 0XA0)####### CONFIG2 ---- 0XA0
writeReg(0X06, 0XFF)####### RLD_SENS ---- 0XFF
writeReg(0X04, 0X10)####### CH1 ---- 0X10
writeReg(0X04, 0X10)####### CH2 ---- 0X10

startconv()    
record=[]
start=time.time()
while  time.time()-start<10:
        read=readECG()
        record.append(read)
        save(record)
    
    
file.close()
GPIO.cleanup()
spi.close
