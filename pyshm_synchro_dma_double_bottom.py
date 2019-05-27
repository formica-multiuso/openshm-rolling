from pyb import I2C
from pyb import SPI
from pyb import ExtInt, Pin, Timer
from pyb import UART
import utime
import micropython
import pyb
from array import array
from struct import *
import ubinascii
import stm
import uctypes
import ustruct
import gc
from parameters import *
from parser import CommandParser
micropython.alloc_emergency_exception_buf(100)

NODENAME = 'DEVNODE'
LOGFILE = NODENAME + '.log'

#gc.disable()

## ADC SECTION ##
ADS131_RDATAC =    b'\x10' # Enable read data continuous mode (Default mode at power-up)
ADS131_SDATAC =    b'\x11' # Stop read data continuous mode
ADS131_RDATA =     b'\x12' # Read data by command

## MAGNETOMETER SECTION ##
MAG_ADDR = 0x0E
MAG_AUTOINCREMENT = 0x01
MAG_X_MSB = 0x01
MAG_X_LSB = 0x02
MAG_Y_MSB = 0x03
MAG_Y_LSB = 0x04
MAG_Z_MSB = 0x05
MAG_Z_LSB = 0x06
MAG_TEMP = 0x0F # Signed 8 bit temperature
MAG_CTRL_1 = 0x10
MAG_CTRL_2 = 0x11

## GYROSCOPE SECTION ##
LGD = 0x6b #Device I2C slave address
LGD_WHOAMI_ADDRESS = 0x0F
LGD_WHOAMI_ID = 0b11010111 #Device self-id
LGD_CTRL_1 = 0x20 #turns on gyro
LGD_CTRL_2 = 0x21 #can set a high-pass filter for gyro
LGD_CTRL_3 = 0x22
LGD_CTRL_4 = 0x23
LGD_CTRL_5 = 0x24
LGD_CTRL_6 = 0x25
LGD_TEMP = 0x26
LGD_GYRO_X_LSB = 0x28
LGD_GYRO_X_MSB = 0x29
LGD_GYRO_Y_LSB = 0x2A
LGD_GYRO_Y_MSB = 0x2B
LGD_GYRO_Z_LSB = 0x2C
LGD_GYRO_Z_MSB = 0x2D
# Auto-increment mode
LGD_GYRO_X = 0xa8
LGD_GYRO_Y = 0xaa
LGD_GYRO_Z = 0xac
LGD_GYRO_TEMP_TWOBYTE = 0xa6

pwr = Pin("PC7", Pin.OUT_PP)
drdy = Pin("AD_DRDY", Pin.IN)
st =  Pin('AD_START', Pin.OUT_PP)
rst = Pin('AD_RESET', Pin.OUT_PP)
cs = Pin('AD_CS', Pin.OUT_PP) # NOT USED
cs_clino = Pin('PB11', Pin.OUT_PP) 
uart_out = Pin('PA14',Pin.OUT_PP)
pa13 = Pin('PA13', Pin.IN)
pa5 = Pin('PA5', Pin.IN) #Radio Sync
pb0 = Pin('PB0', Pin.OUT) # Radio Reset
pb0.high()

ly = Pin('LED_YELLOW', Pin.OUT_PP)
lg = Pin('LED_GREEN', Pin.OUT_PP)
lb = Pin('LED_BLUE', Pin.OUT_PP)
lr = Pin('LED_RED', Pin.OUT_PP)

spi = SPI(4, SPI.MASTER, baudrate=2000000, firstbit=SPI.MSB, polarity=0, phase=1, bits=8, nss=0x40000) # Accel
spi2 = SPI(2, SPI.MASTER, baudrate=2000000, firstbit=SPI.MSB, polarity=1, phase=1, bits=8, nss=0x40000) # Clino
i2c = I2C(1,I2C.MASTER, baudrate=1000000) # Magnetometer, Gyroscope

sync_uart = UART(2, 115200)
sync_uart.init(115200, bits=8, parity=None, stop=1, timeout=1)

cp = CommandParser(LOGFILE,lr,lb,ly)

# ADS131-E4 INIT (Accelerometers: LIS344ALH)
lg.high()
pwr.high()
utime.sleep_ms(150)
rst.low()
utime.sleep_us(3)
rst.high()
utime.sleep_us(10)
st.low()
utime.sleep_ms(1)
spi.write(ADS131_SDATAC)
utime.sleep_ms(1)
spi.write(bytearray([0x41,0x02,0xd6,0xe0,0xcc])) # 4th: 0xe0 normal, 0xf1 square wave test
utime.sleep_ms(1)
spi.write(bytearray([0x45,0x03,0x10,0x10,0x10,0x90])) #3,4,5: 0x15 test with Gain = 1, 0x10 normal
utime.sleep_ms(1)
spi.write(ADS131_SDATAC)
stm.mem8[0x40013404] = 0x07
utime.sleep(1)
lg.low()
utime.sleep(1)

clino_tx_buf_X = bytearray(2)
clino_tx_buf_Y = bytearray(2)
clino_tx_buf_temp = bytearray(2)
clino_rx_buf_X = bytearray(2)
clino_rx_buf_Y = bytearray(2)
clino_rx_buf_temp = bytearray(2)

uart_buf = bytearray(3)

# ADIS16209 (clino) - L3GD20H (gyro) - MAG3110 (mag) INIT
lg.high()
cs_clino.high()
cs_clino.low()
spi2.write(bytearray([0xb4,0x06])) # MSC_CTRL
utime.sleep_ms(1)
spi2.write(bytearray([0xb6,0x03])) # SMPL_PRD
utime.sleep_ms(1)
spi2.write(bytearray([0xb8,0x0C])) # AVG_CNT
clino_tx_buf_X[0] = 0x0C
clino_tx_buf_Y[0] = 0x0E
clino_tx_buf_temp[0] = 0x0A
i2c.mem_write(0x0F,LGD, LGD_CTRL_1)
utime.sleep_ms(1)
i2c.mem_write(0x30,LGD,LGD_CTRL_4) # Set FULL SCALE 2000dps
utime.sleep_ms(1)
i2c.mem_write(0x80,MAG_ADDR,MAG_CTRL_2)
utime.sleep_ms(1)
i2c.mem_write(0x01,MAG_ADDR,MAG_CTRL_1)

txbuf = bytearray(16)
txbuf[0] = 0x12 # RDATA
tx_addr = uctypes.addressof(txbuf)
utime.sleep(1)
lg.low()
utime.sleep(1)

lg.high()
rxbuf0 = bytearray(1488)
rxbuf1 = bytearray(1488)
shmframe = bytearray(1536)
shmframe_read = bytearray(256)
rx0_addr = uctypes.addressof(rxbuf0)
rx1_addr = uctypes.addressof(rxbuf1)
shmframe_read_addr = uctypes.addressof(shmframe_read)
shm_mv = memoryview(shmframe) 
utime.sleep_ms(1)

stm.mem32[stm.RCC + stm.RCC_AHB1ENR] |= (1 << 21)
stm.mem32[stm.RCC + stm.RCC_AHB1ENR] |= (1 << 22)
utime.sleep(1)

# Basetime and TIM/DMA ENABLE
tim = Timer(1)
tim.init(freq=16000)
stm.mem32[stm.TIM1 + stm.TIM_DIER] |= (1 << 8)
stm.mem32[stm.TIM1 + stm.TIM_DIER] |= (1 << 14)
tim.deinit()
utime.sleep(1)

# TX Timed DMA
stm.mem32[stm.DMA2 + 0x8C] = 16
stm.mem32[stm.DMA2 + 0x90] = 0x4001340C
stm.mem32[stm.DMA2 + 0x94] = tx_addr
stm.mem32[stm.DMA2 + 0x88] = 0x0C000540
utime.sleep_ms(10)

# RX DMA
stm.mem32[stm.DMA2 + 0x0014] = 1488
stm.mem32[stm.DMA2 + 0x0018] = 0x4001340C
stm.mem32[stm.DMA2 + 0x001C] = rx0_addr
stm.mem32[stm.DMA2 + 0x0020] = rx1_addr
stm.mem32[stm.DMA2 + 0x0010] = 0x08060500
utime.sleep_ms(10)

# DMA ENABLE
stm.mem32[stm.DMA2 + 0x0088] = 0x0C000541
stm.mem32[stm.DMA2 + 0x0010] = 0x08060501
utime.sleep_ms(10)
lg.low()


#UART DMA TX 
stm.mem32[0x40004414] |= (1<<6)
stm.mem32[0x40004414] |= (1<<7)

uart_tx_buf = bytearray(1488)
uart_tx_buf[0] = 0xFF
uart_tx_buf[700] = 0xFF
uart_tx_buf[1485] = 0xFF

uart_tx_addr = uctypes.addressof(uart_tx_buf)

stm.mem32[stm.DMA1 + 0x00A4] = 16
stm.mem32[stm.DMA1 + 0x00A8] = 0x40004404
stm.mem32[stm.DMA1 + 0x00AC] = uart_tx_addr

stm.mem32[0x40004400] |= (0 << 6)

#UART DMA RX
uart_rx_buf0 = bytearray(16)
uart_rx_addr0 = uctypes.addressof(uart_rx_buf0)
uart_rx_buf1 = bytearray(16)
uart_rx_addr1 = uctypes.addressof(uart_rx_buf1)

stm.mem32[stm.DMA1 + 0x8C] = 16
stm.mem32[stm.DMA1 + 0x90] = 0x40004404
stm.mem32[stm.DMA1 + 0x94] = uart_rx_addr0
stm.mem32[stm.DMA1 + 0x98] = uart_rx_addr1
stm.mem32[stm.DMA1 + 0x88] = 0x08070501
### END ###

# |NETWORK_ID(2)|NODE_ID(2)|STATUS|FILES_NUM|ALARM1|ALARM2|TIMESTAMP(8)| (16 bytes)#
status_buf = bytearray(16)
status_buf_addr = uctypes.addressof(status_buf)
status_mv = memoryview(status_buf)
status_mv[0:2] = ustruct.pack('>H',NETWORK_ID)
status_mv[2:4] = ustruct.pack('>H',NODE_ID)
#status_mv[8:9] = ustruct.pack('>H',31)
#status_mv[9:10] = ustruct.pack('>H',32)
#status_mv[10:11] = ustruct.pack('>H',33)
#status_mv[11:12] = ustruct.pack('>H',34)
#status_mv[12:13] = ustruct.pack('>H',35)
#status_mv[13:14] = ustruct.pack('>H',36)
#status_mv[14:15] = ustruct.pack('>H',37)
#status_mv[15:16] = ustruct.pack('>H',38)

status_mv[4] = 1 # STR

utime.sleep(1)
lg.high()

sync_detect = 0

gtimestamp = bytearray(16) # Global Timestamp
ltimestamp = 0	# Local Timestamp


# Datagram copy
@micropython.viper
def timestamp_cpy(src:ptr8,dest:ptr8):
	for i in range(16):
		dest[i] = src[i]

def rfsync_callback(pin):
	#global rxbuf
	global gtimestamp, ltimestamp, uart_rx_buf, sync_detect
	#lb.high()
	ltimestamp = utime.ticks_us()
	if ((stm.mem32[stm.DMA1 + 0x0088] & (1 << 19)) != 0):
		timestamp_cpy(uart_rx_buf0,gtimestamp)
		sync_detect=1
	else:
		timestamp_cpy(uart_rx_buf1,gtimestamp)
		sync_detect=1
	#sync_uart.readinto(gtimestamp)
	#utime.sleep_ms(1)
	#print(gtimestamp)
	#lb.low()
	#sync_detect = 1

# Datagram copy
@micropython.viper
def datacpy(src:ptr8,dest:ptr8):
	for i in range(1488):
		dest[i+48] = src[i]

rfSyncInt = pyb.ExtInt(pa5, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_NONE, rfsync_callback)
rfSyncInt.disable()

# START ACQ
tim.init(freq=16000)

buf0_mutex = 0
buf1_mutex = 0
frame_counter = 0
file_counter = 0
fdo_count = 0
fdo = 0

gyroX = bytearray(2)
gyroY = bytearray(2)
gyroZ = bytearray(2)
magXY = bytearray(4)
magX = bytearray(2)
magY = bytearray(2)
gyrotemp = bytearray(2)

period = utime.ticks_us()

fd = open('OPENSHM_'+ str(file_counter) + '.shm','wb')
#fd_log = open(LOGFILE,'w')

lg.high()
start1=utime.ticks_us()
start0=utime.ticks_us()

while True:
        if ((stm.mem32[stm.DMA2 + 0x0010] & (1 << 19)) != 0):
		if (buf0_mutex==0):
			# Write Buffer0 while DMA is writing Buffer1
			#period0 = ustruct.pack('>I',utime.ticks_diff(utime.ticks_us(),start0))			
			start1 = utime.ticks_us()

			#sync_uart.read(16)

			# RADIO TIMESTAMP INTERRUPT WINDOW
			rfSyncInt.enable()
			pyb.delay(15)
			rfSyncInt.disable()

			if(sync_detect==1):
				lb.high()

			# 48-bytes STS header
			# Space header
			shm_mv[0:2] = ustruct.pack('>H', NETWORK_ID)
			shm_mv[2:4] = ustruct.pack('>H', NODE_ID)
			shm_mv[4:6] = ustruct.pack('>h', CONST_LATITUDE_H)
			shm_mv[6:8] = ustruct.pack('>h', CONST_LATITUDE_L)
			shm_mv[8:10] = ustruct.pack('>h', CONST_LONGITUDE_H)
			shm_mv[10:12] = ustruct.pack('>h', CONST_LONGITUDE_L)
			shm_mv[12:16] = ustruct.pack('>I', DUMMY_CHECKSUM)
			
			# Time header			
			##shm_mv[16:20] = period0
			shm_mv[16:20] = ustruct.pack('>I',start0)			
			shm_mv[20] = gtimestamp[6]
			shm_mv[21] = gtimestamp[7]
			shm_mv[22] = gtimestamp[8]
			shm_mv[23] = gtimestamp[9]
			shm_mv[24] = gtimestamp[10]
			shm_mv[25] = gtimestamp[11]
			shm_mv[26] = gtimestamp[12]
			shm_mv[27] = gtimestamp[13]
			shm_mv[28:32] = ustruct.pack('>I',ltimestamp)
			
			# COMPLETE RADIO PACKET TEST			
			#shm_mv[16] = gtimestamp[0]
			#shm_mv[17] = gtimestamp[1]
			#shm_mv[18] = gtimestamp[2]
			#shm_mv[19] = gtimestamp[3]
			#shm_mv[20] = gtimestamp[4]
			#shm_mv[21] = gtimestamp[5]
			#shm_mv[22] = gtimestamp[6]
			#shm_mv[23] = gtimestamp[7]
			#shm_mv[24] = gtimestamp[8]
			#shm_mv[25] = gtimestamp[9]
			#shm_mv[26] = gtimestamp[10]
			#shm_mv[27] = gtimestamp[11]
			#shm_mv[28] = gtimestamp[12]
			#shm_mv[29] = gtimestamp[13]
			#shm_mv[30] = gtimestamp[14]
			#shm_mv[31] = gtimestamp[15]
			
		
			# Sensor header aka Secondary Datagram
			spi2.send_recv(clino_tx_buf_X, clino_rx_buf_X)
			spi2.send_recv(clino_tx_buf_Y, clino_rx_buf_Y)
			i2c.mem_read(gyroX,LGD,LGD_GYRO_X)
			i2c.mem_read(gyroY,LGD,LGD_GYRO_Y)
			i2c.mem_read(gyroZ,LGD,LGD_GYRO_Z)
			i2c.mem_read(gyrotemp,LGD,LGD_GYRO_TEMP_TWOBYTE)
			i2c.mem_read(magXY,MAG_ADDR,MAG_AUTOINCREMENT)
			i2c.mem_read(magX,MAG_ADDR,MAG_X_MSB)
			i2c.mem_read(magY,MAG_ADDR,MAG_Y_MSB)

			shm_mv[32:34] = bytes(clino_rx_buf_X) # Sensor0
			shm_mv[34:36] = bytes(clino_rx_buf_Y) # Sensor1
			shm_mv[36:38] = bytes(gyroX) # Sensor 2
			shm_mv[38:40] = bytes(gyroY) # Sensor 3
			shm_mv[40:42] = bytes(gyroZ) # Sensor 4
			shm_mv[42:44] = bytes(magX)
			shm_mv[44:46] = bytes(magY)

			#shm_mv[42:46] = bytes(magXY) # Sensor 5 and 6
			shm_mv[46:48] = bytes(gyrotemp) # Sensor 7
			
			# 1488 bytes Primary Datagram (~93ms)
			datacpy(rxbuf0,shmframe)			
			
			# Write 1536-bytes frame
			fd.write(shmframe)

			buf0_mutex = 1
			buf1_mutex = 0
                	frame_counter += 1
	else:
		if (buf1_mutex==0):
			# Write Buffer1 while DMA is writing Buffer0
			#period1 = ustruct.pack('>I',utime.ticks_diff(utime.ticks_us(),start1))			
			start0 = utime.ticks_us()

			#sync_uart.read(16)

			# RADIO TIMESTAMP INTERRUPT WINDOW
			rfSyncInt.enable()
			pyb.delay(15)
			rfSyncInt.disable()

			lb.low()
			sync_detect=0
		
			# 48-bytes STS header
			# Space header
			shm_mv[0:2] = ustruct.pack('>H', NETWORK_ID)
			shm_mv[2:4] = ustruct.pack('>H', NODE_ID)
			shm_mv[4:6] = ustruct.pack('>h', CONST_LATITUDE_H)
			shm_mv[6:8] = ustruct.pack('>h', CONST_LATITUDE_L)
			shm_mv[8:10] = ustruct.pack('>h', CONST_LONGITUDE_H)
			shm_mv[10:12] = ustruct.pack('>h', CONST_LONGITUDE_L)
			shm_mv[12:16] = ustruct.pack('>I', DUMMY_CHECKSUM)
			
			# Time header			
			##shm_mv[16:20] = period1
			shm_mv[16:20] = ustruct.pack('>I',start1)			
			shm_mv[20] = gtimestamp[6]
			shm_mv[21] = gtimestamp[7]
			shm_mv[22] = gtimestamp[8]
			shm_mv[23] = gtimestamp[9]
			shm_mv[24] = gtimestamp[10]
			shm_mv[25] = gtimestamp[11]
			shm_mv[26] = gtimestamp[12]
			shm_mv[27] = gtimestamp[13]
			shm_mv[28:32] = ustruct.pack('>I',ltimestamp)
			
			# COMPLETE RADIO PACKET TEST			
			#shm_mv[16] = gtimestamp[0]
			#shm_mv[17] = gtimestamp[1]
			#shm_mv[18] = gtimestamp[2]
			#shm_mv[19] = gtimestamp[3]
			#shm_mv[20] = gtimestamp[4]
			#shm_mv[21] = gtimestamp[5]
			#shm_mv[22] = gtimestamp[6]
			#shm_mv[23] = gtimestamp[7]
			#shm_mv[24] = gtimestamp[8]
			#shm_mv[25] = gtimestamp[9]
			#shm_mv[26] = gtimestamp[10]
			#shm_mv[27] = gtimestamp[11]
			#shm_mv[28] = gtimestamp[12]
			#shm_mv[29] = gtimestamp[13]
			#shm_mv[30] = gtimestamp[14]
			#shm_mv[31] = gtimestamp[15]
	
			# Sensor header aka Secondary Datagram
			spi2.send_recv(clino_tx_buf_X, clino_rx_buf_X)
			spi2.send_recv(clino_tx_buf_Y, clino_rx_buf_Y)
			i2c.mem_read(gyroX,LGD,LGD_GYRO_X)
			i2c.mem_read(gyroY,LGD,LGD_GYRO_Y)
			i2c.mem_read(gyroZ,LGD,LGD_GYRO_Z)
			i2c.mem_read(gyrotemp,LGD,LGD_GYRO_TEMP_TWOBYTE)
			i2c.mem_read(magXY,MAG_ADDR,MAG_AUTOINCREMENT)
			i2c.mem_read(magX,MAG_ADDR,MAG_X_MSB)
			i2c.mem_read(magY,MAG_ADDR,MAG_Y_MSB)

			shm_mv[32:34] = bytes(clino_rx_buf_X) # Sensor0
			shm_mv[34:36] = bytes(clino_rx_buf_Y) # Sensor1
			shm_mv[36:38] = bytes(gyroX) # Sensor 2
			shm_mv[38:40] = bytes(gyroY) # Sensor 3
			shm_mv[40:42] = bytes(gyroZ) # Sensor 4
			shm_mv[42:44] = bytes(magX)
			shm_mv[44:46] = bytes(magY)
			#shm_mv[42:46] = bytes(magXY) # Sensor 5 and 6
			shm_mv[46:48] = bytes(gyrotemp) # Sensor 7
				
			
			# 1488 bytes Primary Datagram (~93ms)
			datacpy(rxbuf1,shmframe)			

			# Write 1536-bytes frame
			fd.write(shmframe)

			buf1_mutex = 1
			buf0_mutex = 0	
			frame_counter += 1
		

        if (frame_counter==215):
                ly.high()
                fd.close()
                frame_counter = 0
                file_counter += 1
		gc.collect()
#		fd_log.close()
#		fd_log.open(LOGFILE,'w')
                fd = open('OPENSHM_'+ str(file_counter) + '.shm','wb')
		ly.low()

