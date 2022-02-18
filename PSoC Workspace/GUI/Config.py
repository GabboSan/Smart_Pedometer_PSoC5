import os
import serial 

##########################
#### Globals editable ####
##########################

#general
CALIBRATION_TIME = 5 #seconds

#serial 
EEPROM_LEN = 65536
RESERVED_BYTES = 5
BAUDRATE = 115200

#live
scaling = 1

#dev
SAMPLE_PLOT = 100



#########################
### Globals protected ###
#########################

#general
CURRENT_PAGE = 1
path= os.path.dirname(os.path.realpath(__file__))+'/'

#serial 
CONN_STATUS = False
SERIAL_PORT = serial.Serial()
PORTNAME = ""
Profiles = []    # prototype: {"UsernameID": , "Username": ,"Gender": , "Height": }
CurrentProfile = 0

#Sessions
Sessions_Data = []      # prototype: {"n.session": , "Username_ID": , "Sdata": [total activity time, steps/minute(average cadence),n. of steps,stride] , "Path" : [ [x1,x2,..,xn] , [y1,y2,..,yn] ] }

#live
OpenGL_resize = False
newXY_path = [0,0]

#dev
NEW_DATA = False
class y_Data_plot:
    acc = [[0]*SAMPLE_PLOT]*3
    gyro = [[0]*SAMPLE_PLOT]*3
    magn = [[0]*SAMPLE_PLOT]*3
    RPY = [[0]*SAMPLE_PLOT]*3

x_Data_plot = list(range(SAMPLE_PLOT))