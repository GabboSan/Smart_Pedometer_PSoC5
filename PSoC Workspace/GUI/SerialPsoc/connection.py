import logging
import serial
import serial.tools.list_ports
import time
import Config
from PyQt5.QtCore import QThread,pyqtSignal
import PyQt5.QtWidgets
from math import sqrt


class Bluetooth_autoconnection(QThread):

    closeStart = pyqtSignal()
    def __init__(self, page, mainW):
        QThread.__init__(self,page)
        self.page = page
        self.mainW = mainW
        self.psocFound = False

    def run(self):
        self.scan_ports()

    def scan_ports(self):
        self.port_list = serial.tools.list_ports.comports()
        for idx,p in enumerate(reversed(self.port_list)):
            Config.PORTNAME = p.name
            self.page.progressSerial.setValue((round(100/len(self.port_list))) * (idx+1))
            self.page.port_found.setText(f"{len(self.port_list)} port found, trying {Config.PORTNAME}")

            if (CheckIfPsoc(self.mainW,self.page)):
                self.psocFound = True
                break
            
        if self.psocFound == False: 
            Config.SERIAL_PORT = serial.Serial() #reset port to avoid port open
            self.page.progressSerial.setValue(100)
            self.page.port_found.setText("cannot connect automatically, please do it manually")
            self.page.connect_btn.setText("Start anyway")
            self.page.connect_btn.clicked.disconnect()
            self.page.connect_btn.clicked.connect(self.page.startMain)
        else:
            #close starting window and open the operative one
            self.closeStart.emit()   


def CheckIfPsoc(mainW,serPage):
    try:
        Config.SERIAL_PORT = serial.Serial(Config.PORTNAME, baudrate=Config.BAUDRATE,write_timeout=1, timeout=2) 
        Config.SERIAL_PORT.write(b'$C\n')
        time.sleep(0.2)
        answer = Config.SERIAL_PORT.read(3)


        if answer == b'$A\n':
            #ask current profile from psoc 
            Get_Current_Profile(mainW)   
            #download session data from psoc 
            if serPage!=0:
                Get_Sessions(True,serPage)  # true -> call from connection window
            else:
                mainW.conn_status_label.setText("Downloading sessions")
                mainW.conn_status_label.setStyleSheet("color: rgb(255, 255, 0)")
                PyQt5.QtWidgets.qApp.processEvents()
                Get_Sessions(False,serPage)  # false -> call from main window
            #send signal to update GUI with session data 
            mainW.SessionDownloaded.emit() 

            # change connection status label
            mainW.com_list_widget.setCurrentText(Config.PORTNAME)
            mainW.com_list_widget.setDisabled(True)
            mainW.conn_status_label.setText("Connected")
            time.sleep(0.3) #need to correctly change label without issues 
            mainW.conn_btn.setText(f"Disonnect from port {Config.PORTNAME}")
            mainW.conn_btn.setChecked(True)
            mainW.conn_status_label.setStyleSheet("color: rgb(85, 255, 0)")
            #enable user action 
            mainW.calib_button.setEnabled(True)
            mainW.change_profile_btn.setEnabled(True)
            mainW.remove_profile_btn.setEnabled(True)
            mainW.startLiveBtn.setEnabled(True)
            mainW.dev_btn.setEnabled(True)
            mainW.dev_btn2.setEnabled(True)
            mainW.add_usr_btn.setEnabled(True)
            return True
        
        else:
            Config.SERIAL_PORT.close()
            #self.page.port_found.setText(f"wrong answer from port {p.name}: {self.answer}")
            logging.info(f"wrong answer from port {Config.PORTNAME}: {answer}")
            if (mainW.isVisible()):
                mainW.conn_btn.setChecked(False)
            return False

    except:
        Config.SERIAL_PORT.close()
        logging.info(f"error with port {Config.PORTNAME}")
        # if connection is asked from main window, reset button
        if (mainW.isVisible()):
            mainW.conn_btn.setChecked(False)
        return False


def Get_Current_Profile(mainW):
        # get current profile from psoc 
        Config.SERIAL_PORT.write(b"$P\n")
        time.sleep(0.1)
        package=Config.SERIAL_PORT.read(3)
        if (package[0]==0x24 and package[2]==0x0A):
            Config.CurrentProfile = package[1]
            logging.info(f"profile loaded: {package[1]}")
            #set current profile label into main window 
            txt_tmp=""
            for profile in Config.Profiles:
                if profile['UsernameID']==Config.CurrentProfile:
                    for key, value in profile.items():
                        txt_tmp += f"{key} : {value}\n"
                        mainW.profile_label.setText(txt_tmp)
                    break
            

        else:
            logging.info(f"ERROR: profile not loaded: received {package}")

def Get_Sessions(isConnW,serPage): 
    Config.Sessions_Data = [] # first clear previous session data
    Config.SERIAL_PORT.write(b"$S\n") #ask psoc for sessions data 
    time.sleep(0.1)
    idx = 0
    sessionData=[]
    
    if isConnW:
        while(idx!=Config.EEPROM_LEN-Config.RESERVED_BYTES): 
            idx+=1
            sessionData.append(Config.SERIAL_PORT.read(1))
            serPage.advance_progress.emit(round(100*idx/(Config.EEPROM_LEN-Config.RESERVED_BYTES)))

    else:
        while(idx!=Config.EEPROM_LEN-Config.RESERVED_BYTES): 
            idx+=1
            sessionData.append(Config.SERIAL_PORT.read(1))
        
    
    sessions=[]
    for index in range(0,len(sessionData)):
        if sessionData[index] == b'\xff' and sessionData[index+1] == b'\xfe':
            index2=index+2
            while sessionData[index2]!= b'\xff' or sessionData[index2+1]!= b'\xff':
                index2+=1
            sessions.append((sessionData[index:index2+2]))
            index=index2


    for session in sessions:
        act_time=2*(len(session)-8)/60
        x_coords=[]
        y_coords=[]
        for i in range(4,len(session)-4,4):
            if (len(x_coords)==0):
                x_coords.append( int.from_bytes(session[i]+session[i+1],"little",signed=True) )
                y_coords.append( int.from_bytes(session[i+2]+session[i+3],"little",signed=True) )
            else:
                x_coords.append( int.from_bytes(session[i]+session[i+1],"little",signed=True) + x_coords[-1] )
                y_coords.append( int.from_bytes(session[i+2]+session[i+3],"little",signed=True) + y_coords[-1] )


        steps = int.from_bytes(session[-3],"little")
        sum_dist=0
        for i in range(len(x_coords) - 1):  #compute total path length
            sum_dist += sqrt((x_coords[i+1] - x_coords[i])**2 + (y_coords[i+1] - y_coords[i])**2)

        Config.Sessions_Data.append({"N_Session": int.from_bytes(session[2],"little"), "UsernameID": int.from_bytes(session[3],"little"), "Sdata": [round(act_time,2),round(steps/act_time,2),steps,round(sum_dist)] , "Path" : [ x_coords , y_coords ] })

        #logging.info(Config.Sessions_Data)
        #compute Sdata 
        #Sdata[0]=2*len(path)/60  #activity time (in minutes)
        #Sdata[1]=steps/Sdata[0]  #cadence
        #Sdata[2]=steps 
        #Sdata[3]=total path length