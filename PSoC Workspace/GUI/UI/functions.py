from main import MainWindow
from PyQt5.QtCore import QPropertyAnimation, QEasingCurve, Qt, QThread, pyqtSignal
import Config
from IMU3D import OpenGLwidget
import logging
import serial
import serial.tools.list_ports
import Config 
from SerialPsoc import worker
import time
from IMU3D import PygameOpenGL
from PyQt5.QtWidgets import QListWidgetItem
from UI import Sessions

class UIFunctions(MainWindow):

    def toggleMenu(self, maxWidth, enable):
        if enable:

            # GET WIDTH
            width = self.frame_left_menu.width()
            maxExtend = maxWidth
            standard = 60

            # SET MAX WIDTH
            if width == 60:
                widthExtended = maxExtend
                self.btn_page_1.setToolButtonStyle(Qt.ToolButtonTextOnly)
                self.btn_page_2.setToolButtonStyle(Qt.ToolButtonTextOnly)
                self.btn_page_3.setToolButtonStyle(Qt.ToolButtonTextOnly)
                self.btn_page_4.setToolButtonStyle(Qt.ToolButtonTextOnly)
                self.btn_page_5.setToolButtonStyle(Qt.ToolButtonTextOnly)

            else:
                widthExtended = standard
                self.btn_page_1.setToolButtonStyle(Qt.ToolButtonIconOnly)
                self.btn_page_2.setToolButtonStyle(Qt.ToolButtonIconOnly)
                self.btn_page_3.setToolButtonStyle(Qt.ToolButtonIconOnly)
                self.btn_page_4.setToolButtonStyle(Qt.ToolButtonIconOnly)
                self.btn_page_5.setToolButtonStyle(Qt.ToolButtonIconOnly)

            # ANIMATION
            self.animation = QPropertyAnimation(self.frame_left_menu, b"minimumWidth")
            self.animation.setDuration(400)
            self.animation.setStartValue(width)
            self.animation.setEndValue(widthExtended)
            self.animation.setEasingCurve(QEasingCurve.InOutQuart)
            self.animation.start()

    def changePage(self,page):
        if page != 2 and Config.CURRENT_PAGE == 2: #unlock window size if user is coming from page 2 which need to fix the window size 
            self.setMinimumSize(1000,500)
            self.setMaximumSize(16777215,16777215)
        if page == 1:
            self.stackedWidget.setCurrentWidget(self.page_1)
            Config.CURRENT_PAGE = 1
        elif page == 2:
            self.stackedWidget.setCurrentWidget(self.page_2)
            Config.CURRENT_PAGE = 2
        elif page == 3:
            self.stackedWidget.setCurrentWidget(self.page_3)
            Config.CURRENT_PAGE = 3
        elif page == 4:
            self.stackedWidget.setCurrentWidget(self.page_4)
            Config.CURRENT_PAGE = 4
        elif page == 5:
            self.stackedWidget.setCurrentWidget(self.page_5)
            Config.CURRENT_PAGE = 5

    def Start_live(self):
        #start path element
        self.path_element.event_source.start()

        #start opengl element
        if self.OpenGL_worker.isRunning() == False:
            # setup reading worker
            if self.serial_worker.isRunning() == False:
                self.serial_worker = worker.SerialWorker(self.shared_data) # needs to be re defined
                # connect worker signals to functions
                #self.serial_worker.signals.status.connect(self.check_serialport_status)
                #self.serial_worker.signals.device_port.connect(self.connected_device)
                # execute the worker
                self.serial_worker.start()

            #create context 
            self.OpenGL_worker = OpenGLwidget.OpenGL_Renderer(self.OpenGL_element,self.shared_data)  #need to re-define it 
            #logging.info("------>   INITIAL CONTEXT: "+ str(self.OpenGL_element.context()))
            self.OpenGL_element.doneCurrent()
            #logging.info("####### start moveToThread #######")
            self.OpenGL_element.context().moveToThread(self.OpenGL_worker)
            #logging.info("####### end moveToThread #######")
            self.OpenGL_worker.start()
        
            #lock window resizing (due to bug with OpenGL, cannot return context immediately)
            self.setFixedSize(self.size().width(), self.size().height()) 
             
    

    def update_COM_ports(self):
        self.com_list_widget.clear()
        # acquire list of serial ports and add it to the combo box
        serial_ports = [
                p.name
                for p in serial.tools.list_ports.comports()
            ]
        
        self.com_list_widget.addItems(serial_ports)


    def resizeGLthread(resizing):
        Config.OpenGL_resize = resizing


    def calibrate(self):
        def updateBar(value):
            self.progressBar_magn.setValue(value)
        #send request to psoc
        Config.SERIAL_PORT.write(b'$Z\n')
        time.sleep(0.1)
        if (Config.SERIAL_PORT.read(3) == b"$A\n"):
            self.progressBar_magn.show()
            progress_process = ProgressBar_counter(self)
            progress_process.countChanged.connect(updateBar)
            progress_process.start()
            logging.info("Calibration started")
            self.label_magn_calib.setText("Calibration started, please move Psoc")
            self.label_magn_calib.setStyleSheet("color: rgb(255, 255, 0)")


    def Add_profile(self):
        #choose unique ID, the last one in the profile array is always the higher ID possible
        NewID=Config.Profiles[-1]["UsernameID"]+1
        #Add new profile to database
        Config.Profiles.append( {"UsernameID" : NewID, "Username" : self.Username.text(), "Gender" : "Male" if self.radioMale.isChecked() else "Female", "Height" : self.Height.value()} )
        self.Profile_combo.addItem(self.Username.text())

        #add profile to profiles file
        with open(Config.path + "/profiles/profiles.txt", "a") as txt_profiles:
            new_profile=f"\n{Config.Profiles[-1]['UsernameID']} {Config.Profiles[-1]['Username']} {Config.Profiles[-1]['Gender']} {Config.Profiles[-1]['Height']}"
            txt_profiles.write(new_profile)
            txt_profiles.close()
        
        #reset state and open modal 
        self.Username.setText("")
        self.Height.setValue(170)
        self.Modal.label_modal.setText("New profile added")
        self.Modal.exec()

    def change_profile(self): #send new selected profile to psoc + update current profile label
        
        #change GUI label displaying current profile  
        txt_tmp=""
        for key, value in Config.Profiles[self.Profile_combo.currentIndex()].items():
            txt_tmp += f"{key} : {value}\n"
        self.profile_label.setText(txt_tmp)

        #notify psoc 
        tmp_ID = Config.Profiles[self.Profile_combo.currentIndex()]["UsernameID"]
        tmp_height = Config.Profiles[self.Profile_combo.currentIndex()]["Height"]
        Config.SERIAL_PORT.write(b"$p"+ tmp_ID.to_bytes(1,'little') + tmp_height.to_bytes(1,'little') +b"\n") # package: $ p userID height \n 
        Config.CurrentProfile = tmp_ID

    def remove_profile(self): #remove selected profile from psoc
        #check if profile is selected or not
        if (Config.CurrentProfile == Config.Profiles[self.Profile_combo.currentIndex()]["UsernameID"]):
            self.Modal.label_modal.setText("Cannot remove current profile\nChange it first")
            self.Modal.exec()
        else:
            #remove from combobox list and list
            del Config.Profiles[self.Profile_combo.currentIndex()]
            self.Profile_combo.removeItem(self.Profile_combo.currentIndex())

            #rewrite profile file (easier rather edit it searching ID)
            with open(Config.path + "/profiles/profiles.txt", "w") as txt_profiles:
                new_file = ""
                for profile in Config.Profiles:
                    new_file=f"{new_file}{profile['UsernameID']} {profile['Username']} {profile['Gender']} {profile['Height']}\n"
                new_file=new_file[:-1] #delete last nl
                txt_profiles.write(new_file)
                txt_profiles.close()

            self.Modal.label_modal.setText("Profile removed")
            self.Modal.exec()


    def UpdateSessions(self):
        self.listSession.clear() #empty widget from previous data
        for ses in Config.Sessions_Data:
            itemN = QListWidgetItem()
            row = Sessions.SessionItem(ses["N_Session"],ses["UsernameID"],ses["Sdata"])
            itemN.setSizeHint(row.minimumSizeHint())
            self.listSession.addItem(itemN)
            self.listSession.setItemWidget(itemN,row)

    def loadProfiles(self):
        with open(Config.path + "/profiles/profiles.txt") as txt_profiles:
            profiles=txt_profiles.read().split("\n")
            for profile in profiles:
                attributes=profile.split(" ")
                Config.Profiles.append( {"UsernameID": int(attributes[0]), "Username" : attributes[1], "Gender" : attributes[2], "Height" : int(attributes[3])} )
                self.Profile_combo.addItem(attributes[1])

    def dev_btn_press(self):
        PygameOpenGL.IMU_VISUALIZER(self.shared_data).start()

    def dev_btn2_press(self):
        if self.serial_worker.isRunning() == False:
            self.serial_worker = worker.SerialWorker(self.shared_data) # needs to be re defined
            self.serial_worker.start()
        self.canvas.event_source.start()
        

class ProgressBar_counter(QThread):

    countChanged = pyqtSignal(int)
    def __init__(self, page):
        super().__init__()
        self.daemon = True
        self.page = page

    def run(self):
        for i in range(11):
            time.sleep(Config.CALIBRATION_TIME*0.1)
            # setting value to progress bar
            #self.page.progressBar_magn.setValue(i)
            self.countChanged.emit(i*10)
            
        self.page.label_magn_calib.setText("Calibration completed")
        self.page.label_magn_calib.setStyleSheet("color: rgb(85, 255, 0)")
        time.sleep(1)
        self.page.progressBar_magn.hide()


