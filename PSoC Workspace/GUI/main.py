from multiprocessing import Array
import sys
import logging

from PyQt5.QtCore import (
    Qt,
    QSize,  
    pyqtSlot,
    pyqtSignal
)
from PyQt5.QtGui import QIcon

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow
)

import Config

from SerialPsoc import connection
from IMU3D import OpenGLwidget

# GUI FILE
from PyQt5.uic import loadUi
from UI import functions, Sessions

import matplotlib
from plots import MPL, path
matplotlib.use('Qt5Agg')

from SerialPsoc import worker


# Logging config
logging.basicConfig(format="%(message)s", level=logging.INFO)

QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True) #enable highdpi scaling
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True) #use highdpi icons


#################
# SERIAL WINDOW #
#################
class SerialWindow(QMainWindow):
    advance_progress = pyqtSignal([int])
    def __init__(self):
        super().__init__()
        loadUi(Config.path+'UI/start.ui', self) # Load the .ui file
        self.connect_btn.clicked.connect(self.autoconnect)
        self.progressSerial.hide()
        self.advance_progress.connect(self.updateBar)
        
    def autoconnect(self):
        self.progressSerial.show()
        #try to autoconnect to first COM port 
        self.try_connect_thread = connection.Bluetooth_autoconnection(self,w)
        self.try_connect_thread.start()

        #il signal dal thred fa partire startmain
        self.try_connect_thread.closeStart.connect(self.startMain)

    def updateBar(self,i):
        self.port_found.setText("Downloading sessions")
        self.progressSerial.setValue(i)
          
    def startMain(self):
        w.show()
        self.close()


###############
# MAIN WINDOW #
###############
class MainWindow(QMainWindow):
    SessionDownloaded = pyqtSignal()
    def __init__(self):
        """!
        @brief Init MainWindow.
        """
        #a = QOpenGLWidget()
        #a.context().moveToThread()
        super(MainWindow, self).__init__()

        self.shared_data = Array('f',[0.0]*9)

        #INIT ELEMENTS
        self.initUI()
        self.serial_elements()
        self.plot_elements()
        self.live_elements()
        self.dev_elements()
        self.session_elements()
        self.profile_elements()



    #####################
    # GRAPHIC INTERFACE #
    #####################
    def initUI(self):
        """!
        @brief Set up the graphical interface structure.
        """
        loadUi(Config.path+'UI/main.ui', self) # Load the .ui file for main window

        # TOGGLE/BURGUER MENU
        self.Btn_Toggle.clicked.connect(lambda: functions.UIFunctions.toggleMenu(self, 180, True))

        # PAGE 1
        self.btn_page_1.clicked.connect(lambda: functions.UIFunctions.changePage(self,1))

        # PAGE 2
        self.btn_page_2.clicked.connect(lambda: functions.UIFunctions.changePage(self,2))

        # PAGE 3
        self.btn_page_3.clicked.connect(lambda: functions.UIFunctions.changePage(self,3))

        # PAGE 4
        self.btn_page_4.clicked.connect(lambda: functions.UIFunctions.changePage(self,4))

        # PAGE 5
        self.btn_page_5.clicked.connect(lambda: functions.UIFunctions.changePage(self,5))
        
        #set hamburgher menu icon 
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/Hamico.png", QSize(), QIcon.Normal, QIcon.Off)
        self.Btn_Toggle.setIcon(self.icon)
        self.Btn_Toggle.setIconSize(QSize(32, 32))

        #set other icons
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/serial.png", QSize(), QIcon.Normal, QIcon.Off)
        self.btn_page_1.setIcon(self.icon)
        self.btn_page_1.setIconSize(QSize(26, 26))
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/live.png", QSize(), QIcon.Normal, QIcon.Off)
        self.btn_page_2.setIcon(self.icon)
        self.btn_page_2.setIconSize(QSize(26, 26))
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/dev.png", QSize(), QIcon.Normal, QIcon.Off)
        self.btn_page_3.setIcon(self.icon)
        self.btn_page_3.setIconSize(QSize(26, 26))
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/sport.png", QSize(), QIcon.Normal, QIcon.Off)
        self.btn_page_4.setIcon(self.icon)
        self.btn_page_4.setIconSize(QSize(26, 26))
        self.icon = QIcon()
        self.icon.addFile(Config.path+"/resources/user.png", QSize(), QIcon.Normal, QIcon.Off)
        self.btn_page_5.setIcon(self.icon)
        self.btn_page_5.setIconSize(QSize(26, 26))

        #hide progress bar 
        self.progressBar_magn.hide()


    ####################
    # SETUP ELEMENTS   #
    ####################
    
    def serial_elements(self):
        """!
        @brief Scans all serial ports and create a list.
        """
        # create the combo box to host port list
        self.port_text = ""
        self.com_list_widget.currentTextChanged.connect(self.port_changed)
        self.conn_btn.toggled.connect(self.on_toggle)

        self.refreshBtn.clicked.connect(lambda: functions.UIFunctions.update_COM_ports(self))

        functions.UIFunctions.update_COM_ports(self)

        self.calib_button.clicked.connect(lambda: functions.UIFunctions.calibrate(self))
        self.change_profile_btn.clicked.connect(lambda: functions.UIFunctions.change_profile(self))
        self.remove_profile_btn.clicked.connect(lambda: functions.UIFunctions.remove_profile(self))
        
        functions.UIFunctions.loadProfiles(self)
        self.serial_worker = worker.SerialWorker(self.shared_data)

    def live_elements(self):
        #Canvas path widget  
        self.path_element = path.PathCanvas()
        self.pathlayout.addWidget(self.path_element)
        #OpenGL widget
        self.OpenGL_element = OpenGLwidget.OpenGL_Widget()
        self.OpenGL_element.paintEvent = lambda *args : None  #absolutely necessary to release context to another thread
        self.OpenGL_layout.addWidget(self.OpenGL_element)
        self.startLiveBtn.clicked.connect(lambda: functions.UIFunctions.Start_live(self))

        #renderer for OpenGL widget
        self.OpenGL_worker = OpenGLwidget.OpenGL_Renderer(self.OpenGL_element,self.shared_data)

        #signal to stop renderer thread if OpenGL_element is about to resize (context sharing not possible) 
        self.OpenGL_element.aboutToResize.connect(lambda: functions.UIFunctions.resizeGLthread(True))
        self.OpenGL_element.resized.connect(lambda: functions.UIFunctions.resizeGLthread(False))

        #checkbox for 4k scaling
        self.LargeScreenCheck.stateChanged.connect(self.scale4k)

    def profile_elements(self):
        self.Modal = loadUi(Config.path+'UI/modal.ui') # Load the .ui file for modal
        self.add_usr_btn.clicked.connect(lambda: functions.UIFunctions.Add_profile(self))
        

    def dev_elements(self):        
        self.dev_btn.clicked.connect(lambda: functions.UIFunctions.dev_btn_press(self))
        self.dev_btn2.clicked.connect(lambda: functions.UIFunctions.dev_btn2_press(self))
        

    def plot_elements(self):
        self.canvas = MPL.MplCanvas(self, width=5, height=8, dpi=50)
        self.plot_layout.addWidget(self.canvas)

    def session_elements(self):
        self.listSession.clicked.connect(lambda: self.path_session_element.updatePath(self.listSession.currentRow()))
        self.path_session_element = Sessions.SessionPathCanvas()
        self.pathSession.addWidget(self.path_session_element)

        self.SessionDownloaded.connect(lambda: functions.UIFunctions.UpdateSessions(self))


    def scale4k(self,state):
        if state:
            Config.scaling=3
        else:
            Config.scaling=1
        


    ##################
    # SERIAL SIGNALS #
    ##################
    def port_changed(self):
        """!
        @brief Update conn_btn label based on selected port.
        """
        self.port_text = self.com_list_widget.currentText()
        self.conn_btn.setText("Connect to port {}".format(self.port_text))

    @pyqtSlot(bool)
    def on_toggle(self, checked):
        """!
        @brief Allow connection and disconnection from selected serial port.
        """
        if s.isVisible()==False:    #check if command comes from user and not automatic checking
            if checked:    
                Config.PORTNAME = self.port_text  
                connection.CheckIfPsoc(self,0)
                
            else:
                # kill thread
                Config.CONN_STATUS = False
                
                self.com_list_widget.setDisabled(False) # enable the possibility to change port
                self.conn_btn.setText(
                    "Connect to port {}".format(self.port_text)
                )
                # change connection status label
                self.conn_status_label.setText("Disconnected")
                self.conn_status_label.setStyleSheet("color: rgb(170, 0, 0)") 

                #disable user action 
                self.calib_button.setEnabled(False)
                self.change_profile_btn.setEnabled(False)
                self.remove_profile_btn.setEnabled(False)
                self.startLiveBtn.setEnabled(False)
                self.dev_btn.setEnabled(False)
                self.dev_btn2.setEnabled(False)
                self.add_usr_btn.setEnabled(False)
                
                #update com list
                functions.UIFunctions.update_COM_ports(self)
        
        
        
    def check_serialport_status(self, port_name, status):
        """!
        @brief Handle the status of the serial port connection.

        Available status:
            - 0  --> Error during opening of serial port
            - 1  --> Serial port opened correctly
        """
        if status == 0:
            self.conn_btn.setChecked(False)
            
        elif status == 1:
            # enable all the widgets on the interface
            #self.dev_btn.setDisabled(False)
            self.com_list_widget.setDisabled(True) # disable the possibility to change COM port when already connected
            self.conn_btn.setText(
                "Disconnect from port {}".format(port_name)
            )

    def connected_device(self, port_name):
        """!
        @brief Checks on the termination of the serial worker.
        """
        logging.info("Port {} closed.".format(port_name))


    def ExitHandler(self):
        """!
        @brief Kill every possible running thread upon exiting application.
        """
        if Config.SERIAL_PORT.isOpen():
            Config.SERIAL_PORT.write(b'$c\n')
        Config.CONN_STATUS = False



#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    w = MainWindow()
    w.setWindowTitle("Contapassi")
    s = SerialWindow()
    s.setWindowTitle("Automatic connection")
    app.aboutToQuit.connect(w.ExitHandler)
    s.setWindowIcon(QIcon(Config.path+"/resources/foot.png"))
    w.setWindowIcon(QIcon(Config.path+"/resources/foot.png"))
    s.show()

    sys.exit(app.exec_())
