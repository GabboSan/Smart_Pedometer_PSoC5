from PyQt5.QtCore import (
    QThread, 
    pyqtSlot
)
import time
import logging
import Config


class SerialWorker(QThread):
    """!
    @brief Main class for serial communication: handles connection with device.
    """
    def __init__(self, data):
        """!
        @brief Init worker.
        """
        super().__init__()
        # init port, params and signals
        #self.port = serial.Serial()
        #self.baudrate = 9600 # hard coded but can be a global variable, or an input param
        #self.signals = SerialWorkerSignals()
        self.data = data

    @pyqtSlot()
    def run(self):
        if not Config.CONN_STATUS and Config.SERIAL_PORT.isOpen():
            Config.CONN_STATUS = True
            time.sleep(0.01)
            
            #send read request to psoc
            Config.SERIAL_PORT.write(b'$R\n')

            while Config.CONN_STATUS:
                self.serial_input = Config.SERIAL_PORT.readline()
                if(len(self.serial_input) == 26 and self.serial_input[0] == 0x24 ):     #Roll Pitch Yaw data 
                    self.extract_Acc()
                    self.extract_Gyro()
                    self.extract_RPY()
                    self.update_plot_data()
                    #logging.info(self.data[:])
                elif(len(self.serial_input) == 15 and self.serial_input[0] == 0x23):    #Path data
                    tmp = self.serial_input[1:-1].decode('utf-8').replace("\x00","").split(" ")
                    Config.newXY_path[0]=int(tmp[0])
                    Config.newXY_path[1]=int(tmp[1])


            Config.SERIAL_PORT.write(b"$r\n") #tell psoc to stop sending data (if in sending mode) 
            #Config.SERIAL_PORT.close()
            logging.info("Killing serial reading thread")


        else:
            logging.info("Error with port {}.".format(Config.PORTNAME))
            time.sleep(0.01)


    def extract_Acc(self):
        self.data[0] = int.from_bytes([self.serial_input[4], self.serial_input[3]],byteorder = 'little',signed=True)
        self.data[1] = int.from_bytes([self.serial_input[2], self.serial_input[1]],byteorder = 'little',signed=True)
        self.data[2] = int.from_bytes([self.serial_input[6], self.serial_input[5]],byteorder = 'little',signed=True)

    def extract_Gyro(self):
        self.data[3] = int.from_bytes([self.serial_input[10], self.serial_input[9]],byteorder = 'little',signed=True)
        self.data[4] = int.from_bytes([self.serial_input[8], self.serial_input[7]],byteorder = 'little',signed=True)
        self.data[5] = int.from_bytes([self.serial_input[12], self.serial_input[11]],byteorder = 'little',signed=True)

    def extract_RPY(self):
        self.data[6] = int.from_bytes([self.serial_input[20], self.serial_input[19]],byteorder = 'little',signed=True)
        self.data[7] = int.from_bytes([self.serial_input[22], self.serial_input[21]],byteorder = 'little',signed=True)
        self.data[8] = int.from_bytes([self.serial_input[24], self.serial_input[23]],byteorder = 'little',signed=True)


    def update_plot_data(self):
        # Drop off the first y element, append a new one.
        Config.y_Data_plot.acc[0] = Config.y_Data_plot.acc[0][1:] + [self.data[0]]
        Config.y_Data_plot.acc[1] = Config.y_Data_plot.acc[1][1:] + [self.data[1]]
        Config.y_Data_plot.acc[2] = Config.y_Data_plot.acc[2][1:] + [self.data[2]]

        Config.y_Data_plot.gyro[0] = Config.y_Data_plot.gyro[0][1:] + [self.data[3]]
        Config.y_Data_plot.gyro[1] = Config.y_Data_plot.gyro[1][1:] + [self.data[4]]
        Config.y_Data_plot.gyro[2] = Config.y_Data_plot.gyro[2][1:] + [self.data[5]]

        Config.y_Data_plot.RPY[0] = Config.y_Data_plot.RPY[0][1:] + [self.data[6]]
        Config.y_Data_plot.RPY[1] = Config.y_Data_plot.RPY[1][1:] + [self.data[7]]
        Config.y_Data_plot.RPY[2] = Config.y_Data_plot.RPY[2][1:] + [self.data[8]]

        Config.NEW_DATA = True