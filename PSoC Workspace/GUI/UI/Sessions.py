from PyQt5.QtWidgets import QGridLayout, QLabel, QWidget
from PyQt5.QtGui import QFont
from matplotlib import pyplot as plt
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import Config

class SessionItem(QWidget):
    def __init__(self, SessionID,UsernameID,Sdata, parent=None):   
        super(SessionItem, self).__init__(parent)

        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        font1 = QFont()
        font1.setPointSize(12)
        
        self.SessionID = QLabel()
        self.SessionID.setText(str(SessionID))
        self.SessionID.setMaximumWidth(40)
        self.Username = QLabel()
        self.Username.setText("Profile removed")
        for profile in Config.Profiles:
            if profile['UsernameID']==UsernameID:
                self.Username.setText(profile['Username'])
                break
            
        self.sessionData = QLabel()
        self.sessionData.setText(f"Activity time : {Sdata[0]} s   Cadence : {Sdata[1]} steps/s\nTotal steps : {Sdata[2]}   Total distance : {Sdata[3]} cm")
        self.gridLayout = QGridLayout()
        self.SessionID.setStyleSheet(u"color:white")
        self.Username.setStyleSheet(u"color:white")
        self.sessionData.setStyleSheet(u"color:white")
        self.SessionID.setFont(font)
        self.Username.setFont(font1) 
        self.gridLayout.addWidget(self.SessionID, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.Username, 0, 1, 1, 1)
        self.gridLayout.addWidget(self.sessionData, 1, 1, 1, 1)

        self.setLayout(self.gridLayout)


class SessionPathCanvas(FigureCanvas):

    def __init__(self):

        matplotlib.rcParams['toolbar'] = 'None' #remove toolbar
        self.fig = plt.figure(figsize=(5,5), facecolor="#2d2d2d")
        self.ax = plt.axes()

        #change color of axis and background
        self.ax.set_facecolor("#2d2d2d")
        self.ax.spines['left'].set_color('lightgrey')
        self.ax.spines['bottom'].set_color('lightgrey')
        self.ax.spines['top'].set_color('lightgrey')
        self.ax.spines['right'].set_color('lightgrey')
        self.ax.tick_params(axis='x', colors='lightgrey')
        self.ax.tick_params(axis='y', colors='lightgrey')
        
        self.line, = self.ax.plot([], [], '-o', color='lightgrey', lw=1)

        FigureCanvas.__init__(self, self.fig)

    def updatePath(self,N_session):

        self.x = Config.Sessions_Data[N_session]["Path"][0]
        self.y = Config.Sessions_Data[N_session]["Path"][1]
        self.ax.set_xlim(min(self.x)-1, max(self.x)+1) 
        self.ax.set_ylim(min(self.y)-1, max(self.y)+1) 

        self.line.set_data(self.x,self.y) 

        plt.xticks(ticks=self.x,labels=[str(i)+"cm" for i in self.x],rotation=45)    #set meters along x axis
        plt.yticks(ticks=self.y,labels=[str(i)+"cm" for i in self.y])    #set meters along y axis
        
        #add label to points (slow down + glitches)
        '''for i in range(0,len(self.x)):
            label = f"P{i}"
            self.ax.annotate(label,(self.x[i],self.y[i]),color="white", size=15)'''

        self.draw()

