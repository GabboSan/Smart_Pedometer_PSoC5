import random
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import pyplot as plt
from matplotlib import animation
import random

from numpy import true_divide
import Config

matplotlib.use("Qt5Agg")


class PathCanvas(FigureCanvas,animation.FuncAnimation):
    def __init__(self):

        #plt.style.use('dark_background') #dark bg
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
        self.x=[0]
        self.y=[0]
        self.wait = True #prevent autostart, a button press is required to start animation  

        FigureCanvas.__init__(self, self.fig)
        # call the animator.blit=True means only re-draw the parts that have changed
        animation.FuncAnimation.__init__(self, self.fig, self.animate, init_func=self.init, interval=2000) #if blit=True legend will fail


    # initialization function
    def init(self):
        self.line.set_data([], [])
        return self.line,

    # animation function.  This is called sequentially
    def animate(self,i):

        if Config.CURRENT_PAGE!=2 or self.wait:
            self.wait = False
            self.event_source.stop()

        if len(self.x)==10: #limit max number of elements (otherwise gui will crash)
            self.x = self.x[1:]
            self.y = self.y[1:]

        self.x.append(Config.newXY_path[0] + self.x[-1])
        self.y.append(Config.newXY_path[1] + self.y[-1])

        self.ax.set_xlim(min(self.x)-10, max(self.x)+10) 
        self.ax.set_ylim(min(self.y)-10, max(self.y)+10) 

        self.line.set_data(self.x, self.y)
        
        #set meters along x-y axis
        self.ax.set_xticks(self.x)
        self.ax.set_yticks(self.y)
        self.ax.set_xticklabels([str(i)+"cm" for i in self.x])
        self.ax.set_yticklabels([str(i)+"cm" for i in self.y])
        self.ax.tick_params(axis='x',labelrotation=45)

        #annotation (keep track of n. of points - bad performances)
        '''for i in range(0,len(self.x)):
            label = f"P{i}"
            self.ax.annotate(label,(self.x[i],self.y[i]),color="white", size=15)'''

        return self.line,

