from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import animation
import Config


class MplCanvas(FigureCanvas,animation.FuncAnimation):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.gs = self.fig.add_gridspec(3, hspace=0)
        self.axs = self.gs.subplots()
        
        # Hide x labels and tick labels for all but bottom plot.
        for ax in self.axs:
            ax.label_outer()

        FigureCanvas.__init__(self, self.fig)

        # call the animator
        animation.FuncAnimation.__init__(self, self.fig, self.animate, init_func=self.init, interval=100) 

        
    def init(self):
        #init axis
        self.line1, = self.axs[0].plot(Config.x_Data_plot,Config.y_Data_plot.RPY[0], label="ROLL")
        self.line2, = self.axs[0].plot(Config.x_Data_plot,Config.y_Data_plot.RPY[1], label="PITCH")
        self.line3, = self.axs[0].plot(Config.x_Data_plot,Config.y_Data_plot.RPY[2], label="YAW")
        self.axs[0].legend(loc='upper left')

        self.line4, = self.axs[1].plot(Config.x_Data_plot,Config.y_Data_plot.acc[0], label="ACC X")
        self.line5, = self.axs[1].plot(Config.x_Data_plot,Config.y_Data_plot.acc[1], label="ACC Y")
        self.line6, = self.axs[1].plot(Config.x_Data_plot,Config.y_Data_plot.acc[2], label="ACC Z")
        self.axs[1].legend(loc='upper left')

        self.line7, = self.axs[2].plot(Config.x_Data_plot,Config.y_Data_plot.gyro[0], label="GYRO X")
        self.line8, = self.axs[2].plot(Config.x_Data_plot,Config.y_Data_plot.gyro[1], label="GYRO Y")
        self.line9, = self.axs[2].plot(Config.x_Data_plot,Config.y_Data_plot.gyro[2], label="GYRO Z")
        self.axs[2].legend(loc='upper left')


        return self.line1,self.line2,self.line3,self.line4,self.line5,self.line6,self.line7,self.line8,self.line9

    def animate(self,i):

        if Config.CURRENT_PAGE!=3:
            self.event_source.stop()

        elif Config.NEW_DATA == True: 
            self.line1.set_data(Config.x_Data_plot, Config.y_Data_plot.RPY[0])
            self.line2.set_data(Config.x_Data_plot, Config.y_Data_plot.RPY[1])
            self.line3.set_data(Config.x_Data_plot, Config.y_Data_plot.RPY[2])

            self.line4.set_data(Config.x_Data_plot, Config.y_Data_plot.acc[0])
            self.line5.set_data(Config.x_Data_plot, Config.y_Data_plot.acc[1])
            self.line6.set_data(Config.x_Data_plot, Config.y_Data_plot.acc[2])

            self.line7.set_data(Config.x_Data_plot, Config.y_Data_plot.gyro[0])
            self.line8.set_data(Config.x_Data_plot, Config.y_Data_plot.gyro[1])
            self.line9.set_data(Config.x_Data_plot, Config.y_Data_plot.gyro[2])

            yDataRPY = [Config.y_Data_plot.RPY[0],Config.y_Data_plot.RPY[1],Config.y_Data_plot.RPY[2]]
            yDataAcc = [Config.y_Data_plot.acc[0],Config.y_Data_plot.acc[1],Config.y_Data_plot.acc[2]]
            yDataGyro = [Config.y_Data_plot.gyro[0],Config.y_Data_plot.gyro[1],Config.y_Data_plot.gyro[2]]

            self.axs[0].set_ylim(min(min(x) for x in yDataRPY)-3, max(max(x) for x in yDataRPY)+3)  #-3/+3 to better see plot
            self.axs[1].set_ylim(min(min(x) for x in yDataAcc)-3, max(max(x) for x in yDataAcc)+3)  #-3/+3 to better see plot
            self.axs[2].set_ylim(min(min(x) for x in yDataGyro)-3, max(max(x) for x in yDataGyro)+3)  #-3/+3 to better see plot

            Config.NEW_DATA = False
            return self.line1,self.line2,self.line3,self.line4,self.line5,self.line6,self.line7,self.line8,self.line9


''' def update_plot(self,canvas):

    if Config.NEW_DATA == True:
        # Note: we no longer need to clear the axis.
        if self._plot_ref is None:
            # First time we have no plot reference, so do a normal plot.
            # .plot returns a list of line <reference>s, as we're
            # only getting one we can take the first element.
            plot_refs_acc = canvas.axs[0].plot(Config.x_Data_plot, Config.y_Data_plot.acc, 'r')
            self._plot_ref = plot_refs_acc[0]
        else:
            # We have a reference, we can use it to update the data for that line.
            self._plot_ref.set_ydata(self.ydata)

        # Trigger the canvas to update and redraw.
        self.canvas.draw()
        Config.NEW_DATA = False'''