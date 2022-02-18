import logging
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from IMU3D.figure import *
import time
import multiprocessing

class IMU_VISUALIZER(multiprocessing.Process):
    
    def __init__(self, data):
        super().__init__()
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.data = data

    def run(self):
        self.InitPygame()
        self.InitGL()
        try:
            while True:

                event = pygame.event.poll()
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()
                    break 
                
                self.Roll = self.data[6]
                self.Pitch = self.data[7]
                self.Yaw = self.data[8]
                self.DrawGL()
                
                pygame.time.wait(10)
            
        except:
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            self.DrawText("Sorry, something is wrong :c")
            pygame.display.flip()
            time.sleep(5)


    def InitPygame(self):
        global display
        pygame.init()
        display = (800,600)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('IMU visualizer   (Press Esc to exit)')


    def InitGL(self):
        glClearColor((1.0/255*46),(1.0/255*45),(1.0/255*64),1)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

        gluPerspective(100, (display[0]/display[1]), 0.1, 50.0)
        glTranslatef(0.0,0.0, -5)


    def DrawText(self,textString):     
        font = pygame.font.SysFont ("Courier New",25, True)
        textSurface = font.render(textString, True, (255,255,0), (46,45,64,255))     
        textData = pygame.image.tostring(textSurface, "RGBA", True)         
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)    

    def DrawBoard(self):
        
        glBegin(GL_QUADS)
        x = 0
        
        for surface in surfaces:
            
            for vertex in surface:  
                glColor3fv((colors[x]))          
                glVertex3fv(vertices[vertex])
            x += 1
        glEnd()

    def DrawGL(self):

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glLoadIdentity() 
        gluPerspective(90, (display[0]/display[1]), 0.1, 50.0)
        glTranslatef(0.0,0.0, -5)   

        glRotatef(round(self.Pitch,1), 0, 0, 1)    #glRotatef(	GLfloat angle,GLfloat x,GLfloat y,GLfloat z);
        glRotatef(round(self.Roll,1), 1, 0, 0)
        glRotatef(round(self.Yaw,1), 0, 1, 0)

        self.DrawText("Roll: {}°        Pitch: {}°        Yaw: {}°".format(round(self.Roll,1),round(self.Pitch,1),round(self.Yaw,1)))
        self.DrawBoard()
        pygame.display.flip()


    def compute_axis(self):
        if self.mode == 0:
            pass
        """
            #self.Roll,self.Pitch,self.Yaw = self.No_filtering()
        elif self.mode == 1:
            #self.Roll,self.Pitch,self.Yaw = self.IMU_standard()
        elif self.mode == 2:
            #self.Roll,self.Pitch,self.Yaw = self.complementary_filtering(self.IMU_standard())
        elif self.mode == 3:
            #self.Roll,self.Pitch,self.Yaw = self.only_gyro()
"""

        #########################
        #######  METHODS  #######
        #########################
"""
    def No_filtering(self):
        Roll = self.data[0]/16384.0*90 #rotation around y
        Pitch = self.data[1]/16384.0*90  #rotation around x 
        Yaw = self.data[2]/16384.0*90  #rotation around z
        return Roll,Pitch,Yaw

    def IMU_standard(self):
        Roll = numpy.arctan(self.data[0]/(numpy.square(self.data[1]**2+self.data[2]**2)))*45    #rotation around y
        Pitch = numpy.arctan(self.data[1]/(numpy.square(self.data[0]**2+self.data[2]**2)))*45    #rotation around x 
        Yaw = numpy.arctan(numpy.square(self.data[0]**2+self.data[1]**2)/self.data[2])*45    #rotation around z
        return [Roll,Pitch,Yaw]


    def complementary_filtering(self,Acc):
        tau = 1
        delta_t = 0.04

        #computing gyroscope angles
        Gyro_angle_X = self.Pitch + self.data[3] * delta_t 
        Gyro_angle_Y = self.Roll + self.data[4] * delta_t
        Gyro_angle_Z = self.Yaw + self.data[5] * delta_t

        alpha = tau/(tau + delta_t) # Δt = sampling rate, τ = time constant greater than timescale of typical accelerometer noise 
        Roll = alpha * Gyro_angle_Y + (1-alpha)*Acc[0]  # filtered rotation around y
        Pitch = alpha * Gyro_angle_X + (1-alpha)*Acc[1]  # filtered rotation around x 
        Yaw = alpha * Gyro_angle_Z + (1-alpha)*Acc[2] # filtered rotation around z

        return Roll,Pitch,Yaw

    def only_gyro(self):
        return -self.data[4],self.data[3],self.data[5]

"""