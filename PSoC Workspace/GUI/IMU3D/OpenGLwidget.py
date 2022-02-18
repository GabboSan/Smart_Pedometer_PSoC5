import logging
from OpenGL.GL import *
from OpenGL.GLU import *
from IMU3D.figure import *
import time
import Config 
from PyQt5 import QtCore
from PyQt5.QtWidgets import QOpenGLWidget, QApplication



class OpenGL_Widget(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        
    def initializeGL(self):
        glClearColor((1.0/255*45),(1.0/255*45),(1.0/255*45),1)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

        gluPerspective(100, 1.33, 0.1, 50.0)
        glTranslatef(0.0,0.0, -5)
        logging.info("### ending init OPENGL ###")


class OpenGL_Renderer(QtCore.QThread):
    
    def __init__(self, widget, data):
        super().__init__()
        self.daemon = True
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.data = data
        self.widget = widget


    def run(self):
        
        logging.info("thread OpenGL partito")
        try:
            while Config.CURRENT_PAGE == 2 and Config.OpenGL_resize == False:

                self.widget.makeCurrent()
                self.Roll = self.data[6]
                self.Pitch = 0#self.data[7] 
                self.Yaw = 0#self.data[8]
                
                self.DrawGL()
                self.widget.doneCurrent()
                self.widget.update()
                time.sleep(0.1)

        except:
            logging.info("Sorry, something is wrong")
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            time.sleep(5)

        self.widget.context().moveToThread(QApplication.instance().thread()) #return context to the main thread
        logging.info("thread OpenGL concluso")
        


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
        glViewport(0,0,self.widget.frameGeometry().width()*Config.scaling, self.widget.frameGeometry().height()*Config.scaling)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glLoadIdentity() 
        gluPerspective(90, 1.33, 0.1, 50.0)
        glTranslatef(0.0,0.0, -5)   

        glRotatef(round(self.Pitch,1), 0, 0, 1)    #glRotatef(	GLfloat angle,GLfloat x,GLfloat y,GLfloat z);
        glRotatef(round(self.Roll,1), 1, 0, 0)
        glRotatef(round(self.Yaw,1), 0, 1, 0)

        self.DrawBoard()

