#!/usr/bin/env python

import roslib
roslib.load_manifest('renderer')
import rospy
import math
import numpy as np
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
import cv2 
import cv
from cv_bridge import CvBridge
import threading


import os, pyglet, sys
from pyglet.gl import *

class World(pyglet.window.Window):
    def __init__(self, viewer_pos=(-12., -12., 10.), *args, **kwargs):
        super(World, self).__init__(*args, **kwargs)
        self.viewer_pos = viewer_pos
        glClearColor(1.0, 1.0, 1.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        self.clock = 0
        pyglet.clock.schedule_interval(self.update, 1 / 60.0)
        pyglet.clock.set_fps_limit(60)
        self.dragging = False
        self.angle = (0., 0.)
        self.mouse_pos = (0,0)        
        self.keys = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.keys)
        
        self.gridded_map = None
        
        self.connect()
        
        self.cross_thread_trigger = False
        
    def connect(self):
        rospy.init_node('Interactive_Map')
        rospy.Subscriber('/floor_gridder/gridded_map',Int16MultiArray, self.on_got_gridded_map)
        self.manual_target_pub = rospy.Publisher('/floor_gridder/manual_target', Float32MultiArray)
        rospy_thread = threading.Thread(target = self.spin)
        rospy_thread.start()
        
    def spin(self):
        rospy.spin()
    
    def on_mouse_press(self, x, y, button, modifiers):
        if button == pyglet.window.mouse.LEFT:
            manual_target = self.mouse_to_world(x, y)
            print "Manual target of ", manual_target
            float32MultiArray = Float32MultiArray()
            float32MultiArray.data.append(manual_target[0])
            float32MultiArray.data.append(manual_target[1])
            self.manual_target_pub.publish(float32MultiArray)
            
    
    def on_mouse_motion(self, x, y, dx, dy):
        self.mouse_pos = (x,y)
    
    def mouse_to_world(self, x,y):
        return (float(x*self.projectionWidth)/self.width+self.viewer_pos[0], -(float((self.height-y)*self.projectionHeight)/self.height+self.viewer_pos[1]))

    def on_got_gridded_map(self, gridded_map):
        print "Received Map"        
        self.gridded_map_buffer = np.reshape(np.array(gridded_map.data, dtype=np.uint8), (gridded_map.layout.dim[0].stride, -1))
        self.gridded_map_buffer = np.dstack((self.gridded_map_buffer,self.gridded_map_buffer,self.gridded_map_buffer))
        self.cross_thread_trigger = True
        
        
    @staticmethod
    def load_nparray(npimage):
        # load_nparray MUST be called in the same thread as pyglet is running
        # gl###### texture loading commands will NOT load textures otherwise
        # There is no visible error in this case, but textures are blank white
        
        #cv2.imshow('windowwww', npimage)
        #cv2.waitKey(0)
        
        image = pyglet.image.ImageData(npimage.shape[1], npimage.shape[0], 'RGB', npimage.tostring())        
        
        texture = image.get_texture()
        
        glEnable(texture.target)
        glBindTexture(texture.target, texture.id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
                     0, GL_RGB, GL_UNSIGNED_BYTE,
                     image.get_image_data().get_data('RGB',
                                                     image.width * -3))
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST )
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
                                                     
        return texture
 
    def update(self, _):
        if self.cross_thread_trigger:
            self.gridded_map = Map(self.load_nparray(self.gridded_map_buffer),
                                  -self.gridded_map_buffer.shape[1]/20,
                                  -self.gridded_map_buffer.shape[0]/20,
                                  +self.gridded_map_buffer.shape[1]/20,
                                  +self.gridded_map_buffer.shape[0]/20)
            self.cross_thread_trigger = False
            
        
        if self.keys[pyglet.window.key.W]:
            self.viewer_pos = self.viewer_pos[0], self.viewer_pos[1]+.1, self.viewer_pos[2]
        if self.keys[pyglet.window.key.S]:
            self.viewer_pos = self.viewer_pos[0], self.viewer_pos[1]-.1, self.viewer_pos[2]
        if self.keys[pyglet.window.key.A]:
            self.viewer_pos = self.viewer_pos[0]-.1, self.viewer_pos[1], self.viewer_pos[2]
        if self.keys[pyglet.window.key.D]:
            self.viewer_pos = self.viewer_pos[0]+.1, self.viewer_pos[1], self.viewer_pos[2]

        
        self.on_draw()
        self.clock += .01
 
    
    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)        
        glDisable(GL_TEXTURE_2D)
        
        glLoadIdentity()
        glPushMatrix()
        
        glRotatef(self.angle[1], 1., 0., 0.)
        glRotatef(self.angle[0], 0., 1., 0.)
        
        glTranslatef(-self.viewer_pos[0], -self.viewer_pos[1], -self.viewer_pos[2])
        self.draw_grid()
        
        self.draw_map(self.gridded_map)
        
        self.draw_cursor()
        
        
        glPopMatrix()
        
    def draw_map(self, mapp):
        if mapp == None:
            return
            
            
        glEnable( GL_TEXTURE_2D )
        
        glBindTexture(mapp.texture.target, mapp.texture.id)
        
        glBegin(GL_POLYGON)        
        glTexCoord2f(0., 1.); glVertex3f( mapp.c1[0], mapp.c1[1], 0.)
        glTexCoord2f(0., 0.); glVertex3f( mapp.c2[0], mapp.c2[1], 0.)
        glTexCoord2f(1., 0.); glVertex3f( mapp.c3[0], mapp.c3[1], 0.)
        glTexCoord2f(1., 1.); glVertex3f( mapp.c4[0], mapp.c4[1], 0.)
        glEnd()
        
        glDisable(GL_TEXTURE_2D)
    
    def draw_grid(self):
        glPushAttrib(GL_CURRENT_BIT)
        glColor3f(.9, .9, .9)
        i = -10
        glBegin(GL_LINES)          
        while(i<11):
            glVertex3f(i, -10, 0)
            glVertex3f(i, +10, 0)
            i = i + 1
        glEnd()
        
        j = -10
        glBegin(GL_LINES)           
        while(j<11):
            glVertex3f(-10, j, 0)
            glVertex3f(+10, j, 0)
            j = j + 1
        glEnd()
        glPopAttrib(GL_CURRENT_BIT)
        
    def draw_cursor(self):
        glPushAttrib(GL_CURRENT_BIT)
        glColor3f(1., 0., 0.)
        glBegin(GL_LINES)
        glVertex3f(float(self.mouse_pos[0]-8)*self.projectionWidth/self.width+self.viewer_pos[0], float(self.mouse_pos[1])*self.projectionHeight/self.height+self.viewer_pos[1], 1)
        glVertex3f(float(self.mouse_pos[0]+8)*self.projectionWidth/self.width+self.viewer_pos[0], float(self.mouse_pos[1])*self.projectionHeight/self.height+self.viewer_pos[1], 1)
        glVertex3f(float(self.mouse_pos[0])*self.projectionWidth/self.width+self.viewer_pos[0], float(self.mouse_pos[1]-8)*self.projectionHeight/self.height+self.viewer_pos[1], 1)
        glVertex3f(float(self.mouse_pos[0])*self.projectionWidth/self.width+self.viewer_pos[0], float(self.mouse_pos[1]+8)*self.projectionHeight/self.height+self.viewer_pos[1], 1)
        glEnd()
        glPopAttrib(GL_CURRENT_BIT)
            
 
    def on_resize(self, width, height):
        glViewport(0, 0, width, height) 

        glMatrixMode(gl.GL_PROJECTION) 

        glLoadIdentity()  


        self.projectionWidth = 24 

        self.projectionHeight = 24

        glOrtho(0, self.projectionWidth, 0, self.projectionHeight, -1, 1000.) 

        glMatrixMode(gl.GL_MODELVIEW)

        
class Map():
    def __init__(self, texture,x1, y1, x2, y2):
        corners = []
        self.c1 = (x1, y1)
        self.c2 = (x1, y2)
        self.c3 = (x2, y2)
        self.c4 = (x2, y1)
        self.texture = texture
        


if __name__ == '__main__':
    world = World(width=800, height=600) # Initialise class to preserve vars
    pyglet.app.run()
