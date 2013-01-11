#!/usr/bin/env python  
import roslib
roslib.load_manifest('renderer')
import rospy
import ardrone_autonomy.msg
import tf
import math
import numpy as np
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2 
import cv
from cv_bridge import CvBridge
import threading


import os, pyglet, sys
from pyglet.gl import *

class World(pyglet.window.Window):
    def __init__(self, viewer_pos=(1., 1.125, 0.), *args, **kwargs):
        super(World, self).__init__(*args, **kwargs)
        self.viewer_pos = viewer_pos
        glClearColor(1.0, 1.0, 1.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        self.textures = self.load_textures()
        self.walls = self.load_walls()
        self.clock = 0
        pyglet.clock.schedule_interval(self.update, 1 / 60.0)
        self.dragging = False
        self.angle = (0., 0.)
        self.mouse_pos = (0,0)        
        self.keys = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.keys)
        self.connect()
        self.live1 = False
        self.live2 = False
        
    def connect(self):
        rospy.init_node('Renderer')
        rospy.Subscriber('/template_track/img',Image,self.texture_process)
        rospy.Subscriber('/template_track/corners',Float32MultiArray, self.corner_process)
        rospy.Subscriber("/template_track/wall_trigger", Empty, self.trigger_process)
        rospy_thread = threading.Thread(target = self.spin)
        rospy_thread.start()
        
    def spin(self):
        rospy.spin()
    
    def on_mouse_press(self, x, y, button, modifiers):
        if button == pyglet.window.mouse.LEFT:
            self.mouse_pos = (x, y)
    
    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        pyglet.window.mouse.x = self.mouse_pos[0]
        pyglet.window.mouse.y = self.mouse_pos[1]
        if buttons & pyglet.window.mouse.LEFT:
            self.angle = (self.angle[0] + dx/3., self.angle[1] - dy/3)
            
    def texture_process(self, d):
        """Converts the ROS published image to a cv2 numpy array
        and passes to FeatureTracker"""
        # ROS to cv image
        bridge = CvBridge()
        cvimg = bridge.imgmsg_to_cv(d,"bgr8")
        self.texture_buffer = pyglet.image.ImageData(640,360, "BGR", cvimg.tostring()).get_texture()
        # cv to cv2 numpy array image
        #self.texture_buffer = np.asarray(cvimg)
        self.live1 = True
        
    def corner_process(self, d):
        self.corner_buffer = d.data
        self.live2 = True
        
    def trigger_process(self, d):
        while((not self.live1) or (not self.live2)):
            rospy.sleep(0.01)
            
        c = self.corner_buffer
        c1 = np.array([c[0], c[1], c[2]])
        c2 = np.array([c[3], c[4], c[5]])
        c3 = np.array([c[6], c[7], c[8]])
        c4 = np.array([c[9], c[10], c[11]])
        #img = self.np_to_abstractimage(self.texture_buffer).get_texture()
        
        #self.raw_masked_image = cv.CreateImage((640,480), self.raw_video_image.depth, self.raw_video_image.nChannels)
        #cv.Not(self.raw_depth_image,self.raw_depth_image)
        #cv.CvtColor(self.raw_depth_image,contour_depth_image_bgr, cv.CV_GRAY2BGR)

        #cv.And(self.raw_video_image,contour_depth_image_bgr,self.raw_masked_image)
        wall = Wall(self.texture_buffer, c1, c2, c3, c4)
        self.walls.append(wall)
        
        self.live1 = False
        self.live2 = False
        
        
        
        
    def np_to_abstractimage(self, img):
        # the size of our texture
        dimensions = (640, 360)

        # we need RGBA textures
        # which has 4 channels
        format_size = 1
        bytes_per_channel = 1
        
        '''
        # populate our array with some random data
        data = np.random.random_integers(
            low = 0,
            high = 1,
            size = (dimensions[ 0 ] * dimensions[ 1 ], format_size)
            )

        # convert any 1's to 255
        data *= 255
        
        #data[:, 0] = img

        # set the GB channels (from RGBA) to 0
        data[ :, 1:-1 ] = 0

        # ensure alpha is always 255
        data[ :, 3 ] = 255
        

        # we need to flatten the array
        data.shape = -1
         

        # convert to GLubytes
        tex_data = (pyglet.gl.GLubyte * data.size)( *data.astype('uint8') )
        '''

        # create an image
        # pitch is 'texture width * number of channels per element * per channel size in bytes'
        return pyglet.image.ImageData(
            dimensions[ 0 ],
            dimensions[ 1 ],
            "L",
            img,
            pitch = dimensions[ 1 ] * format_size * bytes_per_channel
            )   
        
    
    @staticmethod
    def load_textures():
        img_dir = 'imgs'
        textures = []
        if not os.path.isdir(img_dir):
            print 'Could not find directory "%s" under "%s"' % (img_dir,
                                                                os.getcwd())
            sys.exit(1)
        for image in os.listdir(img_dir):
            try:
                image = pyglet.image.load(os.path.join(img_dir, image))
            except pyglet.image.codecs.dds.DDSException:
                print '"%s" is not a valid image file' % image
                continue
            textures.append(image.get_texture())
 
            glEnable(textures[-1].target)
            glBindTexture(textures[-1].target, textures[-1].id)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
                         0, GL_RGBA, GL_UNSIGNED_BYTE,
                         image.get_image_data().get_data('RGBA',
                                                         image.width * 4))
        if len(textures) == 0:
            print 'Found no textures to load. Exiting'
            sys.exit(0)
        return textures
        
    def load_walls(self):
        walls = []
        i = 0.
        #for texture in self.textures:
        #    wall = Wall(texture, (0, 0, 0), i)
        #    walls.append(wall)
        #    i = i + 45.
        wall = Wall(self.textures[0], (-1., 0., -3.), (3., 0., -3.), (3., 2.25, -3.), (-1., 2.25, -3.))
        walls.append(wall)
        wall = Wall(self.textures[0], (3., 0., -3.), (3., 0., 1.), (3., 2.25, 1.), (3., 2.25, -3.))
        walls.append(wall)
        wall = Wall(self.textures[0], (-1., 0., 1.), (-1., 0., -3.), (-1., 2.25, -3.), (-1., 2.25, 1.))
        walls.append(wall)
        
        return walls
 
    def update(self, _):
        # Check if the spacebar is currently pressed:
        if self.keys[pyglet.window.key.W]:
            self.viewer_pos = self.viewer_pos[0]+.1*np.sin(self.angle[0]/180.*np.pi),self.viewer_pos[1]-.1*np.sin(self.angle[1]/180.*np.pi) ,self.viewer_pos[2]-.1*np.cos(self.angle[0]/180.*np.pi)*np.cos(self.angle[1]/180.*np.pi)
        if self.keys[pyglet.window.key.S]:
            self.viewer_pos = self.viewer_pos[0]-.1*np.sin(self.angle[0]/180.*np.pi),self.viewer_pos[1]+.1*np.sin(self.angle[1]/180.*np.pi),self.viewer_pos[2]+.1*np.cos(self.angle[0]/180.*np.pi)*np.cos(self.angle[1]/180.*np.pi)
        if self.keys[pyglet.window.key.A]:
            self.viewer_pos = self.viewer_pos[0]-.1*np.cos(self.angle[0]/180.*np.pi)*np.cos(self.angle[1]/180.*np.pi),self.viewer_pos[1],self.viewer_pos[2]-.1*np.sin(self.angle[0]/180.*np.pi);
        if self.keys[pyglet.window.key.D]:
            self.viewer_pos = self.viewer_pos[0]+.1*np.cos(self.angle[0]/180.*np.pi)*np.cos(self.angle[1]/180.*np.pi),self.viewer_pos[1],self.viewer_pos[2]+.1*np.sin(self.angle[0]/180.*np.pi);
        self.on_draw()
        self.clock += .01
 
    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        #self.draw_images()
        for wall in self.walls:
            self.draw_wall(wall)

    def draw_wall(self, wall):        
        glPushMatrix()
        glRotatef(self.angle[1], 1., 0., 0.)
        glRotatef(self.angle[0], 0., 1., 0.)
        
        glTranslatef(-self.viewer_pos[0], -self.viewer_pos[1], -self.viewer_pos[2])        
        glBindTexture(wall.texture.target, wall.texture.id)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0); glVertex3f( wall.c1[0], wall.c1[1], wall.c1[2])
        glTexCoord2f(1.0, 0.0); glVertex3f( wall.c2[0], wall.c2[1], wall.c2[2])
        glTexCoord2f(1.0, 1.0); glVertex3f( wall.c3[0], wall.c3[1], wall.c3[2])
        glTexCoord2f(0.0, 1.0); glVertex3f( wall.c4[0], wall.c4[1], wall.c4[2])
        glEnd()
        glPopMatrix()
 
    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(65.0, width / float(height), 0.1, 1000.0)
        glMatrixMode(GL_MODELVIEW)
        
class Wall():
    def __init__(self, texture, c1, c2, c3, c4):
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        self.texture = texture
        


if __name__ == '__main__':
    world = World(width=800, height=600) # Initialise class to preserve vars
    pyglet.app.run()
