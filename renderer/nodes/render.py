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
from geometry_msgs.msg import PolygonStamped
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
        self.polygons = []
        self.triangles = []
        self.lines = []
        self.clock = 0
        pyglet.clock.schedule_interval(self.update, 1 / 60.0)
        pyglet.clock.set_fps_limit(60)
        self.dragging = False
        self.angle = (0., 0.)
        self.mouse_pos = (0,0)        
        self.keys = pyglet.window.key.KeyStateHandler()
        self.push_handlers(self.keys)
        self.connect()
        self.live1 = False
        self.live2 = False
        
        self.cross_thread_trigger = False
        
    def connect(self):
        rospy.init_node('Renderer')
        rospy.Subscriber('/template_track/img',Image,self.texture_process)
        rospy.Subscriber('/template_track/corners',Float32MultiArray, self.corner_process)
        rospy.Subscriber("/template_track/wall_trigger", Empty, self.trigger_process)
        rospy.Subscriber("/point_handler/polygon", PolygonStamped, self.on_got_polygon)
        rospy.Subscriber("/point_handler/triangle", PolygonStamped, self.on_got_triangle)
        rospy.Subscriber("/point_handler/poly_clear", Empty, self.on_poly_clear)
        rospy.Subscriber("/point_handler/lines", Float32MultiArray, self.on_got_line)
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
        #self.texture_buffer = pyglet.image.ImageData(640,360, "BGR", cvimg.tostring()).get_texture()
        #cv to cv2 numpy array image
        self.texture_buffer = np.asarray(cvimg)
        self.live1 = True
        
    def on_got_polygon(self, polygonStamped):
        """Adds the receieved polygon to the draw list"""
        points = []
        print "New polygon of ", len(polygonStamped.polygon.points), " points"
        for p in polygonStamped.polygon.points:
            points.append(np.array([p.x, p.y, p.z]))
        self.polygons.append(Polygon(points))
        
    def on_got_triangle(self, polygonStamped):
        """Adds the receieved polygon to the draw list"""
        points = []
        for p in polygonStamped.polygon.points:
            points.append(np.array([p.x, p.y, p.z]))
        self.triangles.append(Polygon(points))
        
    def on_got_line(self, float32MultiArray):
        """Adds the receieved line to the draw list"""
        i = 0;
        while (i < len(float32MultiArray.data)):
            self.lines.append(Line(float32MultiArray.data[i],float32MultiArray.data[i+1],float32MultiArray.data[i+2],float32MultiArray.data[i+3]))
            print "Added line with ends (", float32MultiArray.data[i], ", ", float32MultiArray.data[i+1], " and ", float32MultiArray.data[i+2], ", ", float32MultiArray.data[i+3]
            i = i + 4
        
        
        
    def on_poly_clear(self, d):
        """ Clears the polygon list """
        # This is not thread-safe, may need proper handling
        self.polygons = []
        self.triangles = []
        self.lines = []
        
    def corner_process(self, d):
        self.corner_buffer = d.data
        self.live2 = True
        
    def trigger_process(self, d):
        while((not self.live1) or (not self.live2)):
            rospy.sleep(0.01)
            
        self.cross_thread_trigger = True
        while(self.cross_thread_trigger == True):
            rospy.sleep(0.1)
        
        '''    
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
        
        self.textures2 = self.load_nparray()
        #self.walls[-1].texture = None
        wall = Wall(self.textures2[0], (-1., 0., -2.), (3., 0., -2.), (3., 2.25, -2.), (-1., 2.25, -2.))
        self.walls.append(wall)
        '''
        
        self.live1 = False
        self.live2 = False
        
        
        
    @staticmethod
    def load_nparray(npimage):
        # load_nparray MUST be called in the same thread as pyglet is running
        # gl###### texture loading commands will NOT load textures otherwise
        # There is no visible error in this case, but textures are blank white
        
        #cv2.imshow('windowwww', npimage)
        #cv2.waitKey(0)
        
        print npimage.shape
        image = pyglet.image.ImageData(npimage.shape[1], npimage.shape[0], 'BGR', npimage.tostring())
        print image        
        print image.pitch        
        print image.format
        
        
        texture = image.get_texture()
        
        glEnable(texture.target)
        glBindTexture(texture.target, texture.id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
                     0, GL_BGR, GL_UNSIGNED_BYTE,
                     image.get_image_data().get_data('BGR',
                                                     image.width * -3))
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR )
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR )
        #glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE )
        #glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR )
                                                     
        return texture
    
    def load_textures2(self):
        textures = []
        
        npimage = cv2.imread('2.png')
        print npimage.shape
        image = pyglet.image.ImageData(16, 16, 'BGR', npimage.tostring())
        print image
        
        print image.pitch        
        print image.format
        print image.get_image_data()
        
        #cv2.imshow('window', npimage)
        #cv2.waitKey(0)        
        
        textures.append(image.get_texture())
        
        glEnable(textures[-1].target)
        glBindTexture(textures[-1].target, textures[-1].id)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
                     0, GL_BGR, GL_UNSIGNED_BYTE,
                     image.get_image_data().get_data('BGR',
                                                     image.width * 3))
        
        
        return textures
    
    @staticmethod
    def load_textures():
        img_dir = 'imgs'
        textures = []
        img_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+'/nodes/imgs'
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
            glDisable(textures[-1].target)
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
        wall = Wall(self.textures[-1], (-3., 0., -3.), (3., 0., -3.), (3., 2.25, -3.), (-3., 2.25, -3.), (0,0.297), (0.625,0.297), (0.625,1), (0,1))
        walls.append(wall)
        wall = Wall(self.textures[0], (3., 0., -3.), (3., 0., 1.), (3., 2.25, 1.), (3., 2.25, -3.), (0,0), (1,0), (1,1), (0,1))
        walls.append(wall)
        wall = Wall(self.textures[0], (-3., 0., 1.), (-3., 0., -3.), (-3., 2.25, -3.), (-3., 2.25, 1.), (0,0), (1,0), (1,1), (0,1))
        walls.append(wall)
        
        return walls
 
    def update(self, _):
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
        if (self.cross_thread_trigger):
            self.add_wall()
            self.cross_thread_trigger = False
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glDisable(GL_TEXTURE_2D)
        
        glLoadIdentity()
        glPushMatrix()
        
        glRotatef(self.angle[1], 1., 0., 0.)
        glRotatef(self.angle[0], 0., 1., 0.)
        
        glTranslatef(-self.viewer_pos[0], -self.viewer_pos[1], -self.viewer_pos[2])
        self.draw_grid()
        for wall in self.walls:
            self.draw_wall(wall)
        for poly in self.polygons:
            self.draw_polygon(poly)
        #self.draw_triangles(self.polygons)
        
        self.draw_lines(self.lines)
        
        glPopMatrix()
        
        
        
    def draw_polygon(self, poly):
        glPushAttrib(GL_CURRENT_BIT)
        glColor3f(.5, .5, 5.)
        glBegin(GL_POLYGON)
        for p in poly.points:
            glVertex3f( p[0], p[1], p[2] )
        glEnd()
        glColor3f(.25, .25, .25)
        glBegin(GL_LINE_LOOP)
        for p in poly.points:
            glVertex3f( p[0], p[1], p[2] )
        glEnd()
        glPopAttrib(GL_CURRENT_BIT)
            
    def draw_triangles(self, tris):
        glBegin(GL_TRIANGLES)
        for t in tris:
            for p in t.points:
                glVertex3f(p[0], p[1], p[2])
        glEnd()
        
    def draw_lines(self, lines):
        glPushAttrib(GL_CURRENT_BIT)
        glColor3f(0., 0., 0.)
        glBegin(GL_LINES)
        for l in lines:
            # Flips from image to world here (in ground plane)
            # lines were entered as [image_x, image_z]
            # Should probably happen elsewhere
            glVertex3f(l.start[0], l.start[1], 0.01)
            glVertex3f(l.end[0], l.end[1], 0.01)
        glEnd()
        glPopAttrib(GL_CURRENT_BIT)
            
            
    def add_wall(self):
        c = self.corner_buffer
        print "Corner buffer:\r\n",c
        c1 = np.array([c[0], c[1], c[2]])
        c2 = np.array([c[3], c[4], c[5]])
        c3 = np.array([c[6], c[7], c[8]])
        c4 = np.array([c[9], c[10], c[11]])
        t1 = np.array([c[12], c[13]])
        t2 = np.array([c[14], c[15]])
        t3 = np.array([c[16], c[17]])
        t4 = np.array([c[18], c[19]])
        wall = Wall(self.load_nparray(self.texture_buffer), c1, c2, c3, c4, t4, t3, t2, t1)
        #wall = Wall(self.load_nparray(self.texture_buffer), c1, c2, c3, c4, (0,0), (1,0), (1,1), (0,1))
        print wall.t1
        print wall.t2
        print wall.t3
        print wall.t4
        self.walls.append(wall)

    def draw_wall(self, wall):
        glEnable( GL_TEXTURE_2D )
        glBindTexture(wall.texture.target, wall.texture.id)
        glBegin(GL_QUADS)
        glTexCoord2f(wall.t1[0], wall.t1[1]); glVertex3f( wall.c1[0], wall.c1[1], wall.c1[2])
        glTexCoord2f(wall.t2[0], wall.t2[1]); glVertex3f( wall.c2[0], wall.c2[1], wall.c2[2])
        glTexCoord2f(wall.t3[0], wall.t3[1]); glVertex3f( wall.c3[0], wall.c3[1], wall.c3[2])
        glTexCoord2f(wall.t4[0], wall.t4[1]); glVertex3f( wall.c4[0], wall.c4[1], wall.c4[2])
        #glTexCoord2f(0,0); glVertex3f( wall.c1[0], wall.c1[1], wall.c1[2])
        #glTexCoord2f(1,0); glVertex3f( wall.c2[0], wall.c2[1], wall.c2[2])
        #glTexCoord2f(1,1); glVertex3f( wall.c3[0], wall.c3[1], wall.c3[2])
        #glTexCoord2f(0,1); glVertex3f( wall.c4[0], wall.c4[1], wall.c4[2])
        glEnd()
        glDisable( GL_TEXTURE_2D )
    
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
            
 
    def on_resize(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(65.0, width / float(height), 0.1, 1000.0)
        glMatrixMode(GL_MODELVIEW)
        
class Wall():
    def __init__(self, texture, c1, c2, c3, c4, t1, t2, t3, t4):
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.t4 = t4
        self.texture = texture
        
class Polygon():
    def __init__(self, points):
        self.points = points
        
class Line():
    def __init__(self, x1, y1, x2, y2):
        self.start = np.array([x1, y1])
        self.end = np.array([x2, y2])
        


if __name__ == '__main__':
    world = World(width=800, height=600) # Initialise class to preserve vars
    pyglet.app.run()
