import sys
import framebuf
import fontlib
import gc9a01
import time
from machine import Pin,I2C,SPI
import machine
import math
import gc
import fontlib
import random
import neopixel

screen_width = 240
screen_height = 240

#machine.freq(160000000)
spi = SPI(1,baudrate=60000000, sck=Pin(7), mosi=Pin(6))
tft = gc9a01.GC9A01(
    spi,
    dc=Pin(3, Pin.OUT),
    cs=Pin(4, Pin.OUT),
    reset=Pin(5, Pin.OUT),
    #backlight=Pin(14, Pin.OUT),
    rotation=0)
bytebuffer = bytearray(screen_width * screen_height * 2) #two bytes for each pixel
fbuf = framebuf.FrameBuffer(bytebuffer, screen_width, screen_height, framebuf.RGB565)
IBM_font = fontlib.font("IBM BIOS (8,8).bmp") # Loads font to ram 

# Define the pin for the WS2812 LED (GPIO8)
led_pin = machine.Pin(18)
# Define the number of NeoPixels (in this case, 1 for the onboard LED)
num_pixels = 1
# Create a NeoPixel object
np = neopixel.NeoPixel(led_pin, num_pixels)

def color(tup):
    red, green, blue = tup
    greens = [0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7]
    reds = [0x0,0x20,0x40,0x60,0x80,0xa0,0xc0,0xe0]
    blues = [0x0,0x400,0x600,0x800,0xa00,0xc00,0xe00,0xf000]
    ri = red//36
    gi = green//36
    bi = blue//36
    return reds[ri] | greens[gi] | blues[bi]

def dot_(a, b):
    s = 0
    for i in range(0, len(b)):
        s += a[i] * b[i]
    return s

def cross_(a,b):
    return (a[1]*b[2]-a[2]*b[1],-(a[0]*b[2]-a[2]*b[0]),a[0]*b[1]-a[1]*b[0])

def angle_between_vectors(source, dest):
    if source[0] == dest[0] and source[1] == dest[1]:
        return 0
    dt = dot_(source, dest)
    if dt > 1.0:
        dt = 1.0
    if dt < -1.0:
        dt = -1.0
    return math.acos(dt)

def signed_angle_from_to_vectors(source, dest):
    angle = angle_between_vectors(source, dest)
    cross = cross_((source[0],source[1],0),(dest[0],dest[1],0))
    dt = dot_(cross, (0,0,1))
    if dt < 0:
        return -1*angle
    else:
        return angle
    
class boid:
    def __init__(self,x,y,size,col=None):
        self.size = size
        self.pos = [x,y]
        self.speed = random.choice([(0,1),(1,0),(0,-1),(-1,0)])
        self.orblist = [[x,y,size]]    
        self.ang = 0
        self.length = 13
        self.groupRadius = 20
        self.group = []
        self.gcenter = (0,0)
        self.color = color(col)
        if col == None:
            tup = random.choice([(241, 99, 35),(242, 243, 244),(227, 68, 39),(100, 100, 100)])
            self.color = color(tup)
    
            
    def draw(self,fbuf):
        for orb in self.orblist:
            fbuf.ellipse(orb[0],orb[1], int(orb[2]/2), int(orb[2]/2), self.color,1)
    
    def getHeadingNormal(self):
        mag = math.sqrt((self.speed[0])**2+(self.speed[1])**2)
        return [self.speed[0]/mag,self.speed[1]/mag]
        
    def normalizeSpeed(self):
        self.speed = self.getHeadingNormal()
    
    def setSpeed(self,s):
        self.speed[0] = self.speed[0]*s
        self.speed[1] = self.speed[1]*s
    
    def addToSpeed(self,NormalVec,Mag):
        self.speed = (self.speed[0]+NormalVec[0]*Mag,self.speed[1]+NormalVec[1]*Mag)
    
    def limitSpeed(self,Mag=3):
        self.normalizeSpeed()
        self.speed[0] = self.speed[0]*Mag
        self.speed[1] = self.speed[1]*Mag
    
    def setGroup(self,boidList,dist = 50):
        self.gcenter = [0,0]
        self.group = []
        for bd in boidList:
            if self.getDistance((bd.pos[0],bd.pos[1])) < dist:
                self.gcenter[0] += bd.pos[0]
                self.gcenter[1] += bd.pos[1]
                self.group.append(bd)
                
    def atractToGroup(self,mag=1):
        self.gcenter[0] = self.gcenter[0]/len(self.group)
        self.gcenter[1] = self.gcenter[1]/len(self.group)
        gcNormal = self.getNormalToPnt(self.gcenter)
        if len(self.group) > 0:
            self.addToSpeed(gcNormal,mag)
                
    def repelToGroup(self,mag=20,dist_min=30):
        if len(self.group) > 0:
            d = self.getDistance(self.gcenter)
            
            if d < dist_min and d > 1:
                gcNormal = self.getNormalToPnt(self.gcenter)
                invgcNormal = (-gcNormal[0],-gcNormal[1])
                self.addToSpeed(invgcNormal,mag/d)
                
    def alignToGroup(self,mag=0.2):
        if len(self.group) > 0:
            gheading = [0,0]
            for bd in self.group:
                b_hd = bd.getHeadingNormal()
                gheading[0] += b_hd[0]
                gheading[1] += b_hd[1]
            
            gheading[0] = gheading[0]/len(self.group)
            gheading[1] = gheading[1]/len(self.group)
            
            self.addToSpeed(gheading,mag)
                
    def rot(self,rot_speed):
        inc = 0.01745329 # 1 degree in radians
        self.ang += (inc*rot_speed)
        self.ang = self.ang%(2*math.pi)
        
    def move(self):
        self.pos[0] = int(self.pos[0] + self.speed[0])
        self.pos[1] = int(self.pos[1] + self.speed[1])
        for orb in self.orblist:
            orb[2] -= 1
        self.orblist.append([self.pos[0],self.pos[1],self.size])
        if len(self.orblist) >= self.length:
            del self.orblist[0]

    def angleToPoint(self,pnt):
        d = self.getDistance((pnt))
        if d > 0:
            toCVec = ((self.pos[0]-120)/d,(self.pos[1]-120)/d)
            dirVec = (math.cos(self.ang),math.sin(self.ang))
            a = signed_angle_from_to_vectors(toCVec,dirVec)
        else:
            a = 0
        return a
    
    def dotNormalizedTo(self,pnt):
        pntN = self.getNormalToPnt(pnt)
        heading = self.getHeadingNormal()
        return dot_(pntN,heading)
    
    def angleToCenter(self,ct_pnt=(120,120)):
        return self.angleToPoint(ct_pnt)
    
    def getDistance(self,pnt):
        x,y = pnt 
        return math.sqrt((x-self.pos[0])**2+(y-self.pos[1])**2)
    
    def getNormalToPnt(self,pnt):
        d = self.getDistance(pnt)
        if d > 0:
            N = [pnt[0]-self.pos[0],pnt[1]-self.pos[1]]
            return (N[0]/d,N[1]/d)
        else:
            return (0,0)
        
    def drawCenterLine(self,px,py,fbuf):
        return fbuf.line(self.pos[0], self.pos[1], px, py, color(255,0,0))
    
    def drawVec(self,Vec):
        Vec = (Vec[0]*10,Vec[1]*10)
        return fbuf.line(self.pos[0], self.pos[1], int(self.pos[0]+Vec[0]), int(self.pos[1]+Vec[1]), color(255,0,0))

    def drawGCenter(self,fbuf):
        fbuf.pixel(int(self.gcenter[0]),int(self.gcenter[1]),color(255,255,255))
boidlist = []
boidlist.append(boid(130,130,13,col=(100,100,100)))
boidlist.append(boid(100,100,13,col=(242, 243, 244)))
boidlist.append(boid(80,80,13,col=(242, 243, 244)))
boidlist.append(boid(130,130,13,col=(241, 99, 35)))
boidlist.append(boid(100,100,13,col=(241, 99, 35)))
boidlist.append(boid(80,80,13,col=(227, 68, 39)))
screenCenter = (120,120)
np[0] = (241, 99, 35)
np.write()
while True:
    fbuf.fill(0)
    for b in boidlist:
        b.setGroup(boidlist,dist = 50)
        b.atractToGroup()
        b.repelToGroup()
        b.alignToGroup()
        b.draw(fbuf)
        if b.getDistance(screenCenter) > 80:
            Ctnormal = b.getNormalToPnt(screenCenter)
            if b.dotNormalizedTo(screenCenter) < 0.8:
                b.addToSpeed(Ctnormal,0.2+b.getDistance(screenCenter)/240)
        b.limitSpeed()
        b.move()
    tft.blit_buffer(bytebuffer,0,0,screen_width,screen_height)
    