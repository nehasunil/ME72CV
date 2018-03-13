from SimpleCV import *
import operator
import numpy as np
import smbus
import time

# Movement plan: turn with pre-programmed radius
# until orange stripe on bottom of ramp is horizontal
    #send 50 to stop turning
# then move forward/backward until ymax is within range
    #send 90 to move backward, 110 to move forward
# then ready to go up ramp
    #send 120 to give control to arduino

rampOnLeft = TRUE

display_width = 320
display_height = 240

display = SimpleCV.Display((display_width,display_height))
cam = SimpleCV.Camera()

# Hue Segmentation
ORANGE_HUE = 115 #5 for photo
BASE_HUE = 171
BINARIZE_THRESH = 10
BASE_BINARIZE_THRESH = 20
MINSIZE = display_width * display_height * .01

MIN_DIST = 136 #3ft
MAX_DIST = 144 #2ft

# I2C Communication setup
bus = smbus.SMBus(1)
address = 0x04
mode = 0x00

normaldisplay = True
frame_count = 0
straight = False
inPosition = False

text = 'havent seen horizontal lines yet'

while display.isNotDone():
    if display.mouseRight:
        normaldisplay = not(normaldisplay)
        print 'Display Mode:', 'Normal' if normaldisplay else 'segmented'

    img = cam.getImage().adaptiveScale((display_width,display_height))
    dl = DrawingLayer((img.width,img.height))

    orangeDist = img.hueDistance(ORANGE_HUE)
    orangeBin = orangeDist.binarize(BINARIZE_THRESH).erode(3).dilate(6).invert()
    orange = (img-orangeBin).smooth()
    
    # line up to ramp
    if frame_count < 3 and not straight:
        lines = orange.findLines(threshold = 50, minlinelength=20)
        if rampOnLeft:
            temp = lines.filter(lines.angle() > -5)
            horizontal = temp.filter(temp.angle() < 2)
        else:
            temp = lines.filter(lines.angle() < 5)
            horizontal = temp.filter(temp.angle() > -2)
        if horizontal:
            frame_count += 1
            text = 'frame count: {}'.format(frame_count)
            if frame_count >= 3:
                straight = True
                bus.write_byte_data(address, mode, 50)
                text = 'straight!'
        else:
            frame_count = 0
            
    # adjust distance
    elif not inPosition:
        blobs = orange.findBlobs(minsize = 500)
        if blobs:
            blobs.sort(key=operator.methodcaller('maxY'), reverse = True)
            distance = blobs[0].maxY()
            if distance < MIN_DIST: #go forward
                bus.write_byte_data(address, mode, 110)
                text = 'moving forward to get within 3 feet'
            elif distance > MAX_DIST: #go backward
                bus.write_byte_data(address, mode, 90)
                text = 'moving backward to keep 2 foot distance'
            else:
                inPosition = True
                bus.write_byte_data(address, mode, 120)
                text = 'in position for arduino to take over'
        
    if normaldisplay:
        img.drawText(text, x=15, y=40, color = SimpleCV.Color.YELLOW)
        img.show()
    else:
        if horizontal:
            horizontal.show(width = 3)
        else:
            base.show()