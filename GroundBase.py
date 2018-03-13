from SimpleCV import *
import operator
import smbus
import time

display_width = 320
display_height = 240

display = SimpleCV.Display((display_width,display_height))
cam = SimpleCV.Camera()
normaldisplay = True

# I2C Communication setup
bus = smbus.SMBus(1)
address = 0x04
mode = 0x00

# Hue Segmentation
BASE_HUE = 171 #shop
BASE_BINARIZE_THRESH = 30

MINSIZE = display_width * display_height * .01 # blob takes up 1% of display
Y_MAX_THRESH = 125  # base is 4ft away

veerRight = True #change to approach base on left


# Use X and Y bounds to determine necessary servo angle
def servoAngleRight(blob):
    x = float(blob.maxX())
    y = float(blob.maxY())
    return 728.9 - 3.396*x - 7.592*y + 0.001617*x**2 + 0.0223*x*y + 0.0175*y**2

def servoAngleLeft(blob):
    x = display_width - float(blob.minX())
    y = float(blob.maxY())
    return -(730 - 3.396*x - 7.592*y + + 0.001617*x**2 + 0.0223*x*y + 0.0175*y**2)


while display.isNotDone():
    if display.mouseRight:
        normaldisplay = not(normaldisplay)
        print 'Display Mode:', 'Normal' if normaldisplay else 'segmented'

    img = cam.getImage().adaptiveScale((display_width,display_height)) 
    servoDelta = 0
    dl = DrawingLayer((img.width,img.height))
    text = 'no base'

    baseDist = img.hueDistance(BASE_HUE, minsaturation = 40, minvalue = 40)
    baseBin = baseDist.binarize(BASE_BINARIZE_THRESH).erode(4).dilate(8)
    blobs = baseBin.findBlobs(minsize = MINSIZE)

    if blobs:
        blobs[-1].drawOutline(color=Color.ORANGE, width = 2, layer = dl)
        
        # For a base further than 4ft away
        if blobs[-1].maxY() < Y_MAX_THRESH:
            if veerRight: # align to max val
                xmax_aim = 235 - 0.9*blobs[-1].maxY()
                text = 'far away, going straight'
                if blobs[-1].maxX() < xmax_aim -10:
                    servoDelta = -1
                    text = 'far away, turning left to get closer'
                elif blobs[-1].maxX() > xmax_aim:
                    servoDelta = 1
                    text = 'far away, turning right to avoid collision'
            else:
                xmin_aim = blobs[-1].maxY()+70
                if blobs[-1].minX() > xmin_aim + 10:
                    servoDelta = 1
                    text = 'far away, turning right to get closer'
                elif blobs[-1].minX() < xmin_aim:
                    servoDelta = -1
                    text = 'far away, turning left to avoid collision'

        else:
            if veerRight:
                servoDelta = servoAngleRight(blobs[-1])
                text = 'veering right'
                if servoDelta < 0:
                    text = 'error: {}'.format(servoDelta)
                    servoDelta = 0
                    
            else:
                servoDelta = servoAngleLeft(blobs[-1])
                text = 'veering left'
                if servoDelta > 0:
                    servoDelta = 0

    servoVal = int(servoDelta + 100) # range of I2C communication is from 0 to 128
    if servoVal < 73:
        servoVal = 73
    elif servoVal > 127:
        servoVal = 127
    bus.write_byte_data(address, mode, servoVal)

    
    if normaldisplay:
        img.drawText(text, x=15, y=40, color = SimpleCV.Color.YELLOW)
        img.drawText('servoDelta: {}'.format(servoDelta), x=15, y=50, color = SimpleCV.Color.YELLOW)
        img.drawText('servoVal: {}'.format(servoVal), x=15, y=60, color = SimpleCV.Color.YELLOW)
        img.addDrawingLayer(dl)
        img.show()
    else:
        if blobs:
            blobs.show()