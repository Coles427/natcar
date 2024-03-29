# 195b.py
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time, pyb
from pyb import Pin, Timer,LED,USB_VCP

tim4 = Timer(4, freq=300) # Frequency in Hz
# Generate a 2KHz square wave on TIM4 with 50%, 75% and 50% duty cycles on channels 1, 2 and 3 respectively.
#ld = tim4.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=0)
ld = pyb.Pin("P8",pyb.Pin.OUT_PP)
ld.low()
lp = tim4.channel(3, Timer.PWM, pin=Pin("P9"), pulse_width_percent=0)
started = -1
stuck = -1
tim2 = Timer(2, freq = 300)
#rd = tim2.channel(2, Timer.PWM, pin=Pin("P5"), pulse_width_percent = 0)
rd = pyb.Pin("P1", pyb.Pin.OUT_PP)
rd.low()
rp = tim2.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width_percent = 0)
speed=80
white = [(169 , 255)]
roi1= (0,0,160,10)
lroi=(0,0,10,120)
rroi=(150,0,10,120)
move=0 #0停1前2左前3右前4左5右
sensor.reset()                          # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to GRAYSCALE
sensor.set_framesize(sensor.QQVGA)      # Set frame size to QQVGA (160 x 120)
sensor.skip_frames(time = 2000)         # Wait for settings take effect.
sensor.set_auto_gain(False)             # must be turned off for color tracking
sensor.set_auto_whitebal(False)         # must be turned off for color tracking
clock = time.clock()                    # Create a clock object to track the FPS.
usb = pyb.USB_VCP() #for debuging when not connected to IDE
zeroCnt = 0
while(True):
    clock.tick()                        # Update the FPS clock.
    img = sensor.snapshot()             # Take a picture and return the image.
    dmin=81
    midb=-1
    left=-1
    right=-1
    for blob in img.find_blobs(white, roi=roi1, pixels_threshold=4, area_threshold=4, merge=True):
        img.draw_rectangle(blob.rect(), color=0)
        img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
        d=blob.cx()-80
        if (d<0):
            d=-d
        if(d<dmin):
            dmin=d
            midb=blob.cx()
    for blob in img.find_blobs(white, roi=lroi, pixels_threshold=4, area_threshold=4, merge=True):
        img.draw_rectangle(blob.rect(), color=0)
        img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
        left=blob.cy()
    for blob in img.find_blobs(white, roi=rroi, pixels_threshold=4, area_threshold=4, merge=True):
        img.draw_rectangle(blob.rect(), color=0)
        img.draw_cross(blob.cx(), blob.cy(),color=0,size=4,thickness=1)
        right=blob.cy()
    if left>-1:
        left=120-left
    if right>-1:
        right=120-right
    print(midb,left,right)
    if(midb==-1):
        if(right==-1 and left==-1):
            move=0
        if right>left:
            move=5
        if right<left:
            move=4
    elif (midb<50):
        move=2
    elif (midb<110):
        move=1
    else:
        move=3

    if stuck == 1:
        move = -1
        ld.high()
        lp.pulse_width_percent(int(speed/2))
        rd.high()
        rp.pulse_width_percent(int(speed/2))
        print("Stuck, moving back")
        zeroCnt -= 1
        if zeroCnt < 1:
            stuck = 0
    if move==0:
        ld.low()
        lp.pulse_width_percent(0)
        rd.low()
        rp.pulse_width_percent(0)
        zeroCnt += 1
        if zeroCnt > 20:
            stuck = 1
    if move==1:
        started = 1
        ld.low()
        lp.pulse_width_percent(speed)
        rd.low()
        rp.pulse_width_percent(speed)
        zeroCnt = 0
    if move==2:
        started = 1
        ld.low()
        lp.pulse_width_percent(0)
        rd.low()
        rp.pulse_width_percent(int(speed/3))
        zeroCnt = 0
    if move==3:
        started = 1
        ld.low()
        lp.pulse_width_percent(int(speed/3))
        rd.low()
        rp.pulse_width_percent(0)
        zeroCnt = 0
    if move==4:
        started = 1
        ld.high()
        lp.pulse_width_percent(int(speed/3))
        rd.low()
        rp.pulse_width_percent(int(speed/3))
        zeroCnt = 0
    if move==5:
        started = 1
        ld.low()
        lp.pulse_width_percent(int(speed/3))
        rd.low()
        rp.pulse_width_percent(0)
        zeroCnt = 0
    print(move,clock.fps())                  # Note: OpenMV Cam runs about half as fast when connected
                                        # to the IDE. The FPS should increase once disconnected.

