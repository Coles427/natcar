import sensor, image, math, time, pyb
from pyb import LED
from pyb import Pin, Timer


# Tracks a white line. Use the Machine Vision threshold setting tool in the IDE to find the right values for your line (Black or White)
GRAYSCALE_THRESHOLD = [(180, 255)]

red_led   = LED(1)  # in case you want to do some signalling with LEDs
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

measured_angle = 0

old_time = pyb.millis()

timM = Timer(4, freq=2000)
timS = Timer(2, freq=300)
timePerCount = 1/(timS.source_freq() / (timS.prescaler() + 1))

# Frequency in Hz
# Generate a 1KHz square wave on TIM4 with 50%, 75% and 50% duty cycles on channels 1, 2 and 3 respectively.
ch1 = timM.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=10)
ch2 = timS.channel(1, Timer.PWM, pin=Pin("P6"), pulse_width_percent=75)
odd = 0

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (0, 20, 160, 50, 1),
        (0, 30, 53, 30, 0.2), # You'll need to tweak the weights for your app
        (53,  30, 53, 30, 0.6), # depending on how your robot is setup.
        (106,   30, 53, 30, 0.2)
       ]

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(30) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.


while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    #This part determins the center of the line, used to control servo and motor PWM
    blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=ROIS[0][0:4], merge=True) # r[0:4] is roi tuple.
    center_pos = 999
    if blobs:
        # Find the index of the blob with the most pixels.
        most_pixels = 0
        for i in range(len(blobs)):
            if blobs[i].pixels() > most_pixels:
                most_pixels = blobs[i].pixels()
                largest_blob = i

        # Draw a rect around the blob.
        img.draw_rectangle(blobs[largest_blob].rect())
        img.draw_cross(blobs[largest_blob].cx(),
                       blobs[largest_blob].cy())

        center_pos = blobs[largest_blob].cx()

    #This part is used for finish line detection
    areas = []
    edges = []
    width = 0
    for r in ROIS[1:4]:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())
            areas.append(largest_blob.area())
            edges.append(((largest_blob.cx() - largest_blob.w()/2), largest_blob.cx() + largest_blob.w()/2))
            width = largest_blob.w()
    if(len(edges) == 3):
        between = []
        i = 0
        while i < 2:
            between.append(edges[i+1][0] - edges[i][1])
            i = i + 1
    #The next three lines check to see how similar in area the detected blobs are
    mean = sum(areas) / 3
    variance = sum([((x - mean) ** 2) for x in areas]) / 3
    pdev = 999
    if (mean != 0) : pdev = ((variance ** 0.5) / mean) * 100

    #If the differnce in area is close enough and if the space between lines
    #is equal to width of the line, we found the finish line
    if (pdev < 15):
        if(between[0]/width > .875 and between[0]/width < 1.125):
            if(between[1]/width > .875 and between[1]/width < 1.125):
                print ("FINISH")
                break

    now = pyb.millis()
    if  now > old_time + 1.0 :  # time has passed since last measurement
        led_status = [0,0,0]
        if center_pos >= 0 and center_pos <= 60:
            red_led.off()
            green_led.off()
            blue_led.on()
            led_status = [0,0,1]
            #motor_control = 50
        elif center_pos <= 110 and center_pos > 55:
            red_led.off()
            blue_led.off()
            green_led.on()
            led_status = [0,1,0]
           # motor_control = 80
        elif center_pos > 110 and center_pos <=160:
            red_led.on()
            blue_led.off()
            green_led.off()
            led_status = [1,0,0]
           # motor_control = 50
        else:
            red_led.off()
            blue_led.off()
            green_led.off()
            led_status = [0,0,0]
            print ("Line not detected")
           # motor_control = 20
        print ("LED status is: RED", led_status[0], ", GREEN",
                    led_status[1], ", BLUE", led_status[2])
        print ("center_pos: " ,center_pos)
      #  ch1 = timM.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent= motor_control)
        old_time = now  # reset clock
