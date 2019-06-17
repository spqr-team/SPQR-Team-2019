# color tracking - By: paolix - ven mag 18 2018

# Automatic RGB565 Color Tracking Example
#

import sensor, image, time, pyb, math

from pyb import UART
uart = UART(3,19200, timeout_char = 1000)


# LED Setup ##################################################################

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.off()
green_led.off()
blue_led.off()
##############################################################################


#thresholds = [  (30, 100, 15, 127, 15, 127),    # generic_red_thresholds
#                (30, 100, -64, -8, -32, 32),    # generic_green_thresholds
#                (0, 15, 0, 40, -80, -20)]       # generic_blue_thresholds

#thresholds = [  (54, 93, -10, 25, 55, 70),    # thresholds yellow goal
#                (30, 45, 1, 40, -60, -19)]    # thresholds blue goal
#

thresholds = [  (53, 100, -15, 7, 5, 51),    # thresholds yellow goal
                (32, 60, -10, -2, -52, -9)]  # thresholds blue goal (6, 31, -15, 4, -35, 0)



# Camera Setup ###############################################################
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)         # must be turned off for color tracking
sensor.set_auto_whitebal(False)     # must be turned off for color tracking
clock = time.clock()
##############################################################################



# [] list
# () tupla

while(True):
    clock.tick()
    tt_yellow = [(0,999,0,1)]     ## creo una lista di tuple per il giallo, valore x = 999 : non trovata
    tt_blue = [(0,999,0,2)]       ## creo una lista di tuple per il blue, valore x = 999 : non trovata

    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, pixels_threshold=300, area_threshold=700):
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())

        if (blob.code() == 1):
            tt_yellow = tt_yellow +  [ (blob.area(),blob.cx(),blob.cy(),blob.code() ) ]
        if (blob.code() == 2):
            tt_blue = tt_blue +  [ (blob.area(),blob.cx(),blob.cy(),blob.code() ) ]

    tt_yellow.sort(key=lambda tup: tup[0])  ## ordino le liste
    tt_blue.sort(key=lambda tup: tup[0])    ## ordino le liste

    ny = len(tt_yellow)
    nb = len(tt_blue)
    xY = 150
    yY = 3

    area,cx,cy,code = tt_yellow[ny-1]    # coordinata x del piu' grande y se montata al contrario
    Y = ((90 - (int((math.atan2(((cy - yY) / 1.41), cx - xY))* 180 / math.pi))) * -1)
    string_yellow = "Y"+str(Y)+"y"
    #string_yellow = str(cx) + " - " + str(cy);

    #for c in range( 0, ny):
    #    print (tt_yellow[c])


    area,cx,cy,code = tt_blue[nb-1]      # coordinata x del piu' grande y se montata al contrario
    B = ((90 - (int((math.atan2(((cy - yY) / 1.41), cx - xY))* 180 / math.pi))) * -1)
    string_blue = "B"+str(B)+"b"

    uart.write(string_yellow)   # scrivo su seriale
    uart.write(string_blue)     # scrivo su seriale
    print (string_yellow)   # test on serial terminal
    print (string_blue)     # test on serial terminal

    #print ("..................................")

print(clock.fps())
