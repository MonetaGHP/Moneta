import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(13, GPIO.IN)         
GPIO.setup(15, GPIO.IN)         

oldPinA = GPIO.input(13)
Position = 0
inc = 1                         
REmax = 14
REmin = 0                      

while True:
	encoderPinA=GPIO.input(13)
	encoderPinB=GPIO.input(15)
	if (encoderPinA == True) and (oldPinA == False):    # Detect "upward" transition on PinA
				if (encoderPinB == False):              # if PinB is low...
						Position=Position+inc           # we're going clokwise
						if Position > REmax:            # limit max value
								Position = REmin
				else:                                   # otherwise... 
						Position=Position-inc           # we're going anti-clockwise
						if Position < REmin:            # limit min value
								Position = REmax
				with open("rot.txt","w") as text_file:
					text_file.write(str(Position))
				print str(Position)
	oldPinA=encoderPinA
