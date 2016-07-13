import RPi.GPIO as GPIO
import time

pulse_start = time.time()
pulse_end = time.time()

while True:
  GPIO.setmode(GPIO.BOARD) #BCM)

  TRIG = 16#23 
  ECHO = 18#24
  
  #print "Distance Measurement In Progress"

  GPIO.setup(TRIG,GPIO.OUT)
  GPIO.setup(ECHO,GPIO.IN)

  GPIO.output(TRIG, False)
  #print "Waiting For Sensor To Settle"
  time.sleep(.1)

  GPIO.output(TRIG, True)
  time.sleep(0.00001)
  GPIO.output(TRIG, False)

  while GPIO.input(ECHO)==0:
    pulse_start = time.time()

  while GPIO.input(ECHO)==1:
    pulse_end = time.time()

  pulse_duration = pulse_end - pulse_start

  distance = pulse_duration * 17150

  distance = round(distance, 2)

  with open("us.txt","w") as text_file:
	  text_file.write(str(distance)) 
  print str(distance)

GPIO.cleanup()
