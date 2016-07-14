#====IMPORTS====
import RPi.GPIO as GPIO
import time
import math
import smbus
import subprocess
import sys

#====GPIO====
GPIO.setwarnings(False)
GPIO.cleanup()  
GPIO.setmode(GPIO.BOARD)

#====PINS====
TRIG_ = 16 #Trig pin on ultrasonic
ECHO_ = 18 #Echo pin on ultrasonic
PINA_ = 13 #PinA on import smbusrot
PINB_ = 15 #PinB on rot
SWPN = 8 #SW Pin on rot
RED_ = 12 #RGB red pin
GRN_ = 23 #RGB green pin
BLU_ = 24 #RGB blue pin

#====VARIABLES====
counter = 0
distance = 0
tasks = []

#====COLORS====
CYAN = [0,100,100]
MAGE = [100,0,100]
YELO = [100,100,0]
BLUE = [0,0,100]
REDD = [100,0,0]
GREN = [0,100,0]
WHTE = [100,100,100]
BLCK = [0,0,0]

#====STATES====
newTask = False
taskIsNew = False
taskIsOld = False
taskFor = [0,0,0] #Color of the person the task is for
rotClick = True #False
walkBy = False
rotPosit = 0
global MUTE 
MUTE = False
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
GPIO.setup(SWPN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def stopall():
#SEN.stop()
	#def stop(self):
		#self.TRIG.stop()
		#self.ECHO.stop()
#RGB.stop()
	#def stop():
		#self.RED.stop()
		#self.GREEN.stop()
		#self.BLUE.stop()
#ROT.stop()
	#def stop():
		#self.PINA.stop()
		#self.PINB.stop()
	GPIO.cleanup()
	LCD.lcd_string(" Shutting down! ",0x80)
	LCD.lcd_string(" Good night ^_^ ",0xC0)
	time.sleep(2)
	LCD.lcd_string("",0x80)
	LCD.lcd_string("",0xC0)
	sys.exit()
	
	
class rgb(object):
	def __init__(self,R,G,B):
		self.data = "Null"
		self.Freq = 100		
		GPIO.setup(R, GPIO.OUT)
		GPIO.setup(G, GPIO.OUT)
		GPIO.setup(B, GPIO.OUT)
		self.RED = GPIO.PWM(R, self.Freq)
		self.RED.start(0)
		self.GREEN = GPIO.PWM(G, self.Freq)
		self.GREEN.start(0)
		self.BLUE = GPIO.PWM(B, self.Freq)
		self.BLUE.start(0)
		self.timestart = time.time()
	def __str__(self):
		self.ans = "R:",R,"G:",G,"B:",B
		return self.ans	
	def colour(self,R, G, B):
		self.R = 100-int(self.R)
		self.G = 100-int(self.G)
		self.B = 100-int(self.B)
		self.RED.ChangeDutyCycle(self.R)
		self.GREEN.ChangeDutyCycle(self.G)
		self.BLUE.ChangeDutyCycle(self.B)
	def colourM(self,lst):
		self.R = lst[0]
		self.G = lst[1]
		self.B = lst[2]
		self.R = 100-int(self.R)
		self.G = 100-int(self.G)
		self.B = 100-int(self.B)
		self.RED.ChangeDutyCycle(self.R)
		self.GREEN.ChangeDutyCycle(self.G)

		self.BLUE.ChangeDutyCycle(self.B)
	def wave():
		for i in range(0, 720, 5):  
			colour(PosSinWave(50, i, 0.5),  
			PosSinWave(50, i, 1),  
			PosSinWave(50, i, 2))
	def PosSinWave(amplitude, angle, frequency):  
		return self.amplitude + (self.amplitude * math.sin(math.radians(self.angle)*self.frequency) ) 
	def flashN(self,lst):
		RGB.colourM(lst)
		if time.time() - self.timestart > 2:
			RGB.colour(0,0,0)
			self.timestart = time.time()
	def flash(self,lst):
		RGB.colourM(lst)
		time.sleep(.5)
		RGB.colourM(BLCK)

class lcd(object):
	def __init__(self):
		#lcd_string("To Send line one text",LCD_LINE_1)
		#lcd_string("To Send line two text",LCD_LINE_2)
		
		# Define some device parameters
		self.I2C_ADDR  = 0x27 # I2C device address
		self.LCD_WIDTH = 16   # Maximum characters per line
		
		# Define some device constants
		self.LCD_CHR = 1 # Mode - Sending data
		self.LCD_CMD = 0 # Mode - Sending command
		
		self.LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
		self.LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
		self.LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
		self.LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line
		
		self.LCD_BACKLIGHT  = 0x08  # On
		#LCD_BACKLIGHT = 0x00  # Off
		
		self.ENABLE = 0b00000100 # Enable bit
		
		# Timing constants
		self.E_PULSE = 0.0005
		self.E_DELAY = 0.0005
		
		#Open I2C interface
		#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
		self.bus = smbus.SMBus(1) # Rev 2 Pi uses 1
		# Initialise displays
		self.lcd_byte(0x33,self.LCD_CMD) # 110011 Initialise
		self.lcd_byte(0x32,self.LCD_CMD) # 110010 Initialise
		self.lcd_byte(0x06,self.LCD_CMD) # 000110 Cursor move direction
		self.lcd_byte(0x0C,self.LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
		self.lcd_byte(0x28,self.LCD_CMD) # 101000 Data length, number of lines, font size
		self.lcd_byte(0x01,self.LCD_CMD) # 000001 Clear display
		time.sleep(self.E_DELAY)
		self.lcd_byte(0x01, self.LCD_CMD)
	
	def lcd_byte(self,bits, mode):
		# Send byte to data pins
		# bits = the data
		# mode = 1 for data
		#        0 for command
		self.bits_high = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
		self.bits_low = mode | ((bits<<4) & 0xF0) | self.LCD_BACKLIGHT
		
		# High bits
		self.bus.write_byte(self.I2C_ADDR, self.bits_high)
		self.lcd_toggle_enable(self.bits_high)

		# Low bits
		self.bus.write_byte(self.I2C_ADDR, self.bits_low)
		self.lcd_toggle_enable(self.bits_low)
	
	def lcd_toggle_enable(self,bits):
	  # Toggle enable
	  time.sleep(self.E_DELAY)
	  self.bus.write_byte(self.I2C_ADDR, (bits | self.ENABLE))
	  time.sleep(self.E_PULSE)
	  self.bus.write_byte(self.I2C_ADDR,(bits & ~self.ENABLE))
	  time.sleep(self.E_DELAY)
	
	def lcd_string(self,message,line):
	  message = message.ljust(self.LCD_WIDTH," ")
	  self.lcd_byte(line, self.LCD_CMD)
	  for i in range(self.LCD_WIDTH):
		self.lcd_byte(ord(message[i]),self.LCD_CHR)
	
#====Declaring the periphs====
RGB = rgb(RED_,GRN_,BLU_)
LCD = lcd()
	
class task(object):
	def __init__(self, color, text, time=time.time()):
		self.color = color #Color of the person the task is for
		self.time = time #Time the task was assigned
		self.text = text #Actual text of the task itself
		self.isNew = True
		self.text2 = ""
		self.text3 = ""
		self.sanit()
	def __str__(self):
		return self.text
	def sanit(self):
		self.text 
		self.texty = self.text+" "+str(time.strftime('%H'))+":"+str(time.strftime('%M %m'))+"/"+str(time.strftime('%y'))
		self.text2 = ""
		self.text3 = ""
		self.lst = self.texty.split()
		i = 16
		if len(self.texty) > 16:
			while len(self.text3)+len(self.lst[0]) < 16:
				self.text3 = self.text3 +" "+self.lst[0]
				self.lst.remove(self.lst[0])
			while 0 < len(self.lst):
				self.text2 = self.text2 +" "+self.lst[0]
				self.lst.remove(self.lst[0])
	def printToLCD(self):
		LCD.lcd_string(self.text3,0x80)
		LCD.lcd_string(self.text2,0xC0)
	def speak(self):
		if not MUTE:
			subprocess.call(["espeak",str(self.text)])
	def read(self):
		#self.isNew = False
		RGB.colourM(self.color)
		self.printToLCD()
		self.speak()
		print self
	def getColor(self):
		return self.color
	def isNewTask(self):
		return self.isNew
	def getData(self):
		return self.text

def hasNewTasks(tasklist):
	ans = False
	for i in range(0,len(tasklist)):
		if tasklist[i].isNewTask():
			ans = True
	return ans
	
def getUS():
	with open("us.txt","r") as usFile:
		usVal = usFile.read()
	try:
		fusVal = float(usVal)
	except ValueError:
		return 999
	#print fusVal
	return fusVal

def getRot():
	with open("rot.txt","r") as rotFile:
		rotVal = rotFile.read()
	try:
		frotVal = int(rotVal)
	except ValueError:
		return 0
	#print frotVal
	return frotVal

def getNewTask():
	data = "FAILED"
	colo = WHTE
	splitchar = "^"
	ls = [0,0,0]
	ans = task(ls,data)

	with open("tasks.txt","r") as taskFile:
		firstline = taskFile.readline().strip()
	try:
		firstline = str(firstline)
	except ValueError:
		return ""
	data,colo = firstline.split("^")
	ls[0],ls[1],ls[2] = colo.split(" ")
	ans = task(ls,data)	
	return ans

def initNewTasks():
	n = 1
	new = getNewTask()
	added = False
	while n < len(tasks) and not added and new.getData() != tasks[n].getData():
		if tasks[n].getData() == "":
			tasks[n] = new
			added = True
		else:
			n+= 1

def loop():
	while True:
		#print "around"
		initNewTasks()
		index = getRot()
		if hasNewTasks(tasks):
			LCD.lcd_string("    New Task!   ",LCD_LINE_1)
			LCD.lcd_string("                ",LCD_LINE_2)
			RGB.flashN(WHTE)
			if getUS() < 40: #This just flashes when people get near
				for i in range(0,len(tasks)):
					RGB.flash(tasks[i].getColor())
				#subprocess.call(["espeak","New"])
			if not GPIO.input(SWPN): #Read through all the new tasks on click
				for i in range(1,len(tasks)):
					if tasks[i].isNewTask() and tasks[i].getData() != "":
						tasks[i].read()
				#stopall()
		while index > 0:
			index = getRot()
			tasks[index].printToLCD()
			RGB.colourM(tasks[index].getColor())
			if not GPIO.input(SWPN):
				tasks[index].read()
			
		
dogs = task(CYAN,"Take the dogs out")
dish = task(MAGE,"Do the dishes")
walk = task(GREN,"Go for a walk")
EMPTY = task(WHTE,"")

tasks = [EMPTY]*15
tasks[1] = dogs
tasks[2] = dish
tasks[3] = walk
newTask = True

def test(thing):
	for i in range(0,len(thing)):
		thing[i].read()
		time.sleep(2)

initNewTasks()	
#test(tasks)
loop()
stopall()
  
