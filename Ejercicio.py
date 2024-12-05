#Aquí poneis el Docstring que querais
"""""
Este script tiene como objetivo controlar un brazo robótico mediante el uso de servomotores y un controlador PWM PCA9685,
el cual está conectado a una placa Jetson. El brazo cuenta con varios servos que permiten mover las 
articulaciones (hombro, codo, muñeca, base y pinza), y sensores de posición (potenciómetros) que facilitan el 
control de los servomotores. Tambiém hay un botón que permite controlar la apertura y cierre de la pinza del brazo robótico.

¿Cómo funciona?:
- El script hace una configuración los servos mediante el controlador, que gestiona 
 para controlar la posición de cada motor.
- El valor del potenciometro se convierte en pulso analógico para controlar el movimiento de los servos.
- Los potenciómetros se leen a través de los pines GPIO para ajustar el ángulo de los servos de 
las articulaciones (hombro, codo, muñeca, base).
- Un botón que está conectado al pin GPIO 33 nos permite controlar la pinza del brazo robótico. Cuando está pulsado, 
 la pinza se cierra y cuando está presionado, la garra se abre.

Funciones:
- moveMotor(controlIn, motorOut): Lee un valor del potenciometro conectado a un pin GPIO y ajusta el 
  movimiento según la posición de éste.
- El script entra en un bucle infinito donde se ajustan las posiciones continuamente 
  a los valores de los potenciómetros.

Usos:
- adafruit_pca9685: Comunicación con el controlador.
- adafruit_servokit: Gestión de servomotores a través de librería Adafruit.
- Jetson.GPIO: Control de pines GPIO en placa Jetson.
- time: Retrasos entre las acciones y configurarción del sistema.

Pulsos:
- MIN_PULSE_WIDTH: Pulso mínimo. 650 microsegundos.
- MAX_PULSE_WIDTH: Pulso máximo. 2350 microsegundos.
- FREQUENCY: En España. 50 Hz.

"""""

#import Wire 
#import Adafruit_PWMServoDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
from adafruit_servokit import ServoKit
import time 

#Declaro las variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50


#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); (Lenguaje C) 
pwm = adafruit_pca9685.PCA9685(i2c)
kit = ServoKit(channels=16)




#Configuro el SetUP
time.sleep(5)                           #<-- So I have time to get controller to starting position
pwm.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
            
hand = adafruit_motor.servo.Servo(0)   #Usamos entradas analógicas (pwm)
hand = adafruit_motor.servo.Servo(0)
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.begin()                            
pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)                   #Set Gripper to 90 degrees (Close Gripper)
GPIO.setup(33, GPIO.IN)                  #Tiene que tener un pin válido de la jetson


#Asignamos pines
int potWrist    = A3
int potElbow    = A2                    #Assign Potentiometers to pins on Arduino Uno
int potShoulder = A1
int potBase     = A0

int hand      = 11
int wrist     = 12
int elbow     = 13                      #Assign Motors to pins on Servo Driver Board
int shoulder  = 14
int base      = 15


def moveMotor(controlIn, motorOut):
     """""

     Aqui se hace una breve descripción de la función Movemotor.

     Argumentos:
        controlIn (int): El potenciómetro asigna un pin GPI
        motorOut (int): movMotor recibe la señal analógica (pwm) del pin de salida que escogimos.

      Returns: En función del valor que tenga el potenciómetro en ese momento, se reiniciará la posición.

     """""

    pulse_wide, pulse_width, potVal = -7
  
    #potVal = analogRead(controlIn); (Lenguaje C)                         #Read value of Potentiometer
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);    #Map Potentiometer position to Motor
  
    #pwm.setPWM(motorOut, 0, pulse_width); (Lenguaje C) 
    pwm = GPIO.PWM(motorOut, pulse widht)
 
while (True):
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)
    moveMotor(potShoulder, shoulder)    #Assign Motors to corresponding Potentiometers       
    moveMotor(potBase, base)
    pushButton = GPIO.input(33)
    if(pushButton == GPIO.LOW)

        pwm.setPWM(hand, 0, 180)
        print("Grab")
    else
        
        pwm.setPWM(hand, 0, 90)
        print("Release")
GPIO.cleanup()