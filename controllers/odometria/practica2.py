from controller import Robot
from controller import Camera
from controller import DistanceSensor
import time
import threading
from threading import Timer
import numpy as np
import cv2


# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Maximun velocidad para este experimento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 16
NUMBER_OF_ULTRASONIC_SENSORS = 5
NUMBER_OF_IR_SENSORS = 8

#Distancia a la pared
DISTANCIA_MIN = 150
DISTANCIA_STUCK = 200

RADIOUS_WHEEL = 0.21 
SPACE_BETWEEN_WHEELS = 0.1054
RADIOUS_BETWEEN_WHEELS = SPACE_BETWEEN_WHEELS / 2
SIZE_SQUARE = 0.25*.25

SQUARE_LENGTH = 0.25


IMAGE_LIST = []
    
# Nombres de los sensores de distancia
# Infrarojos
infrared_sensor_name = ["rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor"]

ultrasonic_sensor_name = ["left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"]


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.
    """

    robot = Robot()
    
    # Obtener dispositivos correspondientes a las ruedas.
    leftWheel = robot.getDevice('left wheel motor')
    rightWheel = robot.getDevice('right wheel motor')
    
    encoderL = robot.getDevice('left wheel sensor')
    encoderR = robot.getDevice('right wheel sensor')
    
    encoderL.enable(timeStep)
    encoderR.enable(timeStep)
        
    infrared_sensor = []
    for i in range(len(infrared_sensor_name)):
        infrared_sensor.append(robot.getDevice(infrared_sensor_name[i]))
        infrared_sensor[i].enable(timeStep)

    ultrasonic_sensor = []
    for i in range(len(ultrasonic_sensor_name)):
        ultrasonic_sensor.append(robot.getDevice(ultrasonic_sensor_name[i]))
        ultrasonic_sensor[i].enable(timeStep)


    # Utilizamos velocidad, establecemos posición a infinito.
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener y activar el dispositivo de la cámara    
    camera = robot.getDevice('camera')
    camera.enable(timeStep*10)

    # Activar otros sensores necesarios
    # ...
        
    return robot, camera, leftWheel, rightWheel, infrared_sensor, ultrasonic_sensor, encoderL, encoderR


def mse(imageA, imageB):
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageB.shape[1])

    return err


def process_and_compare_image():
    
    W = camera.getWidth()
    H = camera.getHeight() 

    image = np.frombuffer(cameraData, np.uint8).reshape((H, W, 4))
    IMAGE_LIST.append(image)
    timer = threading.Timer(2, process_and_compare_image)
    timer.start()

    if len(IMAGE_LIST) > 1:
        if mse(image, IMAGE_LIST[len(IMAGE_LIST) - 2]) < 5.0:
            params["stuck"] = True
            return params["stuck"]
        else:
            params["stuck"] = False
            return params["stuck"]

               


def process_image():
    """
    Procesamiento de imagen a partir del último frame capturado en el hilo principal.
    - Se libera al hilo prinipal del procesamiento de los frames para no ralentizar la simulación.
    - Para eviar que se liberen los datos del buffer del frame en medio del procesamiento es 
      necesario hacer una copia del mismo (utilizamos Numpy para mayor eficiencia).
    """      
    W = camera.getWidth()
    H = camera.getHeight()  
    offset = CRUISE_SPEED
    # Creamos una copia del frame actual de la cámara utilizando Numpy.
    image = np.frombuffer(cameraData, np.uint8).reshape((H, W, 4))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)      
    img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img_hsv, np.array([40, 40,40]), np.array([70, 255,255]))

    #Buscamos contornos de objetos verdes
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not (len(contours) == 0):
        contour = max(contours, key=cv2.contourArea)
        contour_center = cv2.moments(contour)
        #Para pillar el centro en X
        center = int(contour_center['m10'] / contour_center['m00'])
        offset = W / 2 - center

    return offset

def move_one_square(arg):
    global left_wheel_vel1
    global right_wheel_vel1
    global start_time
    global duration_side
    print("Move one square")
    while not arg["stop"]:
        posL = encoderL.getValue()
        posR = encoderR.getValue()
        increment = SQUARE_LENGTH/RADIOUS_WHEEL
        left_vel = CRUISE_SPEED
        right_vel = CRUISE_SPEED
        leftWheel.setPosition(posL + increment)
        rightWheel.setPosition(posR + increment)
        linear_velocity = RADIOUS_WHEEL*CRUISE_SPEED
    
        left_wheel_vel1 = left_vel
        right_wheel_vel1 = right_vel
        duration_side = SQUARE_LENGTH/linear_velocity
        start_time = robot.getTime()

#  Ir recto, en espiral o buscar pared más cercana. Sin sensores o usando sensores de distancia (infrarrojos y/o ultrasonidos).
def behaviour01(arg):
    global left_wheel_vel1
    global right_wheel_vel1
    print("Thread1")
    while not arg["stop"]:
        if arg["forward"]:
            time.sleep(0.1)
            left_wheel_vel1 = CRUISE_SPEED
            right_wheel_vel1 = CRUISE_SPEED
            if infrared_sensor[3].getValue() > DISTANCIA_MIN:
                arg["forward"] = False
                left_wheel_vel1 = 0
                right_wheel_vel1 = 0


def behaviour02(arg):
    global left_wheel_vel2
    global right_wheel_vel2
    print("Thread2")
    
    while not arg["stop"]:
        #print("while Thread2")
        #time.sleep(1)
        left_wall = infrared_sensor[1].getValue() > DISTANCIA_MIN
        left_front_wall = infrared_sensor[2].getValue() > DISTANCIA_MIN
        front_wall = infrared_sensor[3].getValue() > DISTANCIA_MIN
        
        if front_wall:
            #print("Giramos a la derecha quietos")
            right_vel = -MAX_SPEED
            left_vel = MAX_SPEED
        else:
            if left_wall:
                #print("Vamos recto")
                right_vel = CRUISE_SPEED
                left_vel = CRUISE_SPEED
            else:
                #print("Giramos a la izquierda")
                right_vel = CRUISE_SPEED
                left_vel = CRUISE_SPEED/3
            if left_front_wall:
                #print("Giro a la derecha")
                right_vel = CRUISE_SPEED/2
                left_vel = CRUISE_SPEED

        left_wheel_vel2 = left_vel
        right_wheel_vel2 = right_vel


def behaviour03(arg):
    global left_wheel_vel3
    global right_wheel_vel3

    object_detected = False   
    print("Thread3")

    while not arg["stop"]:
        front_wall = infrared_sensor[3].getValue() > DISTANCIA_STUCK
        left_wall = infrared_sensor[1].getValue() > DISTANCIA_MIN
        
        offset = process_image()
        
        if (offset != CRUISE_SPEED) and (not left_wall) and (not front_wall):
            arg["green_object"] = True
            object_detected = True
            left_wheel_vel3 = (-offset) * 0.05 + CRUISE_SPEED/3
            right_wheel_vel3 = (offset) * 0.05 + CRUISE_SPEED/3

        else:
            if object_detected and (not left_wall):
                params["stop"] = True
                left_wheel_vel3 = 0
                right_wheel_vel3 = 0
            else:
                arg["green_object"] = False

def behaviour04(arg):
    global left_wheel_vel4
    global right_wheel_vel4
    print("Thread4")

    while not arg["stop"]:
        if arg["stuck"]:
            left_wheel_vel4 = -CRUISE_SPEED/2
            right_wheel_vel4 = -CRUISE_SPEED

    
    """
    HILO PRINCIPAL
    - Se inicializan los comportamientos como threads.
    - Se ejecuta el bucle de sincronización de la simulación.
    - La API de Webots es "Thread safe". 
    - ¡ATENCIÓN! Puede ser necesario sincronizar acceso concurrente a variables con bloqueos.
    """

# Activamos los dispositivos necesarios y obtenemos referencias a ellos.
robot, camera, leftWheel, rightWheel, infrared_sensor, ultrasonic_sensor, encoderL, encoderR = init_devices(TIME_STEP)

# Parámetros para las tareas que se ejecutan en threads.
# "stop" controla el final de la ejecución de la tarea.
params = {"stop": False, "forward":True, "if_following_wall":False, "green_object":False, "stuck":False}

# Ejecutamos una sincronización para poder cargar el primer frame de la cámara.
robot.step(TIME_STEP)
cameraData = camera.getImage()
params["stuck"] = process_and_compare_image()

left_wheel_vel1 = 0
right_wheel_vel1 = 0
 
left_wheel_vel2 = 0
right_wheel_vel2 = 0

left_wheel_vel3 = 0
right_wheel_vel3 = 0

left_wheel_vel4 = 0
right_wheel_vel4 = 0

start_time = 0
duration_side = 0


# Creamos la lista de threads a ejecutar
threads = []

# Ejecutamos los threads de las tareas
#thread01 = threading.Thread(target=behaviour01, args=(params, ))
#thread01.start()
#threads.append(thread01)
#thread02 = threading.Thread(target=behaviour02, args=(params, ))
#thread02.start()
#threads.append(thread02)
#thread03 = threading.Thread(target=behaviour03, args=(params, ))
#thread03.start()
#threads.append(thread03)
#thread04 = threading.Thread(target=behaviour04, args=(params, ))
#thread04.start()
#threads.append(thread04)

# Bucle de sincronización en el hilo princiapl.
while robot.step(TIME_STEP) != -1:
    
    if params["stuck"]:
        leftWheel.setVelocity(left_wheel_vel4)
        rightWheel.setVelocity(right_wheel_vel4) 
    elif params["green_object"]:
        leftWheel.setVelocity(left_wheel_vel3)
        rightWheel.setVelocity(right_wheel_vel3)
    elif not params["forward"]:
        leftWheel.setVelocity(left_wheel_vel2)
        rightWheel.setVelocity(right_wheel_vel2)
    else:
        leftWheel.setVelocity(left_wheel_vel1)
        rightWheel.setVelocity(right_wheel_vel1)
        
    move_one_square(params)
        
    #start_time = robot.getTime()    
    #move_one_square()
    current_time = robot.getTime()
    if (current_time > start_time + duration_side):
        print(current_time)
        print(start_time + duration_side)
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(0)
        #move_one_square()

    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)


    # Si vamos a utilizar la cámara actualizamos el frame actual en la variable global.
    cameraData = camera.getImage()
    # Sleep (en segundos)
    time.sleep(0.01)

# Actualización de "params" para que termine ejecución tareas threads. 
params["stop"] = True
for thread in threads:
    thread.join()