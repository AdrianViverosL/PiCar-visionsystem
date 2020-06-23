import picar
import cv2
from hand_coded_lane_follower2 import HandCodedLaneFollower

_SHOW_IMAGE = True

class DeepPiCar(object):
    __INITIAL_SPEED = 0
    __SCREEN_WIDTH = 320
    __SCREEN_HEIGHT = 240

    #Inicialización del carrito
    def __init__(self):
        picar.setup()                                            #Setup propio de la libreria del Kit
        #Se obtiene la cámara abierta por default y se le hace un rezise de width y height
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(3, self.__SCREEN_WIDTH)
        self.camera.set(4, self.__SCREEN_HEIGHT)

        #Se pueden omitir/no conectar, en caso de dejar fija la cámara en el soporte, caso de nuestro carrito
        #self.pan_servo = picar.Servo.Servo(1)
        #self.pan_servo.offset = -30  
        #self.pan_servo.write(90)

        #self.tilt_servo = picar.Servo.Servo(2)
        #self.tilt_servo.offset = 20
        #self.tilt_servo.write(90)

        self.back_wheels = picar.back_wheels.Back_Wheels()
        self.back_wheels.speed = 0                              # Rango de velocidad PWM  0 (stop) - 100 (fastest)

        self.front_wheels = picar.front_wheels.Front_Wheels()
        self.front_wheels.turning_offset = -25                  # Offset por default del archivo de configuración del kit
        self.front_wheels.turn(70)                              # Ángulo recto a 70° para nstrs, equivalencias 45(izq) 90(centro) 135 (drc)
        self.lane_follower = HandCodedLaneFollower(self)
        
    def cleanup(self):
        self.back_wheels.speed = 0                                        #Alto total del carrito
        self.front_wheels.turn(70)                                        #Regreso al centro
        self.camera.release()                                             #Liberación de la cámara y video
        cv2.destroyAllWindows()

    def drive(self, speed=__INITIAL_SPEED):
        self.back_wheels.speed = speed
        i = 0
        while self.camera.isOpened():
            _, image_lane = self.camera.read()
            i += 1
            image_lane = self.follow_lane(image_lane)
            show_image('Lane Lines', image_lane)                            #Muestra el frame final, feedback

            if cv2.waitKey(1) & 0xFF == ord('q'):                           #Condición de salida del carrito
                self.cleanup()                                              #Limpieza de todos los frames
                self.back_wheels.speed = 0                                  #Alto total del carrito
                break

    def follow_lane(self, image):
        image = self.lane_follower.follow_lane(image)
        return image

def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)

def main():
    with DeepPiCar() as car:
        car.drive(28)                                                      #Valor PWM del carrito

if __name__ == '__main__':
    main()