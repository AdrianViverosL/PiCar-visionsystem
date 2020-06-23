import sys
import cv2
import math
import numpy as np

_SHOW_IMAGE = False

class HandCodedLaneFollower(object):

    def __init__(self, car=None):
        #Inicializacion del objeto Picar con angulo de "90°"
        self.car = car
        self.curr_steering_angle = 70                             #Ángulo de 70° debido a un problema fisico en uno de los servo-motores

    def follow_lane(self, frame):                                 #Dibuja las líneas en el frame a mostrar
        lane_lines, frame = detect_lane(frame)
        final_frame = self.steer(frame, lane_lines)

        return final_frame

    def steer(self, frame, lane_lines):                           #Da giro a las llantas
        if len(lane_lines) == 0:
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))

        if self.car is not None:
            self.car.front_wheels.turn(self.curr_steering_angle)
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        show_image("heading", curr_heading_image)

        return curr_heading_image

def show_image(title, frame, show=_SHOW_IMAGE):                       #Se muestra el video en una ventana
    if show:
        cv2.imshow(title, frame)

def average_slope_intercept(frame, line_segments):
    #Combinamos los segmentos de línea para formar las dos lineas del carril
    #Si todas las inclinaciones son mayor a 0, entonces solo hay una linea izquierda, si solo hay menor a 0 entonces solo esta la derecha
    lane_lines = []
    
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3                                                   #Consideramso 1/3 y no 1/2 por precisión y factores externos 
    left_region_boundary = width * (1 - boundary)  
    right_region_boundary = width * boundary

    #Para saber si son lineas izquierdas o derechas se observa su inclinacion
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    #Una vez identificadas todas las lineas, se obtiene un promedio para cada lado
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines                                                 #Se retornan las dos lineas encontradas del carril


def compute_steering_angle(frame, lane_lines):
    #Encontramos el angulo de giro basado en las dos lineas del carril (se asume que la camara se encuentra en el punto medio
    
    #No se detectaron lineas, regresamos el ángulo recto
    if len(lane_lines) == 0:
        return -70
    #Detección de solo una linea
    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    #Detección de las dos lineas
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        #Offset de la cámara 0.0 es el centro, -0.03 es a la izq, 0.03 es a la derecha
        camera_mid_offset_percent = 0.0
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid
        
    #Se encuntra el ángulo de dirección que corresponde al ángulo entre la dirección a la linea central
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)              # Ángulo en radianes a la linea central
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)     # Ángulo en grados
    steering_angle = angle_to_mid_deg + 70                            # Calculamos ángulo con respecto al centro
    
    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    if num_of_lane_lines == 2 :
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):  #Dibuja los segmentos de línea
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ): #Dibuja la linea guia
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    # (x1,y1) de la linea guia siempre esta fija abajo al centro; (x2, y2) se calcula con trigonometria
    # NOTA: (en nuestro caso, estos rangos no son validos debido a un problema mecanico con los servo-motores)
    # 0-89 grados es izquierda
    # 90 grados es ir recto 
    # 91-180 grados es derecha  
    steering_angle_radian = steering_angle / 180.0 * math.pi          #Conversion de grados a radianes
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height                                                        # parte baja del frame
    y2 = int(y1 * 1 / 2)                                               # solo coloca puntos de la mitad del frame para abajo

    # une las coordenadas al frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

#Procesameinto de lineas
def detect_lane(frame):
    edges = detect_edges(frame)                                    #Deteccion de los bordes con máscara de color y canny
    cropped_edges = region_of_interest(edges)                      #Definimos la región de interés de la mitad inferior
    line_segments = detect_line_segments(cropped_edges)            #Deteccion de segmentos de linea
    line_segment_image = display_lines(frame, line_segments)       #Mostrar segmentos de lineas
    lane_lines = average_slope_intercept(frame, line_segments)     #Calculo de obtencion de las lineas de carril
    lane_lines_image = display_lines(frame, lane_lines)            #Mostrar lineas de carril

    return lane_lines, lane_lines_image
    
def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)                   #Aplicamos la conversión BGR a HSV para facilitar procesos
    lower_blue = np.array([70, 40, 0])                             #Rangos bajos correspondientes al color azul, saturación e intensidad
    upper_blue = np.array([150, 255, 255])                         #Rangos altos correspondientes al color azul, saturación e intensidad
    mask = cv2.inRange(hsv, lower_blue, upper_blue)                #Hacemos la máscara con los colores 
    edges = cv2.Canny(mask, 200, 400)                              #Aplicamos función Canny con la máscara y los rangos inf/superior
    return edges

def region_of_interest(canny):
    height, width = canny.shape                                    #Obtenemos height y width de la ventana donde aplicamos Canny
    mask = np.zeros_like(canny)                                    
    #Rectángulo para cubrir la parte superior
    polygon = np.array([[(0, height * 1 / 2),(width, height * 1 / 2),(width, height),(0, height),]], np.int32)
    cv2.fillPoly(mask, polygon, 255)                               
    masked_image = cv2.bitwise_and(canny, mask)                    
    return masked_image

def detect_line_segments(cropped_edges):
    #Aplicamos la trasformada para obtener el máximo de lineas posibles de acuerdo con el resultado de agrupar los pixeles blancos
    #Parámetros de la Hough necesita coordenadas polares
    rho = 1                                                         #Distancia de pixel
    angle = np.pi / 180                                             #Ángulo de precisión en radianes
    min_threshold = 10                                              #Número de pixeles para ser considerado un segmento de linea
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)
        
    #Regresamos todas la lineas que se hayan podido consideras/existentes      
    return line_segments

if __name__ == '__main__':
    print("ok")