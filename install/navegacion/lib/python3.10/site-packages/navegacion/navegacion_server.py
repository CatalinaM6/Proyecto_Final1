import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import time
from functools import wraps
import rclpy
from rclpy.node import Node
from proyecto_interfaces.srv import StartNavigationTest
import serial,time
import cv2
import numpy as np
from queue import PriorityQueue

print('Ingrese la posición actual del robot en x:')
posx = int(input())

print('Ingrese la posición actual del robot en y:')
posy = int (input())

# Se inicia la posición inicial que llega por consola
pos_ini = (posx, posy)
global pos_ini

# Bandera para hacer ruta optima, porque llegaron las coordenadas
llego_coordenadas = False

class Navegacion(Node):
    def __init__(self):
        super().__init__('navio_server')
        self.srv = self.create_service(StartNavigationTest, '/group_'+str(3)+'/start_navigation_test_srv',self.callback)

    def callback(self, request, response):
        global pos_ini, llego_coordenadas
        # Cargar la imagen de PGM y convertirla en una cuadrícula
        grid = load_pgm_image("MapaRobotica.pgm")

        # Definir la posición final que le ingresa
        pos_final = (request.goal_x, request.goal_y) # (x, y)

        # Realizar la planificación de ruta
        path = path_planning(grid, pos_ini, pos_final)

        # Comprobar si se encontró una ruta
        if path:
            response.answer = "Ruta encontrada."
            response.path_x = [coordinate[1] for coordinate in path]
            response.path_y = [coordinate[0] for coordinate in path]
            llego_coordenadas = True
            
        else:
            response.answer = "No se encontró una ruta."

        self.get_logger().info(response.answer)
        return response
    
    def ruta_Optima(pos_final):

        global pos_ini
        if llego_coordenadas == True:

            d=23 # Span de pepe en centimetros
            r=10 # parametro de salto entre vecinos (pixeles)
    
            d=round(round(d*4)/2)  #mitad del span de pepe en pixeles
    
            gscore=np.full((len(mapa), len(mapa[0])), np.inf)
            fscore=np.full((len(mapa), len(mapa[0])), np.inf) 

            gscore[pos_ini[1]][pos_ini[0]]=0
            fscore[pos_ini[1]][pos_ini[0]]=self.h(pos_ini, pos_final)
    
            open=PriorityQueue()
            open.put((self.h(pos_ini, pos_final),self.h(pos_ini, pos_final),pos_ini))
            aPath={}

            while not open.empty():
        
                currPix=open.get()[2]
                    
                if currPix[0] in range(pos_final[0]-r,pos_final[0]+r) and currPix[1] in range(pos_final[1]-r,pos_final[1]+r):
                    posf=currPix
                    break
                    
                    
                #nodos vecindad
                izq=(currPix[0]-r,currPix[1])
                der=(currPix[0]+r,currPix[1])
                sup=(currPix[0],currPix[1]+r)
                inf=(currPix[0],currPix[1]-r)
                    
                vecinos=[izq,der,sup,inf]
                    
                for vecino in vecinos:
                        
                    #revision de celdas libres y tamano del Pepe
                    available=True
                    for i in range(vecino[1]-d,vecino[1]+d):
                        for j in range(vecino[0]-d,vecino[0]+d):
                            if mapa[i][j][0]==94:
                                available=False

                                
                    # analisis fscore y gscore
                    if available==True:
                        temp_gscore=gscore[currPix[1]][currPix[0]]+1
                        temp_fscore=self.h(vecino,pos_final)+temp_gscore
                        if temp_fscore<fscore[vecino[1]][vecino[0]]:
                            fscore[vecino[1]][vecino[0]]=temp_fscore
                            gscore[vecino[1]][vecino[0]]=temp_gscore
                            open.put((temp_fscore,self.h(vecino,pos_final),vecino))
                            aPath[vecino]=currPix

            #Se crea el camino
            cell = pos_final
            path=[cell]
            while cell!=pos_ini:
                path.append(aPath[cell])
                cell=aPath[cell]
            path.reverse()

            #Se imprime el camino
            print("El camino es: ")
            print(path)

            for val in path:
                mapa=self.draw(val,mapa,d)

            #Se muestra el mapa
            cv2.imshow("Mapa",mapa)

            #Simplificar el camino
            distancias = []
            distancia_actual = 0
            orientacion_actual = 0
            for i in range(len(path)-1):
                delta_x = path[i][0] - path[i-1][0]
                delta_y = path[i][1] - path[i-1][1]

                if delta_x > 0:
                    orientacion = 0
                elif delta_x < 0:
                    orientacion = 180
                elif delta_y < 0:
                    orientacion = 90
                elif delta_y > 0:
                    orientacion = -90

                if orientacion == orientacion_actual:
                    distancia_actual += math.sqrt(delta_x**2 + delta_y**2)
                else:
                    distancias.append((distancia_actual, orientacion_actual))
                    distancia_actual = math.sqrt(delta_x**2 + delta_y**2)
                    orientacion_actual = orientacion

            # Agregar la última distancia
            distancias.append((distancia_actual, orientacion_actual))

            #Hacer recorrer el camino
            self.movimiento_camino(distancias)


    def draw(self,pos,img,d):
        #conversion a pixeles
        d=round(round(d*4)/2)
        
        for i in range(pos[1]-d,pos[1]+d):
            for j in range(pos[0]-d,pos[0]+d):
                try:
                    img[i][j]=[0,0,0]
                except:
                    pass
        return img
    
    def h(self,pos0,posf): 
        x=abs(posf[0]-pos0[0])
        y=abs(posf[1]-pos0[1])
     
        return x+y
    

def main(args=None):
    rclpy.init(args=args)
    server = Navegacion()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
