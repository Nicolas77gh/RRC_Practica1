'''
 Sistema Experto para el guiado de un robot
 Esta clase contendrá el código creado por los alumnos de RyRDC para el control 
 y guiado de un robot móvil sobre un plano cartesiano

 Creado por: Diego Viejo
 el 11/09/2026


'''

from objetivo import *

class ExpertSystem:
    def __init__(self) -> None:
        self.objetivoActual = None
        self.nombreAlumno = "Invalido" #IMPORTANTE: Cambia el valor de esta propiedad por tu nombre completo

    # función setObjetivo
    #   Almacena en la propiedad objetivoActual el objetivo al que tiene que moverse el robot
    def setObjetivo(self, objetivo):
        self.objetivoActual = objetivo


    # función tomarDecision. 
    #   Recibe una tupla de 3 valores con la pose del robot: posición X, posición Y, orientación
    #   
    #   Devuelve una tupla con la velocidad lineal y angular que se
    #   quiere dar al robot
    def tomarDecision(self, poseRobot):
        # código del sistema experto. A completar por la alumna o alumno

        return (3, 0.25)
    

