'''
 Sistema Experto Difuso para el guiado de un robot
 Esta clase contendrá el código creado por los alumnos de RyRDC para el control 
 y guiado de un robot móvil sobre un plano cartesiano para recorrer diferentes 
 objetivos utilizando un esquema de sistema experto difuso

 Creado por: Diego Viejo
 el 24/10/2024
 Modificado por: Diego Viejo. 

'''

import numpy as np
import math

from fuzzy_expert.variable import FuzzyVariable
from fuzzy_expert.rule import FuzzyRule
from fuzzy_expert.inference import DecompositionalInference

from objetivo import *

class FuzzySystem:
    def __init__(self) -> None:
        self.objetivoActual = None
        #Completar definiendo las variables difusas y las reglas difusas del sistema experto.
        #Construcción del modelo difuso

    # función setObjetivo
    #   Especifica un objetivo que debe ser recorrido por el robot
    def setObjetivo(self, obj):
        self.objetivoActual = obj

    def tomarDecision(self, poseRobot):
        V = W = 0
        #uso del modelo difuso para objeter V y W
        return (V, W)
    
