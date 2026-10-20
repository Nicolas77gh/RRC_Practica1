'''
 Sistema Experto para el guiado de un robot
 Esta clase contendra el codigo creado por los alumnos de RyRDC para el control
 y guiado de un robot movil sobre un plano cartesiano

 Creado por: Diego Viejo
 el 11/09/2026
'''

import math

from objetivo import *

# --- Parametros de la dinamica del robot (robot.py) ---
VMAX = 3.0      # m/s
WMAX = 1.0      # rad/s
VACC = 1.0      # m/s^2

# --- Parametros de sintonia del sistema experto ---
LOOKAHEAD    = 7.0            # m  (distancia de prevision del pure-pursuit)
MARGEN_FRENO = 0.15           # m  (margen para no pasarse del punto final)
KP_GIRO      = WMAX / 30.0    # ganancia del giro proporcional
VEL_CURVA    = 1.6            # m/s (velocidad con error angular moderado)
VEL_GIRO     = 0.6            # m/s (velocidad con error angular grande)
UMBRAL_CURVA = 18.0           # grados
UMBRAL_GIRO  = 75.0           # grados


class ExpertSystem:
    def __init__(self) -> None:
        self.objetivoActual = None
        self.nombreAlumno = "Invalido"

    def setObjetivo(self, objetivo):
        self.objetivoActual = objetivo

    @staticmethod
    def _normaliza180(ang):
        return (ang + 180.0) % 360.0 - 180.0

    @staticmethod
    def _camino(obj):
        """Devuelve la polilinea (lista de puntos) que el robot debe seguir.
        Segmento: inicio->fin. Triangulo: inicio->medio'->fin, con el vertice
        medio desplazado hacia fuera para no entrar en el triangulo."""
        ix, iy = obj.getInicio()
        fx, fy = obj.getFin()
        if obj.getType() == 1:
            return [(ix, iy), (fx, fy)]

        mx, my = obj.getMedio()
        # desplazamos el vertice medio hacia el exterior del triangulo
        cx, cy = (ix + fx) * 0.5, (iy + fy) * 0.5
        vx, vy = mx - cx, my - cy
        n = math.hypot(vx, vy)
        if n < 1e-6:
            vx, vy, n = 0.0, 1.0, 1.0
        # distancia del vertice a la base = altura; nos separamos esa misma
        # cantidad de nuevo para salir del triangulo con holgura
        altura = n
        despx, despy = mx + (altura * vx / n), my + (altura * vy / n)
        return [(ix, iy), (despx, despy), (fx, fy)]

    @staticmethod
    def _puntoGuiado(camino, x, y, lookahead):
        """Pure pursuit: proyecta el robot sobre la polilinea y devuelve el
        punto del camino situado 'lookahead' metros por delante de la
        proyeccion. Avanza solo, no puede quedarse orbitando."""
        # longitud acumulada de cada tramo
        acumulada = 0.0
        mejor_s = 0.0
        mejor_dist = float('inf')
        for i in range(len(camino) - 1):
            ax, ay = camino[i]
            bx, by = camino[i + 1]
            dx, dy = bx - ax, by - ay
            L = math.hypot(dx, dy)
            if L < 1e-9:
                continue
            ux, uy = dx / L, dy / L
            t = (x - ax) * ux + (y - ay) * uy
            t = max(0.0, min(t, L))
            px, py = ax + t * ux, ay + t * uy
            d = math.hypot(x - px, y - py)
            if d < mejor_dist:
                mejor_dist = d
                mejor_s = acumulada + t
            acumulada += L

        objetivo_s = mejor_s + lookahead
        # localizamos el punto a esa distancia a lo largo de la polilinea
        acumulada = 0.0
        for i in range(len(camino) - 1):
            ax, ay = camino[i]
            bx, by = camino[i + 1]
            L = math.hypot(bx - ax, by - ay)
            if L < 1e-9:
                continue
            if acumulada + L >= objetivo_s:
                r = (objetivo_s - acumulada) / L
                return (ax + r * (bx - ax), ay + r * (by - ay))
            acumulada += L
        return camino[-1]

    def tomarDecision(self, poseRobot):
        if self.objetivoActual is None:
            return (0.0, 0.0)

        x, y, heading = poseRobot[0], poseRobot[1], poseRobot[2]
        obj = self.objetivoActual
        camino = self._camino(obj)

        fx, fy = obj.getFin()
        dFin = math.hypot(fx - x, fy - y)

        # cerca del final: apuntamos directos al punto final para rematar
        if dFin < LOOKAHEAD:
            tx, ty = fx, fy
        else:
            tx, ty = self._puntoGuiado(camino, x, y, LOOKAHEAD)

        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < 1e-3:
            return (0.0, 0.0)

        error = self._normaliza180(math.degrees(math.atan2(dy, dx)) - heading)
        ae = abs(error)

        # --- Reglas de velocidad lineal segun el error de orientacion ---
        if ae > UMBRAL_GIRO:
            V = 0.0
        elif ae > UMBRAL_CURVA:
            V = VEL_CURVA
        else:
            V = VMAX

        # --- Frenado ante el punto final ---
        vFreno = math.sqrt(2.0 * VACC * max(dFin - MARGEN_FRENO, 0.0))
        if V > vFreno:
            V = vFreno

        # --- Velocidad angular proporcional al error ---
        W = KP_GIRO * error
        if W > WMAX:
            W = WMAX
        elif W < -WMAX:
            W = -WMAX

        return (V, W)
