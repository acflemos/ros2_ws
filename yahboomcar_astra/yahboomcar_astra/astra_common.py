#!/usr/bin/env python
# encoding: utf-8
# Utilitários partilhados pelos pacotes yahboomcar_astra e yahboomcar_linefollow.
# Contém: I/O de calibração HSV, deteção de objetos por cor, PID discreto, visualização.

import time
import cv2 as cv
import numpy as np


def write_HSV(wf_path, value):
    """Persiste range HSV em ficheiro no formato: Hmin,Smin,Vmin,Hmax,Smax,Vmax"""
    with open(wf_path, "w") as wf:
        wf_str = "{}, {}, {}, {}, {}, {}".format(
            value[0][0], value[0][1], value[0][2],
            value[1][0], value[1][1], value[1][2])
        wf.write(wf_str)
        wf.flush()


def read_HSV(rf_path):
    """Lê range HSV do ficheiro. Retorna ((Hmin,Smin,Vmin),(Hmax,Smax,Vmax)) ou () se inválido."""
    with open(rf_path, "r") as rf:
        line = rf.readline()
    if len(line) == 0:
        return ()
    parts = line.split(',')
    if len(parts) != 6:
        return ()
    return ((int(parts[0]), int(parts[1]), int(parts[2])),
            (int(parts[3]), int(parts[4]), int(parts[5])))


def ManyImgs(scale, imgarray):
    """Concatena múltiplas imagens numa grelha para visualização com cv.imshow.
    Se imgarray é lista de listas: grelha 2D (vstack de hstacks).
    Se imgarray é lista plana: uma linha horizontal."""
    rows = len(imgarray)
    cols = len(imgarray[0])
    rowsAvailable = isinstance(imgarray[0], list)
    width  = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    if rowsAvailable:
        for x in range(rows):
            for y in range(cols):
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y],
                                               (imgarray[0][0].shape[1], imgarray[0][0].shape[0]),
                                               None, scale, scale)
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows
        for x in range(rows):
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)
    else:
        for x in range(rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x],
                                        (imgarray[0].shape[1], imgarray[0].shape[0]),
                                        None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        hor = np.hstack(imgarray)
        ver = hor
    return ver


class color_follow:
    """Deteção de objetos por cor HSV com contornos OpenCV."""

    def __init__(self):
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def object_follow(self, img, hsv_msg):
        """Deteta o maior objeto da cor definida em hsv_msg.
        Retorna: (imagem com círculo, máscara binária, (cx, cy, raio)).
        Raio=0 se não detetado."""
        src = img.copy()
        src = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        mask = cv.inRange(src, lower, upper)
        color_mask = cv.bitwise_and(src, src, mask=mask)
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # Fechar buracos pequenos na máscara (ruído de quantização)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        _, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)

        # Compatibilidade OpenCV 3 (devolve 3 valores) e OpenCV 4 (devolve 2)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = find_contours[1] if len(find_contours) == 3 else find_contours[0]

        if len(contours) != 0:
            areas = [cv.contourArea(c) for c in contours]
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = np.int0(cv.boxPoints(max_rect))
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(img, (self.Center_x, self.Center_y), self.Center_r, (255, 0, 255), 2)
            cv.circle(img, (self.Center_x, self.Center_y), 2, (0, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return img, binary, (self.Center_x, self.Center_y, self.Center_r)

    def line_follow(self, img, hsv_msg):
        """Igual a object_follow — alias usado pelo pacote linefollow."""
        return self.object_follow(img, hsv_msg)

    def Roi_hsv(self, img, Roi):
        """Aprende o range HSV de uma ROI definida pelo utilizador.
        Roi = (x_min, y_min, x_max, y_max).
        Retorna (imagem com texto de feedback, ((Hmin,Smin,Vmin),(Hmax,Smax,Vmax)))."""
        H, S, V = [], [], []
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        H_min, H_max = min(H), max(H)
        S_min = min(S)
        V_min = min(V)
        # S_max e V_max fixos — máximos da gama OpenCV HSV
        S_max = 253
        V_max = 255
        # Margens de tolerância para variações de iluminação
        H_max = min(255, H_max + 5)
        H_min = max(0,   H_min - 5)
        S_min = max(0,   S_min - 20)
        V_min = max(0,   V_min - 20)
        # Feedback visual: "Learning..." se S/V muito baixo (iluminação insuficiente)
        if S_min < 5 or V_min < 5:
            cv.putText(img, 'Learning ...', (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, 'OK !!!', (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        lowerb = 'lowerb : ({} ,{} ,{})'.format(H_min, S_min, V_min)
        upperb = 'upperb : ({} ,{} ,{})'.format(H_max, S_max, V_max)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID:
    """Controlador PID discreto sem escala de tempo (dt implícito na freq. de chamada).
    NOTA: o acumulador integral cresce sem limites — pode causar windup em erros persistentes."""

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0
        self.intergral = 0    # typo do original mantido para compatibilidade
        self.derivative = 0
        self.prevError = 0

    def compute(self, target, current):
        """Calcula output PID: Kp*e + Ki*∫e + Kd*de/dt (sem dt explícito)."""
        error = target - current
        self.intergral += error
        self.derivative = error - self.prevError
        self.targetpoint = self.kp * error + self.ki * self.intergral + self.kd * self.derivative
        self.prevError = error
        return self.targetpoint

    # Alias vetorial usado pelo linefollow (recebe lista, retorna lista)
    def update(self, current_list):
        """Versão vetorial: recebe lista de erros, retorna lista de outputs."""
        if not isinstance(self.kp, list):
            # modo escalar — envolve em lista
            return [self.compute(0, current_list[0])], [0]
        outputs = []
        for i, cur in enumerate(current_list):
            error = -cur  # target implícito = 0
            self.intergral += error
            self.derivative = error - self.prevError
            out = self.kp[i] * error + self.ki[i] * self.intergral + self.kd[i] * self.derivative
            self.prevError = error
            outputs.append(out)
        return outputs

    def reset(self):
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
