#!/usr/bin/env python3
# encoding: utf-8
# 06_FaceLandmarks.py — Standalone: landmarks faciais com dlib (68 pontos)
# =========================================================================
# Deteta rostos com HOG+SVM do dlib e mapeia 68 landmarks faciais usando o
# modelo shape_predictor_68. Inclui métodos para extrair regiões (olhos,
# lábios, sobrancelhas) e aplicar efeitos visuais (prettify_face).
#
# NÃO é nó ROS2 — executa standalone com VideoCapture(0).
#
# AVISO: `prettify_face` referencia `landmarks` como variável global em vez
#   de usar `self` — só funciona porque `landmarks` é criado no bloco
#   `__main__`; falha se a classe for instanciada de outro módulo.
#
# Dependências: dlib, opencv-python, numpy
#   Requer ficheiro externo: ./file/shape_predictor_68_face_landmarks.dat
#   (não incluído no repositório — download separado de ~100 MB)
# Limitações:   dlib não está disponível por padrão no ROSMASTER X3;
#               instalação pode ser complexa na Jetson Nano (compilar de fonte).
# Relevância para robodog2: referência para landmarks faciais detalhados;
#   MediaPipe Face Mesh (04) é preferível na Jetson Nano por ser mais fácil
#   de instalar e mais rápido.
import time
import dlib
import cv2 as cv
import numpy as np

class FaceLandmarks:
    def __init__(self, dat_file):
        self.hog_face_detector = dlib.get_frontal_face_detector()
        self.dlib_facelandmark = dlib.shape_predictor(dat_file)

    def get_face(self, frame, draw=True):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        self.faces = self.hog_face_detector(gray)
        for face in self.faces:
            self.face_landmarks = self.dlib_facelandmark(gray, face)
            if draw:
                for n in range(68):
                    x = self.face_landmarks.part(n).x
                    y = self.face_landmarks.part(n).y
                    cv.circle(frame, (x, y), 2, (0, 255, 255), 2)
                    cv.putText(frame, str(n), (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def get_lmList(self, frame, p1, p2, draw=True):
        lmList = []
        if len(self.faces) != 0:
            for n in range(p1, p2):
                x = self.face_landmarks.part(n).x
                y = self.face_landmarks.part(n).y
                lmList.append([x, y])
                if draw:
                    next_point = n + 1
                    if n == p2 - 1: next_point = p1
                    x2 = self.face_landmarks.part(next_point).x
                    y2 = self.face_landmarks.part(next_point).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def get_lipList(self, frame, lipIndexlist, draw=True):
        lmList = []
        if len(self.faces) != 0:
            for n in range(len(lipIndexlist)):
                x = self.face_landmarks.part(lipIndexlist[n]).x
                y = self.face_landmarks.part(lipIndexlist[n]).y
                lmList.append([x, y])
                if draw:
                    next_point = n + 1
                    if n == len(lipIndexlist) - 1: next_point = 0
                    x2 = self.face_landmarks.part(lipIndexlist[next_point]).x
                    y2 = self.face_landmarks.part(lipIndexlist[next_point]).y
                    cv.line(frame, (x, y), (x2, y2), (0, 255, 0), 1)
        return lmList

    def prettify_face(self, frame, eye=True, lips=True, eyebrow=True, draw=True):
        if eye:
            leftEye = landmarks.get_lmList(frame, 36, 42)
            rightEye = landmarks.get_lmList(frame, 42, 48)
            if draw:
                if len(leftEye) != 0: frame = cv.fillConvexPoly(frame, np.mat(leftEye), (0, 0, 0))
                if len(rightEye) != 0: frame = cv.fillConvexPoly(frame, np.mat(rightEye), (0, 0, 0))
        if lips:
            lipIndexlistA = [51, 52, 53, 54, 64, 63, 62]
            lipIndexlistB = [48, 49, 50, 51, 62, 61, 60]
            lipsUpA = landmarks.get_lipList(frame, lipIndexlistA, draw=True)
            lipsUpB = landmarks.get_lipList(frame, lipIndexlistB, draw=True)
            lipIndexlistA = [57, 58, 59, 48, 67, 66]
            lipIndexlistB = [54, 55, 56, 57, 66, 65, 64]
            lipsDownA = landmarks.get_lipList(frame, lipIndexlistA, draw=True)
            lipsDownB = landmarks.get_lipList(frame, lipIndexlistB, draw=True)
            if draw:
                if len(lipsUpA) != 0: frame = cv.fillConvexPoly(frame, np.mat(lipsUpA), (249, 0, 226))
                if len(lipsUpB) != 0: frame = cv.fillConvexPoly(frame, np.mat(lipsUpB), (249, 0, 226))
                if len(lipsDownA) != 0: frame = cv.fillConvexPoly(frame, np.mat(lipsDownA), (249, 0, 226))
                if len(lipsDownB) != 0: frame = cv.fillConvexPoly(frame, np.mat(lipsDownB), (249, 0, 226))
        if eyebrow:
            lefteyebrow = landmarks.get_lmList(frame, 17, 22)
            righteyebrow = landmarks.get_lmList(frame, 22, 27)
            if draw:
                if len(lefteyebrow) != 0: frame = cv.fillConvexPoly(frame, np.mat(lefteyebrow), (255, 255, 255))
                if len(righteyebrow) != 0: frame = cv.fillConvexPoly(frame, np.mat(righteyebrow), (255, 255, 255))
        return frame


if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    pTime, cTime = 0, 0
    dat_file = "./file/shape_predictor_68_face_landmarks.dat"
    landmarks = FaceLandmarks(dat_file)
    while capture.isOpened():
        ret, frame = capture.read()
        # frame = cv.flip(frame, 1)
        frame = landmarks.get_face(frame, draw=False)
        frame = landmarks.prettify_face(frame, eye=True, lips=True, eyebrow=True, draw=True)
        if cv.waitKey(1) & 0xFF == ord('q'): break
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
