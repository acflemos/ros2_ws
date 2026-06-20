#!/usr/bin/env python3
# encoding: utf-8
# 08_Objectron.py — Standalone: deteção 3D de objetos com MediaPipe Objectron
# ============================================================================
# Deteta e rastreia objetos 3D em tempo real usando o modelo Objectron do
# MediaPipe. Suporta 4 categorias: Shoe, Chair, Cup, Camera. Alterna entre
# modelos com a tecla 'F'.
#
# ATENÇÃO: MediaPipe Objectron foi DESCONTINUADO em março de 2023.
#   A solução foi removida das versões recentes do mediapipe (>= 0.10).
#   Este código NÃO funciona com mediapipe atual — requer versão antiga (~0.8.x).
#
# NÃO é nó ROS2 — executa standalone com VideoCapture(0).
#
# Dependências: mediapipe (versão antiga ≤ 0.9.x), opencv-python
# Limitações:   API descontinuada — não usar em projetos novos.
#               Resolução configurada para 1080x720 (exige câmara compatível).
# Relevância para robodog2: código de referência histórico; para deteção 3D
#   de objetos na Jetson Nano considerar YOLOv8 ou MediaPipe Object Detection
#   (substituto moderno do Objectron).
import mediapipe as mp
import cv2 as cv
import time


class Objectron:
    def __init__(self, staticMode=False, maxObjects=5, minDetectionCon=0.5, minTrackingCon=0.99):
        self.staticMode=staticMode
        self.maxObjects=maxObjects
        self.minDetectionCon=minDetectionCon
        self.minTrackingCon=minTrackingCon
        self.index=3
        self.modelNames = ['Shoe', 'Chair', 'Cup', 'Camera']
        self.mpObjectron = mp.solutions.objectron
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

    def findObjectron(self, frame):
        cv.putText(frame, self.modelNames[self.index], (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for id, detection in enumerate(results.detected_objects):
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d, self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index>=4:self.index=0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

if __name__ == '__main__':
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 1080)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    pTime = cTime = 0
    objectron = Objectron()
    while capture.isOpened():
        ret, frame = capture.read()
        # frame = cv.flip(frame, 1)
        action = cv.waitKey(1) & 0xFF
        if action == ord('q'): break
        if action == ord('f') or action == ord('F') : objectron.configUP()
        frame = objectron.findObjectron(frame)
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv.imshow('frame', frame)
    capture.release()
    cv.destroyAllWindows()
