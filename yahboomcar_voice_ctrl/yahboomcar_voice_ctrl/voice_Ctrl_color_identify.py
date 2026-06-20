# voice_Ctrl_color_identify.py — Identificação interactiva de cor HSV com feedback de voz
# =========================================================================================
# Descrição: script STANDALONE (não é um nó ROS2 — sem rclpy). Captura vídeo da câmara USB,
#   permite ao utilizador seleccionar uma região de interesse (ROI) com o rato, calcula os
#   limites HSV da cor seleccionada e anuncia a cor identificada por voz.
#   Útil para calibrar manualmente os intervalos HSV usados nos nós de seguimento de linha.
#
# Fluxo de utilização:
#   1. Iniciar o script: python voice_Ctrl_color_identify.py
#   2. Clicar e arrastar na janela para seleccionar ROI com a cor desejada
#   3. O sistema calcula e exibe os valores HSV mínimos e máximos da ROI
#   4. Dizer o comando de voz "Identify" (código 60) para identificar a cor
#
# Comandos de voz suportados (código Speech_Lib → acção):
#   60     → "Identify" — identificar cor da ROI seleccionada e anunciar resultado:
#              H∈[0,179], H∈[23,56]                    → anuncia "yellow" (código 64)
#              H∈[56,…), S<200                          → anuncia "green"  (código 63)
#              H≥60, S>200                              → anuncia "blue"   (código 62)
#              H_min==0 e H_max==179 (vermelho/wrap)    → anuncia "red"    (código 61)
#
# Dependências: follow_common (color_follow — importado com *), Speech_Lib (proprietária Yahboom)
# Limitações: usa `from follow_common import *` sem prefixo de pacote — só funciona se
#   executado directamente na mesma directoria ou com follow_common no Python path.
#   Não integra com Nav2 nem publica tópicos ROS2.
# Relevância para robodog2: ferramenta de calibração HSV; a lógica de identificação de cor
#   por intervalo H pode ser reutilizada nos nós de detecção do robodog2.

import cv2 as cv
from Speech_Lib import Speech
from follow_common import *
import time
class Color_identify():
	def __init__(self):
		self.img = None
		self.hsv_range = ()
		self.Roi_init = ()
		self.cols, self.rows = 0, 0
		self.Mouse_XY = (0, 0)
		self.select_flags = False
		self.spe = Speech()
		self.windows_name = "frame"
		self.color = color_follow()
	def onMouse(self, event, x, y, flags, param):
		if event == 1:
			self.select_flags = True
			self.Mouse_XY = (x,y)
		if event == 4:
			self.select_flags = False
		if self.select_flags == True:
			self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
			self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
			self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
			print(self.Roi_init)
	def process(self,rgb_img):
		#self.spe.void_write(command1_result)
		H = [];
		S = [];
		V = [];
		cv.setMouseCallback(self.windows_name, self.onMouse, 0)
		if self.select_flags == True:
			cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
			cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
			if self.Roi_init[0]!=self.Roi_init[2] and self.Roi_init[1]!=self.Roi_init[3]:
				HSV = cv.cvtColor(rgb_img,cv.COLOR_BGR2HSV)
				for i in range(self.Roi_init[0], self.Roi_init[2]):
					for j in range(self.Roi_init[1], self.Roi_init[3]):
						H.append(HSV[j, i][0])
						S.append(HSV[j, i][1])
						V.append(HSV[j, i][2])
				H_min = min(H); H_max = max(H)
				S_min = min(S); S_max = 253
				V_min = min(V); V_max = 255
				#print("H_max: ",H_max)
				#print("H_min: ",H_min)        
				lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
				upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
				cv.putText(rgb_img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
				cv.putText(rgb_img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
				command_result = self.spe.speech_read()
				#self.spe.void_write(56)
				if command_result !=999:
					print(command_result)                
				if command_result == 60:
					if H_min == 0 and H_max == 179 : 
					#print("red")
						self.spe.void_write(61)
						print("red")
					elif H_min >= 23 and H_min <= 56:
						print("yellow")
						self.spe.void_write(64)
					elif H_min >= 56 and S_min < 200:
						print("green")
						self.spe.void_write(63)
					elif H_min >= 60 and S_min >200: 
						print("blue")
						self.spe.void_write(62)
		#command_result = self.spe.speech_read()
		#self.spe.void_write(command_result)
        

		return rgb_img
						
                    

if __name__ == '__main__':
	color_identify = Color_identify()
	#img = cv.imread('red.jpg')
	#cv.imshow("frame",img)
	capture = cv.VideoCapture(0)
	cv_edition = cv.__version__
	if cv_edition[0]=='3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
	else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
	capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
	capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
	cv.namedWindow("frame",cv.WINDOW_AUTOSIZE)

	while capture.isOpened():
		start = time.time()
		ret, frame = capture.read()
		#cv.imshow("frame", frame)
		action = cv.waitKey(10) & 0xFF
		rgb_img = color_identify.process(frame)
		cv.imshow("frame", rgb_img)
		if action == ord('q') or action == 113: break
	capture.release()
	cv.destroyAllWindows()
	'''while True :
		
		color_identify.process(img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv.destroyAllWindows()'''























		

