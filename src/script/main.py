#!/usr/bin/env python3

import rospy
from std_msgs.msg import String , Bool

import cv2
import numpy as np
from operator import itemgetter
import ultralytics
from ultralytics import YOLO
import supervision as sv

import chess
import chess.engine
import chess.svg
import cairosvg

import time

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from gui import *

import sys
from threading import *

import queue
q = queue.Queue()



model = YOLO("src/python_api/used_data/best(2).pt")
box_annotator = sv.BoxAnnotator(
   thickness=2,
   text_thickness=2,
   text_scale=1
)

ZONE_POLYGON = np.array([
    [0, 0],
    [0.5, 0],
    [0.5, 1],
    [0, 1]
])

zone_polygon = (ZONE_POLYGON * np.array([720,540])).astype(int)
zone = sv.PolygonZone(polygon=zone_polygon, frame_resolution_wh=tuple([720, 540720]))



class VideoPlayerThread(QThread):

    pix_sig = pyqtSignal(int,QPixmap)

    def __init__(self) :  #farmers code change str to dirc 
        super().__init__()

    def run(self) -> None:
        self.__thread_active = True
        
        capture = cv2.VideoCapture('http://10.42.0.184:8080/video') #for debug
        qimg: QImage
            # while self.__thread_active and capture.isOpened():
        while self.__thread_active and True:
                ret, frame = capture.read()
                ret = True
                if ret:
                    # frame =  cv2.imread('/home/omar/chess-engine/chess/IMG_20230415_002011.jpg')
                    cv2.resize(frame,(640,640))
                    result = model(frame, agnostic_nms=True)[0]
                    detections = sv.Detections.from_yolov8(result)
                    q.put(detections)

                    labels = [f"{model.model.names[class_id]} {confidence:0.2f}"
                        for  _,confidence, class_id,_,
                        in detections
                    ]
                    frame = box_annotator.annotate(
                        scene=frame, 
                        detections=detections, 
                        labels=labels
                    )
                    

                    zone.trigger(detections=detections)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = cv2.resize(frame,(740,420)) 
                    qimg = QImage(frame.data, frame.shape[1], frame.shape[0],
                                QImage.Format_RGB888)
                    self.pix_sig.emit(0,QPixmap.fromImage(qimg))
                    self.msleep(1000 // 30) # Lower CPU than cv2.waitKey (WTF!!)
                else:
                    self.pix_sig.emit(1,QPixmap.fromImage(qimg))
        self.pix_sig.emit(1,QPixmap.fromImage(qimg))
       


    def stop(self):
        self.__thread_active = False
        self.quit()



# # for chess board conrer detections 
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# start_board = [
#     [
#             ["b", "b", "b", "b", "b", "b", "b", "b"],
#             ["p", "p", "p", "p", "p", "p", "p", "p"],
#             [" ", " ", " ", " ", " ", " ", " ", " "],
#             [" ", " ", " ", " ", " ", " ", " ", " "],
#             [" ", " ", " ", " ", " ", " ", " ", " "],
#             [" ", " ", " ", " ", " ", " ", " ", " "],
#             ["P", "P", "P", "P", "P", "P", "P", "P"],
#             ["R", "N", "B", "Q", "K", "B", "N", "R"]]
# ]

# def check_the_move(boaed,boardnew):
#     list_of_errors = []
#     if boaed == boardnew:
#         return True , list_of_errors
#     else:
#         for i, (j, k) in enumerate(zip(boaed, boardnew)):
#             if k != j:
#                 list_of_errors.append((i,k,j))
#         return False, list_of_errors

# first_check = True


# def board_check(first_check,board):
#     if first_check:    
#         first_check = False
#         if board != start_board:
#             print("Error in board")
    
#     else:
#          if board != start_board:
#             time.sleep(1.5)


# def reset_game():
#     pass

# engine = chessEngineSF()
# reset = False
# human_move = True

# start_board = 

def start():
    print("start")




class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.show()
        self.move_publisher = rospy.Publisher('/chess_rob/python_api/move', String, queue_size=10)
        rospy.init_node('move_talker_listener', anonymous=True)
        # rospy.spin()
        
        self.video_player_thread = VideoPlayerThread()

        self.engine = chess.engine.SimpleEngine.popen_uci("src/python_api/used_data/stockfish-ubuntu-20.04-x86-64")
        self.engine_board = chess.Board(fen=None)
        self.hardness_level = 1
        self.engine_board.turn = chess.BLACK
        
        self.ui.cameraFeed.setText('No Signal')
        self.cap = cv2.VideoCapture('http://10.42.0.184:8080/video')
        self.Dimensions = (7,7)
        self.pices =  ['b', 'k', 'n', 'p', 'q', 'r', 'B', 'K', 'N', 'P', 'Q', 'R']

        self.ui.human_color.setStyleSheet("background-color: rgb(255, 255, 255);color: rgb(255, 255, 255)")

        self.start = False
        self.human_turn = True
        self.calibrated = False
        self.done = False
        self.pass_check = True

        self.ui.start_b.clicked.connect(lambda: self.thread())
        self.ui.Human_b.clicked.connect(lambda: self.human_button())
        self.ui.cali_check.clicked.connect(lambda: self.calibrate_check())
        self.ui.camera_start.clicked.connect(lambda: self.start_camera_worker())
        self.ui.camera_stop.clicked.connect(lambda: self.stop_camera_worker())
        self.ui.slider_Scale.valueChanged.connect(lambda: self.slider_change())
        self.ui.human_color.clicked.connect(lambda: self.color_change())
        self.pixmap = QPixmap()

    def main_script(self):
        while(True):
            if (self.start==False):
                self.t1._delete()
                break
            _, self.frame = self.cap.read()

            # self.frame = cv2.imread('/home/omar/chess-engine/chess/IMG_20230415_002011.jpg')
            if not self.calibrated:
                self.calibration()

            self.first_check()

            if self.human_turn:
                print("Human turn")
                self.ui.Human_b.setStyleSheet(
                "background-color: green;color: rgb(255, 255, 255)")
                time.sleep(1)
            else:
                
                self.ui.Human_b.setStyleSheet(
                "background-color: rgb(116, 0, 0);color: rgb(255, 255, 255)")
                print("AI turn")
                try:
                    self.read_board()
                except:
                    pass
                try:
                    self.board_converting()
                except:
                    pass
                try:
                    self.predict_move()
                except:
                    self.result = "a7a5"

                try:
                    self.move_executing()
                except:
                    pass
                try:
                    self.read_board()
                except:
                    pass
                try:
                    self.board_converting()
                except:
                    pass

                self.check()
                if not self.pass_check:
                    pass
                else:
                    self.human_turn= True


    def human_button(self):
        self.human_turn = not  self.human_turn
    
    def calibrate_check(self):
        self.calibrated = not self.calibrated
    
    def slider_change(self):
        self.hardness_level = self.ui.slider_Scale.value()

    def color_change(self):
        self.ui.human_color.setStyleSheet("background-color: rgb(0, 0, 0);color: rgb(255, 255, 255)")
        self.engine_board.turn= not self.engine_board.turn
        self.human_turn = False


    def thread(self):
        self.start = not self.start
        if self.start:
            self.t1=Thread(target=self.main_script)
            self.t1.start()

    def calibration(self):
      while (True):
        _, frame = self.cap.read()
        #   frame = cv2.imread("/home/omar/chess-engine/chess/IMG_20230415_001916.jpg")
        grayImage = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret, self.corners = cv2.findChessboardCorners(grayImage, self.Dimensions , None)
        print(ret)
        if ret == True:
            image = cv2.drawChessboardCorners(frame, self.Dimensions, self.corners, ret)
            image = cv2.resize(image,(1000,720))
            cv2.imshow("calibration frame",image)
            if (cv2.waitKey(0) == 27):
                cv2.destroyWindow("calibration frame")
                self.calibrated = True
                self.ui.cali_check.setChecked(True)
                self.ui.cali_check.setEnabled(False)
                self.board_prep()
                break

    def start_camera_worker(self):
        self.video_player_thread.start()
        self.video_player_thread.pix_sig.connect(lambda ret, qpix: self.ui.cameraFeed.setPixmap(
            qpix) if ret == 0 else self.ui.cameraFeed.setText("Signal Stoped"))
    def stop_camera_worker(self):
        self.video_player_thread.stop()

    def first_check(self):
        pass             
    
    def board_prep(self):
        self.square_size = int(np.linalg.norm(self.corners[0] - self.corners[1]))
        points = [
                0,2,4,6,
                14,16,18,20,
                28,30,32,34,
                42,44,46,48
            ]

        self.squares = []

        for i in points:
                x , y = (int(self.corners[i][0][0]),int(self.corners[i][0][1]))
                self.squares.append((y-self.square_size,x-self.square_size))
                self.squares.append((y-self.square_size,x))
                self.squares.append((y ,x-self.square_size))            
                self.squares.append((y,x))
    
        self.squares = sorted( self.squares, key=itemgetter(0))

        self.squares[36],  self.squares[34],  self.squares[37] ,  self.squares[35] =  self.squares[34],  self.squares[36] , self.squares[35] ,  self.squares[37] 
        self.squares[38],  self.squares[36],  self.squares[39] ,  self.squares[37] =  self.squares[36],  self.squares[38] , self.squares[37] ,  self.squares[39]

        self.squares[44],  self.squares[42],  self.squares[45] ,  self.squares[43] =  self.squares[42],  self.squares[44] , self.squares[43] ,  self.squares[45] 
        self.squares[44],  self.squares[46],  self.squares[47] ,  self.squares[45] =  self.squares[46],  self.squares[44] , self.squares[45] ,  self.squares[47]


    def read_board(self):
        self.boardn = np.array([])
        detections = q.get()
        k = 0
        for square in self.squares:
            k = k +1
            l = " "
            for i in range(len(detections.xyxy)):
                # print(i)
                x , y = ((int((detections.xyxy[i][2]+detections.xyxy[i][0])/2),int((detections.xyxy[i][3]+detections.xyxy[i][1])/2)))

                if square[1] < x < square[1] + self.square_size and square[0] < y < square[0] + self.square_size:
                    l = str(self.pices[detections.class_id[i]])
                    # img = cv2.putText(self.frame, str(k), (square[1],square[0]), cv2.FONT_HERSHEY_SIMPLEX, 5, (120,120,120), 3, cv2.LINE_AA)
                    # img = cv2.putText(self.frame, str((x,y)), (square[1]+int(self.square_size/2),square[0]+int(self.square_size/2)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,0), 2, cv2.LINE_AA)
                    # cv2.imshow("ss",img)
            self.boardn = np.append(self.boardn,l)
            # print(len(self.boardn))
            # img = cv2.rectangle(img,(square[1],square[0]),(square[1]+square_size,square[0]+square_size),(0,255,0),3)
        self.boardn = np.reshape(self.boardn,(8,8))
        self.boardn = np.transpose(self.boardn)
        self.boardn = np.flip(self.boardn,1)

    def board_converting(self):
        self.boardn
        for row in range(8):
            for col in range(8):
                piece = self.boardn[row][col]
                if piece != " ":
                    self.engine_board.set_piece_at(chess.square(col, 7-row), chess.Piece.from_symbol(piece))

        self.board_svg = chess.svg.board(board=chess.Board(fen=None))
        self.svg = cairosvg.svg2png(self.board_svg)
        self.pixmap.loadFromData(self.svg)
        self.ui.rendered_board.setPixmap(self.pixmap)


    def predict_move(self):
        print(self.engine_board)
        self.result = self.engine.play(self.engine_board, chess.engine.Limit(time=1.0))
        self.engine_board.push(self.result.move)
        self.temp_board = self.engine_board
        self.result = str(self.result.move)
        print(self.engine_board)

        # self.board_svg = chess.svg.board(board=self.engine_board)
        # self.svg = cairosvg.svg2png(self.board_svg)
        # self.pixmap.loadFromData(self.svg)
        # self.ui.rendered_board.setPixmap(self.pixmap)

    def move_executing(self):
        self.move_publisher.publish(self.result)
        print("ok")
        while not self.done:
            rospy.Subscriber("/chess_rob/python_api/done", Bool, self.done_callback)


    def done_callback(self,ok):
        self.human_turn = True
        self.done = True
        

    def check(self):
        self.pass_check = True
        

    def exit(self):
        self.engine.quit()




def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    app.exec_()


if __name__ == "__main__":
    main()