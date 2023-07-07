import cv2
import numpy as np
from operator import itemgetter

import numpy as np

from ultralytics import YOLO
import supervision as sv
import numpy as np

ZONE_POLYGON = np.array([
    [0, 0],
    [0.5, 0],
    [0.5, 1],
    [0, 1]
])

img = cv2.imread('IMG_20230415_001916.jpg')
model = YOLO("best (2).pt")


box_annotator = sv.BoxAnnotator(
   thickness=2,
   text_thickness=2,
   text_scale=1
)

zone_polygon = (ZONE_POLYGON * np.array([720,540])).astype(int)
zone = sv.PolygonZone(polygon=zone_polygon, frame_resolution_wh=tuple([720, 540720]))

# img = cv2.resize(img,(1000,1000))
# convert the input image to a grayscale
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
dim = (int(img.shape[0]*0.4), int(img.shape[1]*0.4))
print(dim)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (7,7),None)


if ret == True:
   c = 0
   # Draw and display the corners
   # img = cv2.drawChessboardCorners(img, (7,7), corners,ret)
   square_size = int(np.linalg.norm(corners[0] - corners[1]))
   points = [
        0,2,4,6,
        14,16,18,20,
        28,30,32,34,
        42,44,46,48
    ]
   squares = []
   boxs  = np.array([
])
   flag = True
   crops = []
   for i in points:
      x , y = (int(corners[i][0][0]),int(corners[i][0][1]))
      # cv2.circle(img, (x , y), 5, (255,25,29), -1)
      # cv2.circle(img, (int(corners[i][0][0]+square_size),int(corners[i][0][1]+square_size)), 15, (255,25,29), -1)
      # cv2.circle(img, (int(corners[i][0][0]+square_size),int(corners[i][0][1]-square_size)), 15, (255,25,29), -1)
      # cv2.circle(img, (int(corners[i][0][0]-square_size),int(corners[i][0][1]+square_size)), 15, (255,25,29), -1)
      # cv2.circle(img, (int(corners[i][0][0]-square_size),int(corners[i][0][1]-square_size)), 15, (255,25,29), -1)

   # al saf al tany b2sam 3ala 2 w agma3 3laha 8 >> (x/2)+8
   # x/2 

   # 
      img = cv2.imread('IMG_20230415_002011.jpg')
  
      square = img[y-square_size:y , x-square_size:x ]
      squares.append((x-square_size,y-square_size,square))
      box =[x-square_size,y-square_size,x,y]
      boxs = np.append(boxs, box)

      square = img[y:y + square_size, x-square_size:x ]
      squares.append((x-square_size,y ,square))
      box= [x-square_size,y,x,y+square_size]
      boxs = np.append(boxs, box)


      square = img[y-square_size:y , x:x + square_size]
      squares.append((x,y-square_size,square))
      box= [x,y-square_size,x+ square_size,y]
      boxs = np.append(boxs, box)


      square = img[y:y + square_size, x:x + square_size]
      squares.append((x,y,square))
      box= [x,y, x+square_size,y+ square_size,y]
      boxs = np.append(boxs, box)


   
   # boxs = np.array(boxs)
   # boxs = boxs.reshape((-1,4))

   # print(boxs)
   # point = np.array([150, 50])

# Query the k-d tree to find the square that contains the point
   # dist, idx = tree.query(point)
   # print(f"The point is in square {idx}")

   # for box in boxs:
   #    img = cv2.rectangle(img,(box[0],box[1]),(box[2],box[3]),(0,255,0),3)
   squares = sorted(squares, key=itemgetter(1))

   squares[36], squares[34], squares[37] , squares[35] = squares[34], squares[36] ,squares[35] , squares[37] 
   squares[38], squares[36], squares[39] , squares[37] = squares[36], squares[38] ,squares[37] , squares[39]

   squares[44], squares[42], squares[45] , squares[43] = squares[42], squares[44] ,squares[43] , squares[45] 
   squares[44], squares[46], squares[47] , squares[45] = squares[46], squares[44] ,squares[45] , squares[47]
   
   
   
   # squares[34],squares[35],squares[36],squares[37],squares[38],squares[39] ,squares[36],squares[37],squares[38],squares[39],squares[34],squares[35] =  squares[36],squares[37],squares[38],squares[39],squares[43],squares[35] ,squares[38],squares[39],squares[34],squares[35],squares[36],squares[37]

   # squares[35] = square_temp[37]
   # squares[36] = square_temp[34]
   # squares[37] = square_temp[35]  
   # squares[34] = square_temp[36]
   # squares[35] = square_temp[37]
   # squares[38] = square_temp[36]
   # squares[36] = square_temp[34]
   # squares[36] = square_temp[34]

   # for i in range (63):
   #     if i%2==0:
   #       squares[i],squares[i+1] = squares[i+1],squares[i]
   for idx, square in enumerate(squares):

      img = cv2.putText(img, str(idx), (int(square[0]),int(square[1])), cv2.FONT_HERSHEY_SIMPLEX, 5, (0,0,255), 3, cv2.LINE_AA)
      # img = cv2.rectangle(img,(square[1],square[0]),(square[1],square[0]),(0,255,0),3))
      # frame = img
      # frame = square[2]
      # # frame = cv2.resize(frame,(1024,1024))
      # # print(square[2])
      # result = model(frame, agnostic_nms=True)[0]
      # detections = sv.Detections.from_yolov8(result)
      # labels = [
      #    f"{model.model.names[class_id]} {confidence:0.2f}"
      #    for _, confidence, class_id, _
      #    in detections
      # ]
      # frame = box_annotator.annotate(
      #    scene=frame, 
      #    detections=detections, 
      #    labels=labels
      # )

      # zone.trigger(detections=detections)
      # img = cv2.resize(img,(1000,720))
      # cv2.imshow("yolov8", img)

      cv2.waitKey(0)




   #    if flag:
   #        square = cv2.putText(img, str((idx+1)), (int(square[1]),int(square[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,120,120), 1, cv2.LINE_AA)
   #    else:
         
   #        square = cv2.putText(img, str((idx-1)), (int(square[1]),int(square[0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120,120,120), 1, cv2.LINE_AA)

   #    flag = not flag
             
         # cv2.imshow('sq',square)
         # cv2.waitKey(1)
   # print(square_size)
   # for i in range(7):
   #       for j in range(7):
   #             print(c)
   #             # Extract the square
   #             x, y = int(corners[i * 7 + j][0][0]), int(corners[i * 7 + j][0][1])
   #             square = gray[y:y + square_size, x:x + square_size]
   #             cv2.circle(img, (x+ square_size, y+ square_size), 5, (255,25,29), -1)

   #             c = c + 1


   img = cv2.resize(img,(1000,720))
   cv2.imshow('Chessboard',img)
   cv2.waitKey(0)
cv2.destroyAllWindows()