import cv2
import numpy as np
import math


def getpoints(event, x,y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # cv2.circle(img, (x,y), 4, (255,0,0), -1)
        points.append([x,y])
        print(points)
        if len(points)==4:
            getperspective()
    cv2.imshow('image', img)

def getperspective():
    width = points[1][0] - points[0][0]
    height = points[2][1] - points[0][1]
    pts1 = np.float32([points[0:4]])
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgOutput = cv2.warpPerspective(img, matrix, (width, height))
    cv2.setMouseCallback('image', click_event)
    cv2.imshow('output', imgOutput)
    cv2.imwrite('output.jpg',imgOutput)


# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    # img 
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x, y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)
        

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)


        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x, y), font, 1,
                    (0, 0, 255), 2)
        cv2.imshow('image', img)

        npimage = np.array(img)
        red=np.array([255,0,0],dtype=np.uint8)
        reds=np.where(np.all((npimage==red),axis=-1))
        blue=np.array([0,0,255],dtype=np.uint8)
        blues=np.where(np.all((npimage==blue),axis=-1))
        dx2 = (blues[0][0]-reds[0][0])**2          # (200-10)^2
        dy2 = (blues[1][0]-reds[1][0])**2          # (300-20)^2
        distance = math.sqrt(dx2 + dy2)
        print(distance)



points = []
img = cv2.imread("IMG_20230224_215316.jpg")
sz1 = int(img.shape[0]*0.4)
sz2 = int(img.shape[1]*0.4)

while True:
    img = cv2.resize(img, (sz1, sz2))
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', getpoints)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    elif cv2.waitKey(1) & 0xff == ord('r'):
        img = cv2.imread("IMG_20230224_215316.jpg")
        points = []
        img = cv2.resize(img, (720, 960))
        cv2.imshow('image', img)
        cv2.setMouseCallback('image', getpoints)
    else:
        continue



    # reading the image
    img = cv2.imread('IMG_20230224_215316.jpg',1)
    img = cv2.resize(img, (720, 720))

    # displaying the image
    cv2.imshow('image', img)

    # setting mouse hadler for the image
    # and calling the click_event() function

    # wait for a key to be pressed to exit
    cv2.waitKey(0)

    # close the window
    cv2.destroyAllWindows()






