import cv2
# importing the module
import numpy as np
import math
# function to display the coordinates of
# of the points clicked on the image
def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    img 
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

                
                




# driver function
if __name__ == "__main__":
    # reading the image
    img = cv2.imread('output.jpg',1)
    # img = cv2.resize(img, (0, 0), None, 0.2, 0.2)

    # displaying the image
    cv2.imshow('image', img)

    # setting mouse hadler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_event)

    # wait for a key to be pressed to exit
    cv2.waitKey(0)

    # close the window
    cv2.destroyAllWindows()