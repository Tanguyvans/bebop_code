import cv2 
import numpy as np
import math
import rospy


def distance (x,y,a,b):
    return math.sqrt((x-a)**2 + (y-b)**2)

def getContours(imgCanny, img):
    _, contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>500:
            cv2.drawContours(img, cnt, -1, (0,0,255),3)  
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 7:
                coteGauche = distance(x,y,x,y+h)
                coteHaut = distance(x,y,x+w,y)
                if coteGauche > coteHaut:  
                    min_x = 1000000000
                    max_x = 0
                    mid = 0
                    cmt = 0
                    for point in approx:
                        if point[0][1] < min_x:
                            min_x = point[0][1]
                        if point[0][1] > max_x:
                            max_x = point[0][1]
                        else:
                            if cmt <= 5:
                                mid = point[0][1]
                        cmt += 1
                    dMin = abs(min_x-mid)
                    dMax = abs(max_x-mid)
                    
                    if dMin < dMax:
                        objectType = 'verticale haut'
                        rospy.loginfo('haut')
                    else:
                        objectType = "verticale bas" 
                        rospy.loginfo('bas')     

                else : 
                    min_x = 1000000000
                    max_x = 0
                    mid = 0
                    cmt = 0
                    for point in approx:
                        if point[0][0] < min_x:
                            min_x = point[0][0]
                        if point[0][0] > max_x:
                            max_x = point[0][0]
                        else : 
                            if cmt <=5:
                                mid = point[0][0]
                        cmt += 1
                    dMin = abs(min_x-mid)
                    dMax = abs(max_x-mid)
                    
                    if dMin < dMax:
                        objectType = 'horizontale gauche'
                        rospy.loginfo('gauche')
                    else:
                        objectType = "horizontale droite" 
                        print (x, y, w, h)
                        rospy.loginfo('droite')

                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)
                cv2.putText(img, objectType, (x,y-10),cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,0,0),2)     
            
            else: 
                objectType = "None"

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image

    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    resized = cv2.resize(image, dim, interpolation = inter)

    return resized