import cv2
import numpy as np
 






# picture = picture*255
 

def draw(A):
    # A = np.array([[7.6, 0], [4.3, 7.4], [-3.5, 6.2], [-6.6, 0], [-2.9, -5.0], [2.6, -4.5]])
    min_x = 0
    min_y = 0
    for i in A:
        if i[1]<min_y:
            min_y = i[1]
        if i[0]<min_x:
            min_x = i[0]
    print(f"x= {min_x} y={min_y}")
    for i in range(len(A)):
        A[i][0]+=-min_x
        A[i][0]*=10
        A[i][1]+=-min_y
        A[i][1]*=10

    max_x = 0
    max_y = 0
    for i in A:
        if i[1]>max_y:
            max_y = i[1]
        if i[0]>max_x:
            max_x = i[0]
    print(f"x= {max_x} y={max_y}")


    print((int(A[0][0]),int(A[0][1])))


    img = np.ones((int(max_x+50),int(max_y+50),3),'uint8')*255

    for i in range(len(A)):
        cv2.circle(img, (int(A[i][0])+10,int(A[i][1])+10), 1, (0, 0, 255), 2)

    cv2.imshow('line',img)
    cv2.waitKey(0)
