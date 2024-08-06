import sys
import cv2
import mediapipe as mp
import time
import math
import numpy as np 


class HandTracker : 
    def __init__(self, mode=False, maxHands=2, detectionCon=0.8,trackCon=0.8):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(mode, maxHands,1, detectionCon,trackCon)
        self.mpDraw = mp.solutions.drawing_utils
    
        # x represent the raw values 
        x = [300,245,170,145,130,122,112,103,93,87,80,75,70,67,62,59,57]
        y = [20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100]
        self.coefficients = np.polyfit(x,y,3)

    def track(self,img, draw=True):
        #only track hands 
        imageRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)
        
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img;

    def display_instructions(self, img, text):
        cv2.putText(img, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)

    def get_z_coordinate(self, x2, x1):
        return math.abs(x2,x1)

    def get_coordinates(self, img, handNo=0, draw=True):
        lmlist = []
        res = [] 
        imageRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]

            for id,lm in enumerate(Hand.landmark):
                if id == 9:
                    h,w,c = img.shape
                    cx, cy = int(lm.x * w ), int(lm.y * h)                     
                    x1, y1 = (Hand.landmark[5].x) * w , (Hand.landmark[5].y) * h
                    x2, y2 = (Hand.landmark[17].x) * w , (Hand.landmark[17].y) * h

                    A, B, C, D = self.coefficients
                    rawDistance = int(math.sqrt((y2 -y1) ** 2 + (x2 -x1) **2 ))

                    cz = A * rawDistance**3 + B * rawDistance**2 + C * rawDistance + D  
                    cz = max(0,cz)
                    # print(f" Raw values: {x1}, {x2} , {y1} , {y2} , {cz}")
                    print(f"coordinates : {cx} - {cy} - {cz}")

                    res.extend([cx, cy,cz])
                    # lmlist.extend([lm.x, lm.y, lm.z])
            if draw:
                self.mpDraw.draw_landmarks(img, Hand, self.mpHands.HAND_CONNECTIONS)
    
        return res





def get_max(cap, pos, msg):
    min_max = []
    max_w = 0
    min_w = sys.maxsize

    while True : 
        success, img = cap.read()
        hand_tracking.display_instructions(img, msg)
        coordinates = hand_tracking.get_coordinates(img)
        if len(coordinates) > 0 :
            current_width = coordinates[pos]
            max_w = max(current_width, max_w)
            min_w = min(current_width, min_w)

        cv2.imshow('Image', img)
        # Wait for key press (1ms)
        key = cv2.waitKey(1)

        # Break the loop if 'q' is pressed
        if key == ord('q'):
            min_max.extend([min_w, max_w])
            return min_max
            break

    
def get_min(cap, pos, msg):
    res = sys.maxsize
    while True : 
        success, img = cap.read()
        hand_tracking.display_instructions(img, msg)
        coordinates = hand_tracking.get_coordinates(img)
        if len(coordinates) > 0 :
            current_width = coordinates[pos]
            res = min(current_width, res);

        cv2.imshow('Image', img)
        # Wait for key press (1ms)
        key = cv2.waitKey(1)

        # Break the loop if 'q' is pressed
        if key == ord('q'):
            return res
            break

    
def main() : 
    min_max = []
    max_width =0
    min_width = 0
    max_height =0
    min_height = 0
    min_depth = 0
    max_depth=0

    cap = cv2.VideoCapture(0)
    hand_tracking = HandTracker()



    # min_w, max_w =  get_max(cap, 0, "Move hand to left, then to right then press q ")
    # min_h, max_h =  get_max(cap, 1, "Move hand from top to bottom, then press q ")
    # min_z, max_z =  get_max(cap, 2, "Fuck this ... Z, then press q ")
    # print(f"Width : {max_w} - {min_w}")
    # print(f"Height : {max_h} - {min_h}")
    # print(f"Depth : {max_z} - {min_z}")


    while True:
        success, img = cap.read()
        calibration_results = hand_tracking.get_coordinates(img)
        print(calibration_results)

        # img = hand_tracking.track(img)
        # Show the image in a window named 'Image'
        cv2.imshow('Image', img)
        # Wait for key press (1ms)
        key = cv2.waitKey(1)

        # Break the loop if 'q' is pressed
        if key == ord('q'):
            break



    cap.release()
if __name__ == "__main__":
    main()
