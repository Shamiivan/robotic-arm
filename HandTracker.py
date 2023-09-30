import sys
import cv2
import mediapipe as mp
import time

class HandTracker : 
    def __init__(self, mode=False, maxHands=1, detectionCon=0.8,trackCon=0.8):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(mode, maxHands,1, detectionCon,trackCon)
        self.mpDraw = mp.solutions.drawing_utils
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

    def get_coordinates(self, img, handNo=0, draw=True):
        lmlist = []
        imageRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id,lm in enumerate(Hand.landmark):
                if id == 9:
                    h,w,c = img.shape
                    cx, cy,cz = int(lm.x * w ), int(lm.y * h) , int(lm.z * 100)
                    lmlist.extend([cx, cy,cz, lm.x, lm.y])
            if draw:
                self.mpDraw.draw_landmarks(img, Hand, self.mpHands.HAND_CONNECTIONS)
    
        return lmlist




# class HandTracking:
#
#     def __init__(self):
#         self.mpHands = mp.solutions.hands
#         self.hands = self.mpHands.Hands()
#         self.mpDraw = mp.solutions.drawing_utils
#         self.width_values = []
#         self.height_values = []
#         self.cap = cv2.VideoCapture(0)
#         self.calibration_frames = 50
#         self.calibration_instruction = "Starting calibration"
#         
#
#     def display_instructions(self, img):
#         cv2.putText(img, self.calibration_instruction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
#
#     def perform_calibration(self):
#         while True:
#             success, img = self.cap.read()
#             self.display_instructions(img)
#
#             imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
#             results = self.hands.process(imgRGB)
#
#             if results.multi_hand_landmarks:
#                 for handLms in results.multi_hand_landmarks:
#                     for id, lm in enumerate(handLms.landmark):
#                         if id == 9:
#                             h, w, c = img.shape
#                             cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z * 100)
#                             self.width_values.append(cx)
#                             self.height_values.append(cy)
#                             cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
#
#                 self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
#
#                 if len(self.width_values) == self.calibration_frames:
#                     break
#
#             cv2.imshow('Image', img)
#             time.sleep(0.5)
#             key = cv2.waitKey(1)
#
#             if key == ord('q'):
#                 break
#
#     def get_calibration_results(self):
#         min_width = min(self.width_values)
#         max_width = max(self.width_values)
#         min_height = min(self.height_values)
#         max_height = max(self.height_values)
#         return {
#             "min_width": min_width,
#             "max_width": max_width,
#             "min_height": min_height,
#             "max_height": max_height
#         }
#
#     def close(self):
#         self.cap.release()
#         cv2.destroyAllWindows()

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

    
if __name__ == "__main__":
    min_max = []
    max_width =0
    min_width = 0
    max_height =0
    min_height = 0
    min_depth = 0
    max_depth=0

    cap = cv2.VideoCapture(0)
    hand_tracking = HandTracker()



    min_w, max_w =  get_max(cap, 0, "Move hand to left, then to right then press q ")
    min_h, max_h =  get_max(cap, 1, "Move hand from top to bottom, then press q ")
    min_z, max_z =  get_max(cap, 2, "Fuck this ... Z, then press q ")
    print(f"Width : {max_w} - {min_w}")
    print(f"Height : {max_h} - {min_h}")
    print(f"Depth : {max_z} - {min_z}")
    
    cap.release()
    sys.exit() 

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
