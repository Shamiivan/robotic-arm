import cv2
import mediapipe as mp
import numpy as np



## CONSTANT 
calibration_value = 1000

class HandTracker:
    def __init__(self):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils

    def track_hand(self, img):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(imgRGB)
        hand_coordinates = []

        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    hand_coordinates.append(cx)
                    hand_coordinates.append(cy)
                    if id == 4:
                        cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

        return np.array(hand_coordinates)

    def normalize_hand_coordinates(self, hand_coordinates, img):
        h, w, _ = img.shape
        # Normalize x coordinates
        hand_coordinates[::2] = hand_coordinates[::2] / w
        # Normalize y coordinates
        hand_coordinates[1::2] = hand_coordinates[1::2] / h
        return hand_coordinates

class RoboticArm:
    def __init__(self) :
        pass
        


def update(self, angles):
        # Set joint angles
        self.joint_angles = anglesk



if __name__ == "__main__":

    hand_tracker = HandTracker()
    robotic_arm = RoboticArm()
    cap = cv2.VideoCapture(0)


     
    print("\033[91mCalibration\033[0m")
    
    print("Move the most left hand")
    for i in range(1000):
        # Normalize the hand coordinates
        
        success, img = cap.read()
        hand_coordinates = hand_tracker.track_hand(img)
        hand_coordinates = hand_tracker.normalize_hand_coordinates(hand_coordinates, img)
        print(hand_coordinates)


        # Compute the desired end-effector position
        # end_effector_position = robotic_arm.compute_desired_position(hand_coordinates)
        # print("End end-effector :\n", end_effector_position)
        # Perform inverse kinematics to compute the joint angles
        # T_desired = np.eye(4)  # Assume a fixed desired orientation

