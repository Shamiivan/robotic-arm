import cv2
import mediapipe as mp
import serial
import struct
from pySerialTransfer import pySerialTransfer as txfer
import time 
## Setup arduino serial communication

# Create objects
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils


# Set up webcam capture (0 is the default webcam)
cap = cv2.VideoCapture(0)
# continuously get frames from the webcam
while True:
    success, img = cap.read()

    # Convert image color format from BGR to RGB
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Use MediaPipe to find hand landmarks
    results = hands.process(imgRGB)

    # If hands are detected
    if results.multi_hand_landmarks:
        # For every hand detected
        for handLms in results.multi_hand_landmarks:
            # For every hand landmark on a hand
            for id, lm in enumerate(handLms.landmark):
                # Get landmark x, y, and z coordinates (normalized to range [0.0, 1.0])
                h, w, c = img.shape
                cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z * 100) # Convert to pixel and depth coordinates
                print(f"2D Coordinates: ({cx}, {cy}), 3D Depth: {cz}")

                # send the data to arduino 

                send_size = 0
                
                ###################################################################
                # Send a list
                ###################################################################
                list_ = [cx, cy, cx]
                list_size = link.tx_obj(list_)
                send_size += list_size
                if id == 4: # For example, to highlight thumb tip only. Remove for loop to highlight all points
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED) # Draw circle at hand landmark

            # Draw the hand landmarks and connections
            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

    # Show the image in a window named 'Image'
    cv2.imshow('Image', img)

    # Wait for key press (1ms)
    key = cv2.waitKey(1)

    # Break the loop if 'q' is pressed
    if key == ord('q'):
        break

# Release the webcam and destroy windows
cap.release()