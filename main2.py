import cv2
import serial
import mediapipe as mp

# arduino = serial.Serial(port='/dev/cu.usbmodem14101', baudrate=9600, timeout=.1)

# Global variables to store x, y, and z-coordinate values
x_values = []
y_values = []
z_values = []

def calculate_average_coordinates_depth(hand_landmarks):
    if hand_landmarks:
        x_values = [int(lm.x * w) for lm in hand_landmarks.landmark]
        y_values = [int(lm.y * h) for lm in hand_landmarks.landmark]
        z_values = [int(lm.z * 100) for lm in hand_landmarks.landmark]
        avg_x = sum(x_values) / len(x_values)
        avg_y = sum(y_values) / len(y_values)
        avg_z = sum(z_values) / len(z_values)
        return avg_x, avg_y, avg_z
    else:
        return None, None, None

def get_average_values(num_samples):
    global x_values, y_values, z_values
    if len(x_values) >= num_samples:
        avg_x = sum(x_values[-num_samples:]) / num_samples
        avg_y = sum(y_values[-num_samples:]) / num_samples
        avg_z = sum(z_values[-num_samples:]) / num_samples
        return avg_x, avg_y, avg_z
    else:
        return None, None, None

# Create objects
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# Set up webcam capture (0 is the default webcam)
cap = cv2.VideoCapture(0)

# Continuously get frames from the webcam
while True:
    success, img = cap.read()

    # Get frame dimensions
    h, w, c = img.shape

    # Convert image color format from BGR to RGB
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Use MediaPipe to find hand landmarks
    results = hands.process(imgRGB)

    # If hands are detected
    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            avg_x, avg_y, avg_z = calculate_average_coordinates_depth(handLms)
            if avg_x is not None and avg_y is not None and avg_z is not None:
                # print(f"Average X-coordinate: {avg_x:.2f}")
                # print(f"Average Y-coordinate: {avg_y:.2f}")
                avg_x_1000, avg_y_1000, avg_z_1000 = get_average_values(1000)
                # print(f"Average Z-coordinate: {avg_z:.2f} cm")
                #
                # Store x, y, and z-coordinate values
                x_values.append(avg_x)
                y_values.append(avg_y)
                z_values.append(avg_z)

                # Check if we have collected 1000 values
                if len(x_values) == 1000:
                    # Calculate and print the average of the last 1000 coordinates
                    avg_x_1000, avg_y_1000, avg_z_1000 = get_average_values(1000)
                    print("Sent ", avg_x_1000, avg_y_1000, avg_z_1000)
                    print("Received ", read_from_arduino())
                    # print(f"Average of Last 1000 X-coordinates: {avg_x_1000:.2f}")
                    # print(f"Average of Last 1000 Y-coordinates: {avg_y_1000:.2f}")
                    # print(f"Average of Last 1000 Z-coordinates: {avg_z_1000:.2f} cm")
                    #
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
cv2.destroyAllWindows()
