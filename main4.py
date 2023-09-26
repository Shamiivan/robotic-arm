import cv2
import mediapipe as mp
from pySerialTransfer import pySerialTransfer as txfer
import time

# Create objects
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# Initialize variables to store coordinates
cx_values = []
cy_values = []
cz_values = []

# Set up webcam capture (0 is the default webcam)
cap = cv2.VideoCapture(0)

def calculate_and_print_average():
    if cx_values and cy_values and cz_values:
        avg_cx = sum(cx_values) / len(cx_values)
        avg_cy = sum(cy_values) / len(cy_values)
        avg_cz = sum(cz_values) / len(cz_values)
        print(f"Average 2D Coordinates: ({avg_cx:.2f}, {avg_cy:.2f}), Average 3D Depth: {avg_cz:.2f}")

def main():
    # Connect to serial communication
    try:
        link = txfer.SerialTransfer('/dev/cu.usbmodem14101')

        link.open()
        time.sleep(2)  # allow some time for the Arduino to completely reset

        # Continuously get frames from the webcam
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
                        cx, cy, cz = int(lm.x * w), int(lm.y * h), int(lm.z * 100)  # Convert to pixel and depth coordinates

                        # Store the coordinates in lists
                        cx_values.append(cx)
                        cy_values.append(cy)
                        cz_values.append(cz)

                        ## Send the coordinates to arduino
                        send_size = 0

                        ###################################################################
                        # Send a list
                        ###################################################################
                        list_ = [cx, cy, cz]
                        list_size = link.tx_obj(list_)
                        send_size += list_size

                        ###################################################################
                        # Transmit all the data to send in a single packet
                        ###################################################################
                        link.send(send_size)

                        ###################################################################
                        # Wait for a response and report any errors while receiving packets
                        ###################################################################
                        while not link.available():
                            if link.status < 0:
                                if link.status == txfer.CRC_ERROR:
                                    print('ERROR: CRC_ERROR')
                                elif link.status == txfer.PAYLOAD_ERROR:
                                    print('ERROR: PAYLOAD_ERROR')
                                elif link.status == txfer.STOP_BYTE_ERROR:
                                    print('ERROR: STOP_BYTE_ERROR')
                                else:
                                    print('ERROR: {}'.format(link.status))

                        ###################################################################
                        # Parse response list
                        ###################################################################
                        rec_list_ = link.rx_obj(obj_type=type(list_),
                                                obj_byte_size=list_size,
                                                list_format='i')

                        ###################################################################
                        # Display the received data
                        ###################################################################
                        print('SENT: {}'.format(list_))
                        print('RCVD: {}'.format(rec_list_))
                        print(' ')

                        # if id == 4: # For example, to highlight thumb tip only. Remove for loop to highlight all points
                        #     cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED) # Draw circle at hand landmark

                    # Draw the hand landmarks and connections
                    mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

                    # Calculate and print the average coordinates for this input
                    calculate_and_print_average()

                # Reset the average values for the next input
                cx_values.clear()
                cy_values.clear()
                cz_values.clear()

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

    except KeyboardInterrupt:
        try:
            link.close()
        except:
            pass

    except:
        import traceback
        traceback.print_exc()

        try:
            link.close()
        except:
            pass

if __name__ == "__main__":
    main()
