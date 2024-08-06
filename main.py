import cv2
import mediapipe as mp
import serial
import struct
import HandTracker as ht
import time
from pySerialTransfer import pySerialTransfer as txfer
import sys
import serial 

import numpy as np 
import sympy as sp

hand_tracking = ht.HandTracker()
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


def constrain(n, _min, _max):
    if n < _min:
        return _min
    elif n > _max:
        return _max
    else : return n

def get_angles(constrains , coordinates):

    x0 , x1 = constrains[0]
    y0, y1 = constrains [1] 
    z0, z1 = constrains [2] 
    p = 117
    d = 124

    # asshole's script 
    x, y, z = coordinates
    # calculating retraction distance 
    r = (z - z0) / (z1 - z0) * (p + d - abs(p - d)) + abs(p - d)

    alpha = np.arccos((r**2 + p**2 - d**2) / (2 * r * p))
    alpha = np.pi - alpha
    alpha = 0.5* alpha

    beta = np.arccos((p**2 + d**2 - r**2) / (2 * p * d))
    # rotation angle 
    theta_y = 0.5 * np.pi - np.arccos(2 / (x1 - x0) * (x - (x1 + x0) / 2))

    # angle between horizontal and line r
    theta_x = np.arcsin(1 / (y1 - y0) * (y - y0))
    theta_x = np.pi * 0.5 - (theta_x)
    theta_x = theta_x + alpha

    # convert to degree
    theta_x = theta_x * 180/np.pi
    
    m1_angle =  theta_x 

    # convert theta_y to degrees
    #  theta_x is shoulder rotation 
    theta_y = theta_y * 180/np.pi * -1

    # adding theta_x and alpha to get M1 angle and converting to degrees
    #  theta_x is shoulder tilt 

    # Beta is the elbow
    beta = beta * 180/np.pi
    beta = 180 - beta

    print(f"Raw Angles : {alpha}, {beta} , {theta_x}, {theta_y}")
    return [theta_y, theta_x, beta] 

def send_angles(ser, theta_x, theta_y, beta):        
    pass

def main():
    cap = cv2.VideoCapture(0)
    # hand_tracking = ht.HandTracker()

    min_w, max_w =  get_max(cap, 0, "Move hand to left, then to right then press q ")
    min_h, max_h =  get_max(cap, 1, "Move hand from top to bottom, then press q ")
    min_z, max_z =  get_max(cap, 2, "Fuck this ... Z, then press q ")

    constrains = [[min_w, max_w], [min_h, max_h],[min_z, max_z]]
    print(f"Width : {max_w},  {min_w}")
    print(f"Height : {max_h}, {min_h}")
    print(f"Depth : {max_z} , {min_z}")
    try:
        link = txfer.SerialTransfer('/dev/cu.usbmodem14101')


        link.open()
        time.sleep(2) # allow some time for the Arduino to completely reset
        
        while True:
            success, img = cap.read()
            coordinates = hand_tracking.get_coordinates(img)
            if len(coordinates) != 0 : 
                x = constrain(coordinates[0], min_w, max_w)
                y = constrain(coordinates[1], min_h, max_h)
                z = constrain(coordinates[2], min_z, max_z)

                print(f"Width : {max_w}, {min_w}")
                print(f"Height : {max_h}, {min_h}")
                print(f"Depth : {max_z}, {min_z}")
                m1,m2,m3 = get_angles(constrains, [x,y,z]) 
                m1, m2, m3 = int(m1), int(m2), int(m3)
                
                m1 = m1 + 90 
                print(f"coordinates {x}, {y}, {z}")
                print(f"Angles {m1},  {m2}, {m3}")


                send_size = 0
                list_ =  []
                list_size = link.tx_obj(m1)
                list_size = link.tx_obj(m2)
                list_size = link.tx_obj(m3)
                # send_size += list_size# 

                # print(f"list_: {list_}")
                print(f"list_size: {send_size}")

                if list_size is not None and list_size <= txfer.MAX_PACKET_SIZE:
                    link.send(list_size)
                else:
                    print("Error: List size exceeds maximum packet size.")

                print("Sending data to arduino")
                ###################################################################
                # Wait for a response and report any errors while receiving packets
                ###################################################################
                # while not link.available():
                #     if link.status < 0:
                #         if link.status == txfer.CRC_ERROR:
                #             print('ERROR: CRC_ERROR')
                #         elif link.status == txfer.PAYLOAD_ERROR:
                #             print('ERROR: PAYLOAD_ERROR')
                #         elif link.status == txfer.STOP_BYTE_ERROR:
                #             print('ERROR: STOP_BYTE_ERROR')
                #         else:
                #             print('ERROR: {}'.format(link.status))

                # ###################################################################
                # # Parse response list
                # ###################################################################
                # rec_list_  = link.rx_obj(obj_type=type(list_),
                #                          obj_byte_size=list_size,
                #                          list_format='i')
                #
                # ###################################################################
                # # Display the received data
                # ###################################################################
                print('SENT: {}'.format(list_))
                # print('RCVD: {}'.format(rec_list_))
                # print(' ')
                while True:
                    # Check if any data received
                    if txfer.available():
                        # Since we know the Arduino sends 3 bytes, we read 3 bytes from the buffer
                        m1 = transfer.rx_obj(obj_type=ctypes.c_uint8, start_pos=0)
                        m2 = transfer.rx_obj(obj_type=ctypes.c_uint8, start_pos=1)
                        m3 = transfer.rx_obj(obj_type=ctypes.c_uint8, start_pos=2)

                        # Process the received data (print it, use it for further logic, etc.)
                        print(f"Received positions: m1={m1}, m2={m2}, m3={m3}")

            # img = hand_tracking.track(img)
            # Show the image in a window named 'Image'
            cv2.imshow('Image', img)
            # Wait for key press (1ms)
            key = cv2.waitKey(1)

            # Break the loop if 'q' is pressed
            if key == ord('r'):
                break

            
    
    except KeyboardInterrupt:
        try:
            link.close()
            cap.release()
            pass
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            pass
            link.close()
            cap.release()
        except:
            pass
            
if __name__ == '__main__':
    main()
