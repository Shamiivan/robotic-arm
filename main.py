import cv2
import mediapipe as mp
import serial
import struct
import HandTracker as ht
import time
from pySerialTransfer import pySerialTransfer as txfer
import sys

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


def get_angles(constrains , coordinates):

    x0 , x1 = constrains[0]
    y0, y1 = constrains [1] 
    z0, z1 = constrains [2] 
    p = 117
    d = 124

    x, y, z = coordinates
    r = (z - z0) / (z1 - z0) * (p + d - abs(p - d)) + abs(p - d)
    alpha = np.arccos((r**2 + p**2 - d**2) / (2 * r * p))
    beta = np.arccos((p**2 + d**2 - r**2) / (2 * p * d))
    theta_y = 0.5 * np.pi - np.arccos(2 / (x1 - x0) * (x - (x1 + x0) / 2))
    theta_x = np.arcsin(1 / (y1 - y0) * (y - y0))
    
    theta_y = theta_y * 180/np.pi 
    theta_x = (theta_x + alpha) * 180/np.pi
    beta = beta * 180/np.pi
    return [theta_y, theta_x, beta] 


    
    

def main():
    cap = cv2.VideoCapture(0)
    # hand_tracking = ht.HandTracker()

    min_w, max_w =  get_max(cap, 0, "Move hand to left, then to right then press q ")
    min_h, max_h =  get_max(cap, 1, "Move hand from top to bottom, then press q ")
    min_z, max_z =  get_max(cap, 2, "Fuck this ... Z, then press q ")
    # constrains = [[min_w, max_w], [min_h, max_h],[min_z, max_z]]
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
                x, y , z = coordinates[0], coordinates[1], coordinates [2]
                print(f"Width : {max_w}, {min_w}")
                print(f"Height : {max_h}, {min_h}")
                print(f"Depth : {max_z}, {min_z}")
                constrains =[[0,1],[0,1],[0,1]]
                m1,m2,m3 = get_angles(constrains, [x,y,z]) 
                print(f"coordinates {x}, {y}, {z}")
                print(f"Angles {m1},  {m2}, {m3}")


                # send_size = 0
                # list_ =  [m1,m2,m3]
                # list_size = link.tx_obj(list_)
                # send_size += list_size
                #
                # link.send(list_size)
        

                # print("Sending data to arduino")
                # ###################################################################
                # # Wait for a response and report any errors while receiving packets
                # ###################################################################
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
                # 
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
                # print('SENT: {} {} {}'.format(list_,))
                # print('RCVD: {} {} {}'.format(rec_list_))
                # print(' ')
                #
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
        except:
            pass
    
    except:
        import traceback
        traceback.print_exc()
        
        try:
            link.close()
            cap.release()
        except:
            pass
if __name__ == '__main__':
    main()
