import socket
import datetime
import time
import pygame
import ctypes
import struct

import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math



# ==========================================
# CAMERA PARAMETERS
# These are approximate values for a webcam
# ==========================================

camera_matrix = np.array([
    [1000, 0, 640],
    [0, 1000, 360],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1))

# Marker size in meters
MARKER_SIZE = 0.05  # 5 cm

success =0
x=0
y=0
z=0
yaw=0
a=0
b=0
c=0
d=0
e=0          
f=0           
g=0        
h=0



# ==========================================
# OPEN WEBCAM
# ==========================================

cap = cv2.VideoCapture(0)  # you may need to change parameter to 1

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
for i in range(-1, -10, -1):
    cap.set(cv2.CAP_PROP_EXPOSURE, i)
    print(i, cap.get(cv2.CAP_PROP_EXPOSURE))
# ==========================================
# ARUCO SETUP
# ==========================================

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

parameters = aruco.DetectorParameters()

detector = aruco.ArucoDetector(
    aruco_dict,
    parameters
)

# ==========================================
# 3D CORNERS OF MARKER
# ==========================================

obj_points = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
], dtype=np.float32)


prev_time = time.time()



# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if joysticks are available
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No joystick detected.")
else:

    print("Press q to quit.")
    # ==========================================
    # MAIN LOOP
    # ==========================================
 # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Main loop to continuously read joystick input
    running = True
    # Print the current date and time
    #print("Current date and time:", current_datetime)
    # Create a UDP socket
   

   
    #axes_bytes=[]
    #data_to_tx=[]
    joy_data=0
    sequence_num=0


    while True:

    # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Get joystick axes
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            #print("Axes:", axes)

            # Get joystick buttons
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            #print("Buttons:", buttons)

            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        
            #axes_bytes = str(axes).encode('utf-8')
            a=int((axes[0]+1)*128)
            b=int((axes[1]+1)*128)
            c=int((axes[2]+1)*128)
            d=int((axes[3]+1)*128)
            e=int(buttons[0])            
            f=int(buttons[1])            
            g=int(buttons[2])            
            h=int(buttons[3])
            
            a1=a.to_bytes(1)
            b1=b.to_bytes(1)
            c1=c.to_bytes(1)
            d1=d.to_bytes(1)
            e1=e.to_bytes(1)
            f1=f.to_bytes(1)
            g1=g.to_bytes(1)
            h1=h.to_bytes(1)
            hb=sequence_num.to_bytes(1)
            joy_data=0
            joy_data=a1+b1+c1+d1+e1+f1+g1+h1
          


           







        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = detector.detectMarkers(gray)

        # ==========================================
        # FPS
        # ==========================================

        current_time = time.time()
        fps = 1.0 / (current_time - prev_time)
        prev_time = current_time
        #print(f"FPS: {fps:.1f}")
        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # ==========================================
        # PROCESS MARKERS
        # ==========================================

        if ids is not None:

            aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):

                marker_id = ids[i][0]
                if True:
                #if marker_id==5:
                    #print(marker_id)
                    image_points = corners[i][0].astype(np.float32)

                    # Pose estimation
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        image_points,
                        camera_matrix,
                        dist_coeffs
                    )

                    if success:

                        # Position
                        x = tvec[0][0]
                        y = tvec[1][0]
                        z = tvec[2][0]

                        # Draw coordinate axes
                        cv2.drawFrameAxes(
                            frame,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec,
                            0.03
                        )

                        # Rotation matrix
                        R, _ = cv2.Rodrigues(rvec)

                        # Yaw angle
                        yaw = math.degrees(
                            math.atan2(R[1, 0], R[0, 0])
                        )

                        # Marker center
                        c1 = corners[i][0]

                        center_x = int(c1[:, 0].mean())
                        center_y = int(c1[:, 1].mean())

                        # Text
                        lines = [
                            f"ID: {marker_id}",
                            f"X: {x:.3f} m",
                            f"Y: {y:.3f} m",
                            f"Z: {z:.3f} m",
                            f"Yaw: {yaw:.1f} deg"
                        ]

                        for j, text in enumerate(lines):

                            cv2.putText(
                                frame,
                                text,
                                (center_x + 10, center_y + 25 * j),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2
                            )

        # ==========================================
        # SHOW WINDOW
        # ==========================================
        # Get image center
        # Get image center
        h1, w, _ = frame.shape
        cx, cy = w // 2, h1 // 2

        color = (0, 0, 255)
        thickness = 2

        # vertical full line
        cv2.line(frame, (cx, 0), (cx, h1), color, thickness)

        # horizontal full line
        cv2.line(frame, (0, cy), (w, cy), color, thickness)
        cv2.imshow("ArUco Pose Estimation", frame)

        # Quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if success:
            # marker data as floats
          
            marker_data = struct.pack(
                '<ffff',
                float(x),
                float(y),
                float(z),
                float(yaw)

            )
        else: 
            # marker data as floats
            marker_data = struct.pack(
                '<ffff',
                float(0),
                float(0),
                float(0),
                float(0)
            )


        # final packet
        
        seq_nbr=sequence_num.to_bytes(1)
        sccs=success.to_bytes(1)
        tx_data = joy_data + marker_data + sccs + seq_nbr

        
        sequence_num=sequence_num+1
        if (sequence_num>255):
            sequence_num=0
        udp_socket.sendto(tx_data, ("10.42.0.1", 8080))
        if success:
            print("tx 0",a,b,c,d,e,f,g,h,x,y,z,yaw,success,sequence_num)
        else:            
            print("tx 1",a,b,c,d,e,f,g,h,0.0,0.0,0.0,0.0,success,sequence_num)

        success=False

        # Close the socket
udp_socket.close()




