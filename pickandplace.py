import cv2
import numpy as np
import time
from opcua import Client
from opcua import ua

import numpy as np

import numpy as np

import math
prev_angle_deg=None
def calculate_angle(p1, p2, p3, p4):
    global prev_angle_deg
    # Calculate the vectors between the points
    vector1 = complex(p1[0] - p2[0], p1[1] - p2[1])
    vector2 = complex(p3[0] - p4[0], p3[1] - p4[1])
  
    # Calculate the angle between the vectors
    angle_rad = math.degrees(math.atan2(vector2.imag, vector2.real) - math.atan2(vector1.imag, vector1.real))
    
    # Adjust the angle range to 0 to 360
    angle_deg = angle_rad % 360
    if prev_angle_deg != angle_deg :
        prev_angle_deg=angle_deg
        oz_value = ua.Variant(prev_angle_deg, ua.VariantType.Double)
        oa.set_value(oz_value)
        print("Angle between the four points: {:.2f}".format(prev_angle_deg))  
     
    # Adjust the angle to be from 0 to -90 or 0 to 90
    if 0 <= angle_deg <= 180:
        angle_deg = angle_deg
    elif  180 <= angle_deg <= 360:
        angle_deg = -(360-angle_deg)
    return angle_deg


# OPC UA server endpoint URL (UA Expert OPC UA server)
opc_server_url = "opc.tcp://192.168.0.11:4840"

# Connect to the OPC UA server
client = Client(opc_server_url)
client.connect()

# Get the variable node for the trigger variable in UA Expert
trigger_node = client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.k")
trigger_node1= client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.t")
# Create OPC UA variable nodes for the coordinates and angle
x_node = client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.count")
y_node = client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.count1")
angle_node = client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.angle")
oa= client.get_node("ns=4;s=|var|Pilz-x86-WinCE-TV DEMO SM.Application.conveyor.oa")
# Open the camera
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Initialize time variables
start_time = time.time()
output_interval = 1  # Print output every 1 second

# Initialize previous coordinates and angle
prev_x_coord = None
prev_y_coord = None
prev_angle = None

def rob_coord(val1, val2):
    # Conversion factors and offsets for robot coordinates
    if val1 == 0 and val2 == 0:
        X_rob = 0
        Y_rob = 0
    else:
        X_rob = 0
        Y_rob = 0
        if trigger_node.get_value() == True:
            kx = 0.735
            ky = 0.73
            X_off = 238.5
            Y_off = 210.61
            X_rob = X_off - (kx * val1)
            Y_rob = Y_off - (ky * val2) - 7
        elif trigger_node1.get_value() == True:
            kx = 0.364
            ky = 0.370
            X_off = 125
            Y_off = 115
            X_rob = Y_off - (ky * val2)
            Y_rob = X_off - (kx * val1) - 7
        X_rob = int(X_rob)
        Y_rob = int(Y_rob)
    

    print('The Co-Ordinates of detected point in robot base frame are:', (X_rob, Y_rob))
    print("Detected midpoint coordinates:", (val1, val2))

    # Write x-coordinate to OPC UA variable
    x_value = ua.Variant(X_rob, ua.VariantType.Int16)
    x_node.set_value(x_value)
    print("X-coordinate sent to OPC UA server:", X_rob)

    # Write y-coordinate to OPC UA variable
    y_value = ua.Variant(Y_rob, ua.VariantType.Int16)
    y_node.set_value(y_value)
    print("Y-coordinate sent to OPC UA server:", Y_rob)

while True:
    # Check the value of the trigger variable in UA Expert
    if (trigger_node.get_value() == True) or (trigger_node1.get_value() == True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        if (trigger_node.get_value() == True):
            gamma = 0.75
        else:
             gamma = 0.30
    
         # Build a lookup table mapping the pixel values [0, 255] to their adjusted gamma values
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
    
        # Apply the gamma correction using the lookup table
        adjusted_frame = cv2.LUT(frame, table)
        # Convert the frame to grayscale
        gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
       
        _, thresh = cv2.threshold(gray, 101, 255, cv2.THRESH_BINARY)
        # Find contours and hierarchy using RETR_CCOMP
        image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        image2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Create empty images for external and internal contours
        external = np.zeros_like(image)
        internal = np.zeros_like(image2)
        # Iterate over contours and hierarchy
        for i in range(len(contours)):
            
            if hierarchy[0][i][3] == -1:  # External contour
                cv2.drawContours(external, contours, i, 255, -1)
            else:
                cv2.drawContours(internal, contours, i, 255, -1)
        # Find external contours in the binary image
        _, contours_external, _ = cv2.findContours(external.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert the image to RGB format
        external_rgb = cv2.cvtColor(external, cv2.COLOR_GRAY2RGB)
        min_sum = float('inf')  # Initialize with a large value
        min_point = None

        # Store the external midpoint coordinates
        external_midpoints = []

        # Iterate through the external contours
        for contour in contours_external:
            # Calculate the moments of the contour
            moments = cv2.moments(contour)

            # Check if the contour has non-zero area
            if moments['m00'] != 0:
                # Calculate the centroid (midpoint) of the contour
                cx = ((int(moments['m10'] / moments['m00'])))
                cy = ((int(moments['m01'] / moments['m00'])))

                # Calculate the sum of the coordinates
                coord_sum = cx + cy

                # Check if it's the new minimum
                if coord_sum < min_sum:
                    min_sum = coord_sum
                    min_point = (cx, cy)

                # Add the midpoint coordinates to the list
                external_midpoints.append((cx, cy))

                # Draw the midpoint on the RGB image in red color
                cv2.circle(external_rgb, (cx, cy), 5, (255, 0, 0), -1)

                # Display the pixel coordinates of the midpoint
                cv2.putText(
                    external_rgb,
                    f"External Midpoint: ({cx}, {cy})",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA
                )

        # Find internal contours in the binary image
        _, contours_internal, _ = cv2.findContours(internal.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert the image to RGB format
        internal_rgb = cv2.cvtColor(internal, cv2.COLOR_GRAY2RGB)

        # Initialize variables for farthest point
        farthest_internal_point = None
        max_distance = 0

        for contour in contours_internal:
            # Calculate the moments of the contour
            moments = cv2.moments(contour)

            # Check if the contour has non-zero area
            if moments['m00'] != 0:
                # Calculate the centroid (midpoint) of the contour
                mx = int(moments['m10'] / moments['m00'])
                my = int(moments['m01'] / moments['m00'])

                # Draw the midpoint on the RGB image in green color
                cv2.circle(internal_rgb, (mx, my), 5, (0, 255, 0), -1)

                # Display the pixel coordinates of the midpoint
                cv2.putText(
                    internal_rgb,
                    f"Internal Midpoint: ({mx}, {my})",
                    (mx + 10, my),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA
                )

                # Calculate the distance between the external and internal midpoints
                distance = np.sqrt((mx - min_point[0]) ** 2 + (my - min_point[1]) ** 2)

                # Check if it's the new maximum distance
                if distance > max_distance:
                    max_distance = distance
                    farthest_internal_point = (mx, my)
        # Display the RGB image with midpoints for both external and internal contours
        cv2.imshow("External", external_rgb)
        cv2.imshow("internal", internal_rgb)
        # Save x-coordinate and y-coordinate if minimum point exists
        if min_point is not None:
            x_coord = min_point[0]
            y_coord = min_point[1]

            # Calculate the angle between the four points
            if farthest_internal_point is not None:
                p1 = (179, 478)
                p2 = (216, 126)
                p3 = farthest_internal_point
                p4 = min_point

                angle = calculate_angle(p1, p2, p3, p4)

                # Check if the angle is different from the previous angle
                if angle != prev_angle:
                    prev_angle = angle
                    z_value = ua.Variant(angle, ua.VariantType.Double)
                    angle_node.set_value(z_value)
                    print("Angle between the four points1: {:.2f}".format(angle))

            # Save x-coordinate and y-coordinate if they are different from the previous values
            if x_coord != prev_x_coord or y_coord != prev_y_coord:
                prev_x_coord = x_coord
                prev_y_coord = y_coord

                # Perform the robot coordinate conversion and write to OPC UA variables
                rob_coord(x_coord, y_coord)
        else:
            # No contours found, set cx and cy to zero
            cx = 0
            cy = 0
            min_point = (cx, cy)
            print("No contours found.")
            rob_coord(cx, cy)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Delay for a short period to avoid excessive polling
        time.sleep(0.1)
# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()

# Disconnect from the OPC UA server
client.disconnect()
