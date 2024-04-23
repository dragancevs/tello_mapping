import os
import threading
import functions
from djitellopy import Tello
import time
import cv2
import numpy as np
import sys

detected_markers = []
marker_is_on_left = False
current_detected_marker_id = None
# Stop all threads when exit app
exit_application = threading.Event()

# Desired height in cm (100 cm = 1 meter)
desired_height = 200

# Directory where images will be saved
img_directory = os.getcwd()

# Initialize Tello object
tello = Tello()

# Connect to the drone
tello.connect()

# Check battery level
battery_level = tello.get_battery()
if battery_level < 10:
    print(f"Baterie má {tello.get_battery()} % skenování nemůže začít.")
    sys.exit()

print(f"Baterie má {tello.get_battery()} %")

# Start video stream
tello.streamon()

# Shared frame for both threads to use
shared_frame = None
overlap_detected = True  # shared flag for overlap detection

def find_last_image_number(directory):
    max_img_number = 0
    # List all files in the directory
    for filename in os.listdir(directory):
        # If the file is a jpg and matches the expected pattern
        if filename.endswith(".jpg") and "overlap_image_" in filename:
            # Extract the number from the filename using regex
            number = int(re.search(r'overlap_image_(\d+).jpg', filename).group(1))
            # Update max_img_number if this number is higher
            if number > max_img_number:
                max_img_number = number
    return max_img_number

def detect_overlap():
    orb = cv2.ORB_create()
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    prev_frame = None
    prev_kps = None
    prev_des = None
    img_count = find_last_image_number(img_directory)

    while not exit_application.is_set():
        if exit_application.is_set():
            break
        global shared_frame, overlap_detected
        img = shared_frame
        if img is not None:
            kps = orb.detect(img, None)
            kps, des = orb.compute(img, kps)
            #print(f"Keypoints detected: {len(kps)}")
            #logging_function(f"Keypoints detected: {len(kps)} řádek: {inspect.currentframe().f_lineno}")
            if prev_frame is not None and prev_kps is not None:
                matches = matcher.match(des, prev_des)
                overlap_percent = len(matches) / float(len(kps)) * 100
                #print(f"Overlap: {overlap_percent}%")
                if 60 <= overlap_percent <= 70:
                    overlap_detected = True
                    img_count += 1
                    filename = f"overlap_image_{img_count}.jpg"
                    cv2.imwrite(os.path.join(img_directory, filename), img)
                    #print(f"Overlap detected! Image saved as {filename}.")
            prev_frame, prev_kps, prev_des = img, kps, des

def show_video_stream():
    while not exit_application.is_set():
        frame_read = tello.get_frame_read()
        #save frame to shared variable to be accesible for all functions
        global shared_frame
        shared_frame = frame_read.frame
        cv2.imshow("Tello Stream", shared_frame)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            tello.land()
            exit_application.set()
            exit("Exit...")
    cv2.destroyAllWindows()
    sys.exit(0)


def detect_aruco_markers():

    global shared_frame
    global marker_is_on_left
    global current_detected_marker_id
    # load the predefined dictionary
    aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    # create the parameters for the detector
    parameters = cv2.aruco.DetectorParameters_create()

    hi, wi, = 720, 960

    while not exit_application.is_set() or shared_frame is not None:
        img = shared_frame
        if img is not None and img.size != 0:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dictionary, parameters=parameters)

            if ids is not None:
                areas = [cv2.contourArea(c) for c in corners]
                max_index = np.argmax(areas)
                current_detected_marker_id = ids[max_index][0]

                # Adding an ID marker to the list only if it's not the same as the last detected
                if current_detected_marker_id not in detected_markers:
                    detected_markers.append(current_detected_marker_id)
                    print(f"Marker ID {current_detected_marker_id} detected and added to the list.")
                # else:
                    # print(f"Marker ID {current_detected_marker_id} detected again, but not added due to consecutive detection.")

                c = corners[max_index][0]
                cx, cy = np.mean(c, axis=0)

                # Determine the position of the marker
                if cx < wi // 2 and current_detected_marker_id != detected_markers[0]:
                    marker_is_on_left = True
                elif cx > wi // 2:
                    marker_is_on_left = False
                else:
                    marker_is_on_left = None  # Undefined position

# Start video stream thread
video_thread = threading.Thread(target=show_video_stream)
video_thread.start()

# Start overlap detection thread
overlap_thread = threading.Thread(target=detect_overlap)
overlap_thread.start()

# Start aruco detection thread
detect_aruco_thread = threading.Thread(target=detect_aruco_markers)
detect_aruco_thread.start()


# Take off
tello.takeoff()
time.sleep(1)
start_height = tello.get_distance_tof()
minimal_height = start_height
print('Height is cm:', start_height)

# Start scanning the object
print("Scanning object started.")
rotation_made = False  # flag to track whether rotation has occurred

while not exit_application.is_set():
    overlap_detected = False
    moving_sideways = False  # Flag to track when to print moving sideways message
    minimum_height_reached = False  # Flag to track when minimum height is reached
    # Ascend to the desired height
    while tello.get_distance_tof() < desired_height:
        if exit_application.is_set():
            break
        tello.send_rc_control(0, 0, 30, 0)  # Move up at a speed of 20 cm/s
        time.sleep(0.01)

    print("Drone is at maximum height.")
    # When the drone is at desired height, start moving sideways until overlap is detected
    overlap_detected = False

    while overlap_detected == False:
        if exit_application.is_set():
            break
        if moving_sideways == False:
            print("Drone is moving sideways.")
            moving_sideways = True
        tello.send_rc_control(20, 0, 0, 0)  # Move sideways at a speed of 20 cm/s to the right side
        time.sleep(0.01)

    # Descend to the desired height
    overlap_detected = False
    while tello.get_distance_tof() > minimal_height:
        if exit_application.is_set():
            break
        tello.send_rc_control(0, 0, -20, 0)  # Move down at a speed of 20 cm/s
        time.sleep(0.01)

        if minimum_height_reached == False:
            print("Drone is at minimum height.")
            minimum_height_reached = True

    overlap_detected = False

    if marker_is_on_left and len(detected_markers) > 1:
        tello.rotate_counter_clockwise(90)
        rotation_made = True
        print(f"Drone is rotating clockwise. marker_is_on_left: {marker_is_on_left} ")
        # print(f"Overlap: {overlap_percent}%")

    # Check if a marker is found twice => end of scanning
    if marker_is_on_left and current_detected_marker_id == detected_markers[0] and len(detected_markers) == 4: # User enter this value
        tello.land()
        tello.streamoff()
        exit_application.set()
        #print("Scan is complete")
        print(f"Scan is complete.marker_is_on_left: {marker_is_on_left}, len(detected_markers): {len(detected_markers)}")
        break

    overlap_detected = False
    while overlap_detected == False:
        if exit_application.is_set():
            break
        if moving_sideways == False:
            print("Drone is moving sideways.")
            moving_sideways = True
        tello.send_rc_control(20, 0, 0, 0)  # Move sideways at a speed of 20 cm/s to the right side
        time.sleep(0.01)

    overlap_detected=False

    # Reset flags before another pass
    moving_sideways = False
    minimum_height_reached = False

# Join threads
video_thread.join()
overlap_thread.join()
detect_aruco_thread.join()
