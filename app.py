from CommandProcessingModule.SenderModule import EV3CommandSender
import threading
import collections

import time

import csv
import copy
import argparse
import itertools
from collections import deque

import cv2 as cv
import numpy as np
import mediapipe as mp

from utils import CvFpsCalc
from model import KeyPointClassifier


# Initialize the sender (running on the PC)
ev3_sender = EV3CommandSender(  )


# Gesture recognition counter
gesture_counter = collections.defaultdict(int)
last_command_time = 0
last_command = None


def send_command_async(command):
    """Function to send commands asynchronously using a separate thread."""
    global last_command_time, last_command

    current_time = time.time()

    # Check if the new command is different from the last command and if enough time has passed

    if command == last_command and (current_time - last_command_time < 35):
        print("Duplicate command  Ignoring this command.")
        return

    # Update the last command time and command
    last_command_time = current_time
    last_command = command

    # Send the command in a new thread
    send_thread = threading.Thread(target=ev3_sender.send_command, args=(command,))
    send_thread.start()


def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument('--use_static_image_mode', action='store_true')
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.75)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int, default=0.6)

    args = parser.parse_args()

    return args


def main():
    # Argument parsing
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height


    # Camera preparation
    cap = cv.VideoCapture(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)


    use_static_image_mode = args.use_static_image_mode
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    # Initialize mode variable
    mode = 0


    # Model load
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=use_static_image_mode,
        max_num_hands=2,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    # Landmark classifier
    keypoint_classifier = KeyPointClassifier()

    # Read labels
    with open('model/keypoint_classifier/keypoint_classifier_label.csv',
              encoding='utf-8-sig') as f:
        keypoint_classifier_labels = csv.reader(f)
        keypoint_classifier_labels = [
            row[0] for row in keypoint_classifier_labels
        ]

    # FPS 
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Define the ROI in the top-right corner
    roi_x_min = int(cap_width * 0.63)
    roi_y_min = int(cap_height * 0.1)
    roi_x_max = int(cap_width * 0.98)
    roi_y_max = int(cap_height * 0.80)

    hand_sign_text = ""  # To store the current gesture classification
    accuracy_threshold = 0.55  # Define a threshold for gesture confidence

    while True:
        fps = cvFpsCalc.get()

        # Process Key (ESC: end)
        key = cv.waitKey(10)
        if key == 27:  # ESC key
            break
        number, mode = select_mode(key, mode)

        # Camera capture
        ret, image = cap.read()
        if not ret:
            break
        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)

        # Detection implementation
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        hand_in_roi = False

        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                # Bounding box calculation
                boundingRectangle = calc_bounding_rect(debug_image, hand_landmarks)

                # Check if the hand is within the ROI
                if boundingRectangle[0] >= roi_x_min and boundingRectangle[1] >= roi_y_min and boundingRectangle[
                    2] <= roi_x_max and boundingRectangle[3] <= roi_y_max:
                    hand_in_roi = True
                    # Landmark calculation
                    landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                    # Conversion to relative coordinates / normalized coordinates
                    pre_processed_landmark_list = pre_process_landmark(landmark_list)
                    
                    # Write to the dataset file for training
                    logging_csv(number, mode, pre_processed_landmark_list)

                    # Hand sign classification
                    hand_sign_id, hand_sign_prob = keypoint_classifier(pre_processed_landmark_list)

                    # Check if the probability is below the threshold
                    if hand_sign_prob < accuracy_threshold:
                        hand_sign_text = " "
                    else:
                        hand_sign_text = keypoint_classifier_labels[hand_sign_id]
                        if (hand_sign_text != "unknown"):
                            # Increment the counter for the recognized gesture
                            gesture_counter[hand_sign_text] += 1

                            # Check if the gesture has been recognized more than 30 times
                            if gesture_counter[hand_sign_text] > 50:
                                send_command_async(hand_sign_text)
                           
                                # Reset the counter for all gestures after sending a command
                                for gesture in gesture_counter:
                                    gesture_counter[gesture] = 0


        # Highlight the ROI
        if hand_in_roi:
            cv.rectangle(debug_image, (roi_x_min, roi_y_min), (roi_x_max, roi_y_max), (0, 255, 0), 3)
            cv.putText(debug_image, "Ready!", (roi_x_min + 10, roi_y_min - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9,
                       (0, 255, 0), 2)
        else:
            cv.rectangle(debug_image, (roi_x_min, roi_y_min), (roi_x_max, roi_y_max), (0, 0, 255), 3)

        # Display FPS and gesture classification
        debug_image = draw_info(debug_image, fps, mode, number, hand_sign_text)

        # Screen reflection
        cv.imshow('Hand Gesture Recognition', debug_image)

    cap.release()
    cv.destroyAllWindows()


def select_mode(key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    return number, mode


def logging_csv(number, mode, landmark_list):
    if mode == 0:
        pass
    if mode == 1 and (0 <= number <= 9):
        csv_path = 'model/keypoint_classifier/keypoint.csv'
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])

    return


def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []

    # Keypoints
    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point.append([landmark_x, landmark_y])

    return landmark_point


def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convertion to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def draw_info(image, fps, mode, number, hand_sign_text):
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)

    mode_string = ['Logging Key Point', 'Logging Point History']
    if 1 <= mode <= 2:
        cv.putText(image, "MODE:" + mode_string[mode - 1], (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if 0 <= number <= 9:
            cv.putText(image, "NUM:" + str(number), (10, 110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                       cv.LINE_AA)

    # Display current gesture classification
    if hand_sign_text:
        cv.putText(image, "Gesture: " + hand_sign_text, (9, 140), cv.FONT_HERSHEY_SIMPLEX,
           1, (0, 255, 0), 2, cv.LINE_AA)


    return image


if __name__ == '__main__':
    main()
