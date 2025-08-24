#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import cv2
from cv_bridge import CvBridge, CvBridgeError
import base64
import time
import requests
import json
import os
import ocrspace  # Not needed since I will not use the wrapper, in case I would need more info like positioning
import Levenshtein
from medicine_retrieval.srv import OCRService, OCRServiceResponse

cv_bridge = CvBridge()

# OCR Space API key - custom key set up to my email
OCR_SPACE_API_KEY = '#Add your own key!'

# OCR Space API endpoint
OCR_SPACE_API_ENDPOINT = 'https://api.ocr.space/parse/image'

# Load in list of current medicine from the json
meds_filepath = os.path.join(os.path.dirname(__file__), '..', 'config', 'medicines.json')
with open(meds_filepath, 'r') as file:
        meds_data = json.load(file)

MED_NAMES = meds_data.get('medicines', [])


# Threshold for how much of a match it has to be to be considered matched
TEXT_MATCHING_THRESHOLD = 0.2  # Adjust this based on the scores I'm getting, want decent results, as long as they're mostly true (in this case I don't think something would be falsey marked as true, since works are different enough)

# Threshold for when to accept if it is a medicine bottle; right now can be low since we don't have other objects with text on them, but could increase if this is the case #extendability :)
MEDICINE_BOTTLE_THRESHOLD = 0.3

# Whether to save image locally for manual testing/debuggin
SAVE_IMAGE_LOCALLY = False



class OcrServiceHandler:
    def __init__(self):
        self.detected_image = None
        self.bbox_data_latest = None

    def image_callback(self, image_data):
        try:
            # Need to convert the raw image data into OpenCv image
            self.detected_image = cv_bridge.imgmsg_to_cv2(image_data, "bgr8")
            # print('Image Callback occured')
            
        except CvBridgeError as error:
            print(error)


    def bbox_callback(self, bbox_data):
        self.bbox_data_latest = bbox_data



    def handle_ocr_service_request(self, request):
        #darknet_image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback, queue_size=1)
        # Subscribe to the xction image instead
        xction_image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback, queue_size=1)
        darknet_bounding_box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback, queue_size=1)

        time.sleep(5)
        #rospy.sleep(10) # Let darknet send the latest images after ros has settled

        while(self.detected_image is None or self.bbox_data_latest is None): # Only do anything if an image has been first received - note that afterwards there might be a miniscule delay, but the bbox would roughly stay the same and alright for purpose; otherwise could do header time matching
            time.sleep(0.2) # If either thing is not in yet, then have to wait for it to come in


        med_bottle_to_find = request.bottle_to_find
        bottle_found = False
        bottle_number = None

        # Get bounding boxes
        bboxes = self.bbox_data_latest.bounding_boxes

        # Save image for testing/debugging
        if SAVE_IMAGE_LOCALLY:
            # Test by saving to local machine to see it

        # Sort boxes from left to right, using the xmin value of each
        sorted_bboxes = sorted(bboxes, key=lambda box: box.xmin)

        # Loop through each detected object (i.e. each bounding box)
        for i, bbox in enumerate(sorted_bboxes):
            # Get the ROI corresponding to the bounding box
            x1, y1, x2, y2 = bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax
            label = bbox.Class
            prob = bbox.probability  # Probability it is the object it says it is

            object_order = i  # This is the order from left to right, since that's how darknet publishes; for 2d cases, it is probably left to right rowwise, but would have to test if this is the case
            object_bounding_box = [x1, y1, x2, y2]  # Could be useful in future if we want to grab the item - this would help in getting the location since can frame it with where robot is currently standing (since he would be still until all this returns)
            med_name = 'n/a'

            # Perform ocr on it if it is probably a medicine bottle - threshold can be low as we don't have other items with text currently
            if label == 'medicine bottle' and prob >= MEDICINE_BOTTLE_THRESHOLD:
                # since the text is always at the centre of the bottle vertically, I am cropping it here for a better ocr
                bottom_crop_value = int(0.4 * (y2 - y1))
                top_crop_value = int(0.3 * (y2 - y1))
                y2_cropped = y2 - top_crop_value
                y1_cropped = y1 + bottom_crop_value

                # roi = self.detected_image[y1:y2, x1:x2]
                roi = self.detected_image[y1_cropped:y2_cropped, x1:x2]


                # Perform Ocr on the roi (region of interest - the area in the image which has the text)
                print('Performing OCR')
                ocr_result = self.ocr_space_api(roi, object_order)

                # Do the spicy name matching here
                text_read = ocr_result
                med_name, prob_of_name = self.match_text(text_read) # Lev text (or rename) should give most likely med name with certainty percentage - if none above threshold, then unknown
                                          # It should use a list provided to find scores for all which are above threshold, then return highest one, and the corresponding probability
                                            # List and probabilities set above
                if med_name == med_bottle_to_find:
                    bottle_found = True
                    bottle_number = i

                print('Recognized text for object "{}": {} ; with {} probability'.format(object_order, med_name, prob_of_name))
                print('(Original text was "{}")'.format(text_read))


        return OCRServiceResponse(bottle_found, bottle_number)

                # Redundant - For testing, save the file to a folder
                # if SAVE_IMAGE_LOCALLY:
                    # file_name = 'roi' + str(object_order) + med_name + '.png'
                    # file_path = os.path.join(#Your test folder, file_name)
                    # cv2.imwrite(file_path, roi)


    def match_text(self, original_text):
        matched_name = 'Unknown'
        match_probability = 0

        for med_name in MED_NAMES:
            prob = Levenshtein.ratio(med_name, original_text)
            # print (prob)
            if prob > match_probability and prob > TEXT_MATCHING_THRESHOLD:
                match_probability = prob
                matched_name = med_name

        return matched_name, match_probability



    def ocr_space_api(self, roi_image, order_num):
        # Convert the ROI to a base64-encoded string - format it to send to api
        # _, encoded_roi = cv2.imencode('.jpg', roi_image)
        # image_data = encoded_roi.tobytes()
        # image_base64 = base64.b64encode(image_data).decode()

        image_base64 = self._imageprocesandformat_roi(roi_image, order_num)


        # Do not use - since we cannot save/load in deployment
        # with open(testfile_path, 'rb') as img:
        #     img_data = img.read()
        # image_base64 = base64.b64encode(img_data).decode('utf-8')

        # Set paramaters for api
        payload = {
            'apikey': OCR_SPACE_API_KEY,
            'base64Image': 'data:image/jpg;base64,' + image_base64,
            'language': 'eng',
            'filetype': 'jpeg',
            'isOverlayRequired': False  # Since doing 1 roi at a time, this is not necessary at the moment
        }

        # Send Ocr request to the OCR Space api
        response = requests.post(OCR_SPACE_API_ENDPOINT, data=payload)
        response.raise_for_status()

        # Parse the OCR response
        result = json.loads(response.content.decode())
        ocr_result = ''
        try:
            ocr_result = result["ParsedResults"][0]["ParsedText"]

            # if SAVE_IMAGE_LOCALLY:
            #     testresultfile_name = str(order_num) + '_actual_rosResultCode.json'
            #     testresultfile_path = os.path.join(#Your test folder, testresultfile_name)
            #     with open(testresultfile_path, "w") as jsn:
            #         json.dump(result, jsn)

        except KeyError:
            print('Keyerror')
        except IndexError:
            print('Index Error - probably not able to perform - ocr server issue')

            # if SAVE_IMAGE_LOCALLY:
            #     # testfile_name = 'roi.png'
            #     # testfile_path = os.path.join(#Your test folder, testfile_name)
            #     # with open(testfile_path, "wb") as img:
            #     #     img.write(image_base64)
            #     testresultfile_name = 'actual_rosResultCode.json'
            #     testresultfile_path = os.path.join(#Your test folder, testresultfile_name)
            #     with open(testresultfile_path, "w") as jsn:
            #         json.dump(result, jsn)

        return ocr_result



    def _imageprocesandformat_roi(self, roi_image, order_num):
        # Convert to grayscale
        gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        # Enhance contrast with adaptive thresholding
        # threshed = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

        # These are currently good parameters which work well here
        sharpened = cv2.GaussianBlur(gray, (0,0), 3)
        sharpened = cv2.addWeighted(gray, 1.5, sharpened, -0.5, 0)
        # sharpened = cv2.detailEnhance(sharpened, sigma_s=15, sigma_r=0.1)

        # sharpened = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)


        # kernel_size = (5, 5)
        # sigma = 1.0
        # amount = 1.0
        # threshold = 0
        # # Apply an unsharp mask to the image."""
        # blurred = cv2.GaussianBlur(gray, kernel_size, sigma)
        # sharpened = float(amount + 1) * gray - float(amount) * blurred
        # sharpened = cv2.max(sharpened, threshold * gray)
        # sharpened = cv2.min(sharpened, 255)
        # sharpened = sharpened.astype('uint8')

        # Saving the image for manual inspection
        if SAVE_IMAGE_LOCALLY:
            testfile_name = str(order_num) + '_sent_roi.jpg'
            testfile_path = os.path.join(#Your test folder, testfile_name)
            cv2.imwrite(testfile_path, sharpened)

        # Convert to base64 image
        _, encoded_roi = cv2.imencode('.jpg', sharpened)
        image_data = encoded_roi.tobytes()
        image_base64 = base64.b64encode(image_data).decode()

        return image_base64



if __name__ == '__main__':
    rospy.init_node('ocr_service_node', anonymous=False)

    ocr_class = OcrServiceHandler()
    # Subscribers to get the latest image and bounding boxes from Darknet - ready for whenever OCR Service needed
    # darknet_image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, ocr_class.image_callback, queue_size=1)
    # darknet_bounding_box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, ocr_class.bbox_callback, queue_size=1)

    ocr_service = rospy.Service('ocr_service', OCRService, ocr_class.handle_ocr_service_request)

    # print('Class should be started started if this prints, and list of meds:')
    # print(MED_NAMES)
    rospy.spin()

