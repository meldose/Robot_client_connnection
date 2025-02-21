from price_tag import PriceTag
import cv2 as cv

def detect_number_1(image):
    grey_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(grey_image, (5, 5), 0)
 
    _, thresh = cv.threshold(blur, 80, 255, cv.THRESH_BINARY_INV)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (1, 5))
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
    cv.imshow("CropThresh", thresh)
 
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    digits = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        perimeter = cv.arcLength(cnt, True)
 
        epsilon = 0.1 * perimeter
 
        approx = cv.approxPolyDP(cnt, epsilon, True)
        (x, y, w, h) = cv.boundingRect(cnt)
        if 50 < h < 100:
            cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.imshow("Digit", image)
            print(h,w)
 
def detect_number_2(image):
    grey_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(grey_image, (5, 5), 0)
 
    _, thresh = cv.threshold(blur, 80, 255, cv.THRESH_BINARY_INV)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (1, 5))
    thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
    cv.imshow("CropThresh", thresh)
 
    contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    digits = []
    for cnt in contours:
        area = cv.contourArea(cnt)
        perimeter = cv.arcLength(cnt, True)
 
        epsilon = 0.1 * perimeter
 
        approx = cv.approxPolyDP(cnt, epsilon, True)
        (x, y, w, h) = cv.boundingRect(cnt)
        if 50 < h < 100:
            cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.imshow("Digit", image)
            print(h,w)
 
 
# Import Image
tag_image = cv.imread("price_tag_2.jpg")
price_tag = PriceTag(image=tag_image)
 
cropped_images = price_tag.detect_price_tag(image=tag_image)
print(len(cropped_images))
if cropped_images is not None:
    if len(cropped_images) == 2:
        bbox1, bbox2 = cropped_images
        cv.imshow("Price_Tag_1", bbox1)
        cv.imshow("Price_Tag_2", bbox2)
        image_1 = detect_number_1(image=bbox1)
        image_2 = detect_number_2(image=bbox2)
    elif len(cropped_images) == 1:
        bbox = cropped_images[0]
        cv.imshow("Price_Tag", bbox)
        digit_image = price_tag.detect_number(image=bbox)
        price_tag.show_numbers(digit_image)
    else:
        print("More than Two Detection Process, Not Possible")
    #numbers = price_tag.detect_number(image=imgCrop)
    #battery = price_tag.show_numbers(numbers=numbers)
 
# Wait and clean up
cv.waitKey(0)
cv.destroyAllWindows()
 


#######  SAMPLE USAGE #################

 import cv2

# Load the image (replace with a real-time camera feed)
image = cv2.imread("conveyor_frame.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blurred, 50, 150)

# Find contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.imshow("Detected Items", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

#########################################