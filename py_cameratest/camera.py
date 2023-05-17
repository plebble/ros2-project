import cv2
import base64

"""
camera = cv2.VideoCapture(2)
ret, frame = camera.read()
print(frame.shape)
print(ret)
ret, jpg_encoded = cv2.imencode(".jpg",frame)
print(ret)
image_string = base64.b64encode(jpg_encoded)
#print(image_string)

original = base64.b64decode(image_string)
with open('test.jpg', 'wb') as f_output:
    f_output.write(original)

camera.release()
cv2.destroyAllWindows()
"""

for i in range(0,10):
    camera = cv2.VideoCapture(i)

    try:
        print("{} : {}".format(i,camera.getBackendName()))
    except:
        print("{} : nothing".format(i))

