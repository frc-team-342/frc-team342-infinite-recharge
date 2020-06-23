# Requires pip modules pynetworktables, numpy, flask, and opencv-python
from flask import Flask, Response
from networktables import NetworkTables

import numpy as np

import time
import threading

app = Flask(__name__)
camera_switch = False


# Continually returns frames grabbed from the correct camera
def generate():
    while True:
        if camera_switch:
            # Insert grab frame code here
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + bytearray(np.zeros((480, 360, 3), np.uint8)) + b'\r\n\r\n') 
        else:            
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + bytearray(np.zeros((480, 360, 3), np.uint8)) + b'\r\n\r\n') 


@app.before_first_request
def nt_thread():
    # When running on a robot, this should be set to 10.3.42.2, set to localhost when testing with OutlineViewer
    NetworkTables.initialize("localhost")
    table = NetworkTables.getTable("camera")

    def run():
        global camera_switch
        while True:
            camera_switch = table.getBoolean("camera", False)
            time.sleep(0.2)

    thread = threading.Thread(target=run)
    thread.start()


@app.route('/')
def camera_switcher():
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


# Run using "python3 app.py" instead of the Flask module
app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
