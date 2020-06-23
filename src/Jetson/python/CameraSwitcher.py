# Requires pip modules pynetworktables, flask, and opencv-python
from flask import Flask, Response
from networktables import NetworkTables

import time
import threading

app = Flask(__name__)
camera_switch = False


# Continually returns frames grabbed from the correct camera
def generate():
    while True:
        if camera_switch:
            # Insert grab frame code here
            yield "1"
        else:
            yield "2"


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
    if not (next(generate()) == "1" or next(generate()) == "2"):
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return Response(generate())


# Run using "python3 app.py" instead of the Flask module
app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
