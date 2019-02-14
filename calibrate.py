from flask import Flask,jsonify,request,json
from flask_cors import CORS
import numpy as np
import math
from pupper.HardwareInterface import HardwareInterface

try:
    from pupper.HardwareConfig import NEUTRAL_ANGLE_DEGREES
except Exception as e:
    print(e)
    NEUTRAL_ANGLE_DEGREES = np.array([[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,0.]])
angles = []
for i in range(4):
    for j in range(3):
        angles.append(NEUTRAL_ANGLE_DEGREES[j,i])
def toConfig(angles):
    res = np.zeros((3,4))
    for i in range(4):
      for j in range(3):
        res[j][i] = angles[i*3+j]
    return res.tolist()

def toArray(res):
    angles = [0 for i in range(12)]
    for i in range(3):
        for j in range(4):
            angles[j*3+i] = res[i][j]
    return angles

hardware_interface = HardwareInterface()

print(angles)
template = '''
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
SERVO_ARRAY_PLACEHOLDER
,dtype = 'float')

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
'''

app = Flask(__name__)
CORS(app, supports_credentials=True)

@app.route('/')
def index():
    with open('./index.html','r') as f:
        return f.read()

@app.route('/get')
def get():
    return jsonify(angles)

@app.route('/set')
def set():
    global angles
    data = request.args.get('angles')
    try:
      array = json.loads(data)
      if type(array) == list and len(array) == 12:
        angles = array
    except Exception as e:
        print(e)

    stand = [0,0,0,0, 44, 44, 44, 44, -48, -48, -48, -48]
    #stand = [0,0,0,0,45,45,45,45,-45,-45,-45,-45]
    #stand = [-12, 12, -12, 12, 61, 61, 61, 61, -70, -70, -70, -70]
    rad = np.array(stand).reshape((3,4))
    rad = rad/180.0*math.pi
    print(rad)
    hardware_interface.servo_params.neutral_angle_degrees = np.array(toConfig(angles))
    hardware_interface.set_actuator_postions(rad)
    return jsonify(angles)

@app.route('/save')
def save():
    try:
        with open('./pupper/HardwareConfig.py','w') as f:
            f.write(template.replace('SERVO_ARRAY_PLACEHOLDER',str(toConfig(angles))))
    except Exception as e:
        print(e)
        return jsonify({"status":1})
    return jsonify({"status":0})

if __name__ == '__main__':
  app.run(port=8080,host='0.0.0.0')
