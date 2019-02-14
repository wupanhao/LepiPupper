from flask import Flask,jsonify,request,json
from flask_cors import CORS
import numpy as np
try:
    from HardwareConfig import NEUTRAL_ANGLE_DEGREES
except Exception as e:
    print(e)
    NEUTRAL_ANGLE_DEGREES = np.array([[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,0.]])
angles = []
for i in range(4):
    for j in range(3):
        angles.append(NEUTRAL_ANGLE_DEGREES[j,i])
def toConfig(angles):
    degrees = [[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    for i in range(4):
        for j in range(3):
            degrees[j][i] = angles[i*3+j]
    return str(degrees)

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
    return jsonify(angles)

@app.route('/save')
def save():
    try:
        with open('./HardwareConfig.py','w') as f:
            f.write(template.replace('SERVO_ARRAY_PLACEHOLDER',toConfig(angles)))
    except Exception as e:
        print(e)
        return jsonify({"status":1})
    return jsonify({"status":0})

if __name__ == '__main__':
  app.run(port=8080,host='0.0.0.0')
