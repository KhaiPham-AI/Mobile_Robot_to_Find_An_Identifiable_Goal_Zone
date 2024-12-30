from flask import Flask, render_template, Response, request, redirect, url_for, session, jsonify
import threading
import argparse
import time
import cv2
import torch
import time
import numpy as np
import serial
import json

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
model = torch.hub.load(".", 'custom', path='best.pt', source='local')
model.to(device)
model.conf = 0.5
class_names = model.names

outputFrame = None
lock = threading.Lock()
app = Flask(__name__)
app.secret_key = 'supersecretkey'  # Thay thế bằng một giá trị bảo mật hơn


action = False
script = 0
angle = 0
running = False
targetClass = 0
normal_speed = 0 

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=480,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
# Thiết lập kết nối serial với Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600)

cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
time.sleep(1.0)
def generate_data(action):
    global  script, angle, normal_speed
    data = {
        "action": int(action),
        "script": script,
        "angle": angle,
        "normal_speed": normal_speed
    }
    return data
def send_data(ser, data):
    json_data = json.dumps(data)
    ser.write(json_data.encode('utf-8'))
    ser.write(b'\n')  # Gửi thêm ký tự xuống dòng để Arduino biết khi nào kết thúc một gói tin
def send_to_arduino(ser, action):
    data = generate_data(action)
    send_data(ser, data)
def calculate_angle_between_vectors(x1, y1, x2, y2, x3, y3, x4, y4):
    """
    Calculates the angle between two vectors.
    
    Parameters:
    x1, y1 (float): The x and y coordinates of the first point of the first vector.
    x2, y2 (float): The x and y coordinates of the second point of the first vector.
    x3, y3 (float): The x and y coordinates of the first point of the second vector.
    x4, y4 (float): The x and y coordinates of the second point of the second vector.
    
    Returns:
    float: The angle between the two vectors in degrees.
    """
    # Calculate the two vectors
    vector1 = np.array([x2 - x1, y2 - y1])
    vector2 = np.array([x4 - x3, y4 - y3])
    
    # Tính dot product của hai vector
    dot_product = np.dot(vector1, vector2)

    # Tính norm 2 của hai vector
    norm1 = np.linalg.norm(vector1)
    norm2 = np.linalg.norm(vector2)

    # Tính góc giữa hai vector (radian)
    angle_radian = np.arccos(dot_product / (norm1 * norm2))

    # Chuyển góc từ radian sang độ
    angle_degree = np.degrees(angle_radian)
    if vector1[0] > vector2[0]:
        angle_degree = -angle_degree
    
    return angle_degree

def create_and_display_vector(image, x1, y1, x2, y2, color=(0, 255, 0), thickness=2):
    """
    Creates and displays a vector on an image.
    
    Parameters:
    image (numpy.ndarray): The input image.
    x1 (float): The x-coordinate of the first point.
    y1 (float): The y-coordinate of the first point.
    x2 (float): The x-coordinate of the second point.
    y2 (float): The y-coordinate of the second point.
    color (tuple, optional): The color of the vector in BGR format. Defaults to (0, 255, 0) (green).
    thickness (int, optional): The thickness of the vector line. Defaults to 2.
    
    Returns:
    numpy.ndarray: The image with the vector displayed.
    """
    # Create the vector
    vector = np.array([x2 - x1, y2 - y1])
    
    # Display the vector on the image
    img_with_vector = cv2.arrowedLine(image.copy(), (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
    
    return img_with_vector


@app.route("/")
def index():
    if not session.get('logged_in'):
        return redirect(url_for('login'))
    return render_template("view.html")
@app.route("/input", methods=["POST"])
def input():
     global action, script, angle, normal_speed
     global ser
     data = request.get_json()
     input_value = data.get("input")
     normal_speed = float(input_value)
     send_to_arduino(ser,-1)
     return jsonify(success=True)
@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        password = request.form.get("password")
        if password == "121002":  # Thay thế bằng mật khẩu của bạn
            session['logged_in'] = True
            return redirect(url_for('index'))
        else:
            return "Invalid password!"
    return render_template("login.html")

@app.route("/logout")
def logout():
    session['logged_in'] = False
    return redirect(url_for('login'))

@app.route("/button", methods=["POST"])
def button():
    global action
    data = request.get_json()
    is_green = data.get("isGreen")
    if is_green:
        action = True
    else:
        action = False
    return jsonify(success=True)
@app.route("/reset", methods=["POST"])
def reset():
    global action, script, angle, normal_speed, targetClass
    global ser
    action = False  # Đặt lại hành động về False
    script = 0
    angle = 0
    normal_speed = 0
    targetClass = 0
    send_to_arduino(ser,action)
    return jsonify(success=True)

def detect_function(frameCount):
    global cap, outputFrame, lock
    global action, script, angle, targetClass
    global ser
    while True:
        _, frame = cap.read()
        if(action):
            if script == 3:
                action = False
                send_to_arduino(ser, action)
                continue
            # print("Dang hoat dong")
            start_time = time.time()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model(frame)
            results_list = []
            for detection in results.xyxyn[0]:
                results_list.append(detection.tolist())
            
            results_list = list(filter(lambda x: len(x) >= 6 and int(x[5]) == targetClass, results_list))
            ratio_area = 0
            if(len(results_list) > 0):
                results_list = sorted(results_list, key=lambda x: x[4],reverse=True)
                # print(results_list[-1])
                detection = results_list[-1]
                # print(detection)
                # for detection in results_list:
                x1, y1, x2, y2, confidence, class_id = detection
                h, w, _ = frame.shape

                #
                box_area = abs(x1*w-x2*w) * abs(y1*h-y2*h)
                frame_area = h * w
                ratio_area = box_area / frame_area

                x1, y1, x2, y2 = int(x1*w), int(y1*h), int(x2*w), int(y2*h)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                
                if script == 1:
                    script = 2
                
                if(ratio_area > 0.04):
                    if script == 0:
                        script = 1
                        targetClass = 1
                    if script == 2:
                        script = 3
                # print("Tỉ lệ diện tích của box đối tượng so với khung hình:", ratio_area)

                frame = create_and_display_vector(frame, w//2, h, center_x, center_y)
                frame = create_and_display_vector(frame, w//2, h, w//2, 0)
                angle = calculate_angle_between_vectors(w//2, h, w//2, 0,w//2, h, center_x, center_y)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"{class_names[int(class_id)]} {confidence:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            fps = 1 / (time.time() - start_time)
            cv2.putText(frame,f"FPS:{fps:.2f},Action:{action},Script:{script},Angle:{angle:.3f},Ratio:{ratio_area*100:.3f}",(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
            print(f"FPS:{fps:.2f},Action:{action},Script:{script},Angle:{angle:.3f},Ratio:{ratio_area*100:.3f}")
            frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)   
#            cv2.imshow("CSI Camera", frame)

            send_to_arduino(ser,action)
        
        with lock:
            outputFrame = frame.copy()

def generate():
    global outputFrame, lock
    while True:
        with lock:
            if outputFrame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
            if not flag:
                continue
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encodedImage) + b'\r\n')

@app.route("/video_feed")
def video_feed():
    if not session.get('logged_in'):
        return redirect(url_for('login'))
    return Response(generate(),
        mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, default="0.0.0.0",
        help="ip address of the device")
    ap.add_argument("-o", "--port", type=int, default=8000,
        help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=32,
        help="# of frames used to construct the background model")
    args = vars(ap.parse_args())
    t = threading.Thread(target=detect_function, args=(args["frame_count"],))
    t.daemon = True
    t.start()
    app.run(host=args["ip"], port=args["port"], debug=True,
        threaded=True, use_reloader=False)