import serial
import time
import numpy as np
import sys
import pygame.joystick
from PIL import Image
from PIL import ImageDraw
import cv2
import tflite_runtime.interpreter as tflite
import detect
import platform
import collections

EDGETPU_SHARED_LIB = {
  'Linux': 'libedgetpu.so.1',
  'Darwin': 'libedgetpu.1.dylib',
  'Windows': 'edgetpu.dll'
}[platform.system()]

def load_labels(path, encoding='utf-8'):
  """Loads labels from file (with or without index numbers).

  Args:
    path: path to label file.
    encoding: label file encoding.
  Returns:
    Dictionary mapping indices to labels.
  """
  with open(path, 'r', encoding=encoding) as f:
    lines = f.readlines()
    if not lines:
      return {}

    if lines[0].split(' ', maxsplit=1)[0].isdigit():
      pairs = [line.split(' ', maxsplit=1) for line in lines]
      return {int(index): label.strip() for index, label in pairs}
    else:
      return {index: line.strip() for index, line in enumerate(lines)}

def make_interpreter(model_file):
  model_file, *device = model_file.split('@')
  return tflite.Interpreter(
      model_path=model_file,
      experimental_delegates=[
          tflite.load_delegate(EDGETPU_SHARED_LIB,
                               {'device': device[0]} if device else {})
      ])

def initSerial():
    ser = serial.Serial('COM5', 115200, timeout = 2)
    #Wait until serial port opens, or exit if timeout reached
    serStartTime = time.time()
    serTimeout = 2
    serEndTime = serStartTime + serTimeout
    while ser.isOpen == False:
        if time.time() > serEndTime:
            ser.close()
            raise TimeoutError("COM port failed to open")
    return ser

def findController(_ser):
    #TODO don't like that ser is passed to this function
    #Is any controller connected?
    pygame.init()
    try:
        xbox = pygame.joystick.Joystick(0)
    except pygame.error as e:
        #TODO is this the right way to do exception?
        _ser.close()
        print("Controller not connected...exiting")
        raise e

    #Is connected controller an xbox controller?
    if (xbox.get_name() == "Controller (XBOX 360 For Windows)"):
        print("XBOX Controller found")
        xbox.init()
        return xbox
    else:
        _ser.close()
        #TODO is this the right way to do exception?
        raise Exception("XBOX Controller not found...exiting")

def sendArduino(_msg, _ser):
    #expects string
    _ser.write((_msg[:6] + '\n').encode())

def receiveArduino(_ser):
    #returns string
    return _ser.readline().decode('ascii')[:-1] 

def main():
    # setup xbox things
    ser = initSerial()
    xbox = findController(ser)
    
    # xbox mappings
    xboxAxisId = {'LS_H':0,'LS_V':1,'LT/RT':2,'RS_V':3,'RS_H':4}
    xboxAxisPolarity = {'LS_H':1,'LS_V':-1,'LT/RT':0,'RS_V':0,'RS_H':0} #TODO haven't worked out last 3 yet
    xboxButton = {'A':0, 'B':1,'X':2,'Y':3,'LB':4,'RB':5,'BACK':6,'START':7,'LS':8,'RS':9}
    xboxHat = {'DPAD':0}

    # load detection model and labels
    labels = "models/coco_labels.txt"
    model = "models/ssd_mobilenet_v1_coco_quant_postprocess_edgetpu.tflite"
    threshold = 0.6

    # get labels from labels text file
    labels = load_labels(labels)
    # set up interpreter
    interpreter = make_interpreter(model)
    interpreter.allocate_tensors()

    # initialise webcam
    video = cv2.VideoCapture(0)
    fov = 90 # camera fov in deg

    # controller mode constants
    ampH = 45 # peak amplitude of desired H stepper motor movement in degrees
    ampV = 10 # peak amplitude of desired V stepper motor movement in degrees

    angle_buf_len = 5
    angle_buffer = np.zeros(shape=(angle_buf_len,2)).tolist()
    active = True
    mode = "controller"
    while active: #breaks on q key being hit or xbox Y button
        print("Mode: {}".format(mode))
        start = time.perf_counter()

        # get frame from webcam
        ret, frame = video.read()

        # Determine what mode we're in
        if mode == "controller":
            # filter joystick events and send commands to arduino
            for event in pygame.event.get(): #get all events since last loop
                if event.type == pygame.JOYAXISMOTION: #has a joystick moved
                    if event.axis == xboxAxisId['LS_H']:
                        deg = ampH * xboxAxisPolarity['LS_H'] * event.value #scale stick value to amplitude
                        msg = 'H' + str(deg)
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                    elif event.axis == xboxAxisId['LS_V']:
                        deg = ampV * xboxAxisPolarity['LS_V'] * event.value #scale stick value to amplitude
                        msg = 'V' + str(deg)
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == xboxButton['RB']:
                        msg = 'T0'
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                    elif event.button == xboxButton['LB']:
                        # if LB is released, go to controller mode next loop
                        mode = "controller"
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == xboxButton['RB']:
                        msg = 'T1'
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                    elif event.button == xboxButton['LB']:
                        # if LB is pressed, go to auto mode next loop
                        mode = "auto"
                        # reset angle buffer
                        angle_buffer = np.zeros(shape=(angle_buf_len,2)).tolist()
                    elif event.button == xboxButton['Y']:
                        # stop program if Y is pressed
                        active = False

        elif mode == "auto":
            # convert colours
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # get image into PIL format for loop to work
            image = Image.fromarray(frame_rgb)
            scale = detect.set_input(interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS))

            # detect objects in the frame
            interpreter.invoke()
            objs = detect.get_output(interpreter, threshold, scale)

            print('Detection results:')
            if not objs:
                print('No objects detected')
            else:
                # print each detected object's results
                for obj in objs:
                    print('  class: ', labels.get(obj.id, obj.id))
                    print('  id:    ', obj.id)
                    print('  score: ', obj.score)
                    print('  bbox:  ', obj.bbox)
                    # draw bbox
                    cv2.rectangle(frame, (obj.bbox.xmin, obj.bbox.ymax), (obj.bbox.xmax, obj.bbox.ymin), (0,0,255), 1)
            
            # send tracking commands to arduino
            # calculate centre of bbox if something detected
            if objs:
                # get 1st person
                # TODO check if multiple people
                # TODO make sure it's a person, not another object type
                p = objs[0]
                # find the centre of the bbox
                p_centre = [int((p.bbox.xmax + p.bbox.xmin)/2), int((p.bbox.ymax + p.bbox.ymin)/2)]
                cv2.circle(frame, tuple(p_centre), 5, color=(0,0,255), thickness=-1, lineType=8, shift=0)
                # calculate delta angle from centre of bbox to centre of frame in H and V
                height, width, channels = frame.shape
                frame_centre = [int(width/2), int(height/2)]
                # screen coordinates start top left, so a positive y delta, is going down not up
                screen_error = [p_centre[0] - frame_centre[0], frame_centre[1] - p_centre[1]]
                angle_error = [fov * screen_error[0] / width, fov * screen_error[1] / height]
                kp = 0.25
                angle_demand = [kp*angle_error[0], kp*angle_error[1]]
                # draw line from frame centre to target centre
                cv2.line(frame, tuple(frame_centre), tuple(p_centre), color=(0,165,255), thickness=1, lineType=8, shift=0)
                # calc rolling average of the angle based on the last n frames
                # TODO ignore values if they are too far from the last mean
                a = collections.deque(angle_buffer)
                a.rotate(1)
                angle_buffer = list(a)
                angle_buffer[0] = angle_demand
                angle_sum = [0, 0]
                for angle in angle_buffer:
                    angle_sum[0] = angle_sum[0] + angle[0]
                    angle_sum[1] = angle_sum[1] + angle[1]
                angle_mean = [angle_sum[0] / len(angle_buffer), angle_sum[1] / len(angle_buffer)]
                # send angles to arduino
                msgH = 'H' + str(angle_mean[0])
                sendArduino(msgH, ser)
                print(receiveArduino(ser))
                msgV = 'V' + str(angle_mean[1])
                sendArduino(msgV, ser)
                print(receiveArduino(ser))

            # filter joystick events and send user commands to arduino
            for event in pygame.event.get(): #get all events since last loop
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == xboxButton['RB']:
                        msg = 'T0'
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                    elif event.button == xboxButton['LB']:
                        # if LB is released, go to controller mode next loop
                        mode = "controller"
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == xboxButton['RB']:
                        msg = 'T1'
                        sendArduino(msg, ser)
                        print(receiveArduino(ser))
                    elif event.button == xboxButton['LB']:
                        # if LB is pressed, go to auto mode next loop
                        mode = "auto"
                        # reset angle buffer
                        angle_buffer = np.zeros(shape=(angle_buf_len,2)).tolist()
                    elif event.button == xboxButton['Y']:
                        # stop program if Y is pressed
                        active = False
            
        else:
            print("No mode detected, exiting...")
            active = False

        # draw crosshair over everything else
        height, width, channels = frame.shape
        ptv1 = (int(width/2), 0)
        ptv2 = (int(width/2), height)
        cv2.line(frame, ptv1, ptv2, color=(0,0,0), thickness=1, lineType=8, shift=0)
        pth1 = (0, int(height/2))
        pth2 = (width, int(height/2))
        cv2.line(frame, pth1, pth2, color=(0,0,0), thickness=1, lineType=8, shift=0)

        # all the results have been drawn on the frame, so it's time to display it
        cv2.imshow('Turret Cam', frame)

        # print loop approximate fps
        loop_time = time.perf_counter() - start
        print("FPS: %.1f" % (1 / loop_time))

        # check if q has been pressed to quit
        if cv2.waitKey(1) == ord('q'):
            active = False
    
    # clean up
    video.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()