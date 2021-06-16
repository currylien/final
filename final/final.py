import pyb
import sensor, image, time, os, math, tf

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)
tmp = ""
THRESHOLD = (34, 8, -118, 127, -122, -7)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

#labels = ['3', '4', '0', 'other']
f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

def AprilTags_Decoding():
        clock.tick()
        img = sensor.snapshot()
        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
            img.draw_rectangle(tag.rect(), color = (255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
            # The conversion is nearly 6.2cm to 1 -> translation
            print_args = (tag.z_translation())
            # Translation units are unknown. Rotation units are in degrees.
            print("Tz %f" % print_args)
            if tag.z_translation() < -2.6:
                detection = '1'
            elif tag.z_translation() >= -2.6:
                detection = '2'
            else:
                detection = '0'
            return detection

def image_classification():
   img = sensor.snapshot(())
   net = "trained.tflite"
   labels = [line.rstrip('\n') for line in open("labels.txt")]
   # default settings just do one detection... change them to search the image...
   for obj in tf.classify(net, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
      img.draw_rectangle(obj.rect())
      img.draw_string(obj.x()+3, obj.y()-1, labels[obj.output().index(max(obj.output()))], mono_space = False)
   return labels[obj.output().index(max(obj.output()))]
'''
def line_detection():
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    l = img.get_regression([(100,100)], robust = True)
    if (l):
        img.draw_line(l.line(), color = 127)
        print_args = (l.x1(),l.y1(),l.x2(),l.y2())
        print("%f,%f,%f,%f" % print_args)
        if (l.theta() < 100 and l.theta() > 90) or (l.x2() <20): #or (l.x2() < 20):
            detection = '2'
        elif (l.theta() > 70 and l.theta() < 90) or (l.x2() > 70): #or (l.x2() >70):
            detection = '1'
        else:
            detection = '0'
        print(detection)
        return detection
'''
while(1):
   a = uart.readline()
   if a is not None:
      tmp += a.decode()
      #print(tmp)
   if tmp == "AprilTags_Decoding\0":
      tmp =""
      result = AprilTags_Decoding()
      #print("%s" % result)
      uart.write(("%s" % result).encode())
      uart.readchar()
   if tmp == "image_classification\0":
      #print("classify images")
      tmp =""
      label = image_classification()
      print(label)
      uart.write(label.encode())
      uart.readchar()
   if tmp == "line_detection\0":
      #print("classify images")
      '''
      tmp =""
      result = line_detection()
      uart.write(("%s" % result).encode())
      uart.readchar()
      '''
      sensor.reset()
      sensor.set_vflip(True)
      sensor.set_hmirror(True)
      sensor.set_pixformat(sensor.RGB565)
      sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
      #sensor.set_windowing([0,20,80,40])
      sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
      clock = time.clock()                # to process a frame sometimes.

      while(True):
          clock.tick()
          img = sensor.snapshot().binary([THRESHOLD])
          l = img.get_regression([(100,100)], robust = True)
          if (l):
              img.draw_line(l.line(), color = 127)
              print_args = (l.x1(),l.y1(),l.x2(),l.y2())
              print("%f,%f,%f,%f" % print_args)
              if (l.theta() < 100 and l.theta() > 90) or (l.x2() <20): #or (l.x2() < 20):
                 detection = 2
                 s = 0.7
              elif (l.theta() > 70 and l.theta() < 90) or (l.x2() > 70): #or (l.x2() >70):
                 detection = 1
                 s = 0.7

              else:
                 detection = 0
                 s = 0.5
              uart.write(("%d\r\n" % detection).encode())
              #print("%d" % detection)
              #print(l.theta())
              #print(l.rho())
              time.sleep(0.8)
