**Final Project**  
main.cpp  
```c
#include"mbed.h"
#include "bbcar.h"
#include "mbed_rpc.h"
#include "bbcar_rpc.h"
BufferedSerial xbee(D10, D9);

BufferedSerial pc(USBTX,USBRX); //tx,rx
BufferedSerial uart(D1,D0); //tx,rx
Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BBCar car(pin5, pin6, servo_ticker);
DigitalInOut pin8(D8);
Thread thread1;
Thread thread2;
char recv[1];
int mode = 0;


void recieve_thread(){                     // Thread function for recieving data through uart
   while(1) {
      if(uart.readable()){
         uart.read(recv, sizeof(recv));
         // print out the recieved data
         pc.write(recv, sizeof(recv));
         printf("\r\n");
         if (mode == 3) break;
      }
   }
}

void send_thread(){                              // send string through uart to open mv
   while(1){
      // if button is pressed
      if(mode == 1){
         char s[] = "AprilTags_Decoding\0";      // mode for AprilTags detecting
         uart.write(s, sizeof(s));
         printf("send\r\n");
         ThisThread::sleep_for(400ms);
      } else if (mode == 2) {
          char s[] = "image_classification\0";   // mode for image classifying
          uart.write(s, sizeof(s));
          printf("send\r\n");
         ThisThread::sleep_for(1s);
      } else if (mode == 3) {
          char s[] = "line_detection\0";         // mode for line detection
          uart.write(s, sizeof(s));
          printf("send\r\n");
          mode = 0;
      }
   }
}

int main(){
   pc.set_baud(9600);
   xbee.set_baud(9600);
   uart.set_baud(9600);
   thread1.start(send_thread);
   thread2.start(recieve_thread);
   parallax_ping  ping1(pin8);
   char buf[] = "Start to scan AprilTag\r\n";
   xbee.write(buf, sizeof(buf));
   mode = 1;
   while(1) {                                   // scan AprilTags to control BB car
       if (recv[0] == '1') {
           car.goStraight(50);
           ThisThread::sleep_for(200ms);
           //car.stop();
           //ThisThread::sleep_for(200ms);
       } else if (recv[0] == '2') {
           //mode = 1;
           car.stop();
           break;
       } else {
           car.stop();
       }
   }
   ThisThread::sleep_for(500ms);                // got to classifying images
   mode = 0;
   char buf1[] = "Turn left\r\n";
   xbee.write(buf1, sizeof(buf1));
   car.turn(-50, 1);
   ThisThread::sleep_for(1000ms);
   car.stop();
   ThisThread::sleep_for(1s);
   car.goStraight(50);
   ThisThread::sleep_for(1600ms);
   car.stop();
   car.turn(50,1);
   ThisThread::sleep_for(950ms);
   car.stop();
   char buf2[] = "Scan number\r\n";             // start to scan number
   xbee.write(buf2, sizeof(buf2));
   mode = 2;
   ThisThread::sleep_for(5000ms);
   xbee.write(recv, sizeof(recv));              // print out result
   mode = 0;
   char buf5[] = "Turn right\r\n";              // go to next task
   xbee.write(buf5, sizeof(buf5));
   car.turn(50,1);
   ThisThread::sleep_for(1120ms);
   car.goStraight(50);
   ThisThread::sleep_for(5000ms);
   xbee.write(buf5, sizeof(buf5));
   car.turn(50,1);
   ThisThread::sleep_for(980ms);
   car.stop();
   ThisThread::sleep_for(1s);
   char buf3[] = "Start to detect line\r\n";    // start to detect line
   xbee.write(buf3, sizeof(buf3));
   mode = 3;
   char buf4[] = "Go Straight and finish demo\r\n";
   xbee.write(buf4, sizeof(buf4));
   while(1){                                    // using line image to controll BB car
      if(uart.readable()){
            char recv[1];
            uart.read(recv, sizeof(recv));
            if (recv[0] == '1') {
                car.turn(50,0);
                ThisThread::sleep_for(200ms);
                car.stop();
                ThisThread::sleep_for(500ms);
            } else if (recv[0] == '2') {
                car.turn(-50,0);
                ThisThread::sleep_for(200ms);
                car.stop();
                ThisThread::sleep_for(500ms);
            } else if (recv[0] == '0') {
                car.goStraight(50);
                ThisThread::sleep_for(500ms);
                car.stop();
            } else {
                car.stop();
            } 
      }
   }
}
```
XBee_host.py  
```python
import serial
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)               # set up baud rate
line = s.read(25)                             # read string send from XBee on the car
print(line.decode()) 
line = s.read(12)
print(line.decode())
line = s.read(14)
print(line.decode())
line = s.read(4)
print(line.decode())
line = s.read(13)
print(line.decode())
line = s.read(13)
print(line.decode())
line = s.read(23)
print(line.decode())
line = s.read(30)
print(line.decode())
s.close()
```
final.py
```python
import pyb
import sensor, image, time, os, math, tf

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)
tmp = ""
THRESHOLD = (34, 8, -118, 127, -122, -7)                          # Threshold for line detection
sensor.reset()
sensor.set_vflip(True)                                            # since open mv is opposite, we fhave to flip it
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

def AprilTags_Decoding():                                           # function for scanning AprilTags 
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
            return detection                                         # return a char back to BB car

def image_classification():                                          # function for image classification
   img = sensor.snapshot(())
   net = "trained.tflite"
   labels = [line.rstrip('\n') for line in open("labels.txt")]
   # default settings just do one detection... change them to search the image...
   for obj in tf.classify(net, img, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
      img.draw_rectangle(obj.rect())
      img.draw_string(obj.x()+3, obj.y()-1, labels[obj.output().index(max(obj.output()))], mono_space = False)
   return labels[obj.output().index(max(obj.output()))]
while(1):
   a = uart.readline()
   if a is not None:
      tmp += a.decode()
      #print(tmp)
   if tmp == "AprilTags_Decoding\0":                         # string for calling AprilTags scanning
      tmp =""
      result = AprilTags_Decoding()
      #print("%s" % result)
      uart.write(("%s" % result).encode())
      uart.readchar()
   if tmp == "image_classification\0":                       # string for calling image classification
      #print("classify images")
      tmp =""
      label = image_classification()
      print(label)
      uart.write(label.encode())
      uart.readchar()
   if tmp == "line_detection\0":                             # string for calling line detection
      sensor.reset()
      sensor.set_vflip(True)
      sensor.set_hmirror(True)
      sensor.set_pixformat(sensor.RGB565)
      sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
      #sensor.set_windowing([0,20,80,40])
      sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
      clock = time.clock()                # to process a frame sometimes.

      while(True):                        # while loop for line detection
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
```
