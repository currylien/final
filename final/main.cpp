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
//void reply_messange(char *xbee_reply, char *messange);
//void check_addr(char *xbee_reply, char *messenger);


void recieve_thread(){
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

void send_thread(){
   while(1){
      // if button is pressed
      if(mode == 1){
         char s[] = "AprilTags_Decoding\0";
         uart.write(s, sizeof(s));
         printf("send\r\n");
         ThisThread::sleep_for(400ms);
      } else if (mode == 2) {
          char s[] = "image_classification\0";
          uart.write(s, sizeof(s));
          printf("send\r\n");
         ThisThread::sleep_for(1s);
      } else if (mode == 3) {
          char s[] = "line_detection\0";
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
   while(1) {
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
   ThisThread::sleep_for(500ms);
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
   ThisThread::sleep_for(900ms);
   car.stop();
   char buf2[] = "Scan number\r\n";
   xbee.write(buf2, sizeof(buf2));
   mode = 2;
   ThisThread::sleep_for(5000ms);
   xbee.write(recv, sizeof(recv));
   mode = 0;
   car.turn(50,1);
   ThisThread::sleep_for(1120ms);
   car.goStraight(50);
   ThisThread::sleep_for(5000ms);
   car.turn(50,1);
   ThisThread::sleep_for(980ms);
   car.stop();
   ThisThread::sleep_for(1s);
   char buf3[] = "Start to detect line\r\n";
   xbee.write(buf3, sizeof(buf3));
   mode = 3;
   while(1){
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
                char buf4[] = "Go Straight and finish demo\r\n";
                xbee.write(buf4, sizeof(buf4));
                car.stop();
            } else {
                car.stop();
            } 
      }
   }
}
/*void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}*/