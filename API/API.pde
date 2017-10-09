/* athur:zjh @organization:xsspace @data:2017.09.24 */
/* usage: api of rplidar A2M8, support 2 and more device. test@processing3.3.6 in Java mode*/
/* here I design a special storage mothod. U can get a frame or a point according bufMode*/
import processing.serial.*;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;  
import java.util.concurrent.TimeUnit;

ScheduledExecutorService service = Executors.newScheduledThreadPool(2);
ScheduledTask updateData = new ScheduledTask();


/*define a constant for detect lida. How many lidas should be detect*/
final int MAX_LIDAR_NUM = 2;
//final int LIDAR_ES_BUFFER_SIZE = 84*25; /* 5HZ@4K, every circle need so depth*/
//final int LIDAR_S_BUFFER_SIZE = 5 * 400; /*5HZ@2K, every circle need so depth*/
//final int LIDAR_FRAME_BUFFER_SIZE = 4000*2;  /*try to storage one second data: one for angle, one for distance */

/*Using SN to distinguish different RPLIDARs */
final byte[] LIDAR01_SN = {byte(0xca),byte(0xd2),byte(0xe6),byte(0xc1),byte(0xe8),byte(0x83),byte(0x9e),byte(0xf2),
                           byte(0xc0),byte(0xe6),byte(0x9e),byte(0xf7),byte(0x12),byte(0x65),byte(0x4d),byte(0x05)};
final byte[] LIDAR02_SN = {byte(0xd3),byte(0xd2),byte(0x8f),byte(0xc1),byte(0xe8),byte(0x83),byte(0x9e),byte(0xf2),
                           byte(0xc0),byte(0xe6),byte(0x9e),byte(0xf7),byte(0x0b),byte(0x65),byte(0x24),byte(0x05)};
final byte[][] AR_SN = {LIDAR01_SN,LIDAR02_SN}; /*the number of raw must equel to or small than MAX_LIDAR_NUM */

int[][] LidarFrame = new int[MAX_LIDAR_NUM][2400];
volatile boolean[] LidarOnline = new boolean[MAX_LIDAR_NUM];
volatile boolean[] LidarNewData = new boolean[MAX_LIDAR_NUM];


/*variable*/
lidar_api rplidar;

/*just for test*/ 
static StringList recorderPort;
volatile int winPage = 0; /*differetn page have different display*/
volatile int page1Handle = -1;
volatile int buttonNum = 0;
volatile boolean isNewData = false;





void setup(){
  int numLidar;
  
  /*intialize data*/
  for(int i =0; i <MAX_LIDAR_NUM; i++){
    LidarOnline[i] = false;
    LidarNewData[i] = false;
  }
  recorderPort = new StringList();
  
  rplidar = new lidar_api();    /*instance a lidar api class(manage all rplidar)*/
  numLidar = enumLidar();
  
  
  if(numLidar > 0){
    print("There is totally " + numLidar + " rplidar online\r\n");
    for(int i =0; i < MAX_LIDAR_NUM; i++){
      //byte[] selSN = rplidar.theLink.getSN(i);
      String selPortMark = rplidar.theLink.getPortMark(i);
      println("real index of "+ i +" is " + rplidar.theLink.getRealIndex(i));
      if(selPortMark != null){
        println(" PORT is " + selPortMark);
        recorderPort.append("rplidar" + i +"\r\n" + selPortMark);
        LidarOnline[i] = true;
      }
      else{
        recorderPort.append("no device");
        LidarOnline[i] = false;
      }
    }
  } 
  
  println("enum lidar finished");
  rplidar.Init();
  
  /*draw*/
  size(1280,900);
  winPage = 0;
  page1Handle = -1;
  
}


void draw(){
  //thread("task");
  
  switch(winPage){
    case 0: drawPage0(); break;
    case 1: drawPage1(); break;
    case 2: drawPage2(); break;
    default: break;
  }

  //delay(100);

}

void drawPage0(){
  int z = recorderPort.size();
    if(z<1){
    background(200,10,10);
  }
  else{
    background(200,200,200);
    textAlign(CENTER, CENTER);
    textSize(28);
    rectMode(CENTER);
    pushMatrix();
    translate(width*2/(z+3) , height/2);
    for(int i =0; i<z; i++){
      fill(#19D1B0);
      noStroke();
      rect(0,0,150,90,8);
      fill(#DB37A2);
      text(recorderPort.get(i),0,0);
      translate(width/(z+3),0);
    }
    popMatrix();
    
    pushMatrix();
    textSize(28);
    /*add another button*/
    fill(#D3E551);
    noStroke();
    rect(width/2,height-100,220,90,8);
    fill(#1608FA);
    text("Open All Lidar",width/2,height-100);
    popMatrix();
    
    /*add interactive code*/
    buttonNum = -1;
    if(abs(mouseY-height/2)<45){
      int selX = round(mouseX*(z+3)/width);
      if(selX > 1 && selX <(z+2) ){
        if(mouseX - selX*width/(z+3) < 60){
          buttonNum = selX-2;
          pushMatrix();
          translate( selX*width/(z+3), height/2);
          noFill();
          strokeWeight(5);
          stroke(204, 102, 0);   
          rect(0,0,150,90,8);
          popMatrix();
        }
      }
    }
    else if((abs(100+mouseY-height)<45)&&(abs(mouseX-width/2)<110)){
      pushMatrix();
      noFill();
      strokeWeight(5);
      stroke(204, 102, 0);   
      rect(width/2,height-100,220,90,8);
      popMatrix();
      buttonNum = 0xff;
    }
  }
  //delay(50);
}

void drawPage1(){
  
  /*** the framework of page1 ***/
  background(200,200,200);
  textAlign(CENTER, CENTER);
  textSize(28);
  fill(#DB37A2);
  noStroke();
  /*the title of this rplidar*/
  text("rplidar"+page1Handle+" point cloud",width/2,32);
  /*put a circle in center of this page as the lidar*/
  fill(#1FDB49);
  ellipseMode(CENTER);
  ellipse(width/2,height/2, 50,50);
  

  int[] dLen = requestData();
  
  /*draw data*/
  
  fill(#7CED83);
  
  if(isNewData){
    isNewData = false;
    //if(LidarFrame == null) println("code is wrong");
    int pointsNum = floor(dLen[buttonNum]/3); 
    for(int i = 0; i<pointsNum; i++){
      println(i + ":angle is " + (int(LidarFrame[buttonNum][3*i+1])>>6 + (0x0001 & (int(LidarFrame[buttonNum][3*i+1])>>5)) ) + " distance is " + (int(LidarFrame[buttonNum][3*i+2])>>2) );
    }
  }
}

void drawPage2(){
  /*** the framework of page1 ***/
  background(#C1BDFC);
  if (frameCount % 30 == 0) {
    int[] dLen = requestData();
    /*open All lidar*/
    for(int i = 0; i < MAX_LIDAR_NUM; i++){
      if(LidarNewData[i]&&LidarOnline[i]){
        LidarNewData[i] = false;
        println("Lidar " + i +" get data:");
        int pointsNum = floor(dLen[i]/3);
        for(int j = 0; j<pointsNum; j++){
          println(j + ":angle is " + (int(LidarFrame[i][3*j+1])>>6) + " distance is " + (int(LidarFrame[i][3*j+2])>>2) );
        }
      }
    }
  }
  
}



void mouseClicked(){
  println("buttonNum is " + buttonNum);
  if((buttonNum > -1)&&(buttonNum < MAX_LIDAR_NUM)){
    if(recorderPort.get(buttonNum) != "no device"){
      page1Handle = buttonNum;
      winPage = 1;
      isNewData = false;
      
      /*check if port is open*/
      if(rplidar.lidar[buttonNum].status == 2)  openLidarPort(rplidar, buttonNum);
      /*send a stop command firstly*/
      rplidar.lidar[buttonNum].stop();
      delay(50);
      /*start motor*/
      rplidar.lidar[buttonNum].startMotor();
      delay(200);
      
      if(rplidar.lidar[buttonNum].status == 3){
        if(rplidar.lidar[buttonNum].isHealth()){ /*get health*/
          /*startScaning*/
          rplidar.lidar[buttonNum].startEScan();
          println("startEScan");
          service.scheduleWithFixedDelay(updateData,0,16,TimeUnit.MILLISECONDS); 
        }
        else println("No startEScan");
      } 
      
    }
    
  }
  else if(buttonNum == 0xff){
    boolean ifOpenSevice = false;
    /*open all lidar*/
    for(int i = 0; i < MAX_LIDAR_NUM; i++ ){
      /*check if there is device*/
      if(LidarOnline[i]){
        /*check if port is open*/
        if(rplidar.lidar[i].status == 2)  openLidarPort(rplidar, i);
        /*send a stop command firstly*/
        rplidar.lidar[i].stop();
        delay(50);
        /*start the Motor of Lidar*/
        //rplidar.lidar[i].startMotor();
        //delay(200);  
        if(rplidar.lidar[i].status == 3){
          if(rplidar.lidar[i].isHealth()){ /*get health*/
            /*startScaning*/
            rplidar.lidar[i].startEScan();
            println("Lidar " + i +" startEScan");
            ifOpenSevice |= true;
          }
          else println("Not startEScan " + i);
        } 
        
      }
    }
    if(ifOpenSevice) service.scheduleWithFixedDelay(updateData,0,16,TimeUnit.MILLISECONDS); 
    /*switch page*/
    winPage = 2;
  }
}

int[] requestData(){
  int[] retval = new int[MAX_LIDAR_NUM];
  if((buttonNum > -1)&&(buttonNum < MAX_LIDAR_NUM)){
    int readState = rplidar.lidar[buttonNum].readFrame(LidarFrame[buttonNum]);
    
    if(readState <= 0 ) isNewData = false;
    else{
      isNewData = true;
      retval[buttonNum] = readState;
      //println("there is " + readState/3 +" new points in new frame");
    }
  }
  else if(buttonNum == 0xff){
    /*scan all available*/
    for(int i = 0; i < MAX_LIDAR_NUM; i++){
      retval[i] = -1;
      /*try to scan all*/
      int readState = rplidar.lidar[i].readFrame(LidarFrame[i]);
      if(readState <= 0) LidarNewData[i] = false;
      else{
        LidarNewData[i] = true;
        retval[i] = readState;
      }
    }
  }
  return retval;
}

float toRadians(float angle){
  return angle*A2R;
}




  /*to detect how many ralidar is online*/
  int enumLidar(){
    int retval = 0;
    int spNum = 0;
    int lidarNum = 0;
    byte[] wout = {byte(0xA5), 0x50};
    byte[] woutStop = {byte(0xA5), 0x25};
    byte[] rbuf = new byte[20];
    byte[] storSN = new byte[16];
    
    Serial selDev;
    
    
    
    spNum = Serial.list().length;
    if(spNum == 0){
      println("No any available serial port");
      return -1;
    }
    
    print("There is ");print(spNum);println(" Serial port is detected");
    for(int i=0; i < spNum; i++){
      
      try{
        selDev = new Serial(this, Serial.list()[i], 115200);
      }
      catch(RuntimeException e){
        print("Serial port:");print(Serial.list()[i]);println(" can't open!!!");
        continue;
      }
      
      selDev.write(woutStop);
      delay(200);
      selDev.write(wout);
      delay(50);
      if(selDev.available() >=  7) {
        selDev.readBytes(7);
        delay(50);
        if(selDev.available() >= 20){
          rbuf = selDev.readBytes(20);
          /*link port and SN*/
          for(int k =0; k<16; k++) storSN[k] = rbuf[k + 4];          
          rplidar.theLink.setMarkSN(lidarNum, Serial.list()[i],storSN);
          print("SN of Lidar"+ lidarNum +" is 0x");
          for(int j =0; j<16; j++) print(hex(storSN[j]));
          println("\r\nAnd port is " + Serial.list()[i]);
          lidarNum++; 
        }
      }
      
      /*close it anyway*/
      selDev.stop();

      
    }
    retval = lidarNum;
    
    /*allocation port and sn*/
    if(lidarNum >0) rplidar.theLink.linkLidarIndex();
    
    /*here or other place to set available device*/
    rplidar.setAvailable(lidarNum);
    
    return retval;
  }
  
/*to open the lidar port from outside. Cause can not to handle serial from class*/
void openLidarPort(lidar_api handle, int index){
  if(handle.lidar[index].status < 2) return;
  handle.lidar[index].port = new Serial(this, handle.lidar[index].markPort, 115200);
  handle.lidar[index].status = 3;
}

/*  decode express scan data */


  
void serialEvent(Serial myPort){
  int portHandle = -1;
  
  /*if didn't get SN or finish link, return fron it*/
  if(rplidar.theLink.isLink == false) return;
  
  for(int i = 0; i < MAX_LIDAR_NUM; i++){
    if(myPort == rplidar.lidar[i].port){
      portHandle = i;
      break;
    } 
  }
  /*if port is error, exit from this function*/
  if(portHandle == -1) return;
  
  
  try{
  
  if(rplidar.lidar[portHandle].rspdMode == 1){
    int len = 0;
    if(rplidar.lidar[portHandle].port.available() >= 7){
      rplidar.lidar[portHandle].recvBuf = rplidar.lidar[portHandle].port.readBytes(7);
      if((rplidar.lidar[portHandle].recvBuf[0] != 0xA5)&&(rplidar.lidar[portHandle].recvBuf[1] != 0x5A) ) return;
      /*if want to get checksum, di it*/
      //byte checksum = 0;
      //for(int j=0; j<7; j++){
      //  checksum ^= rplidar.lidar[portHandle].recvBuf[j];
      //}
      //if(checksum != 0) return;
      len = rplidar.lidar[portHandle].recvBuf[2];
      if(len != 0){
        if(rplidar.lidar[portHandle].recvBuf[5] !=0) rplidar.lidar[portHandle].rspdMode = 3;
        else rplidar.lidar[portHandle].rspdMode = 2;  
        rplidar.lidar[portHandle].rspdLen = len;
        rplidar.lidar[portHandle].port.buffer(len);
      }
      else{
        rplidar.lidar[portHandle].rspdMode = 0;
        rplidar.lidar[portHandle].rspdLen = 0;
      }
      
    }
  }
  else if(rplidar.lidar[portHandle].rspdMode == 2){
    rplidar.lidar[portHandle].recvBuf = rplidar.lidar[portHandle].port.readBytes(rplidar.lidar[portHandle].rspdLen);
    switch(rplidar.lidar[portHandle].rspdLen){
      /*GET_HEALTH*/
      case 3: {
          if(rplidar.lidar[portHandle].recvBuf[0] == 0){
            rplidar.lidar[portHandle].healthSignal = true;
            rplidar.lidar[portHandle].status = 4;
          } 
          else rplidar.lidar[portHandle].status = 0x10 + rplidar.lidar[portHandle].recvBuf[0];     
          
          /*if want to get error code, deal with recvBuf[1] and recvBuf[2]*/          
      } break;
      /*GET_SAMPLERATE*/
      case 4: {
          ;
          /*if you want to know Tstandard or Texpress, get it from here*/
      } break;
      /*GET_INFO*/
      case 0x14: {
          for(int k =0; k<16; k++){
            rplidar.lidar[portHandle].SN[k] = rplidar.lidar[portHandle].recvBuf[k + 4];
          }
      } break;
      default: break;
    }
    rplidar.lidar[portHandle].rspdLen = 0;
    rplidar.lidar[portHandle].rspdMode = 0;
  }
  else if(rplidar.lidar[portHandle].rspdMode == 3){
    //rplidar.lidar[portHandle].recvBuf = null;
    if(rplidar.lidar[portHandle].port.available() < rplidar.lidar[portHandle].rspdLen ) return; /*check if it is available*/
    rplidar.lidar[portHandle].recvBuf = rplidar.lidar[portHandle].port.readBytes(rplidar.lidar[portHandle].rspdLen);
    switch(rplidar.lidar[portHandle].rspdLen){
      /*SCAN or FORCE_SCAN*/
      case 5: {
          int[] newPoint = new int[3];
          boolean isSOF = false;
          
          if(boolean(rplidar.lidar[portHandle].recvBuf[1]&0x01)){
            newPoint[0] = int(rplidar.lidar[portHandle].recvBuf[0]);
            
            newPoint[1] = (combByte(rplidar.lidar[portHandle].recvBuf[1],rplidar.lidar[portHandle].recvBuf[2]))>>1;
            newPoint[2] = combByte(rplidar.lidar[portHandle].recvBuf[3],rplidar.lidar[portHandle].recvBuf[4]);
            
            isSOF = boolean(rplidar.lidar[portHandle].recvBuf[0] & 0x01);
            
            rplidar.lidar[portHandle].writePoint(newPoint, isSOF);
            // println(": " + (newPoint[1]>>6)+ " , " + (newPoint[2]>>4));
          }
          
      } break;
      case 0x54: {
          boolean isSOF = boolean(rplidar.lidar[portHandle].recvBuf[3] & 0x80);
          /*if want to add check code, please add here*/
          rplidar.lidar[portHandle].cacheESR(rplidar.lidar[portHandle].recvBuf, isSOF);
          
      } break;
      default: break;
    }
  }
  
  }
  catch(RuntimeException e){
    e.printStackTrace();
  }
  
}


/*run in backgroud*/
//void task(){
//  /*check if there is data need convert*/
//  for(int i =0; i< MAX_LIDAR_NUM;i++){
//    /*circle to decode*/
//    if(rplidar.lidar[i].ifCacheNew()) rplidar.lidar[i].decodeESR();  
//  }
//}

static int cir = 0;
public class ScheduledTask implements Runnable{
  public void run(){
    
    /*check if there is data need convert*/
    for(int i =0; i< MAX_LIDAR_NUM;i++){
      /*circle to decode*/
      while(rplidar.lidar[i].ifCacheNew()){
        rplidar.lidar[i].decodeESR();  
      } 
    } 
  }
}