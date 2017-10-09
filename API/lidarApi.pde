/*enum*/
//public class SCAN_TYPEDEF {byte SCAN = 0; byte FORCE_SCAN = 1; byte EXPRESS_SCAN =2; byte AUTO = 3;};
//public class RPLIDAR_CMD {byte STOP = 0x25; byte RESET = 0x40; byte SCAN = 0x20;
//                          byte EXPRESS_SCAN = byte(0x82); byte FORCE_SCAN = 0x21;
//                          byte GET_INFO = 0x50; byte GET_HEALTH = 0x52; byte GET_SAMPLERATE = 0x59;
//                        };
final byte CMD_STOP = 0x25; final byte RSPD_STOP_SIZE = 0;
final byte CMD_RESET = 0x40; final byte RSPD_RESET_SIZE = 0;
final byte CMD_SCAN = 0x20; final byte RSPD_SCAN_SIZE = 5;
final byte CMD_EXPRESS_SCAN = byte(0x82); 
final byte CMD_FORCE_SCAN = 0x21;
final byte CMD_GET_INFO = 0x50; 
final byte CMD_GET_HEALTH = 0x52; 
final byte CMD_GET_SAMPLERATE = 0x59;
final byte CMD_SET_PWM = byte(0xF0);
final float A2R = PI/180;
final int DEFAULT_PWM = 600;


/*for one */
/*!do not use it in user code*/
class lidar_struct{
  public Serial port;  /*connect serial port with rplidar*/
  public String markPort = null; /*give a alias number for every lidar*/
  int[] SN = new int[16]; /*storage the SN of this lidar*/
  //private int cmdRecord; /*record the last command*/
  private IntList buffer;
  private IntList frameSize; /*to record every start Index of every frame*/
  private int[] frameBuf = new int[2400];
  private int frameBufLen = 0;
  private int frameBufState;  /*0-means there is no data in frameBuf; 1-means frameBuf is busy; 2-means framebuf have new data; 3-frame have data but have been update*/
  public byte[] recvBuf = new byte[100];
  public int status = 0; /*0-uninitialized, 1-allcate fail, 2-allcate success - port close, 3- port open, 4 - device ok, no scan, 0x12-device error, 0x11-device warning, 5-scaning */
  public volatile byte rspdMode = 0; /*every command have a head of responde and extra message:0-no responde, 1- wait head,2-responde once,3-responde loop, other-reserve  */
  public volatile int rspdLen = 0;
  public boolean healthSignal;

  
  /*this part is used to cache express scan receive data*/
  private circleBuf ESRBuf = new circleBuf(84,50);
  private byte[] block1st = new byte[84];
  private byte[] block2nd = new byte[84];
  //private byte[] block3rd = new byte[84];
  volatile boolean isCacheNew = false;
  
  
  /*new a class*/
  lidar_struct(){
    status = 0;
    /*can't read undone frame point*/

  }
  
  /*Initialize lidar*/
  void Init(){
    /*initilize the IntList*/
    buffer = new IntList();
    buffer.clear();
    frameSize = new IntList();
    frameSize.clear();

    /*check if status is ok*/
    if(markPort == null) status = 1;
    else status = 2;
    
    /*intilize variable*/
    rspdLen = 0;
    rspdMode = 0;
    
    /*prepare cache*/
    /*needn't do anything*/
    frameBufState = 0;
    frameBufLen = 0;
  }
  
  /*allocate port*/
  void allocatePort(String _mark){
    markPort = _mark;
    if(status == 1) status = 2;
  }
  
  /*open port: can not use. cause Serial can not be handle from this class. Use outside function to open it*/
  //void openPort(){
  //  port = new Serial(this, markPort, 115200);
  //}
  
  /*close port*/
  void closePort(){
    if(status > 2) this.port.stop();
    status = 2;
  }
  
  
  
  /*stop scan*/
  void stop(){
    if(status == 5){
      rspdMode = 0;
      sendMsg(CMD_STOP);
      status = 4;
    }
  }
  
  /*setPWM*/
  void setMotorPMW(int val){
    byte[] pwm_val = new byte[2];
    if(val<0 || val>1023) return;
    pwm_val[0] = byte(val);
    pwm_val[1] = byte(val>>8);
    
    sendMsg(CMD_SET_PWM, pwm_val, byte(2));
  }
  
  /*stop motor*/
  void stopMotor(){
    setMotorPMW(0);
  }
  /*start motor*/
  void startMotor(){
    setMotorPMW(DEFAULT_PWM);
  }
  

  
  /*Give a aliasNum according the fixed SN*/
  
  
  /*read the first unreaded frame .just allow to read one whole frame @bufMode 0 or 1*/
  /*@mode 0, just keep the newest whole frame and the newest unfinished frame. So to get the oldest frame*/
  /*@mode 1, keep all unreaded whole frame and the newest unfinish frame. So to get the newest whole frame*/
  int readFrame(int[] dest){
    int retval = -5;
    if(frameBufState  != 2){
      retval = -frameBufState;
    }
    else{
      frameBufState = 1;
      /*copy data out*/
      try{
        retval = frameBufLen;
        arrayCopy(frameBuf,dest,retval);
        
      }
      catch(RuntimeException e){
        println("this way is wrong");
        e.printStackTrace();
      }
      
      frameBufState = 3;
    }
    return retval;
  }
  
  
  /*access FrameBuf*/
  int accessFrameBuf(){
    int retval = -5;
    if(frameBufState  != 2){
      retval = -frameBufState;
    }
    frameBufState = 1;
    
    return retval;
  }
  /* exit from frameBuf*/
  void exitFrameBuf(){
    if(frameBufState == 1){
      frameBufState = 3;
    }
  }

  
  /*read the newest unfinished */
  int peekUndoneFrame(int[] dest){
    int retval = -1;
     
    int z = frameSize.size();
    if(z < 1) return -1;
    int destLen = frameSize.get(z-1);
    dest = null;
    dest = new int[destLen];
    int offsetIndex = 0;
    for(int ofi = 0; ofi<z-1; ofi++){
      offsetIndex += frameSize.get(ofi);
    }
    for(int i = 0; i< destLen;i++, offsetIndex++){
      dest[i] = buffer.get(offsetIndex);
    }
    
    return retval;
  }
  
  /*get how many points in frameBuf*/
  int numPoint(){
    int retval =0;
    if (frameBufState > 1) retval = frameBuf.length/3;
    return retval;
  }
  
  /*just allow to read one point @bufMode 2 0r 3*/
  int peekNewPoint(int[] point){
    int retval = -1;
    int bSize = buffer.size();
    if(bSize > 2){
      point[0] = buffer.get(bSize - 3);
      point[1] = buffer.get(bSize - 2);
      point[2] = buffer.get(bSize - 1);
    }
    
    return retval;
  }
  
  
  /*check if it is available*/
  //void waitWriteBuf(){
  //  waitWinFlag = true;
  //  int t = millis() + 2;
  //  while(isBufBusy){
  //    if(millis() > t ) return; /*wait for write in */ 
  //  }
  //  isBufBusy = true;
  //  waitWinFlag = false;
  //}
  
  /*storage a new point*/
  void writePoint(int[] npoint, boolean isSOF){
    /*wait buffer*/
    //waitWriteBuf();
    boolean cpSOF = isSOF;
    
    /*if there is too many points(800) in on package, just as a new package */
    if( frameSize.size() > 0 ){
      if(frameSize.get(frameSize.size() - 1) >= 2400){
        cpSOF = true;
      }
    }

    /*remove the old frame, just keep the newest whole frame*/
    if(cpSOF == true){
      if((frameBufState != 1)&&(frameSize.size()>0)){
        frameBufState = 1;
        /*dump data to frameBuf*/
        while(frameSize.size()>1){
           int fbLen = frameSize.remove(0);
           for(int i = 0; i < fbLen;i++) buffer.remove(0);
        }
        frameBuf = null;
        frameBufLen = frameSize.remove(0);
        frameBuf = new int[frameBufLen];
        frameBuf = buffer.array(); 
        buffer.clear();
        
        frameSize.append(0); 
        frameBufState = 2;
      }
      else{
        frameSize.append(0); 
      }
    }
    else{
      if((frameSize.size()>1)&&(frameBufState != 1)){
        frameBufState = 1;
        /*check if there is new frame haven't dumped*/
        /*dump data to frameBuf*/
        int fbLen = 0;
        while(frameSize.size()>2){
           fbLen = frameSize.remove(0);
           for(int i = 0; i < fbLen;i++) buffer.remove(0);
        }
        fbLen = frameSize.remove(0);
        frameBuf = null;
        frameBuf = new int[fbLen]; 
        for(int j =0; j<fbLen; j++) frameBuf[j] = buffer.remove(0);
        frameBufState = 2;
      }
    }
    
    /*add new point to the end of buffer*/
    int z = frameSize.size();
    for(int k = 0; k<3; k++){
      buffer.append(npoint[k]);
      frameSize.increment(z-1);
    }
    
  }
   
  /*send message out*/
  void sendMsg(byte cmd){
    byte[] val;
    if(cmd == CMD_EXPRESS_SCAN){
      val = new byte[]{(byte)0xA5,(byte)0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};
    }
    else{
      val = new byte[]{(byte)0xA5, cmd};
    }
    try{
      port.write(val);
      //cmdRecord = cmd; /*to record every command*/
    }
    catch(RuntimeException e){
      println("serial port 0f liadr is error");
      e.printStackTrace();
    }
    
  }
  /*send message out*/
  void sendMsg(byte cmd,byte[] data, byte dLen){
    /*in case of error input*/
    if(dLen<0 || dLen > 100) return;
    
    byte[] msg = new byte[dLen + 4];
    
    byte checkSum = byte(0xA5^ cmd ^ dLen);
    msg[0] = byte(0xA5);
    msg[1] = cmd;
    msg[2] = dLen;
    for(int i=0; i< dLen; i++){
      msg[3+i] = data[i];
      checkSum ^= data[i];
    }
    msg[3+dLen] = checkSum;

    
    try{
      port.write(msg);
    }
    catch(RuntimeException e){
      println("serial port 0f liadr is error");
      e.printStackTrace();
    }
    
  }
  
  /*check health*/
  boolean isHealth(){
    boolean retVal = false;
    int m = millis() + 500;
    
    rspdMode = 1;
    healthSignal = false;
    this.sendMsg(CMD_GET_HEALTH);
    
    rspdMode = 1;
    port.buffer(7);
    do{
      /*wait response*/
      if(healthSignal){
        retVal = true;
        break;
      }
    }while(millis() < m);
    
    return retVal;
  }
  
  /*detect if there is a lidar device*/
  //boolean isRpliar(){
  //  boolean retVal = false;
  //  this.sendMsg(CMD_GET_INFO);
  //  int m = millis() + 500;
  //  rspdMode = 1;
  //  port.buffer(7);
  //  do{
  //    /*wait response*/
      
  //  }while(millis() < m);
    
  //  return retVal;
  //}
  
  /*start normal SCAN*/
  void startScan(){
    
    /*if it is scanning, stop it. and rescan*/
    if(status > 5) return;
    if(status == 5){
      stop();
      delay(500);
    }
    rspdMode = 1;
    status = 5;
    sendMsg(CMD_SCAN);
  }
  
  /*start express SCAN*/
  void startEScan(){
    
    /*if it is scanning, stop it. and rescan*/
    if(status > 5) return;
    if(status == 5){
      stop();
      delay(100);
    }
    
    rspdMode = 1;
    status = 5;
    sendMsg(CMD_EXPRESS_SCAN);
  }
  
  /**/
  boolean cacheESR(byte[] src, boolean nfFlag){
    if(nfFlag){
      ESRBuf.fakeClear();
      initDecode();
    }      
    
    isCacheNew = ESRBuf.cache(src);
    return isCacheNew;
  }
  
  /**/
  boolean ifCacheNew(){
    boolean retval = false;
    if(ESRBuf.getBNum()>2 ) retval = true;
    return retval;
  }
  
  /*decode express-scan receive data*/
  int decodeESR(){
    int retval = -1;
    
    /*check if there is enough data in it*/
    if(ESRBuf.getBNum() <2) return 0;
    ESRBuf.extract(block1st);
    /*check if this is equel the data storage in block2nd just compare the first 4 byte*/
    int i = 0;
    for(; i< 4; i++){
      if(block1st[i] != block2nd[i]) break;
    }
    if(i >= 4){
      ESRBuf.peak(block2nd);
    }
    else{
      ESRBuf.peak(block2nd);
      println("equel " + i);
      return -2;
    } 
    
    /*check if it is the first data in buffer*/
    //if(block1st[3] == 0x80 && block1st[2] == 0x00){

    //  return false;
    //} 
    
    /*** start decode ***/
    /*data for decode*/
    int w1_q8 = ((combByte(block1st[2], block1st[3]))&0x7fff)<<2;
    int w2_q8 = ((combByte(block2nd[2], block2nd[3]))&0x7fff)<<2;
    int diffAng_q8 = w2_q8 - w1_q8;
    
    if(w1_q8 > w2_q8) diffAng_q8 += (360 << 8);
    
    int angInc_q16 = (diffAng_q8 << 3);
    int w1_q16 = (w1_q8 << 8);
    
    for(int cab = 0; cab < 16; cab++){
      int base = 4 + 5* cab;
      int[] ang_q6 = new int[2];
      int[] dist_q2 = new int[2];
      boolean[] syncBit = new boolean[2];
      int[] ang_offset_q3 = new int[2];
      
      dist_q2[0] = (combByte(block1st[base],block1st[base+1])&0xfffc);
      dist_q2[1] = (combByte(block1st[base+2],block1st[base+3])&0xfffc);
      
      ang_offset_q3[0] = ((int(block1st[base])&0x03)<<4) | (int(block1st[base+4])&0x0f);
      ang_offset_q3[1] = ((int(block1st[base+2])&0x03)<<4) | ((int(block1st[base+4])&0x0f)>>4);
      
      
      /*formula: ang_k = w1 - k*diffAng(w1,w2)/32 - angOffSet_k */
      /*but k is 0 or 1. it didn't say it on datasheet. Maybe start from 0*/
      /*get ang1*/
      ang_q6[0] = (w1_q16 - (ang_offset_q3[0]<<13))>>10;
      syncBit[0] = ((w1_q16 + angInc_q16)%(360<<16) < angInc_q16)?true:false;
      w1_q16 += angInc_q16;
      
      /*get ang2*/
      ang_q6[1] = (w1_q16 - (ang_offset_q3[1]<<13))>>10;
      syncBit[1] = ((w1_q16 + angInc_q16)%(360<<16) < angInc_q16)?true:false;
      w1_q16 += angInc_q16;
      
      
      for(int j=0; j<2; j++){
        /*add function to add this node to Point List*/
        int[] newPoint = new int[3];
        /*limit angle to 0~360 degree*/
        if(ang_q6[j] < 0) ang_q6[j] += (360<<6);
        if(ang_q6[j] >=(360<<6)) ang_q6[j] -= (360<<6);
        
        /*quality*/
        newPoint[0] = ( int(syncBit[j]) | (int(!syncBit[j])<<1) );
        if(dist_q2[j] != 0) newPoint[0] |= 0xBC;  /*quality set to */
        newPoint[1] = ang_q6[j];
        newPoint[2] = dist_q2[j];
        writePoint(newPoint, syncBit[j]); 
        //println( (cab*2+j) + ", "+ syncBit[j] + " ,Ang "+ (newPoint[1]>>6) + " ,Dist " + (newPoint[2]>>2));
      }
      
    }
    
    return retval;
  }

  private void initDecode(){
    int[] newPoint = new int[3];
    newPoint[0] = 0x01;
    newPoint[1] = 0x00;
    newPoint[2] = 0x00;
    writePoint(newPoint,true);
    ESRBuf.peak(block2nd);
  }
  
}


/**Small class just use for record mark of serial port and the sn of lidar**/
class linkLidar{
  private String[] markPort = new String[MAX_LIDAR_NUM];
  private byte[][] SN = new byte[MAX_LIDAR_NUM][16]; 
  private int[] realLidarIndex = new int [MAX_LIDAR_NUM];
  boolean[] isSet = new boolean[MAX_LIDAR_NUM];
  boolean isLink = false;
  
  linkLidar(){
    for(int i=0; i< MAX_LIDAR_NUM; i++){
      isSet[i] = false;
      realLidarIndex[i] = -1;
    }
    isLink = false;
  }
  
  
  void setMarkSN(int index, String mark,byte[] _sn){
    markPort[index] = mark;
    for(int i = 0; i<16; i++) SN[index][i] = _sn[i];
    
    isSet[index] = true;
  }

  void linkLidarIndex(){
    int s = AR_SN.length;
    if(s != MAX_LIDAR_NUM){
      println("There is no enough Lidar can allocation. Please make sure MAX_LIDAR_NUM and AR_SN is fit");
    }
    
    for(int i = 0; i < MAX_LIDAR_NUM; i++){
      for(int j=0; j<MAX_LIDAR_NUM; j++){
        if(isSet[j]){
          int k =0;
          for(; k<16;k++){
            if(SN[j][k] != AR_SN[i][k]) break;
          }
          if(k == 16) realLidarIndex[i] = j;
        }
      }
    }
    isLink = true;
  }
  
  String getPortMark(int lidarIndex){
    if(lidarIndex > MAX_LIDAR_NUM) return null;
    if((realLidarIndex[lidarIndex] > MAX_LIDAR_NUM)||(realLidarIndex[lidarIndex] < 0)) return null;
    
    
    return markPort[ realLidarIndex[lidarIndex] ];
  }
  
  byte[] getSN(int lidarIndex){

    if(lidarIndex > MAX_LIDAR_NUM) return null;
    if((realLidarIndex[lidarIndex] > MAX_LIDAR_NUM)||(realLidarIndex[lidarIndex] < 0)) return null;
    
    return SN[lidarIndex];
  }
  
  int getRealIndex(int lidarIndex){
    return realLidarIndex[lidarIndex];
  }
}


/******handle of lidar******/
/*user can use it to create multiple lidars*/
class lidar_api {
  lidar_struct[] lidar = new lidar_struct[MAX_LIDAR_NUM];
  linkLidar theLink;
  int apiStatus = 0; /*0-uninitlized or fail, 1-intialized succeed*/
  int available = 0;
  
  
  lidar_api(){
    theLink = new linkLidar();
    for(int cir = 0; cir <MAX_LIDAR_NUM; cir++){
      lidar[cir] = new lidar_struct();
    }
  }
  /*Initialze lidar_api*/
  void Init(){
    
    if(!theLink.isLink) return; /*should Link port mark and sn*/
    else{
      for(int i = 0; i< MAX_LIDAR_NUM; i++){
        lidar[i].allocatePort( theLink.getPortMark(i));
      }
    }
    
    for(int i = 0; i < MAX_LIDAR_NUM; i++){
      //lidar[i] = new lidar_struct();
      lidar[i].Init();
    }
    /*reopen the serial*/
  }
  

  
  /*set available rpliar number*/
  void setAvailable(int number){
    available = number;
  }
  /*get available rpliar*/
  int getAvailable(){
    return available;
  }
    
  
  
} /*-end of class: lidar_api-*/


/*it is just a common function */
int combByte(byte l, byte h){
  int retval = int(l);
  retval |= (int(h)<<8);
  return retval;
}