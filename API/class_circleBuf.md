/*这个循环存储类，会假定每次都成块存入和读出数据。数据块大小相同。*/
/*因此对于写入和读出数据块变化或者不想等的操作，这里将会出错。*/

import java.util.Arrays; 

class circleBuf {
  private int p2win;
  private int p2rout;
  private boolean isBusy;
  private int _len = 0;
  private int _bSize = 0;
  byte[] buf;
  
  circleBuf(int bSize,int bNum){
    if(bSize==0 || bNum==0) println("circleBuf Class instance is wrong");
    
    int len = bSize * bNum;
    buf = new byte[len + 1];
    _len = len;
    _bSize = bSize; 
    
    /*reset*/
    Arrays.fill(buf, byte(0));
    /*the last data is used to connect head and tail, and keep not to delete it. 0xa5 is initial state,blank. 0x18 means have data(R<W). 0xc3 means buf is full and there is new data at beginning(R>W) */
    buf[_len] = byte(0xa5);

    /*clear pointer*/
    p2win = 0;
    p2rout = 0;
    isBusy = false;
  }

  /*reset:clear buffer and reset pointer */
  void clear(){
    Arrays.fill(buf,byte(0));
    fakeClear();
  }
  
  /*reset pointer but do not clear buffer*/
  void fakeClear(){
    /*the last data is used to connect head and tail, and keep not to delete it. 0xa5 is initial state,blank. 0x18 means have data(R<W). 0xc3 means buf is full and there is new data at beginning(R>W) */
    buf[_len] = byte(0xa5);

    /*clear pointer*/
    p2win = 0;
    p2rout = 0;
    isBusy = false;
  }

  boolean cache(byte[] src){
    boolean retval = true;
    
    /*in case of dirty input*/
    if(_bSize < 0 || src == null) return false;

    /*setting error*/
    if((buf[_len]==byte(0x18) && p2win < p2rout)||(buf[_len]==byte(0xc3) && p2win > p2rout) ){
        println("circleBuf class-cache is error");
        delay(500);
        return false;
      } 

    /*start lock buffer*/
    if(isBusy) return false;
    isBusy = true;  

    /*in case of flow out*/
    if(p2win == _len){
      p2win = 0;
      if(p2rout > 0) buf[_len] = byte(0xc3);
      else buf[_len] = byte(0xa5);
    }

    /*every time cache a whole continue block*/
    if((p2win + _bSize) > _len){
      isBusy = false; 
      return false;
    }  

    /*if buf is blank, change the state*/
    if(buf[_len] == byte(0xa5)) buf[_len] = 0x18;

    /*start cache data*/
    arrayCopy(src, 0, buf, p2win, _bSize);

    /*change pointer*/
    p2win += _bSize;
    if((buf[_len]== byte(0xc3))&&(p2rout < p2win)) p2rout = p2win;    

    isBusy = false;

    return retval;
  }

  /*extract data from it*/
  boolean extract(byte[] dest){
    boolean retval = true;

    /*if there is no data in it. just cancel it*/
    if(buf[_len] == byte(0xa5)) return false;
    
    /*lock it*/
    if(isBusy) return false;
    
    /*if there is data in it. In correct order. 0x18 */
    if(buf[_len] == byte(0x18) ){
      if((p2rout + _bSize) > p2win) return false;

      /*lock buffer*/
      isBusy = true;

      arrayCopy(buf, p2rout, dest, 0, _bSize);
      p2rout += _bSize;
      if(p2rout == p2win){
        p2rout = 0;
        p2win = 0;
        buf[_len] = byte(0xa5);
      }
    }
    else if(buf[_len] == byte(0xc3)){
      if((p2rout + _bSize)>_len) return false;

      /*lock buffer*/
      isBusy = true;

      arrayCopy(buf, p2rout, dest, 0, _bSize);
      p2rout += _bSize;
      if(p2rout == _len){
        p2rout = 0;
        buf[_len] = 0x18;
        if(p2win == _len){
          p2win = 0;
          buf[_len] = byte(0xa5);
        }
      }
    }
    else return false; /*config is wrong*/

    /*unlock buffer*/
    isBusy = false;
    return retval;
  }

  /*extract data from it, but didn't delete this block*/
  boolean peak(byte[] dest){
    boolean retval = true;

    /*if there is no data in it. just cancel it*/
    if(buf[_len] == byte(0xa5)) return false;

    /*lock buffer*/
    isBusy = true;
    /*just copy data*/
    arrayCopy(buf, p2rout, dest, 0, _bSize);

    isBusy = false;
    
    return retval;
  }
}  