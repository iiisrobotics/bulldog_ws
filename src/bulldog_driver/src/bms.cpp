#include <ros/ros.h>
#include <bulldog_driver/bms.h>

BMS::BMS(string dev)
{
  handle = -1;
  port.assign(dev);
}

BMS::~BMS()
{
  if(handle != -1)
    close(handle);

  handle = -1;
}

int BMS::connect()
{
  if(handle != -1)
    return 0;

  handle = open(port.c_str(), O_RDWR |O_NOCTTY | O_NDELAY);
  /*
  对于open函数来说，第三个参数仅当创建新文件时（即 使用了O_CREAT 时）
  才使用，用于指定文件的访问权限位（access permission bits）。pathname
  是待打开/创建文件的POSIX路径名（如/home/user/a.cpp）；flags 用于指定文
  件的打开/创建模式，这个参数可由以下常量（定义于fcntl.h）通过逻辑位或逻辑构成。
  open返回的文件描述符一定是最小的未被使用的描述符。文件描述符是非负整数
  */
  if(handle == -1)
  {
    //ROS_ERROR("Error opening BMS port");
    return -1;
  }

  fcntl (handle, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);
  /*
  fcntl是计算机中的一种函数，通过fcntl可以改变已打开的文件性质。fcntl
  针对描述符提供控制。参数fd是被参数cmd操作的描述符。针对cmd的值，fcntl能够接受第三个参数int arg。
  fcntl的返回值与命令有关。如果出错，所有命令都返回－1，如果成功则返回某个其他值。

  */
  initPort();

  cout<<"Opening BMS: '"<<port<<"'..."<<"succeeded."<<endl;
  return 0;
}


void BMS::initPort()
{
  if(handle == -1)
    return;

  int BAUDRATE = B57600;
  struct termios newtio;
  tcgetattr (handle, &newtio);
  /*
  tcgetattr是一个函数，用来获取终端参数，成功返回零；失败返回非零，发生失败接口将设置errno错误标识。
  int tcgetattr(int fd, struct termios *termios_p);
  */

  cfsetospeed (&newtio, (speed_t)BAUDRATE);
  cfsetispeed (&newtio, (speed_t)BAUDRATE);
  // 对于波特率的设置通常使用cfsetospeed和cfsetispeed函数来完成。获取波特率信息是通过cfgetispeed和cfgetospeed函数来完成的。

  newtio.c_iflag = IGNBRK;		
  newtio.c_lflag = 0;			
  newtio.c_oflag = 0;			
  newtio.c_cflag |= (CLOCAL | CREAD);	
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
  newtio.c_cflag &= ~CSIZE;		
  newtio.c_cflag |= CS8;			
  newtio.c_cflag &= ~PARENB;		
  newtio.c_cflag &= ~PARODD;		
  newtio.c_cflag &= ~CSTOPB;		
  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 1;

  tcflush (handle, TCIFLUSH);
  /*
  清空终端未完成的输入/输出请求及数据
  TCIFLUSH  // 清除正收到的数据，且不会读取出来。 
  TCOFLUSH  // 清除正写入的数据，且不会发送至终端。
  TCIOFLUSH // 清除所有正在发生的I/O数据。
  */
  tcsetattr (handle, TCSANOW, &newtio);	
  /*
  tcsetattr是用于设置终端参数的函数。
  TCSANOW：不等数据传输完毕就立即改变属性。
  TCSADRAIN：等待所有数据传输结束才改变属性。
  TCSAFLUSH：等待所有数据传输结束,清空输入输出缓冲区才改变属性。 
  &newtio中保存了要改变的属性
  */
}

float BMS::getSOC()
{
  if(handle == -1)
    return -1;

  string str("\x5A\x03\xA1\x02\xF0\x10");
  int countSent = write(handle, str.c_str(), str.length());
  if(countSent < 0)
  {
    return -1;
  }

  int countRcv;
  char buf[512] = "";
  string dataI = "";
  while((countRcv = read(handle, buf, 512)) > 0)
  {
    dataI.append(buf, countRcv);
    if(countRcv < 512)
      break;
  }
  if(countRcv < 0)
  {
    return -1;
  }

  //cout<<"data: "<< dataI <<endl;
  int soc = 0;
  if ((int)dataI[0] == 90)
  {
    //cout<<"SOC: "<< (int)dataI[16] <<endl;
    return (int)dataI[16];
  }
  else
  {
    return -1;
  }
}


