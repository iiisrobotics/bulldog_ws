#ifndef __BMS_H_
#define __BMS_H_

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>

using namespace std;

class BMS {
  private:
    int handle;
    string port;

  protected:
    void initPort();

  public:
    int connect();
    float getSOC();

    BMS(string dev);
    ~BMS();
};

#endif
