#ifndef _KULLER_SLIDE_
#define _KULLER_SLIDE_

#include<iostream>
#include<pigpio.h>
#include<string>
#include<cmath>

class Morter_front{
    private:
    int pwm_front;
    int dir_front;
    int switch_front;

public:
      Morter_front(int pwm,int dir,int sw)
    :pwm_front(pwm),dir_front(dir),switch_front(sw){}
    


    bool init(){
if(gpioInitialise()<0){
    std::cerr<<"pigpioの初期化に失敗しました"<<std::endl;
    return false;
}
gpioSetMode(pwm_front,PI_OUTPUT);
gpioSetMode(dir_front,PI_OUTPUT);
gpioSetMode(switch_front,PI_INPUT);

gpioWrite(dir_front,1);
return true;

    }
    void update(){
        int sw = gpioRead(switch_front);
        if(sw==1){
            gpioPWM(pwm_front,128);
        }else{gpioPWM(pwm_front,0);
    }
}

void shutdown(){
    gpioPWM(pwm_front,0);
    gpioTerminate();
}
};

#endif