#ifndef _Balance_H_
#define _Balance_H_

#include <Arduino.h>
#include "Controller.h"
#include "Robot.h"

class Balance :public Controller{
    public:
        Balance();
        void reset();
        void execute(Robot *robot, Input *input, Output* output, double dt);


 };


#endif /* _BALANCE_H_ */





