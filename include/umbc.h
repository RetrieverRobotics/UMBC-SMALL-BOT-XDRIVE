#ifndef _UMBC_H_
#define _UMBC_H_

#define MSG_DELAY_MS 1000

#ifdef __cplusplus
#include "umbc/controller.hpp"
#include "umbc/controllerinput.hpp"
#include "umbc/controllerrecorder.hpp"
#include "umbc/pcontroller.hpp"
#include "umbc/robot.hpp"
#include "umbc/vcontroller.hpp"
#include "umbc/log.hpp"
#include "okapi\impl\util\configurableTimeUtilFactory.hpp"
#endif

extern okapi::ConfigurableTimeUtilFactory gloabal_time_factory;
extern okapi::TimeUtil global_time;

typedef enum{

    REST = 0,
    GOAL_1 = 1,
    GOAL_2 = 2,
    GOAL_3 = 3,

} ARM_STATE;

typedef enum{

    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1,

} DIGITAL_BUTTON_STATE;

extern DIGITAL_BUTTON_STATE prev_button_state_r1;
extern DIGITAL_BUTTON_STATE cur_button_state_r1;

extern DIGITAL_BUTTON_STATE prev_button_state_r2;
extern DIGITAL_BUTTON_STATE cur_button_state_r2;

extern ARM_STATE cur_state;

#endif // _UMBC_H_