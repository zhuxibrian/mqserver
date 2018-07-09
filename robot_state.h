/*
 * robot_state.h
 *
 * Copyright 2017 DrZ @ JKTech
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <netinet/in.h>
#include "../../../include/global.h"

namespace message_types
{
enum message_type
{
    ROBOT_STATE = 16, ROBOT_MESSAGE = 20, PROGRAM_STATE_MESSAGE = 25
};
}
typedef message_types::message_type messageType;

namespace package_types
{
enum package_type
{
    ROBOT_MODE_DATA = 0,
    JOINT_DATA = 1,
    TOOL_DATA = 2,
    MASTERBOARD_DATA = 3,
    CARTESIAN_INFO = 4,
    KINEMATICS_INFO = 5,
    CONFIGURATION_DATA = 6,
    FORCE_MODE_DATA = 7,
    ADDITIONAL_INFO = 8,
    CALIBRATION_DATA = 9
};
}
typedef package_types::package_type packageType;

namespace robot_message_types
{
enum robot_message_type
{
    ROBOT_MESSAGE_TEXT = 0,
    ROBOT_MESSAGE_PROGRAM_LABEL = 1,
    PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE = 2,
    ROBOT_MESSAGE_VERSION = 3,
    ROBOT_MESSAGE_SAFETY_MODE = 5,
    ROBOT_MESSAGE_ERROR_CODE = 6,
    ROBOT_MESSAGE_KEY = 7,
    ROBOT_MESSAGE_REQUEST_VALUE = 9,
    ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10,

};
}
typedef robot_message_types::robot_message_type robotMessageType;

namespace program_state_types
{
    enum program_state_type
    {
        PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP = 0
    };
}
typedef program_state_types::program_state_type programType;


namespace robot_state_type_v18
{
enum robot_state_type
{
    ROBOT_RUNNING_MODE = 0,
    ROBOT_FREEDRIVE_MODE = 1,
    ROBOT_READY_MODE = 2,
    ROBOT_INITIALIZING_MODE = 3,
    ROBOT_SECURITY_STOPPED_MODE = 4,
    ROBOT_EMERGENCY_STOPPED_MODE = 5,
    ROBOT_FATAL_ERROR_MODE = 6,
    ROBOT_NO_POWER_MODE = 7,
    ROBOT_NOT_CONNECTED_MODE = 8,
    ROBOT_SHUTDOWN_MODE = 9,
    ROBOT_SAFEGUARD_STOP_MODE = 10
};
}
typedef robot_state_type_v18::robot_state_type robotStateTypeV18;
namespace robot_state_type_v30
{
enum robot_state_type
{
    ROBOT_MODE_DISCONNECTED = 0,
    ROBOT_MODE_CONFIRM_SAFETY = 1,
    ROBOT_MODE_BOOTING = 2,
    ROBOT_MODE_POWER_OFF = 3,
    ROBOT_MODE_POWER_ON = 4,
    ROBOT_MODE_IDLE = 5,
    ROBOT_MODE_BACKDRIVE = 6,
    ROBOT_MODE_RUNNING = 7,
    ROBOT_MODE_UPDATING_FIRMWARE = 8
};
}

typedef robot_state_type_v30::robot_state_type robotStateTypeV30;



struct robot_mode_data //ROBOT_MODE_DATA = 0
{
    uint64_t timestamp;
    bool isRobotConnected;
    bool isRealRobotEnabled;
    bool isPowerOnRobot;
    bool isEmergencyStopped;
    bool isProtectiveStopped;
    bool isProgramRunning;
    bool isProgramPaused;
    unsigned char robotMode;
    unsigned char controlMode;
    double targetSpeedFraction;
    double speedScaling;
    double targetSpeedFractionLimit;

    robot_mode_data() {
        timestamp = 4567890;
        isRobotConnected = true;
        isRealRobotEnabled = true;
        isPowerOnRobot = true;
        isEmergencyStopped = true;
        isProtectiveStopped = true;
        isProgramRunning = false;
        isProgramPaused = true;
        robotMode = 12;
        controlMode = 13;
        targetSpeedFraction = 0.7;
        speedScaling = 0.8;
        targetSpeedFractionLimit = 1.8;
    }
};


struct joint_data //JOINT_DATA = 1
{
    double  q_actual_1;
    double  q_target_1;
    double qd_actual_1;
    float  I_actual_1;
    float  V_actual_1;
    float  T_motor_1;
    float  T_micro_1;
    unsigned char  jointMode_1;
    double  q_actual_2;
    double  q_target_2;
    double qd_actual_2;
    float  I_actual_2;
    float  V_actual_2;
    float  T_motor_2;
    float  T_micro_2;
    unsigned char  jointMode_2;
    double  q_actual_3;
    double  q_target_3;
    double qd_actual_3;
    float  I_actual_3;
    float  V_actual_3;
    float  T_motor_3;
    float  T_micro_3;
    unsigned char  jointMode_3;
    double  q_actual_4;
    double  q_target_4;
    double qd_actual_4;
    float  I_actual_4;
    float  V_actual_4;
    float  T_motor_4;
    float  T_micro_4;
    unsigned char  jointMode_4;
    double  q_actual_5;
    double  q_target_5;
    double qd_actual_5;
    float  I_actual_5;
    float  V_actual_5;
    float  T_motor_5;
    float  T_micro_5;
    unsigned char  jointMode_5;
    double  q_actual_6;
    double  q_target_6;
    double qd_actual_6;
    float  I_actual_6;
    float  V_actual_6;
    float  T_motor_6;
    float  T_micro_6;
    unsigned char  jointMode_6;
    double  q_actual_7;
    double  q_target_7;
    double qd_actual_7;
    float  I_actual_7;
    float  V_actual_7;
    float  T_motor_7;
    float  T_micro_7;
    unsigned char  jointMode_7;

    joint_data() {
        q_actual_1 = 0.9;
        q_target_1 = 0.1;
        qd_actual_1 = 0.4;
        I_actual_1 = 1.1;
        T_micro_7 = 0.8;
        jointMode_7 = 1;

    }
};

struct cartesian_info //CARTESIAN_INFO = 4
{
    double X;
    double Y;
    double Z;
    double Rx;
    double Ry;
    double Rz;
    double TCPOffsetX;
    double TCPOffsetY;
    double TCPOffsetZ;
    double TCPOffsetRx;
    double TCPOffsetRy;
    double TCPOffsetRz;
    double Rn;
    cartesian_info() {
        X = 1.23;
        Rn = 2.22;
    }
};

struct masterboard_data //MASTERBOARD_DATA = 3
{
    int digitalInputBits;
    int digitalOutputBits;
    char analogInputRange0;
    char analogInputRange1;
    double analogInput0;
    double analogInput1;
    char analogOutputDomain0;
    char analogOutputDomain1;
    double analogOutput0;
    double analogOutput1;
    float masterBoardTemperature;
    float robotVoltage48V;
    float robotCurrent;
    float masterIOCurrent;
    unsigned char safetyMode;
    unsigned char masterOnOffState;
    char euromap67InterfaceInstalled;
    int euromapInputBits;
    int euromapOutputBits;
    float euromapVoltage;
    float euromapCurrent;
    uint32_t uburso;
    uint8_t operationalModeSelectorInput;
    uint8_t threePositionEnablingDeviceInput;

    masterboard_data() {
        digitalInputBits = 2;
        digitalOutputBits = 4;
        euromap67InterfaceInstalled = 1;
        euromapCurrent = 4.5;
        threePositionEnablingDeviceInput = 9;

    }
};

struct configuration_data
{
    double jointMinLimit_1;
    double jointMaxLimitt_1;
    double jointMinLimit_2;
    double jointMaxLimitt_2;
    double jointMinLimit_3;
    double jointMaxLimitt_3;
    double jointMinLimit_4;
    double jointMaxLimitt_4;
    double jointMinLimit_5;
    double jointMaxLimitt_5;
    double jointMinLimit_6;
    double jointMaxLimitt_6;

    double jointMaxSpeed_1;
    double jointMaxAcceleration_1;
    double vJointDefault_1;
    double aJointDefault_1;
    double vToolDefault_1;
    double aToolDefault_1;
    double eqRadius_1;
    double jointMaxSpeed_2;
    double jointMaxAcceleration_2;
    double vJointDefault_2;
    double aJointDefault_2;
    double vToolDefault_2;
    double aToolDefault_2;
    double eqRadius_2;
    double jointMaxSpeed_3;
    double jointMaxAcceleration_3;
    double vJointDefault_3;
    double aJointDefault_3;
    double vToolDefault_3;
    double aToolDefault_3;
    double eqRadius_3;
    double jointMaxSpeed_4;
    double jointMaxAcceleration_4;
    double vJointDefault_4;
    double aJointDefault_4;
    double vToolDefault_4;
    double aToolDefault_4;
    double eqRadius_4;
    double jointMaxSpeed_5;
    double jointMaxAcceleration_5;
    double vJointDefault_5;
    double aJointDefault_5;
    double vToolDefault_5;
    double aToolDefault_5;
    double eqRadius_5;
    double jointMaxSpeed_6;
    double jointMaxAcceleration_6;
    double vJointDefault_6;
    double aJointDefault_6;
    double vToolDefault_6;
    double aToolDefault_6;
    double eqRadius_6;

    double DHa_1;
    double DHa_2;
    double DHa_3;
    double DHa_4;
    double DHa_5;
    double DHa_6;

    double DHd_1;
    double DHd_2;
    double DHd_3;
    double DHd_4;
    double DHd_5;
    double DHd_6;

    double DHalpha_1;
    double DHalpha_2;
    double DHalpha_3;
    double DHalpha_4;
    double DHalpha_5;
    double DHalpha_6;

    double DHtheta_1;
    double DHtheta_2;
    double DHtheta_3;
    double DHtheta_4;
    double DHtheta_5;
    double DHtheta_6;

    int masterboardVersion;
    int controllerBoxType;
    int robotType;
    int robotSubType;

    configuration_data() {
        jointMinLimit_1 = 9.9;
        jointMaxLimitt_6 = 9.3;

        DHtheta_6 = 1.1;
        masterboardVersion = 2;
        robotSubType = 1;
    }
};

struct additional_info
{
    bool freedriveButtonPressed;
    bool freedriveButtonEnabled;
    bool IOEnabledFreedrive;

    additional_info() {
        freedriveButtonPressed = true;
    }
};

struct version_message
{
    uint64_t timestamp = 0;
    int8_t source;
    int8_t robot_message_type;
    uint8_t project_name_size;
    char *project_name;
    uint8_t major_version;
    uint8_t minor_version;
    int svn_revision;
    uint8_t build_date_size;
    char *build_date;

    version_message () {
        major_version = 3;
        svn_revision = 9;
        project_name_size = 0;
        project_name = nullptr;
        build_date_size = 0;
        build_date = nullptr;
    }
    ~version_message() {
        ZX_DELETE(project_name);
        ZX_DELETE(build_date);
    }
};

struct safetyModeMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    int robotMessageCode;
    int robotMessageArgument;
    char safetyModeType;
    uint8_t textmessage_size;
    char *textMessage;

    safetyModeMessage(){
        timestamp = 0;
        source = 0;
        robot_message_type = 1;
        robotMessageCode = 1;
        robotMessageArgument = 1;
        safetyModeType = 1;
        textmessage_size = 0;
        textMessage = nullptr;
    }
    ~safetyModeMessage(){
        ZX_DELETE(textMessage);
    }
};

struct robotcommMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    int robotMessageCode;
    int robotMessageArgument;
    int warningLevel;
    uint8_t textmessage_size;
    char *textMessage;

    robotcommMessage() {
        robotMessageCode = 1;
        textmessage_size = 0;
        textMessage = nullptr;
    }

    ~robotcommMessage(){
        ZX_DELETE(textMessage);
    }
};

struct keyMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    int robotMessageCode;
    int robotMessageArgument;
    uint8_t titleSize;
    char * messageTitle;
    uint8_t textMessage_size;
    char * textMessage;

    keyMessage() {
        robotMessageArgument = 2;
        titleSize = 0;
        messageTitle = nullptr;
        textMessage_size = 0;
        textMessage = nullptr;
    }

    ~keyMessage(){
        ZX_DELETE(messageTitle);
        ZX_DELETE(textMessage);
    }
};

struct labelMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    int id;
    uint8_t textMessage_size;
    char *textMessage;

    labelMessage(){
        source = 1;
        id = 2;
        textMessage_size = 0;
        textMessage = nullptr;
    }
    ~labelMessage(){
        ZX_DELETE(textMessage);
    }
};

struct globalVariablesSetupMessage
{
    uint64_t timestamp;
    int8_t robot_message_type;
    unsigned short startIndex;
    uint8_t variableNames_size;
    char * variableNames;

    globalVariablesSetupMessage(){
        startIndex = 2;
        variableNames_size = 0;
        variableNames = nullptr;
    }
    ~globalVariablesSetupMessage(){
        ZX_DELETE(variableNames);
    }
};

struct requestValueMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    unsigned int requestId;
    bool warning;
    bool error;
    bool blocking;
    unsigned char titleLength;
    uint8_t messageTitle_size;
    char * messageTitle;
    uint8_t textMessage_size;
    char * textMessage;

    requestValueMessage() {
        messageTitle_size = 0;
        messageTitle = nullptr;
        textMessage_size = 0;
        textMessage = nullptr;
    }
    ~requestValueMessage(){
        ZX_DELETE(messageTitle);
        ZX_DELETE(textMessage);
    }
};

struct textMessageStruct
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    uint8_t textMessage_size;
    char * textMessage;

    textMessageStruct() {
        textMessage_size = 0;
        textMessage = nullptr;
    }
    ~textMessageStruct(){
        ZX_DELETE(textMessage);
    }
};

struct runtimeExceptionMessage
{
    uint64_t timestamp;
    char source;
    int8_t robot_message_type;
    uint8_t textMessage_size;
    char * textMessage;


    runtimeExceptionMessage() {
        textMessage_size = 0;
        textMessage = nullptr;
    }
    ~runtimeExceptionMessage(){
        ZX_DELETE(textMessage);
    }
};

struct varMessage
{
    uint64_t timestamp;
    int8_t robot_message_type;
    uint8_t titleSize;
    char * messageTitle;
    uint8_t messageText_size;
    char * messageText;

    varMessage() {
        titleSize = 0;
        messageTitle = nullptr;
        messageText_size = 0;
        messageText = nullptr;
    }
    ~varMessage(){
        ZX_DELETE(messageTitle);
        ZX_DELETE(messageText);
    }
};

struct globalVariablesUpdateMessage
{
    uint64_t timestamp;
    int8_t robot_message_type;
    unsigned short startIndex;
    unsigned char valueType;
    void* value;

    // todo 析构函数
//    ~globalVariablesUpdateMessage(){
//        delete value;
//        value = nullptr;
//    }
};

struct globalVariablesUpdateMessage_string
{
    unsigned short length;
    char * value;


//    ~globalVariablesUpdateMessage_string(){
//        ZX_DELETE(value);
//    }
};

struct globalVariablesUpdateMessage_pose
{
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
};

struct globalVariablesUpdateMessage_list
{
    unsigned short listLength;
    void * value;
// todo 析构函数
//    ~globalVariablesUpdateMessage_list(){
//        delete[] value;
//    }
};

struct tool_data// TOOL_DATA = 2
{
    char analogInputRange2;
    char analogInputRange3;
    double analogInput2;
    double analogInput3;
    float toolVoltage48V;
    unsigned char toolOutputVoltage;
    float toolCurrent;
    float toolTemperature;
    unsigned char toolMode;
};

struct force_mode_data
{
    double X[7];
    double Y[7];
    double Z[7];
    double Rx[7];
    double Ry[7];
    double Rz[7];
    double robotDexterity[7];
};

struct Calibration_data
{

};

struct Kinematics_info
{

};

class RobotState
{
public:
    robot_mode_data robot_mode_;
	joint_data joint_data_;
	cartesian_info cartesian_info_;
	masterboard_data mb_data_;
	configuration_data configuration_data_;
	additional_info additional_info_;

	version_message version_msg_;
	safetyModeMessage safetyModeMessage_;
	robotcommMessage robotcommMessage_;
	keyMessage keyMessage_;
	labelMessage labelMessage_;
	globalVariablesSetupMessage globalVariablesSetupMessage_;



	//todo 暂不实现
	requestValueMessage requestValueMessage_;
	textMessageStruct textMessage_;
	runtimeExceptionMessage runtimeExceptionMessage_;
	varMessage varMessage_;
	globalVariablesUpdateMessage globalVariablesUpdateMessage_;
	tool_data tool_data_;
	force_mode_data force_mode_data_;
	Calibration_data calibration_data_;
	Kinematics_info kinematics_info_;

private:
    std::recursive_mutex val_lock_; // Locks the variables while unpack parses data;

    std::condition_variable* pMsg_cond_; //Signals that new vars are available
    bool new_data_available_; //to avoid spurious wakes
    unsigned char robot_mode_running_;

    double ntohd(uint64_t nf);
    double htond(uint64_t nf);


public:
    RobotState(std::condition_variable& msg_cond);
    ~RobotState();
    double getVersion();
    double getTime();
    std::vector<double> getQTarget(); //TODO

    // region 这里都是get/set方法，现在把结构体变量都声明为public，不需要单独写get/set了
//    int getDigitalInputBits();
//    int getDigitalOutputBits();
//    char getAnalogInputRange0();
//    char getAnalogInputRange1();
//    double getAnalogInput0();
//    double getAnalogInput1();
//    char getAnalogOutputDomain0();
//    char getAnalogOutputDomain1();
//    double getAnalogOutput0();
//    double getAnalogOutput1();
//    std::vector<double> getVActual(); //TODO
//    float getMasterBoardTemperature();
//    float getRobotVoltage48V();
//    float getRobotCurrent();
//    float getMasterIOCurrent();
//    unsigned char getSafetyMode();
//    unsigned char getInReducedMode();
//    char getEuromap67InterfaceInstalled();
//    int getEuromapInputBits();
//    int getEuromapOutputBits();
//    float getEuromapVoltage();
//    float getEuromapCurrent();
//
//    void setDigitalInputBits(int digitalInputBits);
//    void setDigitalOutputBits(int digitalOutputBits);
//    void setAnalogInputRange0(char analogInput0);
//    void setAnalogInputRange1(char analogInput1);
//    void setAnalogInput0(double analogInput0);
//    void setAnalogInput1(double analogInput1);
//    void setAnalogOutputDomain0(char analogOutputDomain0);
//    void setAnalogOutputDomain1(char analogOutputDomain1);
//    void setAnalogOutput0(double analogOutput0);
//    void setAnalogOutput1(double analogOutput1);
//    void setMasterBoardTemperature(float masterBoardTemperature);
//    void setRobotVoltage48V(float robotVoltage48V);
//    void setRobotCurrent(float robotCurrent);
//    void setMasterIOCurrent(float masterIOCurrent);
//    void setSafetyMode(unsigned char safetyMode);
//    void setInReducedMode(unsigned char masterOnOffState);
//    void setEuromap67InterfaceInstalled(char euromap67InterfaceInstalled);
//    void setEuromapInputBits(int euromapInputBits);
//    void setEuromapOutputBits(int euromapOutputBits);
//    void setEuromapVoltage(float euromapVoltage);
//    void setEuromapCurrent(float euromapCurrent);
//
//	/* Joint data get_ */
//	double get_q_actual_1();
//	double get_q_target_1();
//	double get_qd_actual_1();
//	float get_I_actual_1();
//	float get_V_actual_1();
//	float get_T_motor_1();
//	float get_T_micro_1();
//	unsigned char get_jointMode_1();
//	double get_q_actual_2();
//	double  get_q_target_2();
//	double get_qd_actual_2();
//	float  get_I_actual_2();
//	float  get_V_actual_2();
//	float  get_T_motor_2();
//	float  get_T_micro_2();
//	unsigned char get_jointMode_2();
//	double get_q_actual_3();
//	double get_q_target_3();
//	double get_qd_actual_3();
//	float get_I_actual_3();
//	float get_V_actual_3();
//	float get_T_motor_3();
//	float get_T_micro_3();
//	unsigned char get_jointMode_3();
//	double get_q_actual_4();
//	double get_q_target_4();
//	double get_qd_actual_4();
//	float get_I_actual_4();
//	float get_V_actual_4();
//	float get_T_motor_4();
//	float get_T_micro_4();
//	unsigned char  get_jointMode_4();
//	double get_q_actual_5();
//	double get_q_target_5();
//	double get_qd_actual_5();
//	float get_I_actual_5();
//	float get_V_actual_5();
//	float get_T_motor_5();
//	float get_T_micro_5();
//	unsigned char  get_jointMode_5();
//	double  get_q_actual_6();
//	double  get_q_target_6();
//	double get_qd_actual_6();
//	float get_I_actual_6();
//	float get_V_actual_6();
//	float get_T_motor_6();
//	float get_T_micro_6();
//	unsigned char get_jointMode_6();
//	double get_q_actual_7();
//	double get_q_target_7();
//	double get_qd_actual_7();
//	float get_I_actual_7();
//	float get_V_actual_7();
//	float get_T_motor_7();
//	float get_T_micro_7();
//	unsigned char  get_jointMode_7();
//
//  // Joint data set_
//	void set_q_actual_1(double q_actual_1);
//	void set_q_target_1(double q_target_1);
//	void set_qd_actual_1(double qd_actual_1);
//	void set_I_actual_1(float I_actual_1);
//	void set_V_actual_1(float V_actual_1);
//	void set_T_motor_1(float T_motor_1);
//	void set_T_micro_1(float T_micro_1);
//	void set_jointMode_1(unsigned char jointMode_1);
//	void set_q_actual_2(double q_actual_2);
//	void set_q_target_2(double q_target_2);
//	void set_qd_actual_2(double qd_actual_2);
//	void set_I_actual_2(float I_actual_2);
//	void set_V_actual_2(float V_actual_2);
//	void set_T_motor_2(float T_motor_2);
//	void set_T_micro_2(float T_micro_2);
//	void set_jointMode_2(unsigned char jointMode_2);
//	void set_q_actual_3(double q_actual_3);
//	void set_q_target_3(double q_target_3);
//	void set_qd_actual_3(double qd_actual_3);
//	void set_I_actual_3(float I_actual_3);
//	void set_V_actual_3(float V_actual_3);
//	void set_T_motor_3(float T_motor_3);
//	void set_T_micro_3(float T_micro_3);
//	void set_jointMode_3(unsigned char jointMode_3);
//	void set_q_actual_4(double q_actual_4);
//	void set_q_target_4(double q_target_4);
//	void set_qd_actual_4(double qd_actual_4);
//	void set_I_actual_4(float I_actual_4);
//	void set_V_actual_4(float V_actual_4);
//	void set_T_motor_4(float T_motor_4);
//	void set_T_micro_4(float T_micro_4);
//	void set_jointMode_4(unsigned char jointMode_4);
//	void set_q_actual_5(double q_actual_5);
//	void set_q_target_5(double q_target_5);
//	void set_qd_actual_5(double qd_actual_5);
//	void set_I_actual_5(float I_actual_5);
//	void set_V_actual_5(float V_actual_5);
//	void set_T_motor_5(float T_motor_5);
//	void set_T_micro_5(float T_micro_5);
//	void set_jointMode_5(unsigned char jointMode_5);
//	void set_q_actual_6(double q_actual_6);
//	void set_q_target_6(double q_target_6);
//	void set_qd_actual_6(double qd_actual_6);
//	void set_I_actual_6(float I_actual_6);
//	void set_V_actual_6(float V_actual_6);
//	void set_T_motor_6(float T_motor_6);
//	void set_T_micro_6(float T_micro_6);
//	void set_jointMode_6(unsigned char jointMode_6);
//	void set_q_actual_7(double q_actual_7);
//	void set_q_target_7(double q_target_7);
//	void set_qd_actual_7(double qd_actual_7);
//	void set_I_actual_7(float I_actual_7);
//	void set_V_actual_7(float V_actual_7);
//	void set_T_motor_7(float T_motor_7);
//	void set_T_micro_7(float T_micro_7);
//	void set_jointMode_7(unsigned char jointMode_7);
//
//	/* Cartesian info get_ */
//	double get_X();
//	double get_Y();
//	double get_Z();
//	double get_Rx();
//	double get_Ry();
//	double get_Rz();
//	double get_TCPOffsetX();
//	double get_TCPOffsetY();
//	double get_TCPOffsetZ();
//	double get_TCPOffsetRx();
//	double get_TCPOffsetRy();
//	double get_TCPOffsetRz();
//
//	/* Cartesian info set_ */
//	void set_X(double X);
//	void set_Y(double Y);
//	void set_Z(double Z);
//	void set_Rx(double Rx);
//	void set_Ry(double Ry);
//	void set_Rz(double Rz);
//	void set_TCPOffsetX(double TCPOffsetX);
//	void set_TCPOffsetY(double TCPOffsetY);
//	void set_TCPOffsetZ(double TCPOffsetZ);
//	void set_TCPOffsetRx(double TCPOffsetRx);
//	void set_TCPOffsetRy(double TCPOffsetRy);
//	void set_TCPOffsetRz(double TCPOffsetRz);
//
//
//    bool isRobotConnected();
//    bool isRealRobotEnabled();
//    bool isPowerOnRobot();
//    bool isEmergencyStopped();
//    bool isProtectiveStopped();
//    bool isProgramRunning();
//    bool isProgramPaused();
//    unsigned char getRobotMode();
//    bool isReady();
//
//    void setRobotConnected(bool isRobotConnected);
//    void setRealRobotEnabled(bool isRealRobotEnabled);
//    void setPowerOnRobot(bool isPowerOnRobot);
//    void setEmergencyStopped(bool isEmergencyStopped);
//    void setProtectiveStopped(bool isProtectiveStopped);
//    void setProgramRunning(bool isProgramRunning);
//    void setProgramPaused(bool isProgramPaused);
//    void setRobotMode(unsigned char robotMode);

// endregion

    void setDisconnected();

    bool getNewDataAvailable();
    void finishedReading();

    //todo unpack from net 解释器不用，暂不实现
    unsigned int unpack(uint8_t * buf, unsigned int buf_length);

    unsigned int unpackRobotState(uint8_t * buf, unsigned int offset, uint32_t len);

    unsigned int unpackRobotStateMasterboard(uint8_t * buf, unsigned int offset);

    unsigned int unpackRobotMode(uint8_t * buf, unsigned int offset);

    unsigned int unpackJointData(uint8_t * buf, unsigned int offset);

    unsigned int unpackCartesianInfo(uint8_t * buf, unsigned int offset);

    unsigned int unpackConfigurationData(uint8_t *buf, unsigned int offset);

    unsigned int unpackAdditionalInfo(uint8_t *buf, unsigned int offset);

    unsigned int unpackRobotMessage(uint8_t * buf, unsigned int offset, uint32_t len);

    unsigned int unpackRobotMessageVersion(uint8_t * buf, unsigned int offset, uint32_t len);

    unsigned int unpackRobotMessageVersion(uint8_t * buf, unsigned int offset);
    unsigned int unpackSafetyModeMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackRobotcommMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackKeyMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackLabelMessage(uint8_t *buf, unsigned int offset);

    unsigned int unpackGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset);

    /** pack to net*/
    unsigned int pack(uint8_t* buf);

	unsigned int packRobotState(uint8_t* buf, unsigned int offset, uint8_t package_type);
	unsigned int packRobotMode(uint8_t * buf, unsigned int offset);
	unsigned int packJointData(uint8_t * buf, unsigned int offset);
	unsigned int packCartesianInfo(uint8_t * buf, unsigned int offset);
	unsigned int packRobotStateMasterboard(uint8_t * buf, unsigned int offset);
	unsigned int packConfigurationData(uint8_t *buf, unsigned int offset);
	unsigned int packAdditionalInfo(uint8_t *buf, unsigned int offset);

	unsigned int packRobotMessage(uint8_t* buf, unsigned int offset, uint8_t package_type);
	unsigned int packRobotMessageVersion(uint8_t* buf, unsigned int offset);
    unsigned int packSafetyModeMessage(uint8_t *buf, unsigned int offset);
    unsigned int packRobotcommMessage(uint8_t *buf, unsigned int offset);
    unsigned int packKeyMessage(uint8_t *buf, unsigned int offset);
    unsigned int packLabelMessage(uint8_t *buf, unsigned int offset);

    unsigned int packProgramMessage(uint8_t* buf, unsigned int offset, uint8_t package_type);
    unsigned int packGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset);


    /** unpack from memory*/
    unsigned int unpackFromMem(uint8_t * buf, unsigned int buf_length);

    unsigned int unpackFromMemRobotStateMasterboard(uint8_t * buf, unsigned int offset);

    unsigned int unpackFromMemRobotMode(uint8_t * buf, unsigned int offset);

    unsigned int unpackFromMemJointData(uint8_t * buf, unsigned int offset);

    unsigned int unpackFromMemCartesianInfo(uint8_t * buf, unsigned int offset);

    unsigned int unpackFromMemConfigurationData(uint8_t *buf, unsigned int offset);

    unsigned int unpackFromMemAdditionalInfo(uint8_t *buf, unsigned int offset);

    unsigned int unpackFromMemRobotMessageVersion(uint8_t * buf, unsigned int offset);
    unsigned int unpackFromMemSafetyModeMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackFromMemRobotcommMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackFromMemKeyMessage(uint8_t *buf, unsigned int offset);
    unsigned int unpackFromMemLabelMessage(uint8_t *buf, unsigned int offset);

    unsigned int unpackFromMemGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset);

    /** pack to memory*/
    unsigned int packToMem(uint8_t * buf);
    unsigned int packToMemRobotState(uint8_t* buf, unsigned int offset, uint8_t packToMemage_type);
    unsigned int packToMemRobotMode(uint8_t * buf, unsigned int offset);
    unsigned int packToMemJointData(uint8_t * buf, unsigned int offset);
    unsigned int packToMemCartesianInfo(uint8_t * buf, unsigned int offset);
    unsigned int packToMemRobotStateMasterboard(uint8_t * buf, unsigned int offset);
    unsigned int packToMemConfigurationData(uint8_t *buf, unsigned int offset);
    unsigned int packToMemAdditionalInfo(uint8_t *buf, unsigned int offset);

    unsigned int packToMemRobotMessage(uint8_t* buf, unsigned int offset, uint8_t packToMemage_type);
    unsigned int packToMemRobotMessageVersion(uint8_t* buf, unsigned int offset);
    unsigned int packToMemSafetyModeMessage(uint8_t *buf, unsigned int offset);
    unsigned int packToMemRobotcommMessage(uint8_t *buf, unsigned int offset);
    unsigned int packToMemKeyMessage(uint8_t *buf, unsigned int offset);
    unsigned int packToMemLabelMessage(uint8_t *buf, unsigned int offset);

    unsigned int packToMemProgramMessage(uint8_t* buf, unsigned int offset, uint8_t packToMemage_type);
    unsigned int packToMemGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset);

    };

#endif /* ROBOT_STATE_H_ */
