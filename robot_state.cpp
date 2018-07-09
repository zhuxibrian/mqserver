/*
 * robot_state.cpp
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

#include "robot_state.h"
#include "./include/endian.h"

RobotState::RobotState(std::condition_variable& msg_cond)
{
    version_msg_.major_version = 3;
    version_msg_.minor_version = 0;
    new_data_available_ = false;
    pMsg_cond_ = &msg_cond;
    //RobotState::setDisconnected();
    robot_mode_running_ = robotStateTypeV30::ROBOT_MODE_RUNNING;
}

RobotState::~RobotState()
{
}
double RobotState::ntohd(uint64_t nf)
{
    double x;
    nf = be64toh(nf);
    memcpy(&x, &nf, sizeof(x));
    return x;
}
double RobotState::htond(uint64_t nf)
{
    double x;
    nf = htobe64(nf);
    memcpy(&x, &nf, sizeof(x));
    return x;
}

//Example: two packages (ROBOT_MODE_DATA and MASTERBOARD_DATA) in one message (ROBOT_STATE)
unsigned int RobotState::pack(uint8_t* buf)
{
    unsigned int buf_length = sizeof(buf_length);
    unsigned char message_type;

    message_type = messageType::ROBOT_STATE;
    memcpy(&buf[buf_length], &message_type, sizeof(message_type));
    buf_length += sizeof(message_type);

    buf_length += RobotState::packRobotState(buf, buf_length, packageType::ROBOT_MODE_DATA);
	buf_length += RobotState::packRobotState(buf, buf_length, packageType::JOINT_DATA);
	buf_length += RobotState::packRobotState(buf, buf_length, packageType::CARTESIAN_INFO);
    buf_length += RobotState::packRobotState(buf, buf_length, packageType::MASTERBOARD_DATA);
    buf_length += RobotState::packRobotState(buf, buf_length, packageType::CONFIGURATION_DATA);
    buf_length += RobotState::packRobotState(buf, buf_length, packageType::ADDITIONAL_INFO);


    buf_length += RobotState::packRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_VERSION);
    buf_length += RobotState::packRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_SAFETY_MODE);
    buf_length += RobotState::packRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_ERROR_CODE);
    buf_length += RobotState::packRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_KEY);
    buf_length += RobotState::packRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL);

    buf_length += RobotState::packProgramMessage(buf, buf_length, programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP);

    buf_length = htonl(buf_length);
    memcpy(&buf[0], &buf_length, sizeof(buf_length));
    return ntohl(buf_length);
}

//TODO: whether buf_length is necessary? the first four bytes is the length of overall package
unsigned int RobotState::unpack(uint8_t* buf, unsigned int buf_length)
{
    /* Returns missing bytes to unpack a message, or 0 if all data was parsed */
    unsigned int offset = 0;
    while (buf_length > offset)
    {
        int len;
        unsigned char message_type;
        memcpy(&len, &buf[offset], sizeof(len));
        len = ntohl(len);
        if (len + offset > buf_length)
        {
            return 0;
        }
        memcpy(&message_type, &buf[offset + sizeof(len)], sizeof(message_type));
        switch (message_type)
        {
        case messageType::ROBOT_MESSAGE:
            RobotState::unpackRobotMessage(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
            break;
        case messageType::ROBOT_STATE:
            RobotState::unpackRobotState(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
            break;
        case messageType::PROGRAM_STATE_MESSAGE:
        //Don't do anything atm...
        default:
            break;
        }
        offset += len;

    }
    return offset;
}

unsigned int RobotState::packRobotMessage(uint8_t* buf, unsigned int offset, uint8_t package_type)
{

    int32_t length = 5;
    uint8_t msgType = messageType::ROBOT_MESSAGE;
    switch (package_type)
    {
        case robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL:
            val_lock_.lock();
            length += RobotState::packLabelMessage(buf, offset + 5);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_SAFETY_MODE:
            val_lock_.lock();
            length += RobotState::packSafetyModeMessage(buf, offset + 5);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_ERROR_CODE:
            val_lock_.lock();
            length += RobotState::packRobotcommMessage(buf, offset + 5);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_KEY:
            val_lock_.lock();
            length += RobotState::packKeyMessage(buf, offset + 5);
            val_lock_.unlock();
            break;
        case robotMessageType::ROBOT_MESSAGE_VERSION:
            val_lock_.lock();
            length += RobotState::packRobotMessageVersion(buf, offset + 5);
            val_lock_.unlock();
            break;
        default:
            break;
    }
    memcpy(&buf[offset + sizeof(length)], &msgType, sizeof(msgType));
    length = htonl(length);
    memcpy(&buf[offset], &length, sizeof(length));
    return ntohl(length);
}

unsigned int RobotState::unpackRobotMessage(uint8_t * buf, unsigned int offset,
                                    uint32_t len)
{
//    offset += 5;
//    uint64_t timestamp;
//    int8_t source, robot_message_type;
//    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
//    offset += sizeof(timestamp);
//    memcpy(&source, &buf[offset], sizeof(source));
//    offset += sizeof(source);
//    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
//    offset += sizeof(robot_message_type);
//    switch (robot_message_type)
//    {
//    case robotMessageType::ROBOT_MESSAGE_VERSION:
//        val_lock_.lock();
//        version_msg_.timestamp = timestamp;
//        version_msg_.source = source;
//        version_msg_.robot_message_type = robot_message_type;
//        RobotState::unpackRobotMessageVersion(buf, offset, len);
//        val_lock_.unlock();
//        break;
//    default:
//        break;
//    }
    return 0;

}

unsigned int RobotState::packJointData(uint8_t *buf, unsigned int offset)
{
    unsigned int offset_ = offset;
    memcpy(&buf[offset], &joint_data_.q_actual_1, sizeof(joint_data_.q_actual_1));
    offset += sizeof(joint_data_.q_actual_1);
    memcpy(&buf[offset], &joint_data_.q_target_1, sizeof(joint_data_.q_target_1));
    offset += sizeof(joint_data_.q_target_1);
    memcpy(&buf[offset], &joint_data_.qd_actual_1, sizeof(joint_data_.qd_actual_1));
    offset += sizeof(joint_data_.qd_actual_1);
	memcpy(&buf[offset], &joint_data_.I_actual_1, sizeof(joint_data_.I_actual_1));
    offset += sizeof(joint_data_.I_actual_1);
    memcpy(&buf[offset], &joint_data_.V_actual_1, sizeof(joint_data_.V_actual_1));
    offset += sizeof(joint_data_.V_actual_1);
	memcpy(&buf[offset], &joint_data_.T_motor_1, sizeof(joint_data_.T_motor_1));
    offset += sizeof(joint_data_.T_motor_1);
    memcpy(&buf[offset], &joint_data_.T_micro_1, sizeof(joint_data_.T_micro_1));
    offset += sizeof(joint_data_.T_micro_1);		
    memcpy(&buf[offset], &joint_data_.jointMode_1, sizeof(joint_data_.jointMode_1));
    offset += sizeof(joint_data_.jointMode_1);		
    memcpy(&buf[offset], &joint_data_.q_actual_2, sizeof(joint_data_.q_actual_2));
    offset += sizeof(joint_data_.q_actual_2);	
    memcpy(&buf[offset], &joint_data_.q_target_2, sizeof(joint_data_.q_target_2));
    offset += sizeof(joint_data_.q_target_2);	
    memcpy(&buf[offset], &joint_data_.qd_actual_2, sizeof(joint_data_.qd_actual_2));
    offset += sizeof(joint_data_.qd_actual_2);	
    memcpy(&buf[offset], &joint_data_.I_actual_2, sizeof(joint_data_.I_actual_2));
    offset += sizeof(joint_data_.I_actual_2);		
    memcpy(&buf[offset], &joint_data_.V_actual_2, sizeof(joint_data_.V_actual_2));
    offset += sizeof(joint_data_.V_actual_2);	
    memcpy(&buf[offset], &joint_data_.T_motor_2, sizeof(joint_data_.T_motor_2));
    offset += sizeof(joint_data_.T_motor_2);		
    memcpy(&buf[offset], &joint_data_.T_micro_2, sizeof(joint_data_.T_micro_2));
    offset += sizeof(joint_data_.T_micro_2);		
    memcpy(&buf[offset], &joint_data_.jointMode_2, sizeof(joint_data_.jointMode_2));
    offset += sizeof(joint_data_.jointMode_2);	
	memcpy(&buf[offset], &joint_data_.q_actual_3, sizeof(joint_data_.q_actual_3));
    offset += sizeof(joint_data_.q_actual_3);		
    memcpy(&buf[offset], &joint_data_.q_target_3, sizeof(joint_data_.q_target_3));
    offset += sizeof(joint_data_.q_target_3);		
    memcpy(&buf[offset], &joint_data_.qd_actual_3, sizeof(joint_data_.qd_actual_3));
    offset += sizeof(joint_data_.qd_actual_3);		
    memcpy(&buf[offset], &joint_data_.I_actual_3, sizeof(joint_data_.I_actual_3));
    offset += sizeof(joint_data_.I_actual_3);	
	memcpy(&buf[offset], &joint_data_.V_actual_3, sizeof(joint_data_.V_actual_3));
    offset += sizeof(joint_data_.V_actual_3);		
    memcpy(&buf[offset], &joint_data_.T_motor_3, sizeof(joint_data_.T_motor_3));
    offset += sizeof(joint_data_.T_motor_3);	
    memcpy(&buf[offset], &joint_data_.T_micro_3, sizeof(joint_data_.T_micro_3));
    offset += sizeof(joint_data_.T_micro_3);	
    memcpy(&buf[offset], &joint_data_.jointMode_3, sizeof(joint_data_.jointMode_3));
    offset += sizeof(joint_data_.jointMode_3);		
	memcpy(&buf[offset], &joint_data_.q_actual_4, sizeof(joint_data_.q_actual_4));
    offset += sizeof(joint_data_.q_actual_4);		
    memcpy(&buf[offset], &joint_data_.q_target_4, sizeof(joint_data_.q_target_4));
    offset += sizeof(joint_data_.q_target_4);		
    memcpy(&buf[offset], &joint_data_.qd_actual_4, sizeof(joint_data_.qd_actual_4));
    offset += sizeof(joint_data_.qd_actual_4);		
    memcpy(&buf[offset], &joint_data_.I_actual_4, sizeof(joint_data_.I_actual_4));
    offset += sizeof(joint_data_.I_actual_4);	
	memcpy(&buf[offset], &joint_data_.V_actual_4, sizeof(joint_data_.V_actual_4));
    offset += sizeof(joint_data_.V_actual_4);		
    memcpy(&buf[offset], &joint_data_.T_motor_4, sizeof(joint_data_.T_motor_4));
    offset += sizeof(joint_data_.T_motor_4);	
    memcpy(&buf[offset], &joint_data_.T_micro_4, sizeof(joint_data_.T_micro_4));
    offset += sizeof(joint_data_.T_micro_4);	
    memcpy(&buf[offset], &joint_data_.jointMode_4, sizeof(joint_data_.jointMode_4));
    offset += sizeof(joint_data_.jointMode_4);		
	memcpy(&buf[offset], &joint_data_.q_actual_5, sizeof(joint_data_.q_actual_5));
    offset += sizeof(joint_data_.q_actual_5);		
    memcpy(&buf[offset], &joint_data_.q_target_5, sizeof(joint_data_.q_target_5));
    offset += sizeof(joint_data_.q_target_5);		
    memcpy(&buf[offset], &joint_data_.qd_actual_5, sizeof(joint_data_.qd_actual_5));
    offset += sizeof(joint_data_.qd_actual_5);		
    memcpy(&buf[offset], &joint_data_.I_actual_5, sizeof(joint_data_.I_actual_5));
    offset += sizeof(joint_data_.I_actual_5);	
	memcpy(&buf[offset], &joint_data_.V_actual_5, sizeof(joint_data_.V_actual_5));
    offset += sizeof(joint_data_.V_actual_5);		
    memcpy(&buf[offset], &joint_data_.T_motor_5, sizeof(joint_data_.T_motor_5));
    offset += sizeof(joint_data_.T_motor_5);	
    memcpy(&buf[offset], &joint_data_.T_micro_5, sizeof(joint_data_.T_micro_5));
    offset += sizeof(joint_data_.T_micro_5);	
    memcpy(&buf[offset], &joint_data_.jointMode_5, sizeof(joint_data_.jointMode_5));
    offset += sizeof(joint_data_.jointMode_5);
	memcpy(&buf[offset], &joint_data_.q_actual_6, sizeof(joint_data_.q_actual_6));
    offset += sizeof(joint_data_.q_actual_6);		
    memcpy(&buf[offset], &joint_data_.q_target_6, sizeof(joint_data_.q_target_6));
    offset += sizeof(joint_data_.q_target_6);		
    memcpy(&buf[offset], &joint_data_.qd_actual_6, sizeof(joint_data_.qd_actual_6));
    offset += sizeof(joint_data_.qd_actual_6);		
    memcpy(&buf[offset], &joint_data_.I_actual_6, sizeof(joint_data_.I_actual_6));
    offset += sizeof(joint_data_.I_actual_6);	
	memcpy(&buf[offset], &joint_data_.V_actual_6, sizeof(joint_data_.V_actual_6));
    offset += sizeof(joint_data_.V_actual_6);		
    memcpy(&buf[offset], &joint_data_.T_motor_6, sizeof(joint_data_.T_motor_6));
    offset += sizeof(joint_data_.T_motor_6);	
    memcpy(&buf[offset], &joint_data_.T_micro_6, sizeof(joint_data_.T_micro_6));
    offset += sizeof(joint_data_.T_micro_6);	
    memcpy(&buf[offset], &joint_data_.jointMode_6, sizeof(joint_data_.jointMode_6));
    offset += sizeof(joint_data_.jointMode_6);
	memcpy(&buf[offset], &joint_data_.q_actual_7, sizeof(joint_data_.q_actual_7));
    offset += sizeof(joint_data_.q_actual_7);		
    memcpy(&buf[offset], &joint_data_.q_target_7, sizeof(joint_data_.q_target_7));
    offset += sizeof(joint_data_.q_target_7);		
    memcpy(&buf[offset], &joint_data_.qd_actual_7, sizeof(joint_data_.qd_actual_7));
    offset += sizeof(joint_data_.qd_actual_7);		
    memcpy(&buf[offset], &joint_data_.I_actual_7, sizeof(joint_data_.I_actual_7));
    offset += sizeof(joint_data_.I_actual_7);	
	memcpy(&buf[offset], &joint_data_.V_actual_7, sizeof(joint_data_.V_actual_7));
    offset += sizeof(joint_data_.V_actual_7);		
    memcpy(&buf[offset], &joint_data_.T_motor_7, sizeof(joint_data_.T_motor_7));
    offset += sizeof(joint_data_.T_motor_7);	
    memcpy(&buf[offset], &joint_data_.T_micro_7, sizeof(joint_data_.T_micro_7));
    offset += sizeof(joint_data_.T_micro_7);	
    memcpy(&buf[offset], &joint_data_.jointMode_7, sizeof(joint_data_.jointMode_7));
    offset += sizeof(joint_data_.jointMode_7);	
	
	return (offset - offset_);
}

unsigned int RobotState::unpackJointData(uint8_t * buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    memcpy(&joint_data_.q_actual_1, &buf[offset], sizeof(joint_data_.q_actual_1));
    offset += sizeof(joint_data_.q_actual_1);
    memcpy(&joint_data_.q_target_1, &buf[offset], sizeof(joint_data_.q_target_1));	
	offset += sizeof(joint_data_.q_target_1);
	memcpy(&joint_data_.qd_actual_1, &buf[offset], sizeof(joint_data_.qd_actual_1));	
	offset += sizeof(joint_data_.qd_actual_1);
	memcpy(&joint_data_.I_actual_1, &buf[offset], sizeof(joint_data_.I_actual_1));	
	offset += sizeof(joint_data_.I_actual_1);	
	memcpy(&joint_data_.V_actual_1, &buf[offset], sizeof(joint_data_.V_actual_1));	
	offset += sizeof(joint_data_.V_actual_1);
	memcpy(&joint_data_.T_motor_1, &buf[offset], sizeof(joint_data_.T_motor_1));	
	offset += sizeof(joint_data_.T_motor_1);	
	memcpy(&joint_data_.T_micro_1, &buf[offset], sizeof(joint_data_.T_micro_1));	
	offset += sizeof(joint_data_.T_micro_1);
	memcpy(&joint_data_.jointMode_1, &buf[offset], sizeof(joint_data_.jointMode_1));	
	offset += sizeof(joint_data_.jointMode_1);
    memcpy(&joint_data_.q_actual_2, &buf[offset], sizeof(joint_data_.q_actual_2));
    offset += sizeof(joint_data_.q_actual_2);
    memcpy(&joint_data_.q_target_2, &buf[offset], sizeof(joint_data_.q_target_2));	
	offset += sizeof(joint_data_.q_target_2);
	memcpy(&joint_data_.qd_actual_2, &buf[offset], sizeof(joint_data_.qd_actual_2));	
	offset += sizeof(joint_data_.qd_actual_2);
	memcpy(&joint_data_.I_actual_2, &buf[offset], sizeof(joint_data_.I_actual_2));	
	offset += sizeof(joint_data_.I_actual_2);	
	memcpy(&joint_data_.V_actual_2, &buf[offset], sizeof(joint_data_.V_actual_2));	
	offset += sizeof(joint_data_.V_actual_2);
	memcpy(&joint_data_.T_motor_2, &buf[offset], sizeof(joint_data_.T_motor_2));	
	offset += sizeof(joint_data_.T_motor_2);	
	memcpy(&joint_data_.T_micro_2, &buf[offset], sizeof(joint_data_.T_micro_2));	
	offset += sizeof(joint_data_.T_micro_2);
	memcpy(&joint_data_.jointMode_2, &buf[offset], sizeof(joint_data_.jointMode_2));	
	offset += sizeof(joint_data_.jointMode_2);	
	memcpy(&joint_data_.q_actual_3, &buf[offset], sizeof(joint_data_.q_actual_3));
    offset += sizeof(joint_data_.q_actual_3);
    memcpy(&joint_data_.q_target_3, &buf[offset], sizeof(joint_data_.q_target_3));	
	offset += sizeof(joint_data_.q_target_3);
	memcpy(&joint_data_.qd_actual_3, &buf[offset], sizeof(joint_data_.qd_actual_3));	
	offset += sizeof(joint_data_.qd_actual_3);
	memcpy(&joint_data_.I_actual_3, &buf[offset], sizeof(joint_data_.I_actual_3));	
	offset += sizeof(joint_data_.I_actual_3);	
	memcpy(&joint_data_.V_actual_3, &buf[offset], sizeof(joint_data_.V_actual_3));	
	offset += sizeof(joint_data_.V_actual_3);
	memcpy(&joint_data_.T_motor_3, &buf[offset], sizeof(joint_data_.T_motor_3));	
	offset += sizeof(joint_data_.T_motor_3);	
	memcpy(&joint_data_.T_micro_3, &buf[offset], sizeof(joint_data_.T_micro_3));	
	offset += sizeof(joint_data_.T_micro_3);
	memcpy(&joint_data_.jointMode_3, &buf[offset], sizeof(joint_data_.jointMode_3));	
	offset += sizeof(joint_data_.jointMode_3);	
    memcpy(&joint_data_.q_actual_4, &buf[offset], sizeof(joint_data_.q_actual_4));
    offset += sizeof(joint_data_.q_actual_4);
    memcpy(&joint_data_.q_target_4, &buf[offset], sizeof(joint_data_.q_target_4));	
	offset += sizeof(joint_data_.q_target_4);
	memcpy(&joint_data_.qd_actual_4, &buf[offset], sizeof(joint_data_.qd_actual_4));	
	offset += sizeof(joint_data_.qd_actual_4);
	memcpy(&joint_data_.I_actual_4, &buf[offset], sizeof(joint_data_.I_actual_4));	
	offset += sizeof(joint_data_.I_actual_4);	
	memcpy(&joint_data_.V_actual_4, &buf[offset], sizeof(joint_data_.V_actual_4));	
	offset += sizeof(joint_data_.V_actual_4);
	memcpy(&joint_data_.T_motor_4, &buf[offset], sizeof(joint_data_.T_motor_4));	
	offset += sizeof(joint_data_.T_motor_4);	
	memcpy(&joint_data_.T_micro_4, &buf[offset], sizeof(joint_data_.T_micro_4));	
	offset += sizeof(joint_data_.T_micro_4);
	memcpy(&joint_data_.jointMode_4, &buf[offset], sizeof(joint_data_.jointMode_4));	
	offset += sizeof(joint_data_.jointMode_4);	
    memcpy(&joint_data_.q_actual_5, &buf[offset], sizeof(joint_data_.q_actual_5));
    offset += sizeof(joint_data_.q_actual_5);
    memcpy(&joint_data_.q_target_5, &buf[offset], sizeof(joint_data_.q_target_5));	
	offset += sizeof(joint_data_.q_target_5);
	memcpy(&joint_data_.qd_actual_5, &buf[offset], sizeof(joint_data_.qd_actual_5));	
	offset += sizeof(joint_data_.qd_actual_5);
	memcpy(&joint_data_.I_actual_5, &buf[offset], sizeof(joint_data_.I_actual_5));	
	offset += sizeof(joint_data_.I_actual_5);	
	memcpy(&joint_data_.V_actual_5, &buf[offset], sizeof(joint_data_.V_actual_5));	
	offset += sizeof(joint_data_.V_actual_5);
	memcpy(&joint_data_.T_motor_5, &buf[offset], sizeof(joint_data_.T_motor_5));	
	offset += sizeof(joint_data_.T_motor_5);	
	memcpy(&joint_data_.T_micro_5, &buf[offset], sizeof(joint_data_.T_micro_5));	
	offset += sizeof(joint_data_.T_micro_5);
	memcpy(&joint_data_.jointMode_5, &buf[offset], sizeof(joint_data_.jointMode_5));	
	offset += sizeof(joint_data_.jointMode_5);		
    memcpy(&joint_data_.q_actual_6, &buf[offset], sizeof(joint_data_.q_actual_6));
    offset += sizeof(joint_data_.q_actual_6);
    memcpy(&joint_data_.q_target_6, &buf[offset], sizeof(joint_data_.q_target_6));	
	offset += sizeof(joint_data_.q_target_6);
	memcpy(&joint_data_.qd_actual_6, &buf[offset], sizeof(joint_data_.qd_actual_6));	
	offset += sizeof(joint_data_.qd_actual_6);
	memcpy(&joint_data_.I_actual_6, &buf[offset], sizeof(joint_data_.I_actual_6));	
	offset += sizeof(joint_data_.I_actual_6);	
	memcpy(&joint_data_.V_actual_6, &buf[offset], sizeof(joint_data_.V_actual_6));	
	offset += sizeof(joint_data_.V_actual_6);
	memcpy(&joint_data_.T_motor_6, &buf[offset], sizeof(joint_data_.T_motor_6));	
	offset += sizeof(joint_data_.T_motor_6);	
	memcpy(&joint_data_.T_micro_6, &buf[offset], sizeof(joint_data_.T_micro_6));	
	offset += sizeof(joint_data_.T_micro_6);
	memcpy(&joint_data_.jointMode_6, &buf[offset], sizeof(joint_data_.jointMode_6));	
	offset += sizeof(joint_data_.jointMode_6);
	memcpy(&joint_data_.q_actual_7, &buf[offset], sizeof(joint_data_.q_actual_7));
    offset += sizeof(joint_data_.q_actual_7);
    memcpy(&joint_data_.q_target_7, &buf[offset], sizeof(joint_data_.q_target_7));	
	offset += sizeof(joint_data_.q_target_7);
	memcpy(&joint_data_.qd_actual_7, &buf[offset], sizeof(joint_data_.qd_actual_7));	
	offset += sizeof(joint_data_.qd_actual_7);
	memcpy(&joint_data_.I_actual_7, &buf[offset], sizeof(joint_data_.I_actual_7));	
	offset += sizeof(joint_data_.I_actual_7);	
	memcpy(&joint_data_.V_actual_7, &buf[offset], sizeof(joint_data_.V_actual_7));	
	offset += sizeof(joint_data_.V_actual_7);
	memcpy(&joint_data_.T_motor_7, &buf[offset], sizeof(joint_data_.T_motor_7));	
	offset += sizeof(joint_data_.T_motor_7);	
	memcpy(&joint_data_.T_micro_7, &buf[offset], sizeof(joint_data_.T_micro_7));	
	offset += sizeof(joint_data_.T_micro_7);
	memcpy(&joint_data_.jointMode_7, &buf[offset], sizeof(joint_data_.jointMode_7));	
	offset += sizeof(joint_data_.jointMode_7);

    return (offset - offset_);
}

unsigned int RobotState::packCartesianInfo(uint8_t * buf, unsigned int offset)
{
    unsigned int offset_ = offset;
    memcpy(&buf[offset], &cartesian_info_.X, sizeof(cartesian_info_.X));
    offset += sizeof(cartesian_info_.X);
    memcpy(&buf[offset], &cartesian_info_.Y, sizeof(cartesian_info_.Y));
    offset += sizeof(cartesian_info_.Y);
	memcpy(&buf[offset], &cartesian_info_.Z, sizeof(cartesian_info_.Z));
    offset += sizeof(cartesian_info_.Z);
    memcpy(&buf[offset], &cartesian_info_.Rx, sizeof(cartesian_info_.Rx));
    offset += sizeof(cartesian_info_.Rx);
	memcpy(&buf[offset], &cartesian_info_.Ry, sizeof(cartesian_info_.Ry));
    offset += sizeof(cartesian_info_.Ry);
    memcpy(&buf[offset], &cartesian_info_.Rz, sizeof(cartesian_info_.Rz));
    offset += sizeof(cartesian_info_.Rz);
	memcpy(&buf[offset], &cartesian_info_.TCPOffsetX, sizeof(cartesian_info_.TCPOffsetX));
    offset += sizeof(cartesian_info_.TCPOffsetX);
    memcpy(&buf[offset], &cartesian_info_.TCPOffsetY, sizeof(cartesian_info_.TCPOffsetY));
    offset += sizeof(cartesian_info_.TCPOffsetY);
	memcpy(&buf[offset], &cartesian_info_.TCPOffsetZ, sizeof(cartesian_info_.TCPOffsetZ));
    offset += sizeof(cartesian_info_.TCPOffsetZ);
	memcpy(&buf[offset], &cartesian_info_.TCPOffsetRx, sizeof(cartesian_info_.TCPOffsetRx));
    offset += sizeof(cartesian_info_.TCPOffsetRx);
	memcpy(&buf[offset], &cartesian_info_.TCPOffsetRy, sizeof(cartesian_info_.TCPOffsetRy));
    offset += sizeof(cartesian_info_.TCPOffsetRy);
	memcpy(&buf[offset], &cartesian_info_.TCPOffsetRz, sizeof(cartesian_info_.TCPOffsetRz));
    offset += sizeof(cartesian_info_.TCPOffsetRz);
    memcpy(&buf[offset], &cartesian_info_.Rn, sizeof(cartesian_info_.Rn));
    offset += sizeof(cartesian_info_.Rn);
	
	return (offset - offset_);	
}

unsigned int RobotState::unpackCartesianInfo(uint8_t * buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    memcpy(&cartesian_info_.X, &buf[offset], sizeof(cartesian_info_.X));
	offset += sizeof(cartesian_info_.X);
	memcpy(&cartesian_info_.Y, &buf[offset], sizeof(cartesian_info_.Y));	
	offset += sizeof(cartesian_info_.Y);	
	memcpy(&cartesian_info_.Z, &buf[offset], sizeof(cartesian_info_.Z));	
	offset += sizeof(cartesian_info_.Z);	
	memcpy(&cartesian_info_.Rx, &buf[offset], sizeof(cartesian_info_.Rx));	
	offset += sizeof(cartesian_info_.Rx);	
	memcpy(&cartesian_info_.Ry, &buf[offset], sizeof(cartesian_info_.Ry));	
	offset += sizeof(cartesian_info_.Ry);	
	memcpy(&cartesian_info_.Rz, &buf[offset], sizeof(cartesian_info_.Rz));	
	offset += sizeof(cartesian_info_.Rz);	
	memcpy(&cartesian_info_.TCPOffsetX, &buf[offset], sizeof(cartesian_info_.TCPOffsetX));	
	offset += sizeof(cartesian_info_.TCPOffsetX);	
	memcpy(&cartesian_info_.TCPOffsetY, &buf[offset], sizeof(cartesian_info_.TCPOffsetY));	
	offset += sizeof(cartesian_info_.TCPOffsetY);	
	memcpy(&cartesian_info_.TCPOffsetZ, &buf[offset], sizeof(cartesian_info_.TCPOffsetZ));	
	offset += sizeof(cartesian_info_.TCPOffsetZ);	
	memcpy(&cartesian_info_.TCPOffsetRx, &buf[offset], sizeof(cartesian_info_.TCPOffsetRx));	
	offset += sizeof(cartesian_info_.TCPOffsetRx);	
	memcpy(&cartesian_info_.TCPOffsetRy, &buf[offset], sizeof(cartesian_info_.TCPOffsetRy));	
	offset += sizeof(cartesian_info_.TCPOffsetRy);	
	memcpy(&cartesian_info_.TCPOffsetRz, &buf[offset], sizeof(cartesian_info_.TCPOffsetRz));	
	offset += sizeof(cartesian_info_.TCPOffsetRz);
    memcpy(&cartesian_info_.Rn, &buf[offset], sizeof(cartesian_info_.Rn));
    offset += sizeof(cartesian_info_.Rn);

    return (offset - offset_);
}

unsigned int RobotState::packRobotState(uint8_t* buf, unsigned int offset,
                                        uint8_t package_type)
{
    int32_t length = 5;
    switch (package_type)
    {
    case packageType::ROBOT_MODE_DATA:
        val_lock_.lock();
        length += RobotState::packRobotMode(buf, offset + 5);
        val_lock_.unlock();
        break;

    case packageType::MASTERBOARD_DATA:
        val_lock_.lock();
        length += RobotState::packRobotStateMasterboard(buf, offset + 5);
        val_lock_.unlock();
        break;
	
	case packageType::JOINT_DATA:
		val_lock_.lock();
		length += RobotState::packJointData(buf, offset + 5);
		val_lock_.unlock();
		break;
		
	case packageType::CARTESIAN_INFO:
		val_lock_.lock();
		length += RobotState::packCartesianInfo(buf, offset + 5);
		val_lock_.unlock();
		break;

    case packageType::CONFIGURATION_DATA:
        val_lock_.lock();
        length += RobotState::packConfigurationData(buf, offset + 5);
        val_lock_.unlock();
        break;

    case packageType::ADDITIONAL_INFO:
        val_lock_.lock();
        length += RobotState::packAdditionalInfo(buf, offset + 5);
        val_lock_.unlock();
        break;

        default:
        break;
    }
    memcpy(&buf[offset] + 4, &package_type, 1);
    memcpy(&buf[offset + sizeof(length)], &package_type,
           sizeof(package_type));
    length = htonl(length);
    memcpy(&buf[offset], &length, sizeof(length));
    return ntohl(length);
}

unsigned int RobotState::unpackRobotState(uint8_t * buf, unsigned int offset,
                                  uint32_t len)
{
    offset += 5;
    while (offset < len)
    {
        int32_t length;
        uint8_t package_type;
        memcpy(&length, &buf[offset], sizeof(length));
        length = ntohl(length);
        memcpy(&package_type, &buf[offset + sizeof(length)],
               sizeof(package_type));
        switch (package_type)
        {
        case packageType::ROBOT_MODE_DATA:
            val_lock_.lock();
            RobotState::unpackRobotMode(buf, offset + 5);
            val_lock_.unlock();
            break;

        case packageType::MASTERBOARD_DATA:
            val_lock_.lock();
            RobotState::unpackRobotStateMasterboard(buf, offset + 5);
            val_lock_.unlock();
            break;
		
		case packageType::JOINT_DATA:
            val_lock_.lock();
            RobotState::unpackJointData(buf, offset + 5);
            val_lock_.unlock();
			break;
			
		case packageType::CARTESIAN_INFO:
            val_lock_.lock();
            RobotState::unpackCartesianInfo(buf, offset + 5);
            val_lock_.unlock();
			break;
			
        default:
            break;
        }
        offset += length;
    }
    new_data_available_ = true;
    pMsg_cond_->notify_all();

    return offset;

}

unsigned int RobotState::packRobotMessageVersion(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = version_msg_.timestamp;
    source = version_msg_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_VERSION;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    memcpy(&buf[offset], &version_msg_.project_name_size,
           sizeof(version_msg_.project_name_size));
    offset += sizeof(version_msg_.project_name_size);
    if (version_msg_.project_name_size > 0 ) {
        memcpy(&buf[offset], version_msg_.project_name,
               sizeof(char) * version_msg_.project_name_size);
//    memcpy(&buf[offset], &version_msg_.project_name,
//           sizeof(char) * version_msg_.project_name_size);
        offset += version_msg_.project_name_size;
    }
    memcpy(&buf[offset], &version_msg_.major_version,
           sizeof(version_msg_.major_version));
    offset += sizeof(version_msg_.major_version);
    memcpy(&buf[offset], &version_msg_.minor_version,
           sizeof(version_msg_.minor_version));
    offset += sizeof(version_msg_.minor_version);
    int svn_revision;
    svn_revision = htonl(version_msg_.svn_revision);
    memcpy(&buf[offset], &svn_revision,
           sizeof(version_msg_.svn_revision));
    offset += sizeof(version_msg_.svn_revision);
    memcpy(&buf[offset], &version_msg_.build_date_size,
           sizeof(version_msg_.build_date_size));
    offset += sizeof(version_msg_.build_date_size);

    if (version_msg_.build_date_size > 0 ) {
        memcpy(&buf[offset], version_msg_.build_date,
               sizeof(char) * version_msg_.build_date_size);
        offset += version_msg_.build_date_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packSafetyModeMessage(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = safetyModeMessage_.timestamp;
    source = safetyModeMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_SAFETY_MODE;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = htonl(safetyModeMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp, sizeof(temp));
    offset += sizeof(safetyModeMessage_.robotMessageCode);

    temp = htonl(safetyModeMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp, sizeof(temp));
    offset += sizeof(safetyModeMessage_.robotMessageArgument);

    memcpy(&buf[offset], &safetyModeMessage_.safetyModeType,
           sizeof(safetyModeMessage_.safetyModeType));
    offset += sizeof(safetyModeMessage_.safetyModeType);


    memcpy(&buf[offset], &safetyModeMessage_.textmessage_size,
           sizeof(safetyModeMessage_.textmessage_size));
    offset += sizeof(safetyModeMessage_.textmessage_size);

    if (safetyModeMessage_.textmessage_size) {
        memcpy(&buf[offset], safetyModeMessage_.textMessage,
               sizeof(char) * safetyModeMessage_.textmessage_size);
        offset += safetyModeMessage_.textmessage_size;
    }




    return (offset - offset_);
}

unsigned int RobotState::packRobotcommMessage(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = robotcommMessage_.timestamp;
    source = robotcommMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_ERROR_CODE;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = htonl(robotcommMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.robotMessageCode));
    offset += sizeof(robotcommMessage_.robotMessageCode);

    temp = htonl(robotcommMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.robotMessageArgument));
    offset += sizeof(robotcommMessage_.robotMessageArgument);

    temp = htonl(robotcommMessage_.warningLevel);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.warningLevel));
    offset += sizeof(robotcommMessage_.warningLevel);


    memcpy(&buf[offset], &robotcommMessage_.textmessage_size,
           sizeof(robotcommMessage_.textmessage_size));
    offset += sizeof(robotcommMessage_.textmessage_size);
    if(robotcommMessage_.textmessage_size) {
        memcpy(&buf[offset], robotcommMessage_.textMessage,
               sizeof(char) * robotcommMessage_.textmessage_size);
        offset += robotcommMessage_.textmessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packKeyMessage(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = keyMessage_.timestamp;
    source = keyMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_KEY;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = htonl(keyMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp,
           sizeof(keyMessage_.robotMessageCode));
    offset += sizeof(keyMessage_.robotMessageCode);

    temp = htonl(keyMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp,
           sizeof(keyMessage_.robotMessageArgument));
    offset += sizeof(keyMessage_.robotMessageArgument);


    memcpy(&buf[offset], &keyMessage_.titleSize,
           sizeof(keyMessage_.titleSize));
    offset += sizeof(keyMessage_.titleSize);
    if (keyMessage_.titleSize) {
        memcpy(&buf[offset], keyMessage_.messageTitle,
               sizeof(char) * keyMessage_.titleSize);
        offset += keyMessage_.titleSize;
    }


    memcpy(&buf[offset], &keyMessage_.textMessage_size,
           sizeof(keyMessage_.textMessage_size));
    offset += sizeof(keyMessage_.textMessage_size);
    if(keyMessage_.textMessage_size) {
        memcpy(&buf[offset], keyMessage_.textMessage,
               sizeof(char) * keyMessage_.textMessage_size);
        offset += keyMessage_.textMessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packLabelMessage(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = labelMessage_.timestamp;
    source = labelMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = htonl(labelMessage_.id);
    memcpy(&buf[offset], &temp,
           sizeof(labelMessage_.id));
    offset += sizeof(labelMessage_.id);

    memcpy(&buf[offset], &labelMessage_.textMessage_size,
           sizeof(labelMessage_.textMessage_size));
    offset += sizeof(labelMessage_.textMessage_size);

    if(labelMessage_.textMessage_size) {
        memcpy(&buf[offset], labelMessage_.textMessage,
               sizeof(char) * labelMessage_.textMessage_size);
        offset += labelMessage_.textMessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packGlobalVariablesSetupMessage(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t robot_message_type;
    timestamp = globalVariablesSetupMessage_.timestamp;
    robot_message_type = programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    unsigned short temp;
    temp = htons(globalVariablesSetupMessage_.startIndex);
    memcpy(&buf[offset], &temp,
           sizeof(globalVariablesSetupMessage_.startIndex));
    offset += sizeof(globalVariablesSetupMessage_.startIndex);


    memcpy(&buf[offset], &globalVariablesSetupMessage_.variableNames_size,
           sizeof(globalVariablesSetupMessage_.variableNames_size));
    offset += sizeof(globalVariablesSetupMessage_.variableNames_size);
    if (globalVariablesSetupMessage_.variableNames_size) {
        memcpy(&buf[offset], globalVariablesSetupMessage_.variableNames,
               sizeof(char) * globalVariablesSetupMessage_.variableNames_size);
        offset += globalVariablesSetupMessage_.variableNames_size;
    }

    return (offset - offset_);
}

unsigned int RobotState::unpackRobotMessageVersion(uint8_t * buf, unsigned int offset,
        uint32_t len)
{
//    memcpy(&version_msg_.project_name_size, &buf[offset],
//           sizeof(version_msg_.project_name_size));
//    offset += sizeof(version_msg_.project_name_size);
//    memcpy(&version_msg_.project_name, &buf[offset],
//           sizeof(char) * version_msg_.project_name_size);
//    offset += version_msg_.project_name_size;
//    version_msg_.project_name[version_msg_.project_name_size] = '\0';
//    memcpy(&version_msg_.major_version, &buf[offset],
//           sizeof(version_msg_.major_version));
//    offset += sizeof(version_msg_.major_version);
//    memcpy(&version_msg_.minor_version, &buf[offset],
//           sizeof(version_msg_.minor_version));
//    offset += sizeof(version_msg_.minor_version);
//    int svn_revision;
//    memcpy(&svn_revision, &buf[offset],
//           sizeof(version_msg_.svn_revision));
//    offset += sizeof(version_msg_.svn_revision);
//    version_msg_.svn_revision = ntohl(svn_revision);
//    memcpy(&version_msg_.build_date, &buf[offset], sizeof(char) * len - offset);
//    version_msg_.build_date[len - offset] = '\0';
//    if (version_msg_.major_version < 2)
//    {
//        robot_mode_running_ = robotStateTypeV18::ROBOT_RUNNING_MODE;
//    }

    return 0;
}


unsigned int RobotState::packRobotMode(uint8_t * buf, unsigned int offset)
{
    unsigned int offset_ = offset;
    memcpy(&buf[offset], &robot_mode_.timestamp, sizeof(robot_mode_.timestamp));
    offset += sizeof(robot_mode_.timestamp);
    memcpy(&buf[offset], &robot_mode_.isRobotConnected, sizeof(robot_mode_.isRobotConnected));
    offset += sizeof(robot_mode_.isRobotConnected);
    memcpy(&buf[offset], &robot_mode_.isRealRobotEnabled, sizeof(robot_mode_.isRealRobotEnabled));
    offset += sizeof(robot_mode_.isRealRobotEnabled);
    memcpy(&buf[offset], &robot_mode_.isPowerOnRobot, sizeof(robot_mode_.isPowerOnRobot));
    offset += sizeof(robot_mode_.isPowerOnRobot);
    memcpy(&buf[offset], &robot_mode_.isEmergencyStopped, sizeof(robot_mode_.isEmergencyStopped));
    offset += sizeof(robot_mode_.isEmergencyStopped);
    memcpy(&buf[offset], &robot_mode_.isProtectiveStopped, sizeof(robot_mode_.isProtectiveStopped));
    offset += sizeof(robot_mode_.isProtectiveStopped);
    memcpy(&buf[offset], &robot_mode_.isProgramRunning, sizeof(robot_mode_.isProgramRunning));
    offset += sizeof(robot_mode_.isProgramRunning);
    memcpy(&buf[offset], &robot_mode_.isProgramPaused, sizeof(robot_mode_.isProgramPaused));
    offset += sizeof(robot_mode_.isProgramPaused);
    memcpy(&buf[offset], &robot_mode_.robotMode, sizeof(robot_mode_.robotMode));
    offset += sizeof(robot_mode_.robotMode);
    uint64_t temp;
    if (RobotState::getVersion() > 2.)
    {
        memcpy(&buf[offset], &robot_mode_.controlMode,
               sizeof(robot_mode_.controlMode));
        offset += sizeof(robot_mode_.controlMode);
        //temp_double = RobotState::htond(robot_mode_.targetSpeedFraction);
        memcpy(&buf[offset], &robot_mode_.targetSpeedFraction, sizeof(robot_mode_.targetSpeedFraction));
        offset += sizeof(robot_mode_.targetSpeedFraction);
    }
    //temp_double = RobotState::htond(robot_mode_.speedScaling);
    memcpy(&buf[offset], &robot_mode_.speedScaling, sizeof(robot_mode_.speedScaling));
    offset += sizeof(robot_mode_.speedScaling);
    //robot_mode_.speedScaling = RobotState::htond(robot_mode_.targetSpeedFractionLimit);
    memcpy(&buf[offset], &robot_mode_.targetSpeedFractionLimit, sizeof(robot_mode_.targetSpeedFractionLimit));
    offset += sizeof(robot_mode_.targetSpeedFractionLimit);
    return (offset - offset_);
}

unsigned int RobotState::unpackRobotMode(uint8_t * buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    memcpy(&robot_mode_.timestamp, &buf[offset], sizeof(robot_mode_.timestamp));
    offset += sizeof(robot_mode_.timestamp);
    uint8_t tmp;
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isRobotConnected = true;
    else
        robot_mode_.isRobotConnected = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isRealRobotEnabled = true;
    else
        robot_mode_.isRealRobotEnabled = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    //printf("PowerOnRobot: %d\n", tmp);
    if (tmp > 0)
        robot_mode_.isPowerOnRobot = true;
    else
        robot_mode_.isPowerOnRobot = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isEmergencyStopped = true;
    else
        robot_mode_.isEmergencyStopped = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isProtectiveStopped = true;
    else
        robot_mode_.isProtectiveStopped = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isProgramRunning = true;
    else
        robot_mode_.isProgramRunning = false;
    offset += sizeof(tmp);
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    if (tmp > 0)
        robot_mode_.isProgramPaused = true;
    else
        robot_mode_.isProgramPaused = false;
    offset += sizeof(tmp);
    memcpy(&robot_mode_.robotMode, &buf[offset], sizeof(robot_mode_.robotMode));
    offset += sizeof(robot_mode_.robotMode);
    uint64_t temp;
    if (RobotState::getVersion() > 2.)
    {
        memcpy(&robot_mode_.controlMode, &buf[offset],
               sizeof(robot_mode_.controlMode));
        offset += sizeof(robot_mode_.controlMode);
        memcpy(&robot_mode_.targetSpeedFraction, &buf[offset], sizeof(robot_mode_.targetSpeedFraction));
        offset += sizeof(robot_mode_.targetSpeedFraction);
        //robot_mode_.targetSpeedFraction = RobotState::ntohd(temp);
    }
    memcpy(&robot_mode_.speedScaling, &buf[offset], sizeof(robot_mode_.speedScaling));
    offset += sizeof(robot_mode_.speedScaling);
    //robot_mode_.speedScaling = RobotState::ntohd(temp);
    memcpy(&robot_mode_.targetSpeedFractionLimit, &buf[offset], sizeof(robot_mode_.targetSpeedFractionLimit));
    offset += sizeof(robot_mode_.targetSpeedFractionLimit);
    //robot_mode_.targetSpeedFractionLimit = RobotState::ntohd(temp);
    return (offset - offset_);
}

unsigned int RobotState::packRobotStateMasterboard(uint8_t * buf,
        unsigned int offset)
{
    unsigned int offset_ = offset;
    if (RobotState::getVersion() < 3.0)
    {
        int16_t digital_input_bits, digital_output_bits;
        digital_input_bits = htons(mb_data_.digitalInputBits);
        digital_output_bits = htons(mb_data_.digitalOutputBits);
        memcpy(&buf[offset], &digital_input_bits, sizeof(digital_input_bits));
        offset += sizeof(digital_input_bits);
        memcpy(&buf[offset], &digital_output_bits, sizeof(digital_output_bits));
        offset += sizeof(digital_output_bits);
    }
    else
    {
        int32_t digital_input_bits, digital_output_bits;
        digital_input_bits = htonl(mb_data_.digitalInputBits);
        memcpy(&buf[offset], &digital_input_bits,
               sizeof(mb_data_.digitalInputBits));
        offset += sizeof(mb_data_.digitalInputBits);
        digital_output_bits = htonl(mb_data_.digitalOutputBits);
        memcpy(&buf[offset], &digital_output_bits,
               sizeof(mb_data_.digitalOutputBits));
        offset += sizeof(mb_data_.digitalOutputBits);
    }

    memcpy(&buf[offset], &mb_data_.analogInputRange0,
           sizeof(mb_data_.analogInputRange0));
    offset += sizeof(mb_data_.analogInputRange0);
    memcpy(&buf[offset], &mb_data_.analogInputRange1,
           sizeof(mb_data_.analogInputRange1));
    offset += sizeof(mb_data_.analogInputRange1);
    uint64_t temp64;
    //temp64 = RobotState::htond(mb_data_.analogInput0);
    memcpy(&buf[offset], &mb_data_.analogInput0, sizeof(mb_data_.analogInput0));
    offset += sizeof(mb_data_.analogInput0);
    //temp64 = RobotState::htond(mb_data_.analogInput1);
    memcpy(&buf[offset], &mb_data_.analogInput1, sizeof(mb_data_.analogInput1));
    offset += sizeof(mb_data_.analogInput1);
    memcpy(&buf[offset], &mb_data_.analogOutputDomain0,
           sizeof(mb_data_.analogOutputDomain0));
    offset += sizeof(mb_data_.analogOutputDomain0);
    memcpy(&buf[offset], &mb_data_.analogOutputDomain1,
           sizeof(mb_data_.analogOutputDomain1));
    offset += sizeof(mb_data_.analogOutputDomain1);
    //temp64 = RobotState::htond(mb_data_.analogOutput0);
    memcpy(&buf[offset], &mb_data_.analogOutput0, sizeof(mb_data_.analogOutput0));
    offset += sizeof(mb_data_.analogOutput0);
    //temp64 = RobotState::htond(mb_data_.analogOutput1);
    memcpy(&buf[offset], &mb_data_.analogOutput1, sizeof(mb_data_.analogOutput1));
    offset += sizeof(mb_data_.analogOutput1);

    uint32_t temp32;
    temp32 = htonl(mb_data_.masterBoardTemperature);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.robotVoltage48V);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.robotCurrent);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.masterIOCurrent);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);

    memcpy(&buf[offset], &mb_data_.safetyMode, sizeof(mb_data_.safetyMode));
    offset += sizeof(mb_data_.safetyMode);
    memcpy(&buf[offset], &mb_data_.masterOnOffState,
           sizeof(mb_data_.masterOnOffState));
    offset += sizeof(mb_data_.masterOnOffState);

    memcpy(&buf[offset], &mb_data_.euromap67InterfaceInstalled,
           sizeof(mb_data_.euromap67InterfaceInstalled));
    offset += sizeof(mb_data_.euromap67InterfaceInstalled);
    if (mb_data_.euromap67InterfaceInstalled != 0)
    {
        temp32 = htonl(mb_data_.euromapInputBits);
        memcpy(&buf[offset], &temp32,
               sizeof(temp32));
        offset += sizeof(temp32);
        temp32 = htonl(mb_data_.euromapOutputBits);
        memcpy(&buf[offset], &temp32,
               sizeof(temp32));
        offset += sizeof(temp32);
        if (RobotState::getVersion() < 3.0)
        {
            int16_t euromap_voltage, euromap_current;
            euromap_voltage = htons(mb_data_.euromapVoltage);
            euromap_current = htons(mb_data_.euromapCurrent);
            memcpy(&buf[offset], &euromap_voltage, sizeof(euromap_voltage));
            offset += sizeof(euromap_voltage);
            memcpy(&buf[offset], &euromap_current, sizeof(euromap_current));
            offset += sizeof(euromap_current);
        }
        else
        {
            int32_t euromap_voltage;
            euromap_voltage = htonl(mb_data_.euromapVoltage);
            memcpy(&buf[offset], &euromap_voltage,
                   sizeof(mb_data_.euromapVoltage));
            offset += sizeof(mb_data_.euromapVoltage);
            //euromap_current = htonl(mb_data_.euromapCurrent);
            memcpy(&buf[offset], &mb_data_.euromapCurrent,
                   sizeof(mb_data_.euromapCurrent));
            offset += sizeof(mb_data_.euromapCurrent);
        }

    }

    uint32_t temp;
    temp = htonl(mb_data_.uburso);
    memcpy(&buf[offset], &temp, sizeof(mb_data_.uburso));
    offset += sizeof(mb_data_.uburso);

    memcpy(&buf[offset], &mb_data_.operationalModeSelectorInput, sizeof(mb_data_.operationalModeSelectorInput));
    offset += sizeof(mb_data_.operationalModeSelectorInput);
    memcpy(&buf[offset], &mb_data_.threePositionEnablingDeviceInput, sizeof(mb_data_.threePositionEnablingDeviceInput));
    offset += sizeof(mb_data_.threePositionEnablingDeviceInput);


    return (offset - offset_);
}

unsigned int RobotState::unpackRobotStateMasterboard(uint8_t * buf,
        unsigned int offset)
{
    unsigned int offset_ = offset;
    if (RobotState::getVersion() < 3.0)
    {
        int16_t digital_input_bits, digital_output_bits;
        memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
        offset += sizeof(digital_input_bits);
        memcpy(&digital_output_bits, &buf[offset], sizeof(digital_output_bits));
        offset += sizeof(digital_output_bits);
        mb_data_.digitalInputBits = ntohs(digital_input_bits);
        mb_data_.digitalOutputBits = ntohs(digital_output_bits);
    }
    else
    {
        memcpy(&mb_data_.digitalInputBits, &buf[offset],
               sizeof(mb_data_.digitalInputBits));
        offset += sizeof(mb_data_.digitalInputBits);
        mb_data_.digitalInputBits = ntohl(mb_data_.digitalInputBits);
        memcpy(&mb_data_.digitalOutputBits, &buf[offset],
               sizeof(mb_data_.digitalOutputBits));
        offset += sizeof(mb_data_.digitalOutputBits);
        mb_data_.digitalOutputBits = ntohl(mb_data_.digitalOutputBits);
    }

    memcpy(&mb_data_.analogInputRange0, &buf[offset],
           sizeof(mb_data_.analogInputRange0));
    offset += sizeof(mb_data_.analogInputRange0);
    memcpy(&mb_data_.analogInputRange1, &buf[offset],
           sizeof(mb_data_.analogInputRange1));
    offset += sizeof(mb_data_.analogInputRange1);
    uint64_t temp;
    memcpy(&temp, &buf[offset], sizeof(temp));
    offset += sizeof(temp);
    mb_data_.analogInput0 = RobotState::ntohd(temp);
    memcpy(&temp, &buf[offset], sizeof(temp));
    offset += sizeof(temp);
    mb_data_.analogInput1 = RobotState::ntohd(temp);
    memcpy(&mb_data_.analogOutputDomain0, &buf[offset],
           sizeof(mb_data_.analogOutputDomain0));
    offset += sizeof(mb_data_.analogOutputDomain0);
    memcpy(&mb_data_.analogOutputDomain1, &buf[offset],
           sizeof(mb_data_.analogOutputDomain1));
    offset += sizeof(mb_data_.analogOutputDomain1);
    memcpy(&temp, &buf[offset], sizeof(temp));
    offset += sizeof(temp);
    mb_data_.analogOutput0 = RobotState::ntohd(temp);
    memcpy(&temp, &buf[offset], sizeof(temp));
    offset += sizeof(temp);
    mb_data_.analogOutput1 = RobotState::ntohd(temp);

    memcpy(&mb_data_.masterBoardTemperature, &buf[offset],
           sizeof(mb_data_.masterBoardTemperature));
    offset += sizeof(mb_data_.masterBoardTemperature);
    mb_data_.masterBoardTemperature = ntohl(mb_data_.masterBoardTemperature);
    memcpy(&mb_data_.robotVoltage48V, &buf[offset],
           sizeof(mb_data_.robotVoltage48V));
    offset += sizeof(mb_data_.robotVoltage48V);
    mb_data_.robotVoltage48V = ntohl(mb_data_.robotVoltage48V);
    memcpy(&mb_data_.robotCurrent, &buf[offset], sizeof(mb_data_.robotCurrent));
    offset += sizeof(mb_data_.robotCurrent);
    mb_data_.robotCurrent = ntohl(mb_data_.robotCurrent);
    memcpy(&mb_data_.masterIOCurrent, &buf[offset],
           sizeof(mb_data_.masterIOCurrent));
    offset += sizeof(mb_data_.masterIOCurrent);
    mb_data_.masterIOCurrent = ntohl(mb_data_.masterIOCurrent);

    memcpy(&mb_data_.safetyMode, &buf[offset], sizeof(mb_data_.safetyMode));
    offset += sizeof(mb_data_.safetyMode);
    memcpy(&mb_data_.masterOnOffState, &buf[offset],
           sizeof(mb_data_.masterOnOffState));
    offset += sizeof(mb_data_.masterOnOffState);

    memcpy(&mb_data_.euromap67InterfaceInstalled, &buf[offset],
           sizeof(mb_data_.euromap67InterfaceInstalled));
    offset += sizeof(mb_data_.euromap67InterfaceInstalled);
    if (mb_data_.euromap67InterfaceInstalled != 0)
    {
        memcpy(&mb_data_.euromapInputBits, &buf[offset],
               sizeof(mb_data_.euromapInputBits));
        offset += sizeof(mb_data_.euromapInputBits);
        mb_data_.euromapInputBits = ntohl(mb_data_.euromapInputBits);
        memcpy(&mb_data_.euromapOutputBits, &buf[offset],
               sizeof(mb_data_.euromapOutputBits));
        offset += sizeof(mb_data_.euromapOutputBits);
        mb_data_.euromapOutputBits = ntohl(mb_data_.euromapOutputBits);
        if (RobotState::getVersion() < 3.0)
        {
            int16_t euromap_voltage, euromap_current;
            memcpy(&euromap_voltage, &buf[offset], sizeof(euromap_voltage));
            offset += sizeof(euromap_voltage);
            memcpy(&euromap_current, &buf[offset], sizeof(euromap_current));
            offset += sizeof(euromap_current);
            mb_data_.euromapVoltage = ntohs(euromap_voltage);
            mb_data_.euromapCurrent = ntohs(euromap_current);
        }
        else
        {
            memcpy(&mb_data_.euromapVoltage, &buf[offset],
                   sizeof(mb_data_.euromapVoltage));
            offset += sizeof(mb_data_.euromapVoltage);
            mb_data_.euromapVoltage = ntohl(mb_data_.euromapVoltage);
            memcpy(&mb_data_.euromapCurrent, &buf[offset],
                   sizeof(mb_data_.euromapCurrent));
            offset += sizeof(mb_data_.euromapCurrent);
            mb_data_.euromapCurrent = ntohl(mb_data_.euromapCurrent);
        }

    }

    memcpy(&mb_data_.uburso, &buf[offset],
           sizeof(mb_data_.uburso));
    offset += sizeof(mb_data_.uburso);
    mb_data_.uburso = ntohl(mb_data_.uburso);

    memcpy(&mb_data_.operationalModeSelectorInput, &buf[offset],
           sizeof(mb_data_.operationalModeSelectorInput));
    offset += sizeof(mb_data_.operationalModeSelectorInput);
    memcpy(&mb_data_.threePositionEnablingDeviceInput, &buf[offset],
           sizeof(mb_data_.threePositionEnablingDeviceInput));
    offset += sizeof(mb_data_.threePositionEnablingDeviceInput);

    return (offset - offset_);
}

unsigned int RobotState::packConfigurationData(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_1, sizeof(configuration_data_.jointMinLimit_1));
    offset += sizeof(configuration_data_.jointMinLimit_1);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_2, sizeof(configuration_data_.jointMinLimit_2));
    offset += sizeof(configuration_data_.jointMinLimit_2);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_3, sizeof(configuration_data_.jointMinLimit_3));
    offset += sizeof(configuration_data_.jointMinLimit_3);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_4, sizeof(configuration_data_.jointMinLimit_4));
    offset += sizeof(configuration_data_.jointMinLimit_4);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_5, sizeof(configuration_data_.jointMinLimit_5));
    offset += sizeof(configuration_data_.jointMinLimit_5);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_6, sizeof(configuration_data_.jointMinLimit_6));
    offset += sizeof(configuration_data_.jointMinLimit_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_1, sizeof(configuration_data_.jointMaxLimitt_1));
    offset += sizeof(configuration_data_.jointMaxLimitt_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_2, sizeof(configuration_data_.jointMaxLimitt_2));
    offset += sizeof(configuration_data_.jointMaxLimitt_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_3, sizeof(configuration_data_.jointMaxLimitt_3));
    offset += sizeof(configuration_data_.jointMaxLimitt_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_4, sizeof(configuration_data_.jointMaxLimitt_4));
    offset += sizeof(configuration_data_.jointMaxLimitt_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_5, sizeof(configuration_data_.jointMaxLimitt_5));
    offset += sizeof(configuration_data_.jointMaxLimitt_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_6, sizeof(configuration_data_.jointMaxLimitt_6));
    offset += sizeof(configuration_data_.jointMaxLimitt_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_1, sizeof(configuration_data_.jointMaxSpeed_1));
    offset += sizeof(configuration_data_.jointMaxSpeed_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_2, sizeof(configuration_data_.jointMaxSpeed_2));
    offset += sizeof(configuration_data_.jointMaxSpeed_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_3, sizeof(configuration_data_.jointMaxSpeed_3));
    offset += sizeof(configuration_data_.jointMaxSpeed_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_4, sizeof(configuration_data_.jointMaxSpeed_4));
    offset += sizeof(configuration_data_.jointMaxSpeed_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_5, sizeof(configuration_data_.jointMaxSpeed_5));
    offset += sizeof(configuration_data_.jointMaxSpeed_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_6, sizeof(configuration_data_.jointMaxSpeed_6));
    offset += sizeof(configuration_data_.jointMaxSpeed_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_1, sizeof(configuration_data_.jointMaxAcceleration_1));
    offset += sizeof(configuration_data_.jointMaxAcceleration_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_2, sizeof(configuration_data_.jointMaxAcceleration_2));
    offset += sizeof(configuration_data_.jointMaxAcceleration_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_3, sizeof(configuration_data_.jointMaxAcceleration_3));
    offset += sizeof(configuration_data_.jointMaxAcceleration_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_4, sizeof(configuration_data_.jointMaxAcceleration_4));
    offset += sizeof(configuration_data_.jointMaxAcceleration_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_5, sizeof(configuration_data_.jointMaxAcceleration_5));
    offset += sizeof(configuration_data_.jointMaxAcceleration_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_6, sizeof(configuration_data_.jointMaxAcceleration_6));
    offset += sizeof(configuration_data_.jointMaxAcceleration_6);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_1, sizeof(configuration_data_.vJointDefault_1));
    offset += sizeof(configuration_data_.vJointDefault_1);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_2, sizeof(configuration_data_.vJointDefault_2));
    offset += sizeof(configuration_data_.vJointDefault_2);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_3, sizeof(configuration_data_.vJointDefault_3));
    offset += sizeof(configuration_data_.vJointDefault_3);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_4, sizeof(configuration_data_.vJointDefault_4));
    offset += sizeof(configuration_data_.vJointDefault_4);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_5, sizeof(configuration_data_.vJointDefault_5));
    offset += sizeof(configuration_data_.vJointDefault_5);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_6, sizeof(configuration_data_.vJointDefault_6));
    offset += sizeof(configuration_data_.vJointDefault_6);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_1, sizeof(configuration_data_.aJointDefault_1));
    offset += sizeof(configuration_data_.aJointDefault_1);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_2, sizeof(configuration_data_.aJointDefault_2));
    offset += sizeof(configuration_data_.aJointDefault_2);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_3, sizeof(configuration_data_.aJointDefault_3));
    offset += sizeof(configuration_data_.aJointDefault_3);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_4, sizeof(configuration_data_.aJointDefault_4));
    offset += sizeof(configuration_data_.aJointDefault_4);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_5, sizeof(configuration_data_.aJointDefault_5));
    offset += sizeof(configuration_data_.aJointDefault_5);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_6, sizeof(configuration_data_.aJointDefault_6));
    offset += sizeof(configuration_data_.aJointDefault_6);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_1, sizeof(configuration_data_.vToolDefault_1));
    offset += sizeof(configuration_data_.vToolDefault_1);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_2, sizeof(configuration_data_.vToolDefault_2));
    offset += sizeof(configuration_data_.vToolDefault_2);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_3, sizeof(configuration_data_.vToolDefault_3));
    offset += sizeof(configuration_data_.vToolDefault_3);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_4, sizeof(configuration_data_.vToolDefault_4));
    offset += sizeof(configuration_data_.vToolDefault_4);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_5, sizeof(configuration_data_.vToolDefault_5));
    offset += sizeof(configuration_data_.vToolDefault_5);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_6, sizeof(configuration_data_.vToolDefault_6));
    offset += sizeof(configuration_data_.vToolDefault_6);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_1, sizeof(configuration_data_.aToolDefault_1));
    offset += sizeof(configuration_data_.aToolDefault_1);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_2, sizeof(configuration_data_.aToolDefault_2));
    offset += sizeof(configuration_data_.aToolDefault_2);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_3, sizeof(configuration_data_.aToolDefault_3));
    offset += sizeof(configuration_data_.aToolDefault_3);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_4, sizeof(configuration_data_.aToolDefault_4));
    offset += sizeof(configuration_data_.aToolDefault_4);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_5, sizeof(configuration_data_.aToolDefault_5));
    offset += sizeof(configuration_data_.aToolDefault_5);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_6, sizeof(configuration_data_.aToolDefault_6));
    offset += sizeof(configuration_data_.aToolDefault_6);
    memcpy(&buf[offset], &configuration_data_.eqRadius_1, sizeof(configuration_data_.eqRadius_1));
    offset += sizeof(configuration_data_.eqRadius_1);
    memcpy(&buf[offset], &configuration_data_.eqRadius_2, sizeof(configuration_data_.eqRadius_2));
    offset += sizeof(configuration_data_.eqRadius_2);
    memcpy(&buf[offset], &configuration_data_.eqRadius_3, sizeof(configuration_data_.eqRadius_3));
    offset += sizeof(configuration_data_.eqRadius_3);
    memcpy(&buf[offset], &configuration_data_.eqRadius_4, sizeof(configuration_data_.eqRadius_4));
    offset += sizeof(configuration_data_.eqRadius_4);
    memcpy(&buf[offset], &configuration_data_.eqRadius_5, sizeof(configuration_data_.eqRadius_5));
    offset += sizeof(configuration_data_.eqRadius_5);
    memcpy(&buf[offset], &configuration_data_.eqRadius_6, sizeof(configuration_data_.eqRadius_6));
    offset += sizeof(configuration_data_.eqRadius_6);
    memcpy(&buf[offset], &configuration_data_.DHa_1, sizeof(configuration_data_.DHa_1));
    offset += sizeof(configuration_data_.DHa_1);
    memcpy(&buf[offset], &configuration_data_.DHa_2, sizeof(configuration_data_.DHa_2));
    offset += sizeof(configuration_data_.DHa_2);
    memcpy(&buf[offset], &configuration_data_.DHa_3, sizeof(configuration_data_.DHa_3));
    offset += sizeof(configuration_data_.DHa_3);
    memcpy(&buf[offset], &configuration_data_.DHa_4, sizeof(configuration_data_.DHa_4));
    offset += sizeof(configuration_data_.DHa_4);
    memcpy(&buf[offset], &configuration_data_.DHa_5, sizeof(configuration_data_.DHa_5));
    offset += sizeof(configuration_data_.DHa_5);
    memcpy(&buf[offset], &configuration_data_.DHa_6, sizeof(configuration_data_.DHa_6));
    offset += sizeof(configuration_data_.DHa_6);
    memcpy(&buf[offset], &configuration_data_.DHd_1, sizeof(configuration_data_.DHd_1));
    offset += sizeof(configuration_data_.DHd_1);
    memcpy(&buf[offset], &configuration_data_.DHd_2, sizeof(configuration_data_.DHd_2));
    offset += sizeof(configuration_data_.DHd_2);
    memcpy(&buf[offset], &configuration_data_.DHd_3, sizeof(configuration_data_.DHd_3));
    offset += sizeof(configuration_data_.DHd_3);
    memcpy(&buf[offset], &configuration_data_.DHd_4, sizeof(configuration_data_.DHd_4));
    offset += sizeof(configuration_data_.DHd_4);
    memcpy(&buf[offset], &configuration_data_.DHd_5, sizeof(configuration_data_.DHd_5));
    offset += sizeof(configuration_data_.DHd_5);
    memcpy(&buf[offset], &configuration_data_.DHd_6, sizeof(configuration_data_.DHd_6));
    offset += sizeof(configuration_data_.DHd_6);
    memcpy(&buf[offset], &configuration_data_.DHalpha_1, sizeof(configuration_data_.DHalpha_1));
    offset += sizeof(configuration_data_.DHalpha_1);
    memcpy(&buf[offset], &configuration_data_.DHalpha_2, sizeof(configuration_data_.DHalpha_2));
    offset += sizeof(configuration_data_.DHalpha_2);
    memcpy(&buf[offset], &configuration_data_.DHalpha_3, sizeof(configuration_data_.DHalpha_3));
    offset += sizeof(configuration_data_.DHalpha_3);
    memcpy(&buf[offset], &configuration_data_.DHalpha_4, sizeof(configuration_data_.DHalpha_4));
    offset += sizeof(configuration_data_.DHalpha_4);
    memcpy(&buf[offset], &configuration_data_.DHalpha_5, sizeof(configuration_data_.DHalpha_5));
    offset += sizeof(configuration_data_.DHalpha_5);
    memcpy(&buf[offset], &configuration_data_.DHalpha_6, sizeof(configuration_data_.DHalpha_6));
    offset += sizeof(configuration_data_.DHalpha_6);
    memcpy(&buf[offset], &configuration_data_.DHtheta_1, sizeof(configuration_data_.DHtheta_1));
    offset += sizeof(configuration_data_.DHtheta_1);
    memcpy(&buf[offset], &configuration_data_.DHtheta_2, sizeof(configuration_data_.DHtheta_2));
    offset += sizeof(configuration_data_.DHtheta_2);
    memcpy(&buf[offset], &configuration_data_.DHtheta_3, sizeof(configuration_data_.DHtheta_3));
    offset += sizeof(configuration_data_.DHtheta_3);
    memcpy(&buf[offset], &configuration_data_.DHtheta_4, sizeof(configuration_data_.DHtheta_4));
    offset += sizeof(configuration_data_.DHtheta_4);
    memcpy(&buf[offset], &configuration_data_.DHtheta_5, sizeof(configuration_data_.DHtheta_5));
    offset += sizeof(configuration_data_.DHtheta_5);
    memcpy(&buf[offset], &configuration_data_.DHtheta_6, sizeof(configuration_data_.DHtheta_6));
    offset += sizeof(configuration_data_.DHtheta_6);

    int temp32;
    temp32 = htonl(configuration_data_.masterboardVersion);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.masterboardVersion);

    temp32 = htonl(configuration_data_.controllerBoxType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.controllerBoxType);

    temp32 = htonl(configuration_data_.robotType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.robotType);

    temp32 = htonl(configuration_data_.robotSubType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.robotSubType);

    return (offset - offset_);
}

unsigned int RobotState::packAdditionalInfo(uint8_t* buf, unsigned int offset)
{
    unsigned int offset_ = offset;

    memcpy(&buf[offset], &additional_info_.freedriveButtonPressed, sizeof(additional_info_.freedriveButtonPressed));
    offset += sizeof(additional_info_.freedriveButtonPressed);
    memcpy(&buf[offset], &additional_info_.freedriveButtonEnabled, sizeof(additional_info_.freedriveButtonEnabled));
    offset += sizeof(additional_info_.freedriveButtonEnabled);
    memcpy(&buf[offset], &additional_info_.IOEnabledFreedrive, sizeof(additional_info_.IOEnabledFreedrive));
    offset += sizeof(additional_info_.IOEnabledFreedrive);

    return (offset - offset_);
}


double RobotState::getVersion()
{
//    double ver;
//    val_lock_.lock();
//    ver = version_msg_.major_version + 0.1 * version_msg_.minor_version
//          + .0000001 * version_msg_.svn_revision;
//    val_lock_.unlock();
//    return ver;

    return 3.5;

}

double RobotState::getTime()
{
    //TODO
    return 0;
}

void RobotState::finishedReading()
{
    new_data_available_ = false;
}

bool RobotState::getNewDataAvailable()
{
    return new_data_available_;
}

//region get/set方法
//int RobotState::getDigitalInputBits()
//{
//    return mb_data_.digitalInputBits;
//}
//int RobotState::getDigitalOutputBits()
//{
//    return mb_data_.digitalOutputBits;
//}
//double RobotState::getAnalogInput0()
//{
//    return mb_data_.analogInput0;
//}
//double RobotState::getAnalogInput1()
//{
//    return mb_data_.analogInput1;
//}
//char RobotState::getAnalogOutputDomain0()
//{
//    return mb_data_.analogOutputDomain0;
//}
//char RobotState::getAnalogOutputDomain1()
//{
//    return mb_data_.analogOutputDomain1;
//}
//double RobotState::getAnalogOutput0()
//{
//    return mb_data_.analogOutput0;
//}
//double RobotState::getAnalogOutput1()
//{
//    return mb_data_.analogOutput1;
//}
//float RobotState::getMasterBoardTemperature()
//{
//    return mb_data_.masterBoardTemperature;
//}
//float RobotState::getRobotVoltage48V()
//{
//    return mb_data_.robotVoltage48V;
//}
//float RobotState::getRobotCurrent()
//{
//    return mb_data_.robotCurrent;
//}
//float RobotState::getMasterIOCurrent()
//{
//    return mb_data_.masterIOCurrent;
//}
//unsigned char RobotState::getSafetyMode()
//{
//    return mb_data_.safetyMode;
//}
//unsigned char RobotState::getInReducedMode()
//{
//    return mb_data_.masterOnOffState;
//}
//char RobotState::getEuromap67InterfaceInstalled()
//{
//    return mb_data_.euromap67InterfaceInstalled;
//}
//int RobotState::getEuromapInputBits()
//{
//    return mb_data_.euromapInputBits;
//}
//int RobotState::getEuromapOutputBits()
//{
//    return mb_data_.euromapOutputBits;
//}
//float RobotState::getEuromapVoltage()
//{
//    return mb_data_.euromapVoltage;
//}
//float RobotState::getEuromapCurrent()
//{
//    return mb_data_.euromapCurrent;
//}
//
//bool RobotState::isRobotConnected()
//{
//    return robot_mode_.isRobotConnected;
//}
//bool RobotState::isRealRobotEnabled()
//{
//    return robot_mode_.isRealRobotEnabled;
//}
//bool RobotState::isPowerOnRobot()
//{
//    return robot_mode_.isPowerOnRobot;
//}
//bool RobotState::isEmergencyStopped()
//{
//    return robot_mode_.isEmergencyStopped;
//}
//bool RobotState::isProtectiveStopped()
//{
//    return robot_mode_.isProtectiveStopped;
//}
//bool RobotState::isProgramRunning()
//{
//    return robot_mode_.isProgramRunning;
//}
//bool RobotState::isProgramPaused()
//{
//    return robot_mode_.isProgramPaused;
//}
//unsigned char RobotState::getRobotMode()
//{
//    return robot_mode_.robotMode;
//}
//
//void RobotState::setDigitalInputBits(int digitalInputBits)
//{
//    mb_data_.digitalInputBits = digitalInputBits;
//}
//void RobotState::setDigitalOutputBits(int digitalOutputBits)
//{
//    mb_data_.digitalOutputBits = digitalOutputBits;
//}
//void RobotState::setAnalogInputRange0(char analogInputRange0)
//{
//    mb_data_.analogInputRange0 = analogInputRange0;
//}
//void RobotState::setAnalogInputRange1(char analogInputRange1)
//{
//    mb_data_.analogInputRange1 = analogInputRange1;
//}
//void RobotState::setAnalogInput0(double analogInput0)
//{
//    mb_data_.analogInput0 = analogInput0;
//}
//void RobotState::setAnalogInput1(double analogInput1)
//{
//    mb_data_.analogInput1 = analogInput1;
//}
//void RobotState::setAnalogOutputDomain0(char analogOutputDomain0)
//{
//    mb_data_.analogOutputDomain0 = analogOutputDomain0;
//}
//void RobotState::setAnalogOutputDomain1(char analogOutputDomain1)
//{
//    mb_data_.analogOutputDomain1 = analogOutputDomain1;
//}
//void RobotState::setAnalogOutput0(double analogOutput0)
//{
//    mb_data_.analogOutput0 = analogOutput0;
//}
//void RobotState::setAnalogOutput1(double analogOutput1)
//{
//    mb_data_.analogOutput1 = analogOutput1;
//}
//void RobotState::setMasterBoardTemperature(float masterBoardTemperature)
//{
//    mb_data_.masterBoardTemperature = masterBoardTemperature;
//}
//void RobotState::setRobotVoltage48V(float robotVoltage48V)
//{
//    mb_data_.robotVoltage48V = robotVoltage48V;
//}
//void RobotState::setRobotCurrent(float robotCurrent)
//{
//    mb_data_.robotCurrent = robotCurrent;
//}
//void RobotState::setMasterIOCurrent(float masterIOCurrent)
//{
//    mb_data_.masterIOCurrent = masterIOCurrent;
//}
//void RobotState::setSafetyMode(unsigned char safetyMode)
//{
//    mb_data_.safetyMode = safetyMode;
//}
//void RobotState::setInReducedMode(unsigned char masterOnOffState)
//{
//    mb_data_.masterOnOffState = masterOnOffState;
//}
//void RobotState::setEuromap67InterfaceInstalled(char euromap67InterfaceInstalled)
//{
//    mb_data_.euromap67InterfaceInstalled = euromap67InterfaceInstalled;
//}
//void RobotState::setEuromapInputBits(int euromapInputBits)
//{
//    mb_data_.euromapInputBits = euromapInputBits;
//}
//void RobotState::setEuromapOutputBits(int euromapOutputBits)
//{
//    mb_data_.euromapOutputBits = euromapOutputBits;
//}
//void RobotState::setEuromapVoltage(float euromapVoltage)
//{
//    mb_data_.euromapVoltage = euromapVoltage;
//}
//void RobotState::setEuromapCurrent(float euromapCurrent)
//{
//    mb_data_.euromapCurrent = euromapCurrent;
//}
//
//void RobotState::setRobotConnected(bool isRobotConnected)
//{
//    robot_mode_.isRobotConnected = isRobotConnected;
//}
//void RobotState::setRealRobotEnabled(bool isRealRobotEnabled)
//{
//    robot_mode_.isRealRobotEnabled = isRealRobotEnabled;
//}
//void RobotState::setPowerOnRobot(bool isPowerOnRobot)
//{
//    robot_mode_.isPowerOnRobot = isPowerOnRobot;
//}
//void RobotState::setEmergencyStopped(bool isEmergencyStopped)
//{
//    robot_mode_.isEmergencyStopped = isEmergencyStopped;
//}
//void RobotState::setProtectiveStopped(bool isProtectiveStopped)
//{
//    robot_mode_.isProtectiveStopped = isProtectiveStopped;
//}
//void RobotState::setProgramRunning(bool isProgramRunning)
//{
//    robot_mode_.isProgramRunning = isProgramRunning;
//}
//void RobotState::setProgramPaused(bool isProgramPaused)
//{
//    robot_mode_.isProgramPaused = isProgramPaused;
//}
//void RobotState::setRobotMode(unsigned char robotMode)
//{
//    robot_mode_.robotMode = robotMode;
//}
//
//
//
//// Joint data get_
//double RobotState::get_q_actual_1()
//{
//	return joint_data_.q_actual_1;
//}
//
//double RobotState::get_q_target_1()
//{
//	return joint_data_.q_target_1;
//}
//double RobotState::get_qd_actual_1()
//{
//	return joint_data_.qd_actual_1;
//}
//float RobotState::get_I_actual_1()
//{
//	return joint_data_.I_actual_1;
//}
//float RobotState::get_V_actual_1()
//{
//	return joint_data_.V_actual_1;
//}
//float RobotState::get_T_motor_1()
//{
//	return joint_data_.T_motor_1;
//}
//float RobotState::get_T_micro_1()
//{
//	return joint_data_.T_micro_1;
//}
//unsigned char RobotState::get_jointMode_1()
//{
//	return joint_data_.jointMode_1;
//}
//double RobotState::get_q_actual_2()
//{
//	return joint_data_.q_actual_2;
//}
//double RobotState::get_q_target_2()
//{
//	return joint_data_.q_target_2;
//}
//double RobotState::get_qd_actual_2()
//{
//	return joint_data_.qd_actual_2;
//}
//float RobotState::get_I_actual_2()
//{
//	return joint_data_.I_actual_2;
//}
//float RobotState::get_V_actual_2()
//{
//	return joint_data_.V_actual_2;
//}
//float RobotState::get_T_motor_2()
//{
//	return joint_data_.T_motor_2;
//}
//float RobotState::get_T_micro_2()
//{
//	return joint_data_.T_micro_2;
//}
//unsigned char RobotState::get_jointMode_2()
//{
//	return joint_data_.jointMode_2;
//}
//double RobotState::get_q_actual_3()
//{
//	return joint_data_.q_actual_3;
//}
//double RobotState::get_q_target_3()
//{
//	return joint_data_.q_target_3;
//}
//double RobotState::get_qd_actual_3()
//{
//	return joint_data_.qd_actual_3;
//}
//float RobotState::get_I_actual_3()
//{
//	return joint_data_.I_actual_3;
//}
//float RobotState::get_V_actual_3()
//{
//	return joint_data_.V_actual_3;
//}
//float RobotState::get_T_motor_3()
//{
//	return joint_data_.T_motor_3;
//}
//float RobotState::get_T_micro_3()
//{
//	return joint_data_.T_micro_3;
//}
//unsigned char RobotState::get_jointMode_3()
//{
//	return joint_data_.jointMode_3;
//}
//double RobotState::get_q_actual_4()
//{
//	return joint_data_.q_actual_4;
//}
//double RobotState::get_q_target_4()
//{
//	return joint_data_.q_target_4;
//}
//double RobotState::get_qd_actual_4()
//{
//	return joint_data_.qd_actual_4;
//}
//float RobotState::get_I_actual_4()
//{
//	return joint_data_.I_actual_4;
//}
//float RobotState::get_V_actual_4()
//{
//	return joint_data_.V_actual_4;
//}
//float RobotState::get_T_motor_4()
//{
//	return joint_data_.T_motor_4;
//}
//float RobotState::get_T_micro_4()
//{
//	return joint_data_.T_micro_4;
//}
//unsigned char RobotState::get_jointMode_4()
//{
//	return joint_data_.jointMode_4;
//}
//double RobotState::get_q_actual_5()
//{
//	return joint_data_.q_actual_5;
//}
//double RobotState::get_q_target_5()
//{
//	return joint_data_.q_target_5;
//}
//double RobotState::get_qd_actual_5()
//{
//	return joint_data_.qd_actual_5;
//}
//float RobotState::get_I_actual_5()
//{
//	return joint_data_.I_actual_5;
//}
//float RobotState::get_V_actual_5()
//{
//	return joint_data_.V_actual_5;
//}
//float RobotState::get_T_motor_5()
//{
//	return joint_data_.T_motor_5;
//}
//float RobotState::get_T_micro_5()
//{
//	return joint_data_.T_micro_5;
//}
//unsigned char RobotState::get_jointMode_5()
//{
//	return joint_data_.jointMode_5;
//}
//double RobotState::get_q_actual_6()
//{
//	return joint_data_.q_actual_6;
//}
//double RobotState::get_q_target_6()
//{
//	return joint_data_.q_target_6;
//}
//double RobotState::get_qd_actual_6()
//{
//	return joint_data_.qd_actual_6;
//}
//float RobotState::get_I_actual_6()
//{
//	return joint_data_.I_actual_6;
//}
//float RobotState::get_V_actual_6()
//{
//	return joint_data_.V_actual_6;
//}
//float RobotState::get_T_motor_6()
//{
//	return joint_data_.T_motor_6;
//}
//float RobotState::get_T_micro_6()
//{
//	return joint_data_.T_micro_6;
//}
//unsigned char RobotState::get_jointMode_6()
//{
//	return joint_data_.jointMode_6;
//}
//double RobotState::get_q_actual_7()
//{
//	return joint_data_.q_actual_7;
//}
//double RobotState::get_q_target_7()
//{
//	return joint_data_.q_target_7;
//}
//double RobotState::get_qd_actual_7()
//{
//	return joint_data_.qd_actual_7;
//}
//float RobotState::get_I_actual_7()
//{
//	return joint_data_.I_actual_7;
//}
//float RobotState::get_V_actual_7()
//{
//	return joint_data_.V_actual_7;
//}
//float RobotState::get_T_motor_7()
//{
//	return joint_data_.T_motor_7;
//}
//float RobotState::get_T_micro_7()
//{
//	return joint_data_.T_micro_7;
//}
//unsigned char RobotState::get_jointMode_7()
//{
//	return joint_data_.jointMode_7;
//}
//
//// Joint data set_
//void RobotState::set_q_actual_1(double q_actual_1)
//{
//	joint_data_.q_actual_1 = q_actual_1;
//}
//void RobotState::set_q_target_1(double q_target_1)
//{
//	joint_data_.q_target_1 = q_target_1;
//}
//void RobotState::set_qd_actual_1(double qd_actual_1)
//{
//	joint_data_.qd_actual_1 = qd_actual_1;
//}
//void RobotState::set_I_actual_1(float I_actual_1)
//{
//	joint_data_.I_actual_1 = I_actual_1;
//}
//void RobotState::set_V_actual_1(float V_actual_1)
//{
//	joint_data_.V_actual_1 = V_actual_1;
//}
//void RobotState::set_T_motor_1(float T_motor_1)
//{
//	joint_data_.T_motor_1 = T_motor_1;
//}
//void RobotState::set_T_micro_1(float T_micro_1)
//{
//	joint_data_.T_micro_1 = T_micro_1;
//}
//void RobotState::set_jointMode_1(unsigned char jointMode_1)
//{
//	joint_data_.jointMode_1 = jointMode_1;
//}
//void RobotState::set_q_actual_2(double q_actual_2)
//{
//	joint_data_.q_actual_2 = q_actual_2;
//}
//void RobotState::set_q_target_2(double q_target_2)
//{
//	joint_data_.q_target_2 = q_target_2;
//}
//void RobotState::set_qd_actual_2(double qd_actual_2)
//{
//	joint_data_.qd_actual_2 = qd_actual_2;
//}
//void RobotState::set_I_actual_2(float I_actual_2)
//{
//	joint_data_.I_actual_2 = I_actual_2;
//}
//void RobotState::set_V_actual_2(float V_actual_2)
//{
//	joint_data_.V_actual_2 = V_actual_2;
//}
//void RobotState::set_T_motor_2(float T_motor_2)
//{
//	joint_data_.T_motor_2 = T_motor_2;
//}
//void RobotState::set_T_micro_2(float T_micro_2)
//{
//	joint_data_.T_micro_2 = T_micro_2;
//}
//void RobotState::set_jointMode_2(unsigned char jointMode_2)
//{
//	joint_data_.jointMode_2 = jointMode_2;
//}
//void RobotState::set_q_actual_3(double q_actual_3)
//{
//	joint_data_.q_actual_3 = q_actual_3;
//}
//void RobotState::set_q_target_3(double q_target_3)
//{
//	joint_data_.q_target_3 = q_target_3;
//}
//void RobotState::set_qd_actual_3(double qd_actual_3)
//{
//	joint_data_.qd_actual_3 = qd_actual_3;
//}
//void RobotState::set_I_actual_3(float I_actual_3)
//{
//	joint_data_.I_actual_3 = I_actual_3;
//}
//void RobotState::set_V_actual_3(float V_actual_3)
//{
//	joint_data_.V_actual_3 = V_actual_3;
//}
//void RobotState::set_T_motor_3(float T_motor_3)
//{
//	joint_data_.T_motor_3 = T_motor_3;
//}
//void RobotState::set_T_micro_3(float T_micro_3)
//{
//	joint_data_.T_micro_3 = T_micro_3;
//}
//void RobotState::set_jointMode_3(unsigned char jointMode_3)
//{
//	joint_data_.jointMode_3 = jointMode_3;
//}
//void RobotState::set_q_actual_4(double q_actual_4)
//{
//	joint_data_.q_actual_4 = q_actual_4;
//}
//void RobotState::set_q_target_4(double q_target_4)
//{
//	joint_data_.q_target_4 = q_target_4;
//}
//void RobotState::set_qd_actual_4(double qd_actual_4)
//{
//	joint_data_.qd_actual_4 = qd_actual_4;
//}
//void RobotState::set_I_actual_4(float I_actual_4)
//{
//	joint_data_.I_actual_4 = I_actual_4;
//}
//void RobotState::set_V_actual_4(float V_actual_4)
//{
//	joint_data_.V_actual_4 = V_actual_4;
//}
//void RobotState::set_T_motor_4(float T_motor_4)
//{
//	joint_data_.T_motor_4 = T_motor_4;
//}
//void RobotState::set_T_micro_4(float T_micro_4)
//{
//	joint_data_.T_micro_4 = T_micro_4;
//}
//void RobotState::set_jointMode_4(unsigned char jointMode_4)
//{
//	joint_data_.jointMode_4 = jointMode_4;
//}
//void RobotState::set_q_actual_5(double q_actual_5)
//{
//	joint_data_.q_actual_5 = q_actual_5;
//}
//void RobotState::set_q_target_5(double q_target_5)
//{
//	joint_data_.q_target_5 = q_target_5;
//}
//void RobotState::set_qd_actual_5(double qd_actual_5)
//{
//	joint_data_.qd_actual_5 = qd_actual_5;
//}
//void RobotState::set_I_actual_5(float I_actual_5)
//{
//	joint_data_.I_actual_5 = I_actual_5;
//}
//void RobotState::set_V_actual_5(float V_actual_5)
//{
//	joint_data_.V_actual_5 = V_actual_5;
//}
//void RobotState::set_T_motor_5(float T_motor_5)
//{
//	joint_data_.T_motor_5 = T_motor_5;
//}
//void RobotState::set_T_micro_5(float T_micro_5)
//{
//	joint_data_.T_micro_5 = T_micro_5;
//}
//void RobotState::set_jointMode_5(unsigned char jointMode_5)
//{
//	joint_data_.jointMode_5 = jointMode_5;
//}
//void RobotState::set_q_actual_6(double q_actual_6)
//{
//	joint_data_.q_actual_6 = q_actual_6;
//}
//void RobotState::set_q_target_6(double q_target_6)
//{
//	joint_data_.q_target_6 = q_target_6;
//}
//void RobotState::set_qd_actual_6(double qd_actual_6)
//{
//	joint_data_.qd_actual_6 = qd_actual_6;
//}
//void RobotState::set_I_actual_6(float I_actual_6)
//{
//	joint_data_.I_actual_6 = I_actual_6;
//}
//void RobotState::set_V_actual_6(float V_actual_6)
//{
//	joint_data_.V_actual_6 = V_actual_6;
//}
//void RobotState::set_T_motor_6(float T_motor_6)
//{
//	joint_data_.T_motor_6 = T_motor_6;
//}
//void RobotState::set_T_micro_6(float T_micro_6)
//{
//	joint_data_.T_micro_6 = T_micro_6;
//}
//void RobotState::set_jointMode_6(unsigned char jointMode_6)
//{
//	joint_data_.jointMode_6 = jointMode_6;
//}
//void RobotState::set_q_actual_7(double q_actual_7)
//{
//	joint_data_.q_actual_7 = q_actual_7;
//}
//void RobotState::set_q_target_7(double q_target_7)
//{
//	joint_data_.q_target_7 = q_target_7;
//}
//void RobotState::set_qd_actual_7(double qd_actual_7)
//{
//	joint_data_.qd_actual_7 = qd_actual_7;
//}
//void RobotState::set_I_actual_7(float I_actual_7)
//{
//	joint_data_.I_actual_7 = I_actual_7;
//}
//void RobotState::set_V_actual_7(float V_actual_7)
//{
//	joint_data_.V_actual_7 = V_actual_7;
//}
//void RobotState::set_T_motor_7(float T_motor_7)
//{
//	joint_data_.T_motor_7 = T_motor_7;
//}
//void RobotState::set_T_micro_7(float T_micro_7)
//{
//	joint_data_.T_micro_7 = T_micro_7;
//}
//void RobotState::set_jointMode_7(unsigned char jointMode_7)
//{
//	joint_data_.jointMode_7 = jointMode_7;
//}
//
//
//// Cartesian info get_
//double RobotState::get_X()
//{
//	return cartesian_info_.X;
//}
//double RobotState::get_Y()
//{
//	return cartesian_info_.Y;
//}
//double RobotState::get_Z()
//{
//	return cartesian_info_.Z;
//}
//double RobotState::get_Rx()
//{
//	return cartesian_info_.Rx;
//}
//double RobotState::get_Ry()
//{
//	return cartesian_info_.Ry;
//}
//double RobotState::get_Rz()
//{
//	return cartesian_info_.Rz;
//}
//double RobotState::get_TCPOffsetX()
//{
//	return cartesian_info_.TCPOffsetX;
//}
//double RobotState::get_TCPOffsetY()
//{
//	return cartesian_info_.TCPOffsetY;
//}
//double RobotState::get_TCPOffsetZ()
//{
//	return cartesian_info_.TCPOffsetZ;
//}
//double RobotState::get_TCPOffsetRx()
//{
//	return cartesian_info_.TCPOffsetRx;
//}
//double RobotState::get_TCPOffsetRy()
//{
//	return cartesian_info_.TCPOffsetRy;
//}
//double RobotState::get_TCPOffsetRz()
//{
//	return cartesian_info_.TCPOffsetRz;
//}
//
//
///* Cartesian info set_ */
//void RobotState::set_X(double X)
//{
//	cartesian_info_.X = X;
//}
//void RobotState::set_Y(double Y)
//{
//	cartesian_info_.Y = Y;
//}
//void RobotState::set_Z(double Z)
//{
//	cartesian_info_.Z = Z;
//}
//void RobotState::set_Rx(double Rx)
//{
//	cartesian_info_.Rx = Rx;
//}
//void RobotState::set_Ry(double Ry)
//{
//	cartesian_info_.Ry = Ry;
//}
//void RobotState::set_Rz(double Rz)
//{
//	cartesian_info_.Rz = Rz;
//}
//void RobotState::set_TCPOffsetX(double TCPOffsetX)
//{
//	cartesian_info_.TCPOffsetX = TCPOffsetX;
//}
//void RobotState::set_TCPOffsetY(double TCPOffsetY)
//{
//	cartesian_info_.TCPOffsetY = TCPOffsetY;
//}
//void RobotState::set_TCPOffsetZ(double TCPOffsetZ)
//{
//	cartesian_info_.TCPOffsetZ = TCPOffsetZ;
//}
//void RobotState::set_TCPOffsetRx(double TCPOffsetRx)
//{
//	cartesian_info_.TCPOffsetRx = TCPOffsetRx;
//}
//void RobotState::set_TCPOffsetRy(double TCPOffsetRy)
//{
//	cartesian_info_.TCPOffsetRy = TCPOffsetRy;
//}
//void RobotState::set_TCPOffsetRz(double TCPOffsetRz)
//{
//	cartesian_info_.TCPOffsetRz = TCPOffsetRz;
//}
//
//
//bool RobotState::isReady()
//{
//    if (robot_mode_.robotMode == robot_mode_running_)
//    {
//        return true;
//    }
//    return false;
//}
//endregion

void RobotState::setDisconnected()
{
    robot_mode_.isRobotConnected = false;
    robot_mode_.isRealRobotEnabled = false;
    robot_mode_.isPowerOnRobot = false;
}

unsigned int RobotState::unpackFromMem(uint8_t *buf, unsigned int buf_length) {
    unsigned int offset = 0;

    offset += unpackFromMemRobotMode(buf, offset);
    offset += unpackFromMemJointData(buf, offset);
    offset += unpackFromMemCartesianInfo(buf, offset);
    offset += unpackFromMemRobotStateMasterboard(buf, offset);
    offset += unpackFromMemConfigurationData(buf, offset);
    offset += unpackFromMemAdditionalInfo(buf, offset);

    offset += unpackFromMemRobotMessageVersion(buf, offset);
    offset += unpackFromMemSafetyModeMessage(buf, offset);
    offset += unpackFromMemRobotcommMessage(buf, offset);
    offset += unpackFromMemKeyMessage(buf, offset);
    offset += unpackFromMemLabelMessage(buf, offset);
    offset += unpackFromMemGlobalVariablesSetupMessage(buf, offset);

    return offset;
}

unsigned int RobotState::packToMem(uint8_t *buf) {
    unsigned int buf_length = 0;

    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::ROBOT_MODE_DATA);
    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::JOINT_DATA);
    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::CARTESIAN_INFO);
    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::MASTERBOARD_DATA);
    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::CONFIGURATION_DATA);
    buf_length += RobotState::packToMemRobotState(buf, buf_length, packageType::ADDITIONAL_INFO);

    buf_length += RobotState::packToMemRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_VERSION);
    buf_length += RobotState::packToMemRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_SAFETY_MODE);
    buf_length += RobotState::packToMemRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_ERROR_CODE);
    buf_length += RobotState::packToMemRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_KEY);
    buf_length += RobotState::packToMemRobotMessage(buf, buf_length, robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL);

    buf_length += RobotState::packToMemProgramMessage(buf, buf_length, programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP);

    return (buf_length);
}

unsigned int RobotState::packProgramMessage(uint8_t *buf, unsigned int offset, uint8_t package_type) {
    int32_t length = 5;
    uint8_t msgType = messageType::PROGRAM_STATE_MESSAGE;
    switch (package_type)
    {
        case programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP:
            val_lock_.lock();
            length += RobotState::packGlobalVariablesSetupMessage(buf, offset + 5);
            val_lock_.unlock();
            break;
        default:
            break;
    }
    memcpy(&buf[offset + sizeof(length)], &msgType,
           sizeof(msgType));
    length = htonl(length);
    memcpy(&buf[offset], &length, sizeof(length));
    return ntohl(length);
}

unsigned int RobotState::unpackConfigurationData(uint8_t *buf, unsigned int offset) {
    //double temp;
    unsigned int offset_ = offset;

    unsigned int tempLength = 0;
    for (int i = 0; i < (sizeof(configuration_data) - sizeof(int) * 4)/ sizeof(double); ++i) {
//        memcpy(&temp, &buf[offset],
//               sizeof(double));
//        temp = ntohd(temp);
        memcpy((char*)&configuration_data_.jointMinLimit_1+tempLength, &buf[offset], sizeof(double));
        tempLength += sizeof(double);
        offset += sizeof(double);
    }

    int tempInt;
    for (int i = 0; i < 4; ++i) {
        memcpy(&tempInt, &buf[offset],
               sizeof(int));
        tempInt = ntohl(tempInt);
        memcpy((char*)&configuration_data_.jointMinLimit_1+tempLength, &tempInt, sizeof(int));
        tempLength += sizeof(int);
        offset += sizeof(int);
    }

    return (offset - offset_);
}

unsigned int RobotState::unpackAdditionalInfo(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint8_t tmp;
    memcpy(&tmp, &buf[offset], sizeof(tmp));
    additional_info_.freedriveButtonPressed = tmp>0 ? true : false;
    offset += sizeof(tmp);

    memcpy(&tmp, &buf[offset], sizeof(tmp));
    additional_info_.freedriveButtonEnabled = tmp>0 ? true : false;
    offset += sizeof(tmp);

    memcpy(&tmp, &buf[offset], sizeof(tmp));
    additional_info_.IOEnabledFreedrive = tmp>0 ? true : false;
    offset += sizeof(tmp);

    return (offset - offset_);
}

unsigned int RobotState::unpackSafetyModeMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    safetyModeMessage_.timestamp = timestamp;
    safetyModeMessage_.source = source;
    safetyModeMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(safetyModeMessage_.robotMessageCode));
    offset += sizeof(safetyModeMessage_.robotMessageCode);
    safetyModeMessage_.robotMessageCode = ntohl(temp);

    memcpy(&temp, &buf[offset],
           sizeof(safetyModeMessage_.robotMessageArgument));
    offset += sizeof(safetyModeMessage_.robotMessageArgument);
    safetyModeMessage_.robotMessageArgument = ntohl(temp);

    memcpy(&safetyModeMessage_.safetyModeType, &buf[offset],
           sizeof(safetyModeMessage_.safetyModeType));
    offset += sizeof(safetyModeMessage_.safetyModeType);


    memcpy(&safetyModeMessage_.textmessage_size, &buf[offset],
           sizeof(safetyModeMessage_.textmessage_size));
    offset += sizeof(safetyModeMessage_.textmessage_size);
    ZX_MEMCPY(safetyModeMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * safetyModeMessage_.textmessage_size);
    offset += safetyModeMessage_.textmessage_size;

    val_lock_.unlock();

    return offset-offset_;
}

unsigned int RobotState::unpackRobotcommMessage(uint8_t *buf, unsigned int offset) {

    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    robotcommMessage_.timestamp = timestamp;
    robotcommMessage_.source = source;
    robotcommMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.robotMessageCode));
    offset += sizeof(robotcommMessage_.robotMessageCode);
    robotcommMessage_.robotMessageCode = ntohl(temp);

    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.robotMessageArgument));
    offset += sizeof(robotcommMessage_.robotMessageArgument);
    robotcommMessage_.robotMessageArgument = ntohl(temp);

    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.warningLevel));
    offset += sizeof(robotcommMessage_.warningLevel);
    robotcommMessage_.warningLevel = ntohl(temp);

    memcpy(&robotcommMessage_.textmessage_size, &buf[offset],
           sizeof(robotcommMessage_.textmessage_size));
    offset += sizeof(robotcommMessage_.textmessage_size);
    ZX_MEMCPY(robotcommMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * robotcommMessage_.textmessage_size);
    offset += robotcommMessage_.textmessage_size;

    val_lock_.unlock();
    return offset-offset_;

}

unsigned int RobotState::unpackKeyMessage(uint8_t *buf, unsigned int offset) {

    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    keyMessage_.timestamp = timestamp;
    keyMessage_.source = source;
    keyMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(keyMessage_.robotMessageCode));
    offset += sizeof(keyMessage_.robotMessageCode);
    keyMessage_.robotMessageCode = ntohl(temp);

    memcpy(&temp, &buf[offset],
           sizeof(keyMessage_.robotMessageArgument));
    offset += sizeof(keyMessage_.robotMessageArgument);
    keyMessage_.robotMessageArgument = ntohl(temp);

    memcpy(&keyMessage_.titleSize, &buf[offset],
           sizeof(keyMessage_.titleSize));
    offset += sizeof(keyMessage_.titleSize);
    ZX_MEMCPY(keyMessage_.messageTitle, (char*)(buf+offset),
              sizeof(char) * keyMessage_.titleSize);
    offset += keyMessage_.titleSize;

    memcpy(&keyMessage_.textMessage_size, &buf[offset],
           sizeof(keyMessage_.textMessage_size));
    offset += sizeof(keyMessage_.textMessage_size);
    ZX_MEMCPY(keyMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * keyMessage_.textMessage_size);
    offset += keyMessage_.textMessage_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackLabelMessage(uint8_t *buf, unsigned int offset) {

    unsigned int offset_ = offset;
    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    labelMessage_.timestamp = timestamp;
    labelMessage_.source = source;
    labelMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(labelMessage_.id));
    offset += sizeof(labelMessage_.id);
    labelMessage_.id = ntohl(temp);

    memcpy(&labelMessage_.textMessage_size, &buf[offset],
           sizeof(labelMessage_.textMessage_size));
    offset += sizeof(labelMessage_.textMessage_size);
    ZX_MEMCPY(labelMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * labelMessage_.textMessage_size);
    offset += labelMessage_.textMessage_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackRobotMessageVersion(uint8_t *buf, unsigned int offset) {

    unsigned int offset_ = offset;
    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);


    val_lock_.lock();

    version_msg_.timestamp = timestamp;
    version_msg_.source = source;
    version_msg_.robot_message_type = robot_message_type;

    memcpy(&version_msg_.project_name_size, &buf[offset],
           sizeof(version_msg_.project_name_size));
    offset += sizeof(version_msg_.project_name_size);
    ZX_MEMCPY(version_msg_.project_name, (char*)(buf+offset),
           sizeof(char) * version_msg_.project_name_size);
    offset += version_msg_.project_name_size;

    memcpy(&version_msg_.major_version, &buf[offset],
           sizeof(version_msg_.major_version));
    offset += sizeof(version_msg_.major_version);
    memcpy(&version_msg_.minor_version, &buf[offset],
           sizeof(version_msg_.minor_version));
    offset += sizeof(version_msg_.minor_version);
    int svn_revision;
    memcpy(&svn_revision, &buf[offset],
           sizeof(version_msg_.svn_revision));
    offset += sizeof(version_msg_.svn_revision);
    version_msg_.svn_revision = ntohl(svn_revision);

    memcpy(&version_msg_.build_date_size, &buf[offset],
           sizeof(version_msg_.build_date_size));
    offset += sizeof(version_msg_.build_date_size);
    ZX_MEMCPY(version_msg_.build_date, (char*)(buf+offset),
              sizeof(char) * version_msg_.build_date_size);
    offset += version_msg_.build_date_size;

    if (version_msg_.major_version < 2)
    {
        robot_mode_running_ = robotStateTypeV18::ROBOT_RUNNING_MODE;
    }
    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    globalVariablesSetupMessage_.timestamp = timestamp;
    globalVariablesSetupMessage_.robot_message_type = robot_message_type;

    unsigned short temp;
    memcpy(&temp, &buf[offset],
           sizeof(globalVariablesSetupMessage_.startIndex));
    offset += sizeof(globalVariablesSetupMessage_.startIndex);
    globalVariablesSetupMessage_.startIndex = ntohs(temp);

    memcpy(&globalVariablesSetupMessage_.variableNames_size, &buf[offset],
           sizeof(globalVariablesSetupMessage_.variableNames_size));
    offset += sizeof(globalVariablesSetupMessage_.variableNames_size);
    ZX_MEMCPY(globalVariablesSetupMessage_.variableNames, (char*)(buf+offset),
              sizeof(char) * globalVariablesSetupMessage_.variableNames_size);
    offset += globalVariablesSetupMessage_.variableNames_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackFromMemGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    uint64_t timestamp;
    int8_t robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    globalVariablesSetupMessage_.timestamp = timestamp;
    globalVariablesSetupMessage_.robot_message_type = robot_message_type;

    unsigned short temp;
    memcpy(&temp, &buf[offset],
           sizeof(globalVariablesSetupMessage_.startIndex));
    offset += sizeof(globalVariablesSetupMessage_.startIndex);
    globalVariablesSetupMessage_.startIndex = (temp);

    memcpy(&globalVariablesSetupMessage_.variableNames_size, &buf[offset],
           sizeof(globalVariablesSetupMessage_.variableNames_size));
    offset += sizeof(globalVariablesSetupMessage_.variableNames_size);
    ZX_MEMCPY(globalVariablesSetupMessage_.variableNames, (char*)(buf+offset),
              sizeof(char) * globalVariablesSetupMessage_.variableNames_size);
    offset += globalVariablesSetupMessage_.variableNames_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackFromMemLabelMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    labelMessage_.timestamp = timestamp;
    labelMessage_.source = source;
    labelMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(labelMessage_.id));
    offset += sizeof(labelMessage_.id);
    labelMessage_.id = (temp);

    memcpy(&labelMessage_.textMessage_size, &buf[offset],
           sizeof(labelMessage_.textMessage_size));
    offset += sizeof(labelMessage_.textMessage_size);
    ZX_MEMCPY(labelMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * labelMessage_.textMessage_size);
    offset += labelMessage_.textMessage_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackFromMemRobotStateMasterboard(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    if (RobotState::getVersion() < 3.0)
    {
        int16_t digital_input_bits, digital_output_bits;
        memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
        offset += sizeof(digital_input_bits);
        memcpy(&digital_output_bits, &buf[offset], sizeof(digital_output_bits));
        offset += sizeof(digital_output_bits);
        mb_data_.digitalInputBits = (digital_input_bits);
        mb_data_.digitalOutputBits = (digital_output_bits);
    }
    else
    {
        memcpy(&mb_data_.digitalInputBits, &buf[offset],
               sizeof(mb_data_.digitalInputBits));
        offset += sizeof(mb_data_.digitalInputBits);
        mb_data_.digitalInputBits = (mb_data_.digitalInputBits);
        memcpy(&mb_data_.digitalOutputBits, &buf[offset],
               sizeof(mb_data_.digitalOutputBits));
        offset += sizeof(mb_data_.digitalOutputBits);
        mb_data_.digitalOutputBits = (mb_data_.digitalOutputBits);
    }

    memcpy(&mb_data_.analogInputRange0, &buf[offset],
           sizeof(mb_data_.analogInputRange0));
    offset += sizeof(mb_data_.analogInputRange0);
    memcpy(&mb_data_.analogInputRange1, &buf[offset],
           sizeof(mb_data_.analogInputRange1));
    offset += sizeof(mb_data_.analogInputRange1);
    //uint64_t temp;
    memcpy(&mb_data_.analogInput0, &buf[offset], sizeof(mb_data_.analogInput0));
    offset += sizeof(mb_data_.analogInput0);
    //mb_data_.analogInput0 = RobotState::ntohd(temp);
    memcpy(&mb_data_.analogInput1, &buf[offset], sizeof(mb_data_.analogInput1));
    offset += sizeof(mb_data_.analogInput1);
    //mb_data_.analogInput1 = RobotState::ntohd(temp);
    memcpy(&mb_data_.analogOutputDomain0, &buf[offset],
           sizeof(mb_data_.analogOutputDomain0));
    offset += sizeof(mb_data_.analogOutputDomain0);
    memcpy(&mb_data_.analogOutputDomain1, &buf[offset],
           sizeof(mb_data_.analogOutputDomain1));
    offset += sizeof(mb_data_.analogOutputDomain1);
    memcpy(&mb_data_.analogOutput0, &buf[offset], sizeof(mb_data_.analogOutput0));
    offset += sizeof(mb_data_.analogOutput0);
    //mb_data_.analogOutput0 = RobotState::ntohd(temp);
    memcpy(&mb_data_.analogOutput1, &buf[offset], sizeof(mb_data_.analogOutput1));
    offset += sizeof(mb_data_.analogOutput1);
    //mb_data_.analogOutput1 = RobotState::ntohd(temp);

    memcpy(&mb_data_.masterBoardTemperature, &buf[offset],
           sizeof(mb_data_.masterBoardTemperature));
    offset += sizeof(mb_data_.masterBoardTemperature);
    mb_data_.masterBoardTemperature = (mb_data_.masterBoardTemperature);
    memcpy(&mb_data_.robotVoltage48V, &buf[offset],
           sizeof(mb_data_.robotVoltage48V));
    offset += sizeof(mb_data_.robotVoltage48V);
    mb_data_.robotVoltage48V = (mb_data_.robotVoltage48V);
    memcpy(&mb_data_.robotCurrent, &buf[offset], sizeof(mb_data_.robotCurrent));
    offset += sizeof(mb_data_.robotCurrent);
    mb_data_.robotCurrent = (mb_data_.robotCurrent);
    memcpy(&mb_data_.masterIOCurrent, &buf[offset],
           sizeof(mb_data_.masterIOCurrent));
    offset += sizeof(mb_data_.masterIOCurrent);
    mb_data_.masterIOCurrent = (mb_data_.masterIOCurrent);

    memcpy(&mb_data_.safetyMode, &buf[offset], sizeof(mb_data_.safetyMode));
    offset += sizeof(mb_data_.safetyMode);
    memcpy(&mb_data_.masterOnOffState, &buf[offset],
           sizeof(mb_data_.masterOnOffState));
    offset += sizeof(mb_data_.masterOnOffState);

    memcpy(&mb_data_.euromap67InterfaceInstalled, &buf[offset],
           sizeof(mb_data_.euromap67InterfaceInstalled));
    offset += sizeof(mb_data_.euromap67InterfaceInstalled);
    if (mb_data_.euromap67InterfaceInstalled != 0)
    {
        memcpy(&mb_data_.euromapInputBits, &buf[offset],
               sizeof(mb_data_.euromapInputBits));
        offset += sizeof(mb_data_.euromapInputBits);
        mb_data_.euromapInputBits = (mb_data_.euromapInputBits);
        memcpy(&mb_data_.euromapOutputBits, &buf[offset],
               sizeof(mb_data_.euromapOutputBits));
        offset += sizeof(mb_data_.euromapOutputBits);
        mb_data_.euromapOutputBits = (mb_data_.euromapOutputBits);
        if (RobotState::getVersion() < 3.0)
        {
            int16_t euromap_voltage, euromap_current;
            memcpy(&euromap_voltage, &buf[offset], sizeof(euromap_voltage));
            offset += sizeof(euromap_voltage);
            memcpy(&euromap_current, &buf[offset], sizeof(euromap_current));
            offset += sizeof(euromap_current);
            mb_data_.euromapVoltage = (euromap_voltage);
            mb_data_.euromapCurrent = (euromap_current);
        }
        else
        {
            memcpy(&mb_data_.euromapVoltage, &buf[offset],
                   sizeof(mb_data_.euromapVoltage));
            offset += sizeof(mb_data_.euromapVoltage);
            mb_data_.euromapVoltage = (mb_data_.euromapVoltage);
            memcpy(&mb_data_.euromapCurrent, &buf[offset],
                   sizeof(mb_data_.euromapCurrent));
            offset += sizeof(mb_data_.euromapCurrent);
            mb_data_.euromapCurrent = (mb_data_.euromapCurrent);
        }

    }

    memcpy(&mb_data_.uburso, &buf[offset],
           sizeof(mb_data_.uburso));
    offset += sizeof(mb_data_.uburso);
    mb_data_.uburso = (mb_data_.uburso);

    memcpy(&mb_data_.operationalModeSelectorInput, &buf[offset],
           sizeof(mb_data_.operationalModeSelectorInput));
    offset += sizeof(mb_data_.operationalModeSelectorInput);
    memcpy(&mb_data_.threePositionEnablingDeviceInput, &buf[offset],
           sizeof(mb_data_.threePositionEnablingDeviceInput));
    offset += sizeof(mb_data_.threePositionEnablingDeviceInput);

    return (offset - offset_);
}

unsigned int RobotState::unpackFromMemRobotMode(uint8_t *buf, unsigned int offset) {
    return unpackRobotMode(buf, offset);
}

unsigned int RobotState::unpackFromMemJointData(uint8_t *buf, unsigned int offset) {
    return unpackJointData(buf, offset);
}

unsigned int RobotState::unpackFromMemCartesianInfo(uint8_t *buf, unsigned int offset) {
    return unpackCartesianInfo(buf, offset);
}

unsigned int RobotState::unpackFromMemConfigurationData(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    unsigned int tempLength = 0;
    for (int i = 0; i < (sizeof(configuration_data) - sizeof(int) * 4)/ sizeof(double); ++i) {
        memcpy((char*)&configuration_data_.jointMinLimit_1+tempLength, &buf[offset], sizeof(double));
        tempLength += sizeof(double);
        offset += sizeof(double);
    }

    int tempInt;
    for (int i = 0; i < 4; ++i) {
        memcpy((char*)&configuration_data_.jointMinLimit_1+tempLength, &buf[offset], sizeof(int));
        tempLength += sizeof(int);
        offset += sizeof(int);
    }

    return (offset - offset_);


}

unsigned int RobotState::unpackFromMemAdditionalInfo(uint8_t *buf, unsigned int offset) {
    return unpackAdditionalInfo(buf,offset);
}

unsigned int RobotState::unpackFromMemRobotMessageVersion(uint8_t *buf, unsigned int offset) {
    return unpackRobotMessageVersion(buf, offset);
}

unsigned int RobotState::unpackFromMemSafetyModeMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    safetyModeMessage_.timestamp = timestamp;
    safetyModeMessage_.source = source;
    safetyModeMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(safetyModeMessage_.robotMessageCode));
    offset += sizeof(safetyModeMessage_.robotMessageCode);
    safetyModeMessage_.robotMessageCode = (temp);

    memcpy(&temp, &buf[offset],
           sizeof(safetyModeMessage_.robotMessageArgument));
    offset += sizeof(safetyModeMessage_.robotMessageArgument);
    safetyModeMessage_.robotMessageArgument = (temp);

    memcpy(&safetyModeMessage_.safetyModeType, &buf[offset],
           sizeof(safetyModeMessage_.safetyModeType));
    offset += sizeof(safetyModeMessage_.safetyModeType);


    memcpy(&safetyModeMessage_.textmessage_size, &buf[offset],
           sizeof(safetyModeMessage_.textmessage_size));
    offset += sizeof(safetyModeMessage_.textmessage_size);
    ZX_MEMCPY(safetyModeMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * safetyModeMessage_.textmessage_size);
    offset += safetyModeMessage_.textmessage_size;

    val_lock_.unlock();

    return offset-offset_;
}

unsigned int RobotState::unpackFromMemRobotcommMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    robotcommMessage_.timestamp = timestamp;
    robotcommMessage_.source = source;
    robotcommMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.robotMessageCode));
    offset += sizeof(robotcommMessage_.robotMessageCode);
    robotcommMessage_.robotMessageCode = (temp);

    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.robotMessageArgument));
    offset += sizeof(robotcommMessage_.robotMessageArgument);
    robotcommMessage_.robotMessageArgument = (temp);

    memcpy(&temp, &buf[offset],
           sizeof(robotcommMessage_.warningLevel));
    offset += sizeof(robotcommMessage_.warningLevel);
    robotcommMessage_.warningLevel = (temp);

    memcpy(&robotcommMessage_.textmessage_size, &buf[offset],
           sizeof(robotcommMessage_.textmessage_size));
    offset += sizeof(robotcommMessage_.textmessage_size);
    ZX_MEMCPY(robotcommMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * robotcommMessage_.textmessage_size);
    offset += robotcommMessage_.textmessage_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::unpackFromMemKeyMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    memcpy(&timestamp, &buf[offset], sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&source, &buf[offset], sizeof(source));
    offset += sizeof(source);
    memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    val_lock_.lock();

    keyMessage_.timestamp = timestamp;
    keyMessage_.source = source;
    keyMessage_.robot_message_type = robot_message_type;

    int temp;
    memcpy(&temp, &buf[offset],
           sizeof(keyMessage_.robotMessageCode));
    offset += sizeof(keyMessage_.robotMessageCode);
    keyMessage_.robotMessageCode = (temp);

    memcpy(&temp, &buf[offset],
           sizeof(keyMessage_.robotMessageArgument));
    offset += sizeof(keyMessage_.robotMessageArgument);
    keyMessage_.robotMessageArgument = (temp);

    memcpy(&keyMessage_.titleSize, &buf[offset],
           sizeof(keyMessage_.titleSize));
    offset += sizeof(keyMessage_.titleSize);
    ZX_MEMCPY(keyMessage_.messageTitle, (char*)(buf+offset),
              sizeof(char) * keyMessage_.titleSize);
    offset += keyMessage_.titleSize;

    memcpy(&keyMessage_.textMessage_size, &buf[offset],
           sizeof(keyMessage_.textMessage_size));
    offset += sizeof(keyMessage_.textMessage_size);
    ZX_MEMCPY(keyMessage_.textMessage, (char*)(buf+offset),
              sizeof(char) * keyMessage_.textMessage_size);
    offset += keyMessage_.textMessage_size;

    val_lock_.unlock();
    return offset-offset_;
}

unsigned int RobotState::packToMemRobotState(uint8_t *buf, unsigned int offset, uint8_t packToMemage_type) {
    int32_t length = 0;
    switch (packToMemage_type)
    {
        case packageType::ROBOT_MODE_DATA:
            val_lock_.lock();
            length += RobotState::packToMemRobotMode(buf, offset);
            val_lock_.unlock();
            break;

        case packageType::MASTERBOARD_DATA:
            val_lock_.lock();
            length += RobotState::packToMemRobotStateMasterboard(buf, offset);
            val_lock_.unlock();
            break;

        case packageType::JOINT_DATA:
            val_lock_.lock();
            length += RobotState::packToMemJointData(buf, offset);
            val_lock_.unlock();
            break;

        case packageType::CARTESIAN_INFO:
            val_lock_.lock();
            length += RobotState::packToMemCartesianInfo(buf, offset);
            val_lock_.unlock();
            break;

        case packageType::CONFIGURATION_DATA:
            val_lock_.lock();
            length += RobotState::packToMemConfigurationData(buf, offset);
            val_lock_.unlock();
            break;

        case packageType::ADDITIONAL_INFO:
            val_lock_.lock();
            length += RobotState::packToMemAdditionalInfo(buf, offset);
            val_lock_.unlock();
            break;

        default:
            break;
    }

    return (length);
}

unsigned int RobotState::packToMemRobotMode(uint8_t *buf, unsigned int offset) {
    return RobotState::packRobotMode(buf, offset);
}

unsigned int RobotState::packToMemJointData(uint8_t *buf, unsigned int offset) {
    return RobotState::packJointData(buf, offset);
}

unsigned int RobotState::packToMemCartesianInfo(uint8_t *buf, unsigned int offset) {
    return RobotState::packCartesianInfo(buf, offset);
}

unsigned int RobotState::packToMemRobotStateMasterboard(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    if (RobotState::getVersion() < 3.0)
    {
        int16_t digital_input_bits, digital_output_bits;
        digital_input_bits = (mb_data_.digitalInputBits);
        digital_output_bits = (mb_data_.digitalOutputBits);
        memcpy(&buf[offset], &digital_input_bits, sizeof(digital_input_bits));
        offset += sizeof(digital_input_bits);
        memcpy(&buf[offset], &digital_output_bits, sizeof(digital_output_bits));
        offset += sizeof(digital_output_bits);
    }
    else
    {
        int32_t digital_input_bits, digital_output_bits;
        digital_input_bits = (mb_data_.digitalInputBits);
        memcpy(&buf[offset], &digital_input_bits,
               sizeof(mb_data_.digitalInputBits));
        offset += sizeof(mb_data_.digitalInputBits);
        digital_output_bits = (mb_data_.digitalOutputBits);
        memcpy(&buf[offset], &digital_output_bits,
               sizeof(mb_data_.digitalOutputBits));
        offset += sizeof(mb_data_.digitalOutputBits);
    }

    memcpy(&buf[offset], &mb_data_.analogInputRange0,
           sizeof(mb_data_.analogInputRange0));
    offset += sizeof(mb_data_.analogInputRange0);
    memcpy(&buf[offset], &mb_data_.analogInputRange1,
           sizeof(mb_data_.analogInputRange1));
    offset += sizeof(mb_data_.analogInputRange1);
    uint64_t temp64;
    //temp64 = RobotState::htond(mb_data_.analogInput0);
    memcpy(&buf[offset], &mb_data_.analogInput0, sizeof(mb_data_.analogInput0));
    offset += sizeof(mb_data_.analogInput0);
    //temp64 = RobotState::htond(mb_data_.analogInput1);
    memcpy(&buf[offset], &mb_data_.analogInput1, sizeof(mb_data_.analogInput1));
    offset += sizeof(mb_data_.analogInput1);
    memcpy(&buf[offset], &mb_data_.analogOutputDomain0,
           sizeof(mb_data_.analogOutputDomain0));
    offset += sizeof(mb_data_.analogOutputDomain0);
    memcpy(&buf[offset], &mb_data_.analogOutputDomain1,
           sizeof(mb_data_.analogOutputDomain1));
    offset += sizeof(mb_data_.analogOutputDomain1);
    //temp64 = RobotState::htond(mb_data_.analogOutput0);
    memcpy(&buf[offset], &mb_data_.analogOutput0, sizeof(mb_data_.analogOutput0));
    offset += sizeof(mb_data_.analogOutput0);
    //temp64 = RobotState::htond(mb_data_.analogOutput1);
    memcpy(&buf[offset], &mb_data_.analogOutput1, sizeof(mb_data_.analogOutput1));
    offset += sizeof(mb_data_.analogOutput1);

    uint32_t temp32;
    temp32 = htonl(mb_data_.masterBoardTemperature);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.robotVoltage48V);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.robotCurrent);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);
    temp32 = htonl(mb_data_.masterIOCurrent);
    memcpy(&buf[offset], &temp32,
           sizeof(temp32));
    offset += sizeof(temp32);

    memcpy(&buf[offset], &mb_data_.safetyMode, sizeof(mb_data_.safetyMode));
    offset += sizeof(mb_data_.safetyMode);
    memcpy(&buf[offset], &mb_data_.masterOnOffState,
           sizeof(mb_data_.masterOnOffState));
    offset += sizeof(mb_data_.masterOnOffState);

    memcpy(&buf[offset], &mb_data_.euromap67InterfaceInstalled,
           sizeof(mb_data_.euromap67InterfaceInstalled));
    offset += sizeof(mb_data_.euromap67InterfaceInstalled);
    if (mb_data_.euromap67InterfaceInstalled != 0)
    {
        temp32 = htonl(mb_data_.euromapInputBits);
        memcpy(&buf[offset], &temp32,
               sizeof(temp32));
        offset += sizeof(temp32);
        temp32 = htonl(mb_data_.euromapOutputBits);
        memcpy(&buf[offset], &temp32,
               sizeof(temp32));
        offset += sizeof(temp32);
        if (RobotState::getVersion() < 3.0)
        {
            int16_t euromap_voltage, euromap_current;
            euromap_voltage = htons(mb_data_.euromapVoltage);
            euromap_current = htons(mb_data_.euromapCurrent);
            memcpy(&buf[offset], &euromap_voltage, sizeof(euromap_voltage));
            offset += sizeof(euromap_voltage);
            memcpy(&buf[offset], &euromap_current, sizeof(euromap_current));
            offset += sizeof(euromap_current);
        }
        else
        {
            int32_t euromap_voltage;
            euromap_voltage = htonl(mb_data_.euromapVoltage);
            memcpy(&buf[offset], &euromap_voltage,
                   sizeof(mb_data_.euromapVoltage));
            offset += sizeof(mb_data_.euromapVoltage);
            //euromap_current = htonl(mb_data_.euromapCurrent);
            memcpy(&buf[offset], &mb_data_.euromapCurrent,
                   sizeof(mb_data_.euromapCurrent));
            offset += sizeof(mb_data_.euromapCurrent);
        }

    }

    uint32_t temp;
    temp = (mb_data_.uburso);
    memcpy(&buf[offset], &temp, sizeof(mb_data_.uburso));
    offset += sizeof(mb_data_.uburso);

    memcpy(&buf[offset], &mb_data_.operationalModeSelectorInput, sizeof(mb_data_.operationalModeSelectorInput));
    offset += sizeof(mb_data_.operationalModeSelectorInput);
    memcpy(&buf[offset], &mb_data_.threePositionEnablingDeviceInput, sizeof(mb_data_.threePositionEnablingDeviceInput));
    offset += sizeof(mb_data_.threePositionEnablingDeviceInput);


    return (offset - offset_);
}

unsigned int RobotState::packToMemConfigurationData(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_1, sizeof(configuration_data_.jointMinLimit_1));
    offset += sizeof(configuration_data_.jointMinLimit_1);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_2, sizeof(configuration_data_.jointMinLimit_2));
    offset += sizeof(configuration_data_.jointMinLimit_2);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_3, sizeof(configuration_data_.jointMinLimit_3));
    offset += sizeof(configuration_data_.jointMinLimit_3);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_4, sizeof(configuration_data_.jointMinLimit_4));
    offset += sizeof(configuration_data_.jointMinLimit_4);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_5, sizeof(configuration_data_.jointMinLimit_5));
    offset += sizeof(configuration_data_.jointMinLimit_5);
    memcpy(&buf[offset], &configuration_data_.jointMinLimit_6, sizeof(configuration_data_.jointMinLimit_6));
    offset += sizeof(configuration_data_.jointMinLimit_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_1, sizeof(configuration_data_.jointMaxLimitt_1));
    offset += sizeof(configuration_data_.jointMaxLimitt_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_2, sizeof(configuration_data_.jointMaxLimitt_2));
    offset += sizeof(configuration_data_.jointMaxLimitt_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_3, sizeof(configuration_data_.jointMaxLimitt_3));
    offset += sizeof(configuration_data_.jointMaxLimitt_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_4, sizeof(configuration_data_.jointMaxLimitt_4));
    offset += sizeof(configuration_data_.jointMaxLimitt_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_5, sizeof(configuration_data_.jointMaxLimitt_5));
    offset += sizeof(configuration_data_.jointMaxLimitt_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxLimitt_6, sizeof(configuration_data_.jointMaxLimitt_6));
    offset += sizeof(configuration_data_.jointMaxLimitt_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_1, sizeof(configuration_data_.jointMaxSpeed_1));
    offset += sizeof(configuration_data_.jointMaxSpeed_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_2, sizeof(configuration_data_.jointMaxSpeed_2));
    offset += sizeof(configuration_data_.jointMaxSpeed_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_3, sizeof(configuration_data_.jointMaxSpeed_3));
    offset += sizeof(configuration_data_.jointMaxSpeed_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_4, sizeof(configuration_data_.jointMaxSpeed_4));
    offset += sizeof(configuration_data_.jointMaxSpeed_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_5, sizeof(configuration_data_.jointMaxSpeed_5));
    offset += sizeof(configuration_data_.jointMaxSpeed_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxSpeed_6, sizeof(configuration_data_.jointMaxSpeed_6));
    offset += sizeof(configuration_data_.jointMaxSpeed_6);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_1, sizeof(configuration_data_.jointMaxAcceleration_1));
    offset += sizeof(configuration_data_.jointMaxAcceleration_1);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_2, sizeof(configuration_data_.jointMaxAcceleration_2));
    offset += sizeof(configuration_data_.jointMaxAcceleration_2);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_3, sizeof(configuration_data_.jointMaxAcceleration_3));
    offset += sizeof(configuration_data_.jointMaxAcceleration_3);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_4, sizeof(configuration_data_.jointMaxAcceleration_4));
    offset += sizeof(configuration_data_.jointMaxAcceleration_4);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_5, sizeof(configuration_data_.jointMaxAcceleration_5));
    offset += sizeof(configuration_data_.jointMaxAcceleration_5);
    memcpy(&buf[offset], &configuration_data_.jointMaxAcceleration_6, sizeof(configuration_data_.jointMaxAcceleration_6));
    offset += sizeof(configuration_data_.jointMaxAcceleration_6);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_1, sizeof(configuration_data_.vJointDefault_1));
    offset += sizeof(configuration_data_.vJointDefault_1);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_2, sizeof(configuration_data_.vJointDefault_2));
    offset += sizeof(configuration_data_.vJointDefault_2);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_3, sizeof(configuration_data_.vJointDefault_3));
    offset += sizeof(configuration_data_.vJointDefault_3);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_4, sizeof(configuration_data_.vJointDefault_4));
    offset += sizeof(configuration_data_.vJointDefault_4);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_5, sizeof(configuration_data_.vJointDefault_5));
    offset += sizeof(configuration_data_.vJointDefault_5);
    memcpy(&buf[offset], &configuration_data_.vJointDefault_6, sizeof(configuration_data_.vJointDefault_6));
    offset += sizeof(configuration_data_.vJointDefault_6);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_1, sizeof(configuration_data_.aJointDefault_1));
    offset += sizeof(configuration_data_.aJointDefault_1);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_2, sizeof(configuration_data_.aJointDefault_2));
    offset += sizeof(configuration_data_.aJointDefault_2);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_3, sizeof(configuration_data_.aJointDefault_3));
    offset += sizeof(configuration_data_.aJointDefault_3);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_4, sizeof(configuration_data_.aJointDefault_4));
    offset += sizeof(configuration_data_.aJointDefault_4);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_5, sizeof(configuration_data_.aJointDefault_5));
    offset += sizeof(configuration_data_.aJointDefault_5);
    memcpy(&buf[offset], &configuration_data_.aJointDefault_6, sizeof(configuration_data_.aJointDefault_6));
    offset += sizeof(configuration_data_.aJointDefault_6);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_1, sizeof(configuration_data_.vToolDefault_1));
    offset += sizeof(configuration_data_.vToolDefault_1);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_2, sizeof(configuration_data_.vToolDefault_2));
    offset += sizeof(configuration_data_.vToolDefault_2);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_3, sizeof(configuration_data_.vToolDefault_3));
    offset += sizeof(configuration_data_.vToolDefault_3);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_4, sizeof(configuration_data_.vToolDefault_4));
    offset += sizeof(configuration_data_.vToolDefault_4);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_5, sizeof(configuration_data_.vToolDefault_5));
    offset += sizeof(configuration_data_.vToolDefault_5);
    memcpy(&buf[offset], &configuration_data_.vToolDefault_6, sizeof(configuration_data_.vToolDefault_6));
    offset += sizeof(configuration_data_.vToolDefault_6);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_1, sizeof(configuration_data_.aToolDefault_1));
    offset += sizeof(configuration_data_.aToolDefault_1);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_2, sizeof(configuration_data_.aToolDefault_2));
    offset += sizeof(configuration_data_.aToolDefault_2);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_3, sizeof(configuration_data_.aToolDefault_3));
    offset += sizeof(configuration_data_.aToolDefault_3);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_4, sizeof(configuration_data_.aToolDefault_4));
    offset += sizeof(configuration_data_.aToolDefault_4);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_5, sizeof(configuration_data_.aToolDefault_5));
    offset += sizeof(configuration_data_.aToolDefault_5);
    memcpy(&buf[offset], &configuration_data_.aToolDefault_6, sizeof(configuration_data_.aToolDefault_6));
    offset += sizeof(configuration_data_.aToolDefault_6);
    memcpy(&buf[offset], &configuration_data_.eqRadius_1, sizeof(configuration_data_.eqRadius_1));
    offset += sizeof(configuration_data_.eqRadius_1);
    memcpy(&buf[offset], &configuration_data_.eqRadius_2, sizeof(configuration_data_.eqRadius_2));
    offset += sizeof(configuration_data_.eqRadius_2);
    memcpy(&buf[offset], &configuration_data_.eqRadius_3, sizeof(configuration_data_.eqRadius_3));
    offset += sizeof(configuration_data_.eqRadius_3);
    memcpy(&buf[offset], &configuration_data_.eqRadius_4, sizeof(configuration_data_.eqRadius_4));
    offset += sizeof(configuration_data_.eqRadius_4);
    memcpy(&buf[offset], &configuration_data_.eqRadius_5, sizeof(configuration_data_.eqRadius_5));
    offset += sizeof(configuration_data_.eqRadius_5);
    memcpy(&buf[offset], &configuration_data_.eqRadius_6, sizeof(configuration_data_.eqRadius_6));
    offset += sizeof(configuration_data_.eqRadius_6);
    memcpy(&buf[offset], &configuration_data_.DHa_1, sizeof(configuration_data_.DHa_1));
    offset += sizeof(configuration_data_.DHa_1);
    memcpy(&buf[offset], &configuration_data_.DHa_2, sizeof(configuration_data_.DHa_2));
    offset += sizeof(configuration_data_.DHa_2);
    memcpy(&buf[offset], &configuration_data_.DHa_3, sizeof(configuration_data_.DHa_3));
    offset += sizeof(configuration_data_.DHa_3);
    memcpy(&buf[offset], &configuration_data_.DHa_4, sizeof(configuration_data_.DHa_4));
    offset += sizeof(configuration_data_.DHa_4);
    memcpy(&buf[offset], &configuration_data_.DHa_5, sizeof(configuration_data_.DHa_5));
    offset += sizeof(configuration_data_.DHa_5);
    memcpy(&buf[offset], &configuration_data_.DHa_6, sizeof(configuration_data_.DHa_6));
    offset += sizeof(configuration_data_.DHa_6);
    memcpy(&buf[offset], &configuration_data_.DHd_1, sizeof(configuration_data_.DHd_1));
    offset += sizeof(configuration_data_.DHd_1);
    memcpy(&buf[offset], &configuration_data_.DHd_2, sizeof(configuration_data_.DHd_2));
    offset += sizeof(configuration_data_.DHd_2);
    memcpy(&buf[offset], &configuration_data_.DHd_3, sizeof(configuration_data_.DHd_3));
    offset += sizeof(configuration_data_.DHd_3);
    memcpy(&buf[offset], &configuration_data_.DHd_4, sizeof(configuration_data_.DHd_4));
    offset += sizeof(configuration_data_.DHd_4);
    memcpy(&buf[offset], &configuration_data_.DHd_5, sizeof(configuration_data_.DHd_5));
    offset += sizeof(configuration_data_.DHd_5);
    memcpy(&buf[offset], &configuration_data_.DHd_6, sizeof(configuration_data_.DHd_6));
    offset += sizeof(configuration_data_.DHd_6);
    memcpy(&buf[offset], &configuration_data_.DHalpha_1, sizeof(configuration_data_.DHalpha_1));
    offset += sizeof(configuration_data_.DHalpha_1);
    memcpy(&buf[offset], &configuration_data_.DHalpha_2, sizeof(configuration_data_.DHalpha_2));
    offset += sizeof(configuration_data_.DHalpha_2);
    memcpy(&buf[offset], &configuration_data_.DHalpha_3, sizeof(configuration_data_.DHalpha_3));
    offset += sizeof(configuration_data_.DHalpha_3);
    memcpy(&buf[offset], &configuration_data_.DHalpha_4, sizeof(configuration_data_.DHalpha_4));
    offset += sizeof(configuration_data_.DHalpha_4);
    memcpy(&buf[offset], &configuration_data_.DHalpha_5, sizeof(configuration_data_.DHalpha_5));
    offset += sizeof(configuration_data_.DHalpha_5);
    memcpy(&buf[offset], &configuration_data_.DHalpha_6, sizeof(configuration_data_.DHalpha_6));
    offset += sizeof(configuration_data_.DHalpha_6);
    memcpy(&buf[offset], &configuration_data_.DHtheta_1, sizeof(configuration_data_.DHtheta_1));
    offset += sizeof(configuration_data_.DHtheta_1);
    memcpy(&buf[offset], &configuration_data_.DHtheta_2, sizeof(configuration_data_.DHtheta_2));
    offset += sizeof(configuration_data_.DHtheta_2);
    memcpy(&buf[offset], &configuration_data_.DHtheta_3, sizeof(configuration_data_.DHtheta_3));
    offset += sizeof(configuration_data_.DHtheta_3);
    memcpy(&buf[offset], &configuration_data_.DHtheta_4, sizeof(configuration_data_.DHtheta_4));
    offset += sizeof(configuration_data_.DHtheta_4);
    memcpy(&buf[offset], &configuration_data_.DHtheta_5, sizeof(configuration_data_.DHtheta_5));
    offset += sizeof(configuration_data_.DHtheta_5);
    memcpy(&buf[offset], &configuration_data_.DHtheta_6, sizeof(configuration_data_.DHtheta_6));
    offset += sizeof(configuration_data_.DHtheta_6);

    int temp32;
    temp32 = (configuration_data_.masterboardVersion);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.masterboardVersion);

    temp32 = (configuration_data_.controllerBoxType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.controllerBoxType);

    temp32 = (configuration_data_.robotType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.robotType);

    temp32 = (configuration_data_.robotSubType);
    memcpy(&buf[offset], &temp32, sizeof(temp32));
    offset += sizeof(configuration_data_.robotSubType);

    return (offset - offset_);
}

unsigned int RobotState::packToMemAdditionalInfo(uint8_t *buf, unsigned int offset) {
    return RobotState::packAdditionalInfo(buf, offset);
}

unsigned int RobotState::packToMemRobotMessage(uint8_t *buf, unsigned int offset, uint8_t packToMemage_type) {
    int32_t length = 0;
    uint8_t msgType = messageType::ROBOT_MESSAGE;
    switch (packToMemage_type)
    {
        case robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL:
            val_lock_.lock();
            length += RobotState::packToMemLabelMessage(buf, offset);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_SAFETY_MODE:
            val_lock_.lock();
            length += RobotState::packToMemSafetyModeMessage(buf, offset);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_ERROR_CODE:
            val_lock_.lock();
            length += RobotState::packToMemRobotcommMessage(buf, offset);
            val_lock_.unlock();
            break;

        case robotMessageType::ROBOT_MESSAGE_KEY:
            val_lock_.lock();
            length += RobotState::packToMemKeyMessage(buf, offset);
            val_lock_.unlock();
            break;
        case robotMessageType::ROBOT_MESSAGE_VERSION:
            val_lock_.lock();
            length += RobotState::packToMemRobotMessageVersion(buf, offset);
            val_lock_.unlock();
            break;
        default:
            break;
    }
    return (length);
}

unsigned int RobotState::packToMemRobotMessageVersion(uint8_t *buf, unsigned int offset) {
    return RobotState::packRobotMessageVersion(buf, offset);
}

unsigned int RobotState::packToMemSafetyModeMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = safetyModeMessage_.timestamp;
    source = safetyModeMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_SAFETY_MODE;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = (safetyModeMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp, sizeof(temp));
    offset += sizeof(safetyModeMessage_.robotMessageCode);

    temp = (safetyModeMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp, sizeof(temp));
    offset += sizeof(safetyModeMessage_.robotMessageArgument);

    memcpy(&buf[offset], &safetyModeMessage_.safetyModeType,
           sizeof(safetyModeMessage_.safetyModeType));
    offset += sizeof(safetyModeMessage_.safetyModeType);


    memcpy(&buf[offset], &safetyModeMessage_.textmessage_size,
           sizeof(safetyModeMessage_.textmessage_size));
    offset += sizeof(safetyModeMessage_.textmessage_size);

    if (safetyModeMessage_.textmessage_size) {
        memcpy(&buf[offset], safetyModeMessage_.textMessage,
               sizeof(char) * safetyModeMessage_.textmessage_size);
        offset += safetyModeMessage_.textmessage_size;
    }




    return (offset - offset_);
}

unsigned int RobotState::packToMemRobotcommMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = robotcommMessage_.timestamp;
    source = robotcommMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_ERROR_CODE;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = (robotcommMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.robotMessageCode));
    offset += sizeof(robotcommMessage_.robotMessageCode);

    temp = (robotcommMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.robotMessageArgument));
    offset += sizeof(robotcommMessage_.robotMessageArgument);

    temp = (robotcommMessage_.warningLevel);
    memcpy(&buf[offset], &temp,
           sizeof(robotcommMessage_.warningLevel));
    offset += sizeof(robotcommMessage_.warningLevel);


    memcpy(&buf[offset], &robotcommMessage_.textmessage_size,
           sizeof(robotcommMessage_.textmessage_size));
    offset += sizeof(robotcommMessage_.textmessage_size);
    if(robotcommMessage_.textmessage_size) {
        memcpy(&buf[offset], robotcommMessage_.textMessage,
               sizeof(char) * robotcommMessage_.textmessage_size);
        offset += robotcommMessage_.textmessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packToMemKeyMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = keyMessage_.timestamp;
    source = keyMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_KEY;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = (keyMessage_.robotMessageCode);
    memcpy(&buf[offset], &temp,
           sizeof(keyMessage_.robotMessageCode));
    offset += sizeof(keyMessage_.robotMessageCode);

    temp = (keyMessage_.robotMessageArgument);
    memcpy(&buf[offset], &temp,
           sizeof(keyMessage_.robotMessageArgument));
    offset += sizeof(keyMessage_.robotMessageArgument);


    memcpy(&buf[offset], &keyMessage_.titleSize,
           sizeof(keyMessage_.titleSize));
    offset += sizeof(keyMessage_.titleSize);
    if (keyMessage_.titleSize) {
        memcpy(&buf[offset], keyMessage_.messageTitle,
               sizeof(char) * keyMessage_.titleSize);
        offset += keyMessage_.titleSize;
    }


    memcpy(&buf[offset], &keyMessage_.textMessage_size,
           sizeof(keyMessage_.textMessage_size));
    offset += sizeof(keyMessage_.textMessage_size);
    if(keyMessage_.textMessage_size) {
        memcpy(&buf[offset], keyMessage_.textMessage,
               sizeof(char) * keyMessage_.textMessage_size);
        offset += keyMessage_.textMessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packToMemLabelMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t source, robot_message_type;
    timestamp = labelMessage_.timestamp;
    source = labelMessage_.source;
    robot_message_type = robotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &source, sizeof(source));
    offset += sizeof(source);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    int temp;
    temp = (labelMessage_.id);
    memcpy(&buf[offset], &temp,
           sizeof(labelMessage_.id));
    offset += sizeof(labelMessage_.id);

    memcpy(&buf[offset], &labelMessage_.textMessage_size,
           sizeof(labelMessage_.textMessage_size));
    offset += sizeof(labelMessage_.textMessage_size);

    if(labelMessage_.textMessage_size) {
        memcpy(&buf[offset], labelMessage_.textMessage,
               sizeof(char) * labelMessage_.textMessage_size);
        offset += labelMessage_.textMessage_size;
    }


    return (offset - offset_);
}

unsigned int RobotState::packToMemProgramMessage(uint8_t *buf, unsigned int offset, uint8_t packToMemage_type) {
    int32_t length = 0;
    uint8_t msgType = messageType::PROGRAM_STATE_MESSAGE;
    switch (packToMemage_type)
    {
        case programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP:
            val_lock_.lock();
            length += RobotState::packToMemGlobalVariablesSetupMessage(buf, offset);
            val_lock_.unlock();
            break;
        default:
            break;
    }

    return (length);
}

unsigned int RobotState::packToMemGlobalVariablesSetupMessage(uint8_t *buf, unsigned int offset) {
    unsigned int offset_ = offset;

    uint64_t timestamp;
    int8_t robot_message_type;
    timestamp = globalVariablesSetupMessage_.timestamp;
    robot_message_type = programType::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP;

    memcpy(&buf[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    memcpy(&buf[offset], &robot_message_type, sizeof(robot_message_type));
    offset += sizeof(robot_message_type);

    unsigned short temp;
    temp = (globalVariablesSetupMessage_.startIndex);
    memcpy(&buf[offset], &temp,
           sizeof(globalVariablesSetupMessage_.startIndex));
    offset += sizeof(globalVariablesSetupMessage_.startIndex);


    memcpy(&buf[offset], &globalVariablesSetupMessage_.variableNames_size,
           sizeof(globalVariablesSetupMessage_.variableNames_size));
    offset += sizeof(globalVariablesSetupMessage_.variableNames_size);
    if (globalVariablesSetupMessage_.variableNames_size) {
        memcpy(&buf[offset], globalVariablesSetupMessage_.variableNames,
               sizeof(char) * globalVariablesSetupMessage_.variableNames_size);
        offset += globalVariablesSetupMessage_.variableNames_size;
    }

    return (offset - offset_);
}


