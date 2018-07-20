#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include "./include/global.h"
#include <boost/interprocess/mapped_region.hpp>
#include "robot_state.h"


using namespace boost::asio;
using namespace boost::interprocess;
using namespace std;

#define MEM_FN(x)       boost::bind(&self_type::x, shared_from_this())
#define MEM_FN1(x,y)    boost::bind(&self_type::x, shared_from_this(),y)
#define MEM_FN2(x,y,z)  boost::bind(&self_type::x, shared_from_this(),y,z)

class server
{
    typedef server this_type;
    typedef ip::tcp::acceptor acceptor_type;
    typedef ip::tcp::endpoint endpoint_type;
    typedef ip::tcp::socket socket_type;
    typedef ip::address address_type;
    typedef boost::shared_ptr<socket_type> sock_ptr;
    typedef vector<char> buffer_type;


private:
    buffer_type m_buf;


public:
    server()
    {

    }

    void run(){
        //Open a message queue.
        message_queue *mq1;
                mq1 = new message_queue(open_only        //only create
                        ,"message_queue_send"  //name
                );

        message_queue *mq2;
        mq2 = new message_queue(open_only        //only create
                ,"message_queue_recv"  //name
        );



        unsigned int priority;
        message_queue::size_type recvd_size;


        int code = 0;
        string strCode;
        while (1) {
            char number[256] = {0};
            if (mq1->try_receive(number, sizeof(number), recvd_size, priority)) {
                cout<<number<<endl;
                if (recvd_size > 0) {
                    usleep(100000);
                    strCode = "CODE_0";// + to_string(code%10000);
                    mq2->send(strCode.c_str(), strCode.length(), 0);
                    cout << strCode << endl;
                    code++;
                }

            }


        }
    }

    void setRobotState() {
        shared_memory_object m_shm;
        mapped_region m_region;
        m_shm = shared_memory_object(open_only, "RobotState", read_write);

        m_shm.truncate(SHARE_MEMORY_SIZE);
        m_region = mapped_region(m_shm, read_write);
        memset(m_region.get_address(), 0, m_region.get_size());

        condition_variable m_cv;
        RobotState robotState(m_cv);

        // robot_mode_data
        robotState.robot_mode_.timestamp = 123456;
        robotState.robot_mode_.isRobotConnected = true;
        robotState.robot_mode_.isRealRobotEnabled = true;
        robotState.robot_mode_.isPowerOnRobot = true;
        robotState.robot_mode_.isEmergencyStopped = true;
        robotState.robot_mode_.isProtectiveStopped = true;
        robotState.robot_mode_.isProgramRunning = false;
        robotState.robot_mode_.isProgramPaused = true;
        robotState.robot_mode_.robotMode = 1;
        robotState.robot_mode_.controlMode = 2;
        robotState.robot_mode_.targetSpeedFraction = 0.4;
        robotState.robot_mode_.speedScaling = 0.5;
        robotState.robot_mode_.targetSpeedFractionLimit = 1.2;

        // joint_data
        robotState.joint_data_.q_actual_1 = 0.3;
        robotState.joint_data_.q_target_1 = 0.2;
        robotState.joint_data_.qd_actual_1 = 0.1;
        robotState.joint_data_.I_actual_1 = 1.2;
        robotState.joint_data_.T_micro_7 = 0.4;
        robotState.joint_data_.jointMode_7 = 5;

        // cartesian_info
        robotState.cartesian_info_.X = 1.66;
        robotState.cartesian_info_.Rn = 33.22;

        // masterboard_data
        robotState.mb_data_.digitalInputBits = 3;
        robotState.mb_data_.digitalOutputBits = 5;
        robotState.mb_data_.euromap67InterfaceInstalled = 2;
        robotState.mb_data_.euromapCurrent = 23.6;
        robotState.mb_data_.threePositionEnablingDeviceInput = 5;

        // configuration_data
        robotState.configuration_data_.jointMinLimit_1 = 9.2;
        robotState.configuration_data_.jointMaxLimitt_6 = 9.1;
        robotState.configuration_data_.DHtheta_6 = 1.2;
        robotState.configuration_data_.masterboardVersion = 1;
        robotState.configuration_data_.robotSubType = 6;

        // additional_info
        robotState.additional_info_.freedriveButtonPressed = true;

        //version_message
        robotState.version_msg_.major_version = 8;
        robotState.version_msg_.minor_version = 1;
        robotState.version_msg_.svn_revision = 3;
        robotState.version_msg_.project_name_size = 4;
        ZX_MEMCPY(robotState.version_msg_.project_name, "test", 4);
        robotState.version_msg_.build_date_size = 4;
        ZX_MEMCPY(robotState.version_msg_.build_date, "date", 4);

        // safetyModeMessage
        robotState.safetyModeMessage_.timestamp = 11;
        robotState.safetyModeMessage_.source = 2;
        robotState.safetyModeMessage_.robot_message_type = 2;
        robotState.safetyModeMessage_.robotMessageCode = 1;
        robotState.safetyModeMessage_.robotMessageArgument = 33;
        robotState.safetyModeMessage_.safetyModeType = 2;
        robotState.safetyModeMessage_.textmessage_size = 6;
        ZX_MEMCPY(robotState.safetyModeMessage_.textMessage, "safety", 6);

        //robotcommMessage
        robotState.robotcommMessage_.robotMessageCode = 2;
        robotState.robotcommMessage_.textmessage_size = 4;
        ZX_MEMCPY(robotState.robotcommMessage_.textMessage, "comm", 4);

        // keyMessage
        robotState.keyMessage_.robotMessageArgument = 22;
        robotState.keyMessage_.titleSize = 5;
        ZX_MEMCPY(robotState.keyMessage_.messageTitle, "title", 5);
        robotState.keyMessage_.textMessage_size = 4;
        ZX_MEMCPY(robotState.keyMessage_.textMessage, "text", 4);

        //labelMessage
        robotState.labelMessage_.source = 2;
        robotState.labelMessage_.id = 1;
        robotState.labelMessage_.textMessage_size = 4;
        ZX_MEMCPY(robotState.labelMessage_.textMessage, "text", 4);

        //globalVariablesSetupMessage
        robotState. globalVariablesSetupMessage_.startIndex = 3;
        robotState. globalVariablesSetupMessage_.variableNames_size = 6;
        ZX_MEMCPY(robotState. globalVariablesSetupMessage_.variableNames, "global", 6);

        uint8_t * data = new uint8_t[SHARE_MEMORY_SIZE];
        robotState.packToMem(data);

        memcpy(m_region.get_address(), data, m_region.get_size());

        delete[] data;
        data = nullptr;

    }

};




int main()
{
    try
    {
        cout<<"Server start."<<endl;
        server srv;
        srv.setRobotState();
        srv.run();
    }
    catch (std::exception &e)
    {
        cout<<e.what()<<endl;
    }

    return 0;
}