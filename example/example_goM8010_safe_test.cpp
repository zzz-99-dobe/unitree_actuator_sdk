#include <unistd.h>
#include <iostream>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"


struct JointCommand
{
    float q;
    float dq;
    float kp;
    float kd;
    float tau;
};


struct JointState
{
    float q;
    float dq;
    float tau;

    int temp;
    int error;

    bool online;
};


struct JointConfig
{
    int motor_id;
    
    float gear_ratio;
    float zero_offset;

    int direction;
};


bool updateJoint(SerialPort& serial,
                 const JointConfig& config,
                 const JointCommand& joint_cmd,
                 JointState& joint_state)
{
    MotorCmd motor_cmd;
    MotorData motor_data;

    motor_cmd.motorType = MotorType::GO_M8010_6;
    motor_data.motorType = MotorType::GO_M8010_6;

    motor_cmd.mode =
        queryMotorMode(MotorType::GO_M8010_6,
                       MotorMode::FOC);

    motor_cmd.id = config.motor_id;

    float r = config.gear_ratio;

    motor_cmd.q =r * (config.direction * joint_cmd.q + config.zero_offset);

    motor_cmd.dq = config.direction *joint_cmd.dq * r;

    motor_cmd.kp =joint_cmd.kp / (r * r);

    motor_cmd.kd =joint_cmd.kd / (r * r);

    motor_cmd.tau =config.direction *joint_cmd.tau / r;

    bool ok = serial.sendRecv(&motor_cmd, &motor_data);

    joint_state.online = ok && motor_data.correct;

// 先判断这次反馈能不能用
    if (!joint_state.online)
{
    return false;
}

// 确认有效以后，才更新状态
     joint_state.q =config.direction *motor_data.q;

     joint_state.dq =config.direction *motor_data.dq;

     joint_state.tau =config.direction  * motor_data.tau * r;

     joint_state.temp =motor_data.temp;

     joint_state.error =motor_data.merror;

     return true;
}


int main()
{
    SerialPort serial("/dev/ttyUSB0");

    JointConfig joint_cfg[2]{};

    // Joint0
    joint_cfg[0].motor_id = 1;
    joint_cfg[0].gear_ratio =
        queryGearRatio(MotorType::GO_M8010_6);

    joint_cfg[0].zero_offset = 0.560795f;
    joint_cfg[0].direction   = -1;


    // Joint1
    joint_cfg[1].motor_id = 3;
    joint_cfg[1].gear_ratio =
        queryGearRatio(MotorType::GO_M8010_6);

    joint_cfg[1].zero_offset = 0.0785168f;
    joint_cfg[1].direction   =-1;



    JointCommand joint_cmd[2]{};
    JointState joint_state[2]{};


    // 默认安全命令
    for (int i = 0; i < 2; i++)
    {
        joint_cmd[i].q   = 0.0f;
        joint_cmd[i].dq  = 0.0f;
        joint_cmd[i].kp  = 0.0f;
        joint_cmd[i].kd  = 0.0f;
        joint_cmd[i].tau = 0.0f;
    }

    while (true)
    {
        
    bool ok0 =
        updateJoint(serial,
                    joint_cfg[0],
                    joint_cmd[0],
                    joint_state[0]);

    bool ok1 =
        updateJoint(serial,
                    joint_cfg[1],
                    joint_cmd[1],
                    joint_state[1]);


    std::cout
        << "J0 online=" << ok0
        << " q=" << joint_state[0].q
	<< " dq="<< joint_state[0].dq
        << " | "
        << "J1 online=" << ok1
        << " q=" << joint_state[1].q
	<< " dq="<< joint_state[1].dq
        << std::endl;


    usleep(10000);
}         
}
