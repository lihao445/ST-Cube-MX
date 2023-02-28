#include "chassis_task.h"

float Target_1,Target_2,Target_3,Target_4;
float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4;
float	Position_Motor_Target_1,Position_Motor_Target_2,Position_Motor_Target_3,Position_Motor_Target_4;

float Vcx,Vcy,Wcr;
float a,b;

extern RC_ctrl_t rc_ctrl;
extern motor_measure_t motor_can1[8];




void chassis_task(void const * argument)
{
  //wait a time 
  //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
	//���̳�ʼ��
	pid_chassis_init();
	
  //make sure all chassis motor is online,
  //�жϵ��̵���Ƿ�����
  while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
  {
		//chassis task control time  2ms
		//����������Ƽ�� 2ms
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
	

	
  while(1)
  {
		//chassis data update
    //�������ݸ���
		chassis_feedback_update(&motor_can1[8]);
    //set chassis control set-point 
    //���̿���������
    chassis_set_contorl(&motor_can1[8]);
    //chassis control pid calculate
    //���̿���PID����
    chassis_control_loop();
		
		//make sure  one motor is online at least, so that the control CAN message can be received
    //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
							//send control current
							//���Ϳ��Ƶ���
							CAN_cmd_chassis(Target_1,Target_2,Target_3,Target_4);
            }
        }
		

		
		
		
		//os delay
    //ϵͳ��ʱ
	  vTaskDelay(CHASSIS_CONTROL_TIME_MS);	
//    osDelay(2);
  }

}






	/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(motor_measure_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

//    uint8_t i = 0;
//    for (i = 0; i < 4; i++)
//    {
//        //update motor speed, accel is differential of speed PID
//        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
//        chassis_move_update->motor_chassis[i].speed = motor_chassis[i].chassis_motor_measure->speed_rpm;
//        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
//    }
}



	/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(motor_measure_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }
		
		RC_speed_chassis_data();
		
		Speed_Motor_Target_1 = - Vcx + Vcy + Wcr * ( a + b ) ;
		Speed_Motor_Target_2 =   Vcx + Vcy - Wcr * ( a + b ) ;
		Speed_Motor_Target_3 = - Vcx + Vcy - Wcr * ( a + b ) ;
		Speed_Motor_Target_4 =   Vcx + Vcy + Wcr * ( a + b ) ;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(void)
{
	Target_1 = PID_velocity_realize_1(Speed_Motor_Target_1,1);
	Target_2 = PID_velocity_realize_1(Speed_Motor_Target_2,2);
	Target_3 = PID_velocity_realize_1(Speed_Motor_Target_3,3);
	Target_4 = PID_velocity_realize_1(Speed_Motor_Target_4,4);
}





/*
�����ٶȽ���ʽ

V1 = - Vcx + Vcy + Wcr * ( a + b )        (c��Chassis)
V2 =   Vcx + Vcy - Wcr * ( a + b )
V3 = - Vcx + Vcy - Wcr * ( a + b )
V4 =   Vcx + Vcy + Wcr * ( a + b )

*/


void RC_speed_chassis_data(void)
{
	Vcx = 0.707106f * rc_ctrl.rc.ch[2];
	Vcy = 0.707106f * rc_ctrl.rc.ch[3];
	Wcr = 0.707106f * rc_ctrl.rc.ch[4];
	
	Vcx *= 17;
	Vcy *= 17;
	Wcr *= -17;
	
	a =  0.085;           //���ӵ��������ĵ� X�����
	b =  0.065;					 //���ӵ��������ĵ� Y�����
}












