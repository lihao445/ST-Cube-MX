/***      《 舵轮角度计算 》

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │              
 *   │       ───       │++ + + +
 *   ███████───███████│+
 *   │                 │+
 *   │      ─┴─        │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *              
 *
 */
#include "helm_wheel.h"

#define ABS(x)	((x>0) ? (x) : (-x))  //通用型绝对值函数

//计算遥控器遥杆角度
float calc_angle_helm_wheel(float set_ch2,float set_ch3){			
			if(ABS(set_ch2)>120||ABS(set_ch3)>120)
            return (atan2(-set_ch2,set_ch3))*180.00f/3.14159f;   //atan2函数math.h
			else 
					  return 0.00f;					
}


//计算电机应该转动的圈数，防止跳变
float calc_motor_round_cnt(float angle,float last_angle){
	 static float round_cnt=0.00f;
	 if(angle - last_angle > 260)//记录轮子转动圈数
					round_cnt --;
	 else if(angle - last_angle < -260)
			 		round_cnt ++;
	 return round_cnt;
}


