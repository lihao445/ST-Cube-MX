/***      �� ���ֽǶȼ��� ��

 *      ������       ������ + +
 *   �������� �ة��������������� �ة�����++
 *   ��                 ��              
 *   ��       ������       ��++ + + +
 *   ������������������������������������+
 *   ��                 ��+
 *   ��      ���ة�        ��
 *   ��                 ��
 *   ����������         ����������
 *       ��         ��
 *       ��         ��   + +
 *       ��         ��
 *       ��         ��������������������������������
 *       ��                        ��
 *       ��                        ������
 *       ��                        ������
 *       ��                        ��
 *       ������  ��  �����������������Щ�����  ��������  + + + +
 *         �� ���� ����       �� ���� ����
 *         �������ة�����       �������ة�����  + + + +
 *              
 *
 */
#include "helm_wheel.h"

#define ABS(x)	((x>0) ? (x) : (-x))  //ͨ���;���ֵ����

//����ң����ң�˽Ƕ�
float calc_angle_helm_wheel(float set_ch2,float set_ch3){			
			if(ABS(set_ch2)>120||ABS(set_ch3)>120)
            return (atan2(-set_ch2,set_ch3))*180.00f/3.14159f;   //atan2����math.h
			else 
					  return 0.00f;					
}


//������Ӧ��ת����Ȧ������ֹ����
float calc_motor_round_cnt(float angle,float last_angle){
	 static float round_cnt=0.00f;
	 if(angle - last_angle > 260)//��¼����ת��Ȧ��
					round_cnt --;
	 else if(angle - last_angle < -260)
			 		round_cnt ++;
	 return round_cnt;
}


