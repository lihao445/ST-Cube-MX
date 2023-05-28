#include "gnss.h"
 
 uint8_t USART_RX_BUF[1];
 bool_t GPS_flag;
 uint16_t USART_RX_STA;
 uint8_t OTA_buff[1023];
 Mgps_msg Mgps;
 
 
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{		 
	if(huart->Instance == USART1)
		{
//			HAL_UART_Transmit(&huart2,OTA_buff,1,100);//����1���ͽ���buff��Ķ���	
			HAL_UART_Receive_IT(&huart1,USART_RX_BUF,1);	//���¿�������3�����ж�
					
			if(USART_RX_STA < 1023)	//�����Խ�������
			{
				OTA_buff[USART_RX_STA]=USART_RX_BUF[0];	
				USART_RX_STA++;	//��¼���յ���ֵ	
				if(strstr((const char *)OTA_buff,"RMC")!=NULL) 
				{
					GPS_flag =1;//�ӵ�RMC
                
				}
			}
		}
}
 
    
//m^n����
//����ֵ:m^n�η�.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;     
    while(n--)result*=m;    
    return result;
}
 
void Andly_GPS(Mgps_msg *Mgps,uint8_t *buf)//������λ��Ϣ
{
    uint16_t i=0,j=0,k=0;
    uint8_t *p1,dx;
    uint32_t temp;
    fp32 rs;
    p1 = (uint8_t *)(strstr((const char *)buf,"RMC"));
//    char *json_buff;
    if(buf[0]==0) buf[0]=1;//��ֹ��ʱ�����ֽ�Ϊ0 ���ֽ���ʧ��
    buf = (uint8_t *)(strstr((const char *)buf,"RMC"));
        if(strstr((const char *)buf,"RMC")!=NULL)
        {
            buf = (uint8_t *)(strstr((const char *)buf,",A"));    
            
            i=NMEA_Comma_Pos(buf,2);//��N����������λ��
            j=i;k=i;
            for(;i<i+10;i++)
            {
                if(buf[i]==',') break;
                Mgps->latitude[i-j]=buf[i];//ά��
                if(buf[i]!='.')Mgps->lat[i-k]=buf[i];//ά��
                else k=k+1;
            }
            
            i=NMEA_Comma_Pos(buf,1);//��N����������λ��
            j=i;k=i;
            for(;i<i+20;i++)
            {
                if(buf[i]==',') break;
                Mgps->Location[i-j]=buf[i];//��λA V
                if(buf[i]!='.')Mgps->lat[i-k]=buf[i];//ά��
                else k=k+1;
            }
            
            i=NMEA_Comma_Pos(buf,4);//��N����������λ��
            j=i;k=i;
            for(;i<i+11;i++)
            {
                if(buf[i]==',') break;
                Mgps->longitude[i-j]=buf[i];//����
                if(buf[i]!='.')Mgps->lon[i-k]=buf[i];//����
                else k=k+1;
            }
            
        }
    
}
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{                 
    uint8_t *p=buf;
    while(cx)
    {         
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//����'*'���߷Ƿ��ַ�,�򲻴��ڵ�cx������
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;     
}
 
//strת��Ϊ����,��','����'*'����
//buf:���ִ洢��
//dx:С����λ��,���ظ����ú���
//����ֵ:ת�������ֵ
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
    uint8_t *p=buf;
    uint32_t ires=0,fres=0;
    uint8_t ilen=0,flen=0,i;
    uint8_t mask=0;
    int res;
    while(1) //�õ�������С���ĳ���
    {
        if(*p=='-'){mask|=0X02;p++;}//�Ǹ���
        if(*p==','||(*p=='*'))break;//����������
        if(*p=='.'){mask|=0X01;p++;}//����С������
        else if(*p>'9'||(*p<'0'))    //�зǷ��ַ�
        {    
            ilen=0;
            flen=0;
            break;
        }    
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;    //ȥ������
    for(i=0;i<ilen;i++)    //�õ�������������
    {  
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;    //���ȡ5λС��
    *dx=flen;             //С����λ��
    for(i=0;i<flen;i++)    //�õ�С����������
    {  
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    } 
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;           
    return res;
}
 
