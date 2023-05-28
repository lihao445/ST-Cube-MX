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
//			HAL_UART_Transmit(&huart2,OTA_buff,1,100);//串口1发送接收buff里的东西	
			HAL_UART_Receive_IT(&huart1,USART_RX_BUF,1);	//重新开启串口3接收中断
					
			if(USART_RX_STA < 1023)	//还可以接收数据
			{
				OTA_buff[USART_RX_STA]=USART_RX_BUF[0];	
				USART_RX_STA++;	//记录接收到的值	
				if(strstr((const char *)OTA_buff,"RMC")!=NULL) 
				{
					GPS_flag =1;//接到RMC
                
				}
			}
		}
}
 
    
//m^n函数
//返回值:m^n次方.
uint32_t NMEA_Pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;     
    while(n--)result*=m;    
    return result;
}
 
void Andly_GPS(Mgps_msg *Mgps,uint8_t *buf)//解析定位信息
{
    uint16_t i=0,j=0,k=0;
    uint8_t *p1,dx;
    uint32_t temp;
    fp32 rs;
    p1 = (uint8_t *)(strstr((const char *)buf,"RMC"));
//    char *json_buff;
    if(buf[0]==0) buf[0]=1;//防止有时候首字节为0 出现解析失败
    buf = (uint8_t *)(strstr((const char *)buf,"RMC"));
        if(strstr((const char *)buf,"RMC")!=NULL)
        {
            buf = (uint8_t *)(strstr((const char *)buf,",A"));    
            
            i=NMEA_Comma_Pos(buf,2);//第N个逗号所在位置
            j=i;k=i;
            for(;i<i+10;i++)
            {
                if(buf[i]==',') break;
                Mgps->latitude[i-j]=buf[i];//维度
                if(buf[i]!='.')Mgps->lat[i-k]=buf[i];//维度
                else k=k+1;
            }
            
            i=NMEA_Comma_Pos(buf,1);//第N个逗号所在位置
            j=i;k=i;
            for(;i<i+20;i++)
            {
                if(buf[i]==',') break;
                Mgps->Location[i-j]=buf[i];//定位A V
                if(buf[i]!='.')Mgps->lat[i-k]=buf[i];//维度
                else k=k+1;
            }
            
            i=NMEA_Comma_Pos(buf,4);//第N个逗号所在位置
            j=i;k=i;
            for(;i<i+11;i++)
            {
                if(buf[i]==',') break;
                Mgps->longitude[i-j]=buf[i];//经度
                if(buf[i]!='.')Mgps->lon[i-k]=buf[i];//经度
                else k=k+1;
            }
            
        }
    
}
uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx)
{                 
    uint8_t *p=buf;
    while(cx)
    {         
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;     
}
 
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(uint8_t *buf,uint8_t*dx)
{
    uint8_t *p=buf;
    uint32_t ires=0,fres=0;
    uint8_t ilen=0,flen=0,i;
    uint8_t mask=0;
    int res;
    while(1) //得到整数和小数的长度
    {
        if(*p=='-'){mask|=0X02;p++;}//是负数
        if(*p==','||(*p=='*'))break;//遇到结束了
        if(*p=='.'){mask|=0X01;p++;}//遇到小数点了
        else if(*p>'9'||(*p<'0'))    //有非法字符
        {    
            ilen=0;
            flen=0;
            break;
        }    
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;    //去掉负号
    for(i=0;i<ilen;i++)    //得到整数部分数据
    {  
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;    //最多取5位小数
    *dx=flen;             //小数点位数
    for(i=0;i<flen;i++)    //得到小数部分数据
    {  
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    } 
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;           
    return res;
}
 
