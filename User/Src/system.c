#include "system.h"

void Delay_Us(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=10;  //自己定义
      while(i--) ;    
   }
}
//毫秒级的延时
void Delay_Ms(uint16_t time)
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}