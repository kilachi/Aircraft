/******************** (C) COPYRIGHT 2016 Cyrus ***************************
 * 作者		 ：Cyrus
 * 描述    : Aircraft
 * 代码版本：V2.0
 * 时间		 ：2016/5/26
**********************************************************************************/
#include "Aircraft_Config.h"

_flag flag={1,0,0};//lock aready mode

int main(void){
	system_initialize();
	while(1){
		fast_loop();
	}
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
