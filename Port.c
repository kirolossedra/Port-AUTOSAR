/*
 * Port.c
 *
 *  Created on: 6 Feb 2022
 *      Author: kirol
 */


#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"
#include "Det.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif




STATIC uint8 PORT_STATUS = PORT_UNINITIALIZED;
STATIC Port_ConfigPins * ptr = NULL_PTR;



/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr)
{


 #if (PORT_DEV_ERROR_DETECT == STD_ON)
	if (ConfigPtr == NULL_PTR) {
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID, PORT_INIT_ID,
		PORT_E_PARAM_CONFIG);
	} else
#endif
        {  
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    
    PortConf = ConfigPtr->Pins;     /*points to the 1st address of the array of structure*/
    Port_Status = PORT_INITIALIZED;
    uint8 Pin_Index;              /* index to configure each pin (43)*/
    
    
 for (Pin_Index = 0; Pin_Index < PORT_NUMBER_OF_PORT_PINS ; Pin_Index++) 
                {
    switch(PortConf[Pin_Index].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break; 
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1<<PortConf[Pin_Index].port_num);
    delay = SYSCTL_REGCGC2_REG;
    
    if( ((PortConf[Pin_Index].port_num== 3) && (PortConf[Pin_Index].port_num== 7)) || ((PortConf[Pin_Index].port_num == 5) && (PortConf[Pin_Index].port_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , PortConf[Pin_Index].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (PortConf[Pin_Index].port_num == 2) && (PortConf[Pin_Index].pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
       
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    /********************************************Direction,initial_value,resistor**************************************************/
    
    if(PortConf[Pin_Index].direction == PORT_PIN_OUT)
    {
	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PortConf[Pin_Index].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(PortConf[Pin_Index].initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PortConf[Pin_Index].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PortConf[Pin_Index].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(PortConf[Pin_Index].direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PortConf[Pin_Index].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(PortConf[Pin_Index].resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PortConf[Pin_Index].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(PortConf[Pin_Index].resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PortConf[Pin_Index].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PortConf[Pin_Index].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PortConf[Pin_Index].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
         }
    else
    {
        /* Do Nothing */
    }
    
    /*********************************************Check Mode**************************************************/
    if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_DIO)
    {
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PortConf[Pin_Index].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (PortConf[Pin_Index].pin_num * 4));     /* Clear the PMCx bits for this pin */   
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
          }
    
          /*********************************************************************************/   
               
     if(PortConf[Pin_Index].pin_mode == PORT_PIN_MODE_UART)
        { 
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PortConf[Pin_Index].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);               /* ENable Alternative function for this pin*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);      /*Enable Digital*/ 
          if(Pin_Index==20||Pin_Index==21) /*PC4 ,PC5*/
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (PortConf[Pin_Index].pin_num * 4));
          }
          else
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (PortConf[Pin_Index].pin_num * 4));
          }
         
        }
      /*************************************************/  
        
     
        else if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_ADC)
        {  
          /*Should i enable RCGCADC and PLL ??*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);               /* ENable Alternative function for this pin*/
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PortConf[Pin_Index].pin_num);            /*SEt as an input pin*/
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);  /*Disable Digital mode */      
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PortConf[Pin_Index].pin_num);   /*Enable analog functionality on this pin */
        }
         /**************************************/
        else if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_CAN)
         {
           
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);               /* ENable Alternative function for this pin*/
          if(Pin_Index==38 ||Pin_Index==41)/*PF0,PF3*/
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (PortConf[Pin_Index].pin_num * 4));
          }
          else
          {
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (PortConf[Pin_Index].pin_num * 4));         
         }
         /*Tx(output) pin or Rx(input)*/
         }
         /********************************************/
        else if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_SSI)
        {    /*RCGCSSI register???*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num); /*set as Digital*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PortConf[Pin_Index].pin_num);           /*PULLUP */
          if(Pin_Index==24 ||Pin_Index==25 ||Pin_Index==26 ||Pin_Index==27)/*PD0,PD1,PD2,PD3*/
          { *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (PortConf[Pin_Index].pin_num * 4));  
          }
          else
          {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (PortConf[Pin_Index].pin_num * 4)); 
         
        }
        }
     /*******************************************/
       else if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_I2C)
       {
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_OPEN_DRAIN) , PortConf[Pin_Index].pin_num);  /*OPen drain select*/
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (PortConf[Pin_Index].pin_num * 4));
          
       }
      /****************************************/
     else if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_PWM)
     {   
       
       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
       if(Pin_Index==12 ||Pin_Index==13 ||Pin_Index==14 ||Pin_Index==15||Pin_Index==20 ||Pin_Index==21 ||Pin_Index==24||Pin_Index==25|Pin_Index==36 ||Pin_Index==37)/*PB4--PB7 , PC4,5  ,PD0,1 ,PE4,5*/
       {
       *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (PortConf[Pin_Index].pin_num * 4));
      }
       else
       {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005<< (PortConf[Pin_Index].pin_num * 4));
       }
     }
       /**********************************/
       else 
         if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_USB)
       {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (PortConf[Pin_Index].pin_num * 4));
       }           
               
                else 
         if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_QEI) 
         {   
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006<< (PortConf[Pin_Index].pin_num * 4));
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         }
  
      else 
         if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_GPT) 
         {   
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);               /* ENable Alternative function for this pin*/
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007<< (PortConf[Pin_Index].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         } 
      else
        if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_NMI) 
         {    
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (PortConf[Pin_Index].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         }  
  
        if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_ANALOG_COMP) 
         {    
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
            if(Pin_Index==20 ||Pin_Index==21 ||Pin_Index==22 ||Pin_Index==23 )  /*PC4,5,6,7 analog pins  C1-,C1+,C0+,C0-*/
            { 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PortConf[Pin_Index].pin_num);   /*Enable analog functionality on this pin */
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);  /*Disable digital mode*/
            } else
            {
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000009<< (PortConf[Pin_Index].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
          
            }  
         }
       if(PortConf[Pin_Index].pin_mode==PORT_PIN_MODE_CORE) 
         {    
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PortConf[Pin_Index].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000E<< (PortConf[Pin_Index].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PortConf[Pin_Index].pin_num);
         }  
       
       
    
               
                } 
        }      
}

/************************************************************************************
* Service Name: Port Set Pin Direction
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin and desired direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/
#if(PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
if(Pin<0 || Pin>PORT_ALL_CHANNELS){
	Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		     PORT_E_PARAM_PIN);

}
if (PORT_STATUS == PORT_UNINITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		PORT_E_UNINIT);

	}
if(ptr[Pin].Change !=PIN_CHANGEABLE){
	Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
			PORT_E_MODE_UNCHANGEABLE);

}
else{
#endif
	if(PORTA_START_PIN<=Pin<PORTB_START_PIN){
		if(PORT_PIN_OUT==Direction)
		ACCESS(GPIO_PORTA_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTA_START_PIN-Pin);
		if(PORT_PIN_IN==Direction)
		ACCESS(GPIO_PORTA_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTA_START_PIN-Pin));

	}
	if(PORTB_START_PIN<=Pin<PORTC_START_PIN){
		if(PORT_PIN_OUT==Direction)
		ACCESS(GPIO_PORTB_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTB_START_PIN-Pin);
		if(PORT_PIN_IN==Direction)
		ACCESS(GPIO_PORTB_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTB_START_PIN-Pin));

	}
	if(PORTC_START_PIN<=Pin<PORTD_START_PIN){
		if(PORT_PIN_OUT==Direction)
		ACCESS(GPIO_PORTC_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTC_START_PIN-Pin);
		if(PORT_PIN_IN==Direction)
		ACCESS(GPIO_PORTC_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTC_START_PIN-Pin));

	}
	if(PORTD_START_PIN<=Pin<PORTE_START_PIN){
			if(PORT_PIN_OUT==Direction)
			ACCESS(GPIO_PORTD_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTD_START_PIN-Pin);
			if(PORT_PIN_IN==Direction)
			ACCESS(GPIO_PORTD_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTD_START_PIN-Pin));

		}
	if(PORTE_START_PIN<=Pin<PORTF_START_PIN){
				if(PORT_PIN_OUT==Direction)
				ACCESS(GPIO_PORTE_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTE_START_PIN-Pin);
				if(PORT_PIN_IN==Direction)
				ACCESS(GPIO_PORTD_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTD_START_PIN-Pin));

			}
	else{

				if(PORT_PIN_OUT==Direction)
				ACCESS(GPIO_PORTF_BASE_ADDRESS, PORT_DIR_REG_OFFSET)|=1<<(PORTF_START_PIN-Pin);
				if(PORT_PIN_IN==Direction)
				ACCESS(GPIO_PORTF_BASE_ADDRESS, PORT_DIR_REG_OFFSET)&=~(1<<(PORTF_START_PIN-Pin));
	}

}
}
#endif
/************************************************************************************
* Service Name: Port Refresh Pin Direction
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Pin and desired direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/
#if(PORT_REFERSH_PORT_DIRECTION_API == STD_ON)
void Port_RefreshPortDirection(void){
#if (PORT_DEV_ERROR_DETECT == STD_ON)
/* check if the Port driver is initialized*/
	if (Port_Status == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_ID,
		PORT_E_UNINIT);

	}
	for(itr i=0; i<PORT_ALL_CHANNELS; i++){
		Port_SetPinMode((Port_PinType)i,ptr[Pin].Direction);
	}
#endif


}
#endif
/************************************************************************************
* Service Name: Port Get Version Info
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Pin and desired direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/

#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif
/************************************************************************************
* Service Name: Port Set Pin Mode
* Service ID[hex]: 0x05
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Pin and desired direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/
#if ( PORT_SET_PIN_MODE_API==STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )

{ 
  if ( PORT_STATUS == PORT_UNINITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_ID,
		PORT_E_UNINIT);
                
	}
	else
	{

	}
 
  /* Incorrect Port Pin ID passed*/
   if (Pin >= PORT_ALL_CHANNELS )
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_ID,
		PORT_E_PARAM_PIN);
	} else
	{

	}
         /* Incorrect Port Pin Mode passed*/
    if ((Mode > PORT_PIN_MODE_CORE) || (Mode < PORT_PIN_MODE_ADC ))
	{
		
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SETPINMODE_ID ,
		     PORT_E_PARAM_INVALID_MODE);
        } else
        {
        }
  
    /* check if Port Pin mode not configured as changeable */
	if( PORT_PIN_MODE_CHANGEABLE==STD_OFF)
        {
          Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SETPINMODE_ID ,
		     PORT_E_MODE_UNCHANGEABLE);
        }
   else
   {}
  
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  switch(ptr[Pin].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break; 
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }


        
       if(Mode == PORT_PIN_MODE_UART)
        { 
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PortConf[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);               /* ENable Alternative function for this pin*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);      /*Enable Digital*/ 
          if(Pin==20||Pin==21) /*PC4 ,PC5*/
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr[Pin].pin_num * 4));
          }
          else
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr[Pin].pin_num * 4));
          }
           
        }
      /*************************************************/  
        
      else if(Mode==PORT_PIN_MODE_DIO)
        {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) ,ptr[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ptr[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */ 
        }
       /********************************************/ 
        else if(Mode==PORT_PIN_MODE_ADC)
        {  
          /*Should i enable RCGCADC and PLL ??*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);               /* ENable Alternative function for this pin*/
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr[Pin].pin_num);            /*SEt as an input pin*/
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);  /*Disable Digital mode */      
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) ,ptr[Pin].pin_num);   /*Enable analog functionality on this pin */
        }
         /**************************************/
        else if(Mode==PORT_PIN_MODE_CAN)
         {        
           
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);               /* ENable Alternative function for this pin*/
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
           if(Pin==38 ||Pin==41)/*PF0,PF3*/
          {
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr[Pin].pin_num * 4));
          }
          else
          {
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (ptr[Pin].pin_num * 4));         
         }
         /*Tx(output) pin or Rx(input)*/
         }
         /********************************************/
        else if(Mode==PORT_PIN_MODE_SSI)
        {    /*RCGCSSI register???*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num); /*set as Digital*/
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , ptr[Pin].pin_num);           /*PULLUP */
          if(Pin==24 ||Pin==25 ||Pin==26 ||Pin==27)/*PD0,PD1,PD2,PD3*/
          { *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (ptr[Pin].pin_num * 4));  
          }
          else
          {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (ptr[Pin].pin_num * 4)); 
         
        }
        }
     /*******************************************/
       else if(Mode==PORT_PIN_MODE_I2C)
       {
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) ,ptr[Pin].pin_num);
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_OPEN_DRAIN) , ptr[Pin].pin_num);  /*OPen drain select*/
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (ptr[Pin].pin_num * 4));
          
       }
      /****************************************/
     else if(Mode==PORT_PIN_MODE_PWM)
     {   
       
       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
       if(Pin==12 ||Pin==13 ||Pin==14 ||Pin==15||Pin==20 ||Pin==21 ||Pin==24||Pin==25|Pin==36 ||Pin==37)/*PB4--PB7 , PC4,5  ,PD0,1 ,PE4,5*/
       {
       *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (ptr[Pin].pin_num * 4));
       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
      }
       else
       {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005<< (ptr[Pin].pin_num * 4));
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
       }}
       /**********************************/
       else 
         if(Mode==PORT_PIN_MODE_USB)
       {
         *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr[Pin].pin_num * 4));
       }
  
       else 
         if(Mode==PORT_PIN_MODE_QEI) 
         {   
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000006<< (ptr[Pin].pin_num * 4));
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
         }
  
      else 
         if(Mode==PORT_PIN_MODE_GPT) 
         {   
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) ,ptr[Pin].pin_num);               /* ENable Alternative function for this pin*/
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007<< (ptr[Pin].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
         } 
      else
        if(Mode==PORT_PIN_MODE_NMI) 
         {    
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008<< (ptr[Pin].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
         }  
  
        if(Mode==PORT_PIN_MODE_ANALOG_COMP) 
         {    
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
            if(Pin==20 ||Pin==21 ||Pin==22 ||Pin==23 )  /*PC4,5,6,7 analog pins  C1-,C1+,C0+,C0-*/
            { 
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ptr[Pin].pin_num);   /*Enable analog functionality on this pin */
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);  /*Disable digital mode*/
            } else
            {
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000009<< (ptr[Pin].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
          
            }  
         }
       if(Mode==PORT_PIN_MODE_CORE) 
         {    
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ptr[Pin].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000E<< (ptr[Pin].pin_num * 4));
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ptr[Pin].pin_num);
         }  
  
        
}
#endif

