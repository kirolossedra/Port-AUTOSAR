/*
 * Port_Cfg.h
 *
 *  Created on: 5 Feb 2022
 *      Author: kirol
 */

#ifndef PORT_CFG_H_
#define PORT_CFG_H_



#endif /* PORT_CFG_H_ */

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (1U)


/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION   (3U)


/*******************************************************************************
 *                      MODES TYPES                                            *
 *******************************************************************************/
#define PORT_PIN_MODE_ADC		(uint8)0
#define PORT_PIN_MODE_DIO		(uint8)1
#define PORT_PIN_MODE_UART		(uint8)2
#define PORT_PIN_MODE_USB		(uint8)3
#define PORT_PIN_MODE_I2C		(uint8)4
#define PORT_PIN_MODE_CAN		(uint8)5
#define PORT_PIN_MODE_PWM		(uint8)6
#define PORT_PIN_MODE_SSI		(uint8)7
#define PORT_PIN_MODE_QEI		(uint8)8
#define PORT_PIN_MODE_GPT		(uint8)9  
#define PORT_PIN_MODE_NMI		(uint8)10
#define PORT_PIN_MODE_ANALOG_COMP	(uint8)11 
#define PORT_PIN_MODE_CORE		(uint8)12  

/*******************************************************************************
 *                      PORT SETTINGS                                          *
 *******************************************************************************/
#define PORT_ALL_CHANNELS				   (43U)
#define PORTA_START_PIN                    (0U)
#define PORTB_START_PIN                    (8U)
#define PORTC_START_PIN                    (16U)
#define PORTD_START_PIN					   (24U)
#define PORTE_START_PIN					   (32U)
#define PORTF_START_PIN					   (38U)
#define CHANNEL_DEFAULT           {PORT_PIN_OUT,CHANNEL_GPIO_MODE,NO_RES,PIN_CHANGEABLE}
#define SWITCH					  {PORT_PIN_IN,CHANNEL_GPIO_MODE,PUP,PIN_CHANGEABLE}
#define LED						  {PORT_PIN_OUT,CHANNEL_GPIO_MODE,NO_RES,PIN_CHANGEABLE}

/*******************************************************************************
 *                      ADDITIONAL SETTINGS                                          *
 *******************************************************************************/

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

#define PORT_SET_PIN_DIRECTION_API			(STD_ON)

#define PORT_SET_PIN_MODE_API				(STD_ON)

#define PORT_REFRESH_PORT_DIRECTION_API		(STD_OFF)



/*******************************************************************************
 *                      SELECTED  PINS                                         *
 *******************************************************************************/


#define LED_PIN					  (Port_PinType) 39
#define SWITCH_PIN                (Port_PinType) 42



