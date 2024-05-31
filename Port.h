/*
 * port.h
 *
 *  Created on: 5 Feb 2022
 *      Author: kirol
 */

#ifndef PORT_H_
#define PORT_H_




/*assume Sedra vendor id is 1000*/
#define PORT_VENDOR_ID (1000U)

/*PORT module ID*/
#define PORT_MODULE_ID (124U)


/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)


/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION              (1U)
#define PORT_SW_MINOR_VERSION              (0U)
#define PORT_SW_PATCH_VERSION              (1U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)


/* Standard AUTOSAR types */
#include "Std_Types.h"



/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Dio_Cfg.h and Dio.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Dio_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Dio_Cfg.h and Dio.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Dio_Cfg.h does not match the expected version"
#endif


/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/*Service ID for PORT_INIT */
#define PORT_INIT_SID (uint8)0x00

/*Service ID for Port_SetPinDirection  */
#define PORT_SET_PIN_DIRECTION_SID (uint8)0x01

/*Service ID for Port_RefreshPortDirection  */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02

/*Service ID for Port_GetVersionInfo  */
#define PORT_GET_VERSION_INFO_SID (uint8)0x03

/*Service ID for Port_SetPinMode  */
#define PORT_SET_PIN_MODE_SID (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* DET code to report invalid Port Pin ID */

#define PORT_E_PARAM_PIN (uint8)0x0A

/* DET code to report Port Pin Not Configured as Changeable */

#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/* DET code to report Port Iinit Service Called with Wrong Paramter */

#define PORT_E_PARAM_CONFIG (uint8)0x0C

/* DET code to report Invalid mode for channel */

#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D

/* DET code to report mode Unchangeable   */

#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E

/* DET code to report service API called without module initialization */

#define PORT_E_UNINIT 			(uint8)0x0F

/* DET code to report API is called with a NULL Pointer  */

#define PORT_E_PARAM_POINTER	(uint8)0x10


#define PORT_INITIALIZED			(1U)

#define PORT_UNINITIALIZED			(0U)

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
typedef enum{
	NO_RES,
	PUP,
	PDN

}Resistance_Type;

typedef enum{
	PIN_CHANGEABLE,
	PIN_UNCHANGEABLE

}Changeability;
/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;


/* Type definition for Port_PinModeType used by the Port APIs */
typedef uint8 Port_PinModeType;
typedef struct {
	Port_PinDirectionType Direction;
	Port_PinModeType mode;
	Resistance_Type resistor;
	Changeability Change;



}PinConfig;

/* Type definition for Port_PinDirectionType used by the PORT APIs */
typedef enum {
	PORT_PIN_IN,
	PORT_PIN_OUT
}Port_PinDirectionType;

typedef struct {
	PinConfig PIN_SETTINGS[PORT_ALL_CHANNELS];


}Port_ConfigType;
typedef struct 
{
    uint8 port_num; 
    uint8 pin_num; 
    Port_PinDirection direction;
    /*pin direction changeable during runtime*/
    uint8 Pin_direction_Change;
    Port_InternalResistor resistor;
    uint8 initial_value;
    
    /*Pin mode*/
   uint8 pin_mode;
}Port_ConfigPins;



/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Function for Port Initialization  API */

void Port_Init(const Port_ConfigType * ConfigPtr);

/* Function for Port set pin direction   API */

void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);

/* Function for Port Refresh Port Direction   API */

void Port_RefreshPortDirection(void);

/* Function for Port Get Version Info   API */

void Port_GetVersionInfo(Std_VersionInfoType * versioninfo);

/* Function for setting Pin Mode   API */

void Port_SetPinMode(Port_PinType Pin , Port_PinModeType Mode);


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H_ */

