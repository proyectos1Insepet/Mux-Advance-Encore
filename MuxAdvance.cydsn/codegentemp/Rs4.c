/*******************************************************************************
* File Name: Rs4.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Rs4.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Rs4__PORT == 15 && ((Rs4__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Rs4_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None
*  
*******************************************************************************/
void Rs4_Write(uint8 value) 
{
    uint8 staticBits = (Rs4_DR & (uint8)(~Rs4_MASK));
    Rs4_DR = staticBits | ((uint8)(value << Rs4_SHIFT) & Rs4_MASK);
}


/*******************************************************************************
* Function Name: Rs4_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Rs4_DM_STRONG     Strong Drive 
*  Rs4_DM_OD_HI      Open Drain, Drives High 
*  Rs4_DM_OD_LO      Open Drain, Drives Low 
*  Rs4_DM_RES_UP     Resistive Pull Up 
*  Rs4_DM_RES_DWN    Resistive Pull Down 
*  Rs4_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Rs4_DM_DIG_HIZ    High Impedance Digital 
*  Rs4_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Rs4_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Rs4_0, mode);
}


/*******************************************************************************
* Function Name: Rs4_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro Rs4_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Rs4_Read(void) 
{
    return (Rs4_PS & Rs4_MASK) >> Rs4_SHIFT;
}


/*******************************************************************************
* Function Name: Rs4_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 Rs4_ReadDataReg(void) 
{
    return (Rs4_DR & Rs4_MASK) >> Rs4_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Rs4_INTSTAT) 

    /*******************************************************************************
    * Function Name: Rs4_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 Rs4_ClearInterrupt(void) 
    {
        return (Rs4_INTSTAT & Rs4_MASK) >> Rs4_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
