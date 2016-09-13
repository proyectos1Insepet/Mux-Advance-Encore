/*******************************************************************************
* File Name: Rs1.c  
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
#include "Rs1.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Rs1__PORT == 15 && ((Rs1__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Rs1_Write
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
void Rs1_Write(uint8 value) 
{
    uint8 staticBits = (Rs1_DR & (uint8)(~Rs1_MASK));
    Rs1_DR = staticBits | ((uint8)(value << Rs1_SHIFT) & Rs1_MASK);
}


/*******************************************************************************
* Function Name: Rs1_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Rs1_DM_STRONG     Strong Drive 
*  Rs1_DM_OD_HI      Open Drain, Drives High 
*  Rs1_DM_OD_LO      Open Drain, Drives Low 
*  Rs1_DM_RES_UP     Resistive Pull Up 
*  Rs1_DM_RES_DWN    Resistive Pull Down 
*  Rs1_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Rs1_DM_DIG_HIZ    High Impedance Digital 
*  Rs1_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Rs1_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Rs1_0, mode);
}


/*******************************************************************************
* Function Name: Rs1_Read
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
*  Macro Rs1_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Rs1_Read(void) 
{
    return (Rs1_PS & Rs1_MASK) >> Rs1_SHIFT;
}


/*******************************************************************************
* Function Name: Rs1_ReadDataReg
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
uint8 Rs1_ReadDataReg(void) 
{
    return (Rs1_DR & Rs1_MASK) >> Rs1_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Rs1_INTSTAT) 

    /*******************************************************************************
    * Function Name: Rs1_ClearInterrupt
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
    uint8 Rs1_ClearInterrupt(void) 
    {
        return (Rs1_INTSTAT & Rs1_MASK) >> Rs1_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
