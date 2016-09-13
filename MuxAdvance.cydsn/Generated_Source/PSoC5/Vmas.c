/*******************************************************************************
* File Name: Vmas.c  
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
#include "Vmas.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Vmas__PORT == 15 && ((Vmas__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Vmas_Write
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
void Vmas_Write(uint8 value) 
{
    uint8 staticBits = (Vmas_DR & (uint8)(~Vmas_MASK));
    Vmas_DR = staticBits | ((uint8)(value << Vmas_SHIFT) & Vmas_MASK);
}


/*******************************************************************************
* Function Name: Vmas_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Vmas_DM_STRONG     Strong Drive 
*  Vmas_DM_OD_HI      Open Drain, Drives High 
*  Vmas_DM_OD_LO      Open Drain, Drives Low 
*  Vmas_DM_RES_UP     Resistive Pull Up 
*  Vmas_DM_RES_DWN    Resistive Pull Down 
*  Vmas_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Vmas_DM_DIG_HIZ    High Impedance Digital 
*  Vmas_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Vmas_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Vmas_0, mode);
}


/*******************************************************************************
* Function Name: Vmas_Read
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
*  Macro Vmas_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Vmas_Read(void) 
{
    return (Vmas_PS & Vmas_MASK) >> Vmas_SHIFT;
}


/*******************************************************************************
* Function Name: Vmas_ReadDataReg
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
uint8 Vmas_ReadDataReg(void) 
{
    return (Vmas_DR & Vmas_MASK) >> Vmas_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Vmas_INTSTAT) 

    /*******************************************************************************
    * Function Name: Vmas_ClearInterrupt
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
    uint8 Vmas_ClearInterrupt(void) 
    {
        return (Vmas_INTSTAT & Vmas_MASK) >> Vmas_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
