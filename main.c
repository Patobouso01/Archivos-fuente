/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master sends the
* command packets to the I2C slave to control an user LED.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "i2c_utils.h"
#include "i2c_clcd.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Delay of 1000ms between commands */
#define CMD_TO_CMD_DELAY        (1000UL)

/* Command valid status */
#define STATUS_CMD_DONE         (0x00UL)

/* Packet size */
#define PACKET_SIZE             (3UL)


void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}

void init_hardware(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize the retarget-io */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

//    result = i2c_init() ;
//    handle_error(result) ;

    /* Enable interrupts */
    __enable_irq();
}


int main(void)
{


    init_hardware() ;


    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "HAL: I2C Master CharLCD "
           "****************** \r\n\n");

    I2C_CLCD_Init() ;
    I2C_CLCD_PutString("CARGANDO") ;

    cyhal_gpio_init(P9_2,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1); //BOTON BAJO
    cyhal_gpio_init(P9_1,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1); //BOTON ALTO
    cyhal_gpio_init(P9_4,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true); //LED
    bool      read_val1;
    bool      read_val2;




    for (;;)
    {
        read_val1 = cyhal_gpio_read(P9_2);
        read_val2 = cyhal_gpio_read(P9_1);
        if(read_val1 == 1){
        	I2C_CLCD_Clear();
        	I2C_CLCD_PutString("NIVEL BAJO") ;
        	  cyhal_gpio_write(P9_4, 0);
        	  cyhal_system_delay_ms (200);
        	  cyhal_gpio_write(P9_4, 1);
        	  cyhal_system_delay_ms (200);
        	  cyhal_gpio_write(P9_4, 0);
        	  cyhal_system_delay_ms (200);
        	  cyhal_gpio_write(P9_4, 1);
        	  cyhal_system_delay_ms (200);
          	  cyhal_gpio_write(P9_4, 1);


        }
        if(read_val2 == 1){
        	I2C_CLCD_Clear();
        	I2C_CLCD_PutString("NIVEL ALTO");
        	cyhal_gpio_write(P9_4, 0);
            cyhal_system_delay_ms (200);
        	cyhal_gpio_write(P9_4, 1);
        	cyhal_system_delay_ms (200);
        	cyhal_gpio_write(P9_4, 0);
        	cyhal_system_delay_ms (200);
        	cyhal_gpio_write(P9_4, 1);
            cyhal_system_delay_ms (200);
        	cyhal_gpio_write(P9_4, 0);

        }




    }
}
