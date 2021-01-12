/******************************************************************************
* File Name: main.c
*
* Description: This example demonstrates the Cypress' QSPI F-RAM access
*              using PSoC 6 QSPI HAL (cyhal_qspi) API
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/******************************************************************************
* Header files
******************************************************************************/
#include "serial_fram_api.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PACKET_SIZE_BYTES       (64u)        /* Memory Read/Write size */
#define LED_TOGGLE_DELAY_MSEC   (1000u)      /* LED blink delay */
#define MODE_BYTE               (0x00)       /* Value of Mode_Byte for XIP enabled commands */ 
#define USER_DATA               (0xA0)       /* User data for memory write and read */
#define MEM_ADDRESS             (0u)         /* Start address for memory write and read */
#define REG_DEFAULT_VAL         (0x00)       /* Factory default configuration register value */ 

cyhal_qspi_t qspi_host_fram_obj;

/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status by turning the LED on, and
*  asserts the non-zero status.
*
* Parameters: 
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
void check_status(char *message, uint32_t status)
{
    if(0u != status)
    {
        printf("\r\n================================================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08lX\r\n", (long unsigned int)status);
        printf("\r\n================================================================================\r\n");
        
        /* On failure, turn the LED ON */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        while(true); /* Wait forever here when error occurs. */
    }
}

/************************************************************************************
* Function Name: reset_to_factory_default
*********************************************************************************//**
* Summary:
* This function resets the status and configuration volatile registers 
*  to their factory default.
*
************************************************************************************/
void reset_to_factory_default (void)
{
    uint8_t tx[ONE_BYTE_ACCESS];
    cy_rslt_t result;

    /* Write to CR2 to set the interface mode to SPI */    
    tx[0] = REG_DEFAULT_VAL;

    /* Try in DPI mode, in case device access mode is DPI after power up */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, FRAM_BUS_TYPE_DPI);
    check_status("WREN command failed", result);
    
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR2_ADDR_VOLATILE, tx, FRAM_BUS_TYPE_DPI);
    check_status("WRAR command failed", result);

    /* Try in QPI mode, in case device access mode is QPI after power up */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, FRAM_BUS_TYPE_QPI);
    check_status("WREN command failed", result);
    
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR2_ADDR_VOLATILE, tx, FRAM_BUS_TYPE_QPI);
    check_status("WRAR command failed", result);
    
    /* At this point device is confugred to a known (SPI) access mode. 
     * Write to CR1 to clear memory latency setting.
     */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, FRAM_BUS_TYPE_SPI);
    check_status("WREN command failed", result);
    
    tx[0] = REG_DEFAULT_VAL;
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR1_ADDR_VOLATILE, tx, FRAM_BUS_TYPE_SPI); /* Set the access mode */
    check_status("WRAR command failed", result);
    
    /* Write to CR5 to clear register latency setting */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, FRAM_BUS_TYPE_SPI);
    check_status("WREN command failed", result);
    
    tx[0] = REG_DEFAULT_VAL;
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR5_ADDR_VOLATILE, tx, FRAM_BUS_TYPE_SPI); /* Set the access mode */    
    check_status("WRAR command failed", result);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
* Sets the device to factory default (SPI) access mode. 
* Reads device ID, unique ID, and displays on UART
* Reads all status & configuration registers, and displays on UART 
* Writes user defined data pattern into F-RAM memory, reads, and verifies.
* Writtent and read data are also displayed on UART terminal. 
* Results can be verified through LED (green for PASS and red for FAIL)
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    uint8_t rx[DEV_ID_LENGTH_BYTE];
    uint8_t tx[DEV_ID_LENGTH_BYTE];
    size_t length = DEV_ID_LENGTH_BYTE;
    uint8_t write_data[PACKET_SIZE_BYTES];
    uint8_t read_data[PACKET_SIZE_BYTES];
    uint32_t index;
    fram_bus_type_t access_mode;
    cy_rslt_t result;

    /* Set up the device based on configurator selections */
    result = cybsp_init();

    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    /* Enable interrupts */
    __enable_irq();
    
    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    if(result != CY_RSLT_SUCCESS)
    {
        __disable_irq();        
        CY_ASSERT(0);
    }   
    
    printf("\x1b[2J\x1b[;H");

    printf("\r\n******************* CE229303 - PSOC 6 MCU: QSPI F-RAM ACCESS *******************");
    printf("\r\n*******************************************************************************\r\n");

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
            CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    check_status("User LED initialization failed", result);

    /* Initialize the QSPI control and pins */
    result = cyhal_qspi_init(&qspi_host_fram_obj, CYBSP_QSPI_D0, CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC,
                            NC, NC, CYBSP_QSPI_SCK, CYBSP_FRAM_CS, QSPI_BUS_FREQUENCY_HZ, 0);
    check_status("CYHAL QSPI initialization failed", result);


    /* This function brings the device from DPI or QPI (if set) to 
     * factory default status and configuration register settings. 
     */
    reset_to_factory_default();
    access_mode = FRAM_BUS_TYPE_SPI; /* Sets the access mode for the host */
    
    /* This sets the device access mode configuration in volatile register. To make this setting 
     * persistent (nonvolatile), you must write into nonvolatile register address. Refer to 
     * Excelon(TM) Ultra CY15x104QSN datasheet for register address details. 
     */  
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, access_mode); /* Send the WREN opcode prior to register write */
    check_status("WREN command failed", result);
    
    /* DPI_EN - to set the DPI protocol enable bit '1' in F-RAM CR2 (CR2[4]).
     * QPI_EN - to set the QPI protocol enable bit '1' in F-RAM CR2 (CR2[6]). 
     */
    tx[0] = QPI_EN; 
    
    /* Write to CR2 to set the F-RAM access mode */           
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR2_ADDR_VOLATILE, tx, access_mode); 
    check_status("WRAR command failed", result);
    
    /* Sets the new access mode for the host, as per the F-RAM access mode setting */  
    access_mode = FRAM_BUS_TYPE_QPI;                       

    /*******************************************************************************
    * Set the F-RAM memory and register access latency
    ********************************************************************************/

    /* Send the WREN opcode prior to write into config reg 1 (CR1) for the memory latency setting */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, access_mode);
    check_status("WREN command failed", result);
    
    tx[0] = (MEM_LATENCY << MEM_LATENCY_POS);
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR1_ADDR_VOLATILE, tx, access_mode); /* Sets the memory latency (MLC) */
    check_status("WRAR command failed", result);

    /* Send the WREN opcode prior to write into config reg 5 (CR5) for the register latency setting */
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, access_mode);
    check_status("WREN command failed", result);
    
    tx[0] = (REG_LATENCY << REG_LATENCY_POS);
    result = fram_wrar_cmd(&qspi_host_fram_obj, CR5_ADDR_VOLATILE, tx, access_mode); /* Sets the register latency (RLC) */
    check_status("WRAR command failed", result);

    /*******************************************************************************
    * Read and Display Device ID and Unique ID of QSPI F-RAM
    ********************************************************************************/
    
    printf("Read QSPI F-RAM 8-byte DID - ");
    result = fram_read_id_cmd (&qspi_host_fram_obj, MEM_CMD_RDID, rx, &length, access_mode, REG_LATENCY);
    check_status("RDID command failed", result);

    for(index = 0; index < length; index++)
    {
        printf("0x%02X ", rx[index]);
    }
    printf("\r\n");

    printf("Read QSPI F-RAM 8-byte UDID - ");
    result = fram_read_id_cmd (&qspi_host_fram_obj, MEM_CMD_RUID, rx, &length, access_mode, REG_LATENCY);
    check_status("RUID command failed", result);

    for(index = 0; index < length; index++)
    {
        printf("0x%02X ", rx[index]);
    }
    printf("\r\n\r\n");

    /*******************************************************************************
    * Read and Display All Status and Configuration Registers
    ********************************************************************************/
    
    printf("Set WEN in SR0\r\n");
    result = fram_opcode_only_cmd(&qspi_host_fram_obj, MEM_CMD_WREN, access_mode);
    check_status("WREN command failed", result);

    printf("Read QSPI F-RAM SR0 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDSR1, rx, access_mode, REG_LATENCY);
    check_status("RDSR1 command failed", result);
    printf("0x%02X\r\n", rx[0]);    

    printf("Read QSPI F-RAM SR1 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDSR2, rx, access_mode, REG_LATENCY);
    check_status("RDSR2 command failed", result);
    printf("0x%02X\r\n", rx[0]);

    printf("Read QSPI F-RAM CR1 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDCR1, rx, access_mode, REG_LATENCY);
    check_status("RDCR1 command failed", result);
    printf("0x%02X\r\n", rx[0]);

    printf("Read QSPI F-RAM CR2 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDCR2, rx, access_mode, REG_LATENCY);
    check_status("RDCR2 command failed", result);
    printf("0x%02X\r\n", rx[0]);

    printf("Read QSPI F-RAM CR4 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDCR4, rx, access_mode, REG_LATENCY);
    check_status("RDCR4 command failed", result);
    printf("0x%02X\r\n", rx[0]);

    printf("Read QSPI F-RAM CR5 ");
    result = fram_read_SRxCRx_cmd (&qspi_host_fram_obj, MEM_CMD_RDCR5, rx, access_mode, REG_LATENCY);
    check_status("RDCR5 command failed", result);
    printf("0x%02X\r\n\r\n", rx[0]);

    /*******************************************************************************
    * Write to F-RAM; print written data via UART
    ********************************************************************************/

    for(index = 0; index < PACKET_SIZE_BYTES; index++)
    {
        write_data[index] = (uint8_t)index + USER_DATA;
    }

    length = PACKET_SIZE_BYTES;
    result = fram_write_cmd (&qspi_host_fram_obj, MEM_CMD_WRITE, MEM_ADDRESS, MODE_BYTE, write_data, &length, access_mode);
    check_status("Memory write command failed", result);

    printf("F-RAM write data\r\n");

    for(index = 0; index < PACKET_SIZE_BYTES; index++)
    {
        printf("0x%02X ", write_data[index]);
    }

    printf("\r\n\r\n");

    /*******************************************************************************
    * Read from F-RAM; print written data via UART
    ********************************************************************************/

    result = fram_read_cmd (&qspi_host_fram_obj, MEM_CMD_READ, MEM_ADDRESS, MODE_BYTE, read_data, &length, access_mode, MEM_LATENCY);
    check_status("Memory read command failed", result);

    printf("F-RAM read data\r\n");

    for(index = 0; index < PACKET_SIZE_BYTES; index++)
    {
        printf("0x%02X ", read_data[index]);
    }

    printf("\r\n");

    for(index = 0; index < PACKET_SIZE_BYTES; index++)
    {
        if(read_data[index] != write_data[index])
        {
            check_status("Data mismatch", PACKET_SIZE_BYTES);
        }
    }

    printf("\r\n================================================================================\r\n");
    printf("\r\nSUCCESS: Read data matches with written data!\r\n");
    printf("\r\n================================================================================\r\n");

    /* Toggle LED on if read data matches written data */
    for(;;)
    {
        cyhal_gpio_toggle(CYBSP_USER_LED);
        cyhal_system_delay_ms(LED_TOGGLE_DELAY_MSEC);
    }
}
