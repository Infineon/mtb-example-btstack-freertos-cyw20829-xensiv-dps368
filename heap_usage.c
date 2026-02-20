/******************************************************************************
* File Name:   heap_usage.c
*
* Description: This file contains the code for printing heap usage.
*              Supports only GCC_ARM compiler. Define PRINT_HEAP_USAGE for
*              printing the heap usage numbers.
*
* Related Document: See README.md
*
*
*******************************************************************************
 * (c) 2021-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>


/* ARM compiler also defines __GNUC__ */
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
#include <malloc.h>
#endif /* #if defined (__GNUC__) && !defined(__ARMCC_VERSION) */


/*******************************************************************************
 * Macros
 ******************************************************************************/
#define TO_KB(size_bytes)  ((float)(size_bytes)/1024)


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/


/*******************************************************************************
* Function Name: print_heap_usage
********************************************************************************
* Summary:
* Prints the available heap and utilized heap by using mallinfo().
*
*******************************************************************************/
void print_heap_usage(char *msg)
{
    /* ARM compiler also defines __GNUC__ */
#if defined(PRINT_HEAP_USAGE) && defined (__GNUC__) && !defined(__ARMCC_VERSION)
    struct mallinfo mall_info = mallinfo();

    extern uint8_t __HeapBase;  /* Symbol exported by the linker. */
    extern uint8_t __HeapLimit; /* Symbol exported by the linker. */

    uint8_t* heap_base = (uint8_t *)&__HeapBase;
    uint8_t* heap_limit = (uint8_t *)&__HeapLimit;
    uint32_t heap_size = (uint32_t)(heap_limit - heap_base);

    printf("\r\n\n********** Heap Usage **********\r\n");
    printf(msg);
    printf("\r\nTotal available heap        : %"PRIu32" bytes/%.2f KB\r\n", heap_size, TO_KB(heap_size));

    printf("Maximum heap utilized so far: %u bytes/%.2f KB, %.2f%% of available heap\r\n",
            mall_info.arena, TO_KB(mall_info.arena), ((float) mall_info.arena * 100u)/heap_size);

    printf("Heap in use at this point   : %u bytes/%.2f KB, %.2f%% of available heap\r\n",
            mall_info.uordblks, TO_KB(mall_info.uordblks), ((float) mall_info.uordblks * 100u)/heap_size);

    printf("********************************\r\n\n");
#endif /* #if defined(PRINT_HEAP_USAGE) && defined (__GNUC__) && !defined(__ARMCC_VERSION) */
}


/* [] END OF FILE */
