/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 */

/** @file
 *
 * Mesh AES encryption related functionality
 *
 */

#include "bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_aes.h"
#include "wiced_bt_mesh_core.h"
#include "mesh_application.h"
#include "wiced_bt_trace.h"

// TBD: below code has to be moved to the middleware/WICED layer
// ---------------------------------- Code for WICED layer
#if defined(CYW20706A2)
static void wiced_request_aes_exclusive_access()
{
}

static void wiced_release_aes_exclusive_access()
{
}
#else
extern unsigned int _tx_v7m_get_and_disable_int(void);
extern void _tx_v7m_set_int(unsigned int posture);
extern void bcs_pmuWaitForBtClock(void);
extern void bcs_pmuReleaseBtClock(void);


static unsigned int interrupt_save;
#define osapi_LOCK_CONTEXT          { interrupt_save = _tx_v7m_get_and_disable_int(); };
#define osapi_UNLOCK_CONTEXT        { _tx_v7m_set_int(interrupt_save); };

static void wiced_request_aes_exclusive_access()
{
    bcs_pmuWaitForBtClock();
    osapi_LOCK_CONTEXT;
}

static void wiced_release_aes_exclusive_access()
{
    osapi_UNLOCK_CONTEXT;
    bcs_pmuReleaseBtClock();
}
#endif
// ---------------------------------- End of code WICED layer

void mesh_app_aes_encrypt(uint8_t* in_data, uint8_t* out_data, uint8_t* key)
{
    uint8_t temp_data[WICED_BT_MESH_KEY_LEN];
    uint8_t temp_key[WICED_BT_MESH_KEY_LEN];
    int i, j;
    // Copy in_data and key to the temp buffers and revert data in these buffers
    for (i = 0, j = (WICED_BT_MESH_KEY_LEN - 1); i < WICED_BT_MESH_KEY_LEN; i++, j--)
    {
        temp_data[i] = in_data[j];
        temp_key[i] = key[j];
    }

    wiced_request_aes_exclusive_access();
    wiced_bcsulp_AES(temp_key, temp_data, out_data);
    wiced_release_aes_exclusive_access();

    // Revert data in out_data
    i = 0;
    j = WICED_BT_MESH_KEY_LEN - 1;

    while (i < j)
    {
        out_data[i] = out_data[i] ^ out_data[j];
        out_data[j] = out_data[i] ^ out_data[j];
        out_data[i] = out_data[i] ^ out_data[j];
        i++;
        j--;
    }
}
