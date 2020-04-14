/*
* Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
* Cypress Semiconductor Corporation. All Rights Reserved.
*
* This software, including source code, documentation and related
* materials ("Software"), is owned by Cypress Semiconductor Corporation
* or one of its subsidiaries ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
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
 * Mesh DFU related functionality
 *
 */

#include "bt_types.h"
#include "sha2.h"
#include "p_256_ecc_pp.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_bt_mesh_model_utils.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_wdog.h"


/******************************************************
 *          Constants
 ******************************************************/

//#define DONT_ALLOW_TO_DOWNLOAD_SAME_VERSION

/* FW ID is 2 bytes CID, 2 bytes PID, 2 bytes VID, 1 byte major version, 1 byte minor version, 1 byte revision, 2 bytes build */
#define MESH_DFU_FW_ID_LEN                          11

#define FW_UPDATE_NVRAM_MAX_READ_SIZE               512

#define HASH_SIZE                                   32
#define SIGNATURE_SIZE                              64

#ifndef WICED_SDK_MAJOR_VER
#define WICED_SDK_BUILD_NUMBER  0
#define WICED_SDK_MAJOR_VER     0
#define WICED_SDK_MINOR_VER     0
#define WICED_SDK_REV_NUMBER    0
#endif

/******************************************************
 *          Function Prototypes
 ******************************************************/

static wiced_bool_t mesh_get_active_fw_id(mesh_dfu_fw_id_t *p_id);
static wiced_bool_t mesh_fw_id_acceptable(mesh_dfu_fw_id_t* p_id);
static wiced_bool_t mesh_fw_verify(uint32_t fw_size, mesh_dfu_fw_id_t *p_fw_id, mesh_dfu_meta_data_t *p_meta_data);

// public key. Ecdsa256_pub.c should be generated and included in the project.
extern Point ecdsa256_public_key;

extern BOOL32 ecdsa_verify_(unsigned char* digest, unsigned char* signature, Point* key);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t mesh_app_dfu_init()
{
    WICED_BT_TRACE("Mesh DFU init\n");

    wiced_bt_mesh_set_dfu_callbacks(mesh_get_active_fw_id, mesh_fw_id_acceptable, mesh_fw_verify);

    return WICED_SUCCESS;
}

/*
 * Get FW ID of active firmware
 */
wiced_bool_t mesh_get_active_fw_id(mesh_dfu_fw_id_t *p_id)
{
    uint8_t *p = p_id->fw_id;

    UINT16_TO_STREAM(p, mesh_config.company_id);
    UINT16_TO_STREAM(p, mesh_config.product_id);
    UINT16_TO_STREAM(p, mesh_config.vendor_id);
    UINT8_TO_STREAM(p, WICED_SDK_MAJOR_VER);
    UINT8_TO_STREAM(p, WICED_SDK_MINOR_VER);
    UINT8_TO_STREAM(p, WICED_SDK_REV_NUMBER);
    UINT16_TO_STREAM(p, WICED_SDK_BUILD_NUMBER);
    p_id->fw_id_len = MESH_DFU_FW_ID_LEN;

    return WICED_TRUE;
}

/*
 * Check if the FW ID is suitable for this device
 */
wiced_bool_t mesh_fw_id_acceptable(mesh_dfu_fw_id_t *p_id)
{
    uint16_t cid, pid, vid;
    uint8_t ver_major, ver_minor, revision;
    uint16_t build;
    uint8_t *p = p_id->fw_id;

    if (p_id->fw_id_len != MESH_DFU_FW_ID_LEN)
        return WICED_FALSE;

    STREAM_TO_UINT16(cid, p);
    STREAM_TO_UINT16(pid, p);
    STREAM_TO_UINT16(vid, p);
    STREAM_TO_UINT8(ver_major, p);
    STREAM_TO_UINT8(ver_minor, p);
    STREAM_TO_UINT8(revision, p);
    STREAM_TO_UINT16(build, p);

    if (cid != mesh_config.company_id || pid != mesh_config.product_id || vid != mesh_config.vendor_id)
    {
        WICED_BT_TRACE("FW ID not acceptable cid:%d/%d\n", cid, mesh_config.company_id);
        WICED_BT_TRACE("FW ID not acceptable pid:%d/%d\n", pid, mesh_config.product_id);
        WICED_BT_TRACE("FW ID not acceptable vid:%d/%d\n", vid, mesh_config.vendor_id);
        return WICED_FALSE;
    }
    if (ver_major > WICED_SDK_MAJOR_VER)
        return WICED_TRUE;
    if (ver_major < WICED_SDK_MAJOR_VER)
    {
        WICED_BT_TRACE("FW ID not acceptable ver_major:%d/%d\n", ver_major, WICED_SDK_MAJOR_VER);
        return WICED_FALSE;
    }
    // It is typically allowed to downgrade to earlier minor/revision
    if (ver_minor != WICED_SDK_MINOR_VER)
        return WICED_TRUE;
    if (revision != WICED_SDK_REV_NUMBER)
        return WICED_TRUE;
#ifdef DONT_ALLOW_TO_DOWNLOAD_SAME_VERSION
    if (build == WICED_SDK_BUILD_NUMBER)
        return WICED_FALSE;
#else
    UNUSED_VARIABLE(build);
#endif
    return WICED_TRUE;
}

/*
 * Verify the signature of received FW
 */
wiced_bool_t mesh_fw_verify(uint32_t fw_size, mesh_dfu_fw_id_t *p_fw_id, mesh_dfu_meta_data_t *p_meta_data)
{
    sha2_context sha2_ctx;
    uint8_t digest[HASH_SIZE];
    uint8_t buf[FW_UPDATE_NVRAM_MAX_READ_SIZE];
    uint32_t nv_read_size;
    uint32_t nv_read_offset = 0;
    uint32_t bytes_remain = fw_size;

#ifdef PTS
    wiced_firmware_upgrade_retrieve_from_nv(0, buf, 4);
    if (buf[0] == 0)
        return WICED_FALSE;
    return WICED_TRUE;
#endif

    if (p_meta_data->len != SIGNATURE_SIZE)
        return WICED_FALSE;

    // initialize sha256 context
    sha2_starts(&sha2_ctx, 0);

    // generate digest
    sha2_update(&sha2_ctx, p_fw_id->fw_id, p_fw_id->fw_id_len);
    while (bytes_remain)
    {
        nv_read_size = bytes_remain < FW_UPDATE_NVRAM_MAX_READ_SIZE ? bytes_remain : FW_UPDATE_NVRAM_MAX_READ_SIZE;
        wiced_firmware_upgrade_retrieve_from_nv(nv_read_offset, buf, (nv_read_size + 3) & 0xFFFFFFFC);
        sha2_update(&sha2_ctx, buf, nv_read_size);
        nv_read_offset += nv_read_size;
        bytes_remain -= nv_read_size;
    }
    wiced_hal_wdog_restart();
    sha2_finish(&sha2_ctx, digest);

    return ecdsa_verify_(digest, p_meta_data->data, &ecdsa256_public_key);
}
