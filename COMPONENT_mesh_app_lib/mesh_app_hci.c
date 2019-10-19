/*
* Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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
 * This file contains functions which embeddded application can use to send and
 * receive data from the host.
 */
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_wdog.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_trace.h"
#include "mesh_application.h"
#include "hci_control_api.h"
#include "wiced_hal_puart.h"
#ifdef CYW20706A2
#include "wiced_bt_hci_defs.h"
#else
#include "hcidefs.h"
#endif

// if defined then prints mesh stats every _DEB_PRINT_MESH_STATS seconds
//#define _DEB_PRINT_MESH_STATS 30

// dump wiced bt buffer statistics on every _DEB_PRINT_BUF_USE seconds to monitor buffer usage
#define _DEB_PRINT_BUF_USE  5

// Enables HCI trace to first(same as app download) com port.
//#define _DEB_ENABLE_HCI_TRACE
#define _DEB_ENABLE_HCI_TRACE_NO_BLE_ADV_EVT

static void  mesh_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
static void  mesh_app_timer(uint32_t count);

// app timer
wiced_timer_t app_timer;

/******************************************************
 *          Function Prototypes
 ******************************************************/

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint32_t mesh_application_proc_rx_cmd(uint8_t *p_buffer, uint32_t length);

/* transport configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    .type                = WICED_TRANSPORT_UART,
#ifdef HCI_UART_BAUDRATE
    .cfg.uart_cfg        = { .mode = WICED_TRANSPORT_UART_HCI_MODE, .baud_rate = HCI_UART_BAUDRATE },
#else
    .cfg.uart_cfg        = { .mode = WICED_TRANSPORT_UART_HCI_MODE, .baud_rate = HCI_UART_DEFAULT_BAUD },   // default it 3mbps
#endif
    .rx_buff_pool_cfg    = {  .buffer_size = 1024, .buffer_count = 2 },
    .p_status_handler    = NULL,
    .p_data_handler      = mesh_application_proc_rx_cmd,
    .p_tx_complete_cback = NULL
};

wiced_transport_buffer_pool_t* host_trans_pool = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_hci_init(void)
{
#ifndef MESH_HOMEKIT_COMBO_APP
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(1024, 2);
#endif

#ifndef WICEDX_LINUX
#ifdef WICED_BT_TRACE_ENABLE
#ifndef _DEB_ENABLE_HCI_TRACE
    //For the 24 MHz board set the UART type as WICED_ROUTE_DEBUG_TO_PUART
    // For BCM920706V2_EVAL board make sure SW5.2 and SW5.4 are ON and all other SW5 are off
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#if (defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0))
    wiced_hal_puart_select_uart_pads(WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#ifndef CYW20706A2
    wiced_hal_puart_configuration(921600, PARITY_NONE, STOP_BIT_1);
#endif
#else
    // WICED_ROUTE_DEBUG_TO_WICED_UART to send debug strings over the WICED debug interface */
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
#endif
#endif
    wiced_bt_dev_register_hci_trace(mesh_hci_trace_cback);
}

void mesh_app_timer_init(void)
{
    /* Starting the app timer  */
    memset(&app_timer, 0, sizeof(wiced_timer_t));
    if (wiced_init_timer(&app_timer, mesh_app_timer, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    //if (wiced_init_timer(&app_timer, mesh_app_timer, 0, WICED_SECONDS_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&app_timer, MESH_APP_TIMEOUT_IN_SECONDS) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("APP START Timer FAILED!!\n");
        }
    }
    else
    {
        WICED_BT_TRACE("APP INIT Timer FAILED!!\n");
    }
}

wiced_bt_mesh_hci_event_t *wiced_bt_mesh_create_hci_event(wiced_bt_mesh_event_t *p_event)
{
    wiced_bt_mesh_hci_event_t *p_hci_event = (wiced_bt_mesh_hci_event_t *)wiced_transport_allocate_buffer(host_trans_pool);
    if (p_hci_event == NULL)
    {
        WICED_BT_TRACE("no trans buffer\n");
    }
    else if (p_event != NULL)
    {
        p_hci_event->src         = p_event->src;
        p_hci_event->dst         = p_event->dst;
        p_hci_event->app_key_idx = p_event->app_key_idx;
        p_hci_event->element_idx = p_event->element_idx;
        p_hci_event->rssi        = p_event->rssi;
        p_hci_event->ttl         = p_event->ttl;
        p_hci_event->company_id  = p_event->company_id;
        p_hci_event->opcode      = p_event->opcode;
    }
    return p_hci_event;
}

wiced_bt_mesh_hci_event_t *wiced_bt_mesh_alloc_hci_event(uint8_t element_idx)
{
    wiced_bt_mesh_hci_event_t *p_hci_event = (wiced_bt_mesh_hci_event_t *)wiced_transport_allocate_buffer(host_trans_pool);
    if (p_hci_event == NULL)
    {
        WICED_BT_TRACE("no trans buffer\n");
    }
    else
    {
        p_hci_event->src         = 0;
        p_hci_event->app_key_idx = 0;
        p_hci_event->element_idx = element_idx;
    }
    return p_hci_event;
}

void mesh_transport_send_data(uint16_t opcode, uint8_t *p_trans_buf, uint16_t data_len)
{
    wiced_transport_send_buffer(opcode, p_trans_buf, data_len);
}

#ifdef _DEB_PRINT_BUF_USE
// dump wiced bt buffer statistics on every 10 seconds to monitor buffer usage
void _deb_print_buf_use()
{
    wiced_bt_buffer_statistics_t buff_stat[4];
    wiced_bt_get_buffer_usage(buff_stat, sizeof(buff_stat));
    WICED_BT_TRACE("pool size/cur/max/total %d/%d/%d/%d %d/%d/%d/%d %d/%d/%d/%d %d/%d/%d/%d\n",
        buff_stat[0].pool_size, buff_stat[0].current_allocated_count, buff_stat[0].max_allocated_count, buff_stat[0].total_count,
        buff_stat[1].pool_size, buff_stat[1].current_allocated_count, buff_stat[1].max_allocated_count, buff_stat[1].total_count,
        buff_stat[2].pool_size, buff_stat[2].current_allocated_count, buff_stat[2].max_allocated_count, buff_stat[2].total_count,
        buff_stat[3].pool_size, buff_stat[3].current_allocated_count, buff_stat[3].max_allocated_count, buff_stat[3].total_count);
}
#endif

#ifdef _DEB_PRINT_MESH_STATS
// dump mesh statistics on every _DEB_PRINT_MESH_STATS
void _deb_print_mesh_stats()
{
    wiced_bt_mesh_core_statistics_t network_stats;
    wiced_bt_mesh_core_transport_statistics_t transport_stats;
    wiced_bt_mesh_core_statistics_get(&network_stats);
    wiced_bt_mesh_core_transport_statistics_get(&transport_stats);
    wiced_bt_mesh_core_statistics_reset();
    wiced_bt_mesh_core_transport_statistics_reset();
    WICED_BT_TRACE("rcv msg cnt:total/proxy-cfg/relay/u-cast/group/lpn:       %d/%d/%d/%d/%d/%d\n",
        network_stats.received_msg_cnt, network_stats.received_proxy_cfg_msg_cnt, network_stats.relayed_msg_cnt,
        network_stats.accepted_unicast_msg_cnt, network_stats.accepted_group_msg_cnt, network_stats.received_for_lpn_msg_cnt);
    WICED_BT_TRACE("dropped msg cnt:invalid/nid/decr/cache/relay-ttl/group:   %d/%d/%d/%d/%d/%d\n",
        network_stats.dropped_invalid_msg_cnt, network_stats.dropped_by_nid_msg_cnt, network_stats.dropped_not_decrypted_msg_cnt,
        network_stats.dropped_by_net_cache_msg_cnt, network_stats.not_relayed_by_ttl_msg_cnt, network_stats.dropped_group_msg_cnt);
    WICED_BT_TRACE("sent msg cnt: adv/gatt/proxy_cfg/clnt/net-cred/frnd-cred: %d/%d/%d/%d/%d/%d\n",
        network_stats.sent_adv_msg_cnt, network_stats.sent_gatt_msg_cnt, network_stats.sent_proxy_cfg_msg_cnt,
        network_stats.sent_proxy_clnt_msg_cnt, network_stats.sent_net_credentials_msg_cnt, network_stats.sent_frnd_credentials_msg_cnt);
    WICED_BT_TRACE("sent msg cnt: u-cast/group/access/unseg/seg/ack: %d/%d/%d/%d/%d/%d\n",
        network_stats.sent_adv_unicast_msg_cnt, network_stats.sent_adv_group_msg_cnt,
        transport_stats.sent_access_layer_msg_cnt, transport_stats.sent_unseg_msg_cnt, transport_stats.sent_seg_msg_cnt,
        transport_stats.sent_ack_msg_cnt);
    WICED_BT_TRACE("rcv msg cnt: access/unseg/seg/ack/dropped access: %d/%d/%d/%d\n",
        transport_stats.received_access_layer_msg_cnt, transport_stats.received_unseg_msg_cnt, transport_stats.received_seg_msg_cnt,
        transport_stats.received_ack_msg_cnt, transport_stats.dropped_access_layer_msg_cnt);
}
#endif

// App timer event handler
void mesh_app_timer(uint32_t arg)
{
    static uint32_t app_timer_count = 1;
    app_timer_count++;

#ifdef _DEB_PRINT_BUF_USE
    /* dump wiced bt buffer statistics on every 10 seconds to monitor buffer usage */
    if (!(app_timer_count % _DEB_PRINT_BUF_USE))
    {
        _deb_print_buf_use();
    }
#endif
#ifdef _DEB_PRINT_MESH_STATS
    // dump mesh statistics on every _DEB_PRINT_MESH_STATS
    if ((app_timer_count % _DEB_PRINT_MESH_STATS) == 2)
    {
        _deb_print_mesh_stats();
    }
#endif
}

//  Pass protocol traces up through the UART
static void mesh_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    // handle HCI_HARDWARE_ERROR_EVT - print trace and reboot.
    if (type == HCI_TRACE_EVENT && p_data[0] == HCI_HARDWARE_ERROR_EVT && p_data[2] != 0)
    {
        // allow retry for HARDWARE_CODE_VS_UART_PARSING_ERROR (p_data[2] == 0)
        WICED_BT_TRACE("Rebooting on HCI_HARDWARE_ERROR_EVT(%x)...\n", p_data[2]);
        wiced_hal_wdog_reset_system();
    }
#ifdef _DEB_ENABLE_HCI_TRACE
#ifdef _DEB_ENABLE_HCI_TRACE_NO_BLE_ADV_EVT
    // filter out HCI_BLE_Advertising_Report_Event
    if (type == 0               // if HCI event
        && p_data[0] == 0x3e    // and HCI_BLE_Event
        && p_data[2] == 2)      // and HCI_BLE_Advertising_Report_Event
        return;
#endif
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
//#else
//    if ((type == HCI_TRACE_COMMAND) || (type == HCI_TRACE_EVENT))
//    {
//        WICED_BT_TRACE("hci_trace_cback: type:%x length:%d\n", type, length);
//        WICED_BT_TRACE_ARRAY((char*)p_data, length, "");
//    }
#endif
}

uint32_t mesh_application_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;

    if (p_buffer == NULL)
    {
        return 0;
    }

    // Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE("invalid params\n");
    }
    else
    {
        STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
        STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
#ifdef MESH_APPLICATION_MCU_MEMORY
        if(!mesh_application_process_hci_cmd(opcode, p_data, payload_len))
#endif
#ifdef PTS
        if (wiced_bt_mesh_core_test_mode_signal(opcode, p_data, payload_len))
#endif
        {
            if (wiced_bt_mesh_app_func_table.p_mesh_app_proc_rx_cmd)
            {
                wiced_bt_mesh_app_func_table.p_mesh_app_proc_rx_cmd(opcode, p_data, payload_len);
            }
        }
    }
    wiced_transport_free_buffer(p_buffer);
    return 0;
}

/*
 * This function can be used to process WICED HCI header common all WICED HCI commands and create corresponing mesh event
 */
wiced_bt_mesh_event_t *wiced_bt_mesh_create_event_from_wiced_hci(uint16_t opcode, uint16_t company_id, uint16_t model_id, uint8_t **p_data, uint32_t *len)
{
    uint16_t dst;
    uint16_t app_key_idx;
    uint8_t  element_idx;
    int i;
    uint8_t  *p = *p_data;
    wiced_bt_mesh_event_t *p_event;

    STREAM_TO_UINT16(dst, p);
    STREAM_TO_UINT16(app_key_idx, p);
    STREAM_TO_UINT8(element_idx, p);

    *len = *len - 5;

    p_event = wiced_bt_mesh_create_event(element_idx, company_id, model_id, dst, app_key_idx);
    if (p_event != NULL)
    {
        p_event->hci_opcode = opcode;

        STREAM_TO_UINT8(p_event->reply, p);
        STREAM_TO_UINT8(p_event->send_segmented, p);
        STREAM_TO_UINT8(p_event->retrans_cnt, p);
        STREAM_TO_UINT8(p_event->retrans_time, p);
        STREAM_TO_UINT8(p_event->reply_timeout, p);
        STREAM_TO_UINT16(p_event->num_in_group, p);

        *len = *len - 7;

        if (p_event->num_in_group != 0)
        {
            p_event->group_list = (uint16_t *)wiced_bt_get_buffer(sizeof(uint16_t) * p_event->num_in_group);
            for (i = 0; i < p_event->num_in_group; i++)
            {
                STREAM_TO_UINT16(p_event->group_list[i], p);
                *len -= 2;
            }
        }
    }
    *p_data = p;
    return p_event;
}

/*
 * This function can be used to skip mesh WICED HCI header
 */
void wiced_bt_mesh_skip_wiced_hci_hdr(uint8_t **p_data, uint32_t *len)
{
    uint16_t num_in_group;
    uint8_t  *p = *p_data;

    // skip dst, app_key_idx, element_idx, reply, send_segmented, retrans_cnt, retrans_time, reply_timeout
    p += 10;
    STREAM_TO_UINT16(num_in_group, p);
    *len = *len - 12;

    if (num_in_group != 0)
    {
        p -= (2 * num_in_group);
        *len -= (2 * num_in_group);
    }
    *p_data = p;
}

/*
 * This function can be used to parse WICED HCI header and extract needed info
 */
uint8_t wiced_bt_mesh_get_element_idx_from_wiced_hci(uint8_t **p_data, uint32_t *len)
{
    uint8_t  element_idx;
    uint16_t num_in_group;
    int i;
    uint8_t  *p = *p_data;
    wiced_bt_mesh_event_t *p_event;

    p += 4;  // skip dst and app_key_idx
    STREAM_TO_UINT8(element_idx, p);

    *len = *len - 5;

    p += 5;             // reply, send_segmented, retrans_cnt, retrans_time, reply_timeout
    STREAM_TO_UINT16(num_in_group, p);
    {
        *len = *len - 6;
        *len = *len - (2 * num_in_group);
        p    = p + (2 * num_in_group);
    }
    *p_data = p;
    return element_idx;
}

/*
 * This function can be used to send TX Complete event to the MCU
 */
void wiced_bt_mesh_send_hci_tx_complete(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_event_t *p_event)
{
    uint8_t *p = p_hci_event->data;
    int i;
    uint8_t num_addrs = 1;
    uint16_t    ui16val;

    if ((p_event->tx_status) && (p_event->dst >= 0xC000))
    {
        num_addrs = 0;
        for (i = 0; i < p_event->num_in_group; i++)
        {
            if (p_event->group_list[i] != 0)
            {
                num_addrs++;
            }
        }
    }

    UINT16_TO_STREAM(p, p_event->hci_opcode);
    UINT8_TO_STREAM(p, p_event->tx_status);

    // if failed, put in all addresses from the Group that have not acked, otherwise just send out DST
    if (p_event->tx_status)
    {
        if (p_event->dst < 0xC000)
        {
            UINT16_TO_STREAM(p, 1);
            UINT16_TO_STREAM(p, p_event->dst);
        }
        else
        {
            ui16val = num_addrs;
            UINT16_TO_STREAM(p, ui16val);
            for (i = 0; i < p_event->num_in_group; i++)
            {
                if (p_event->group_list[i] != 0)
                {
                    UINT16_TO_STREAM(p, p_event->group_list[i]);
                }
            }
        }
    }
    else
    {
        UINT16_TO_STREAM(p, p_event->dst);
    }
    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_TX_COMPLETE, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

void mesh_app_hci_send_seq_changed(wiced_bt_mesh_core_state_seq_t *p_seq)
{
    uint8_t *p_hci_event;
    uint8_t *p;
    p_hci_event = wiced_transport_allocate_buffer(host_trans_pool);
    if (p_hci_event != NULL)
    {
        p = p_hci_event;
        UINT16_TO_STREAM(p, p_seq->addr);
        UINT32_TO_STREAM(p, p_seq->seq);
        UINT8_TO_STREAM(p, p_seq->previous_iv_idx);
        UINT16_TO_STREAM(p, p_seq->rpl_entry_idx);
        mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_CORE_SEQ_CHANGED, p_hci_event, (uint16_t)(p - p_hci_event));
    }
}

void mesh_app_hci_sleep(void)
{
    if (wiced_is_timer_in_use(&app_timer))
        wiced_stop_timer(&app_timer);
}

wiced_result_t mesh_application_send_hci_event(uint16_t opcode, const uint8_t *p_data, uint16_t data_len)
{
    uint8_t                *p_hci_event;
    p_hci_event = wiced_transport_allocate_buffer(host_trans_pool);
    if (p_hci_event != NULL)
    {
        if (data_len != 0)
            memcpy(p_hci_event, p_data, data_len);
        mesh_transport_send_data(opcode, p_hci_event, data_len);
    }
    return WICED_BT_SUCCESS;
}

typedef struct
{
    void        *next;
    uint16_t    id;
    uint16_t    len;
    uint8_t     data[1];
} mesh_nvram_block_t;

mesh_nvram_block_t  *mesh_application_mcu_memory_first_block = NULL;

mesh_nvram_block_t *mesh_application_mcu_memory_find(uint16_t id)
{
    mesh_nvram_block_t  *p;
    for (p = mesh_application_mcu_memory_first_block; p != NULL; p = (mesh_nvram_block_t*)p->next)
    {
        if (p->id == id)
            break;
    }
    return p;
}

uint16_t mesh_application_mcu_memory_read(uint16_t id, uint16_t buf_len, uint8_t *p_buf)
{
    mesh_nvram_block_t  *p;
    if ((p = mesh_application_mcu_memory_find(id)) == NULL)
    {
        WICED_BT_TRACE("mcu_memory_read: no NVRAM id:%d\n", id);
        return 0;
    }
    WICED_BT_TRACE("mcu_memory_read: id:%d buf_len:%d len:%d\n", id, buf_len, p->len);
    if (buf_len > p->len)
        buf_len = p->len;
    memcpy(p_buf, p->data, buf_len);
    return buf_len;
}

uint16_t mesh_application_mcu_memory_write(uint16_t id, uint16_t data_len, const uint8_t *p_data)
{
    uint8_t buf[257];// data_len can't be > 255. 0 data_len means delete NVRAM ID
    WICED_BT_TRACE("mcu_memory_write: id:%d data_len:%d\n", id, data_len);
    buf[0] = (uint8_t)id;
    buf[1] = (uint8_t)(id >> 8);
    if(data_len)
        memcpy(&buf[2], p_data, data_len);
    mesh_application_send_hci_event(HCI_CONTROL_MESH_EVENT_NVRAM_DATA, buf, data_len + 2);
    return data_len;
}

wiced_result_t mesh_application_process_hci_cmd(uint16_t opcode, const uint8_t *p_data, uint16_t data_len)
{
    wiced_result_t      ret = WICED_TRUE;
    uint16_t            id;
    mesh_nvram_block_t  *p;

    switch (opcode)
    {
    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        // make sure it contains NVRAM ID and at least one byte of data
        if (data_len <= 2)
        {
            WICED_BT_TRACE("mesh_application_process_hci_cmd: invalid NVRAM len %d\n", data_len);
            break;
        }
        // make sure NVRAM ID is correct
        id = (uint16_t)p_data[0] | (((uint16_t)p_data[1]) << 8);
        p_data += 2;
        data_len -= 2;

        // make sure this MVRAM ID hasn't been pushed already
        if (NULL != mesh_application_mcu_memory_find(id))
        {
            WICED_BT_TRACE("mesh_application_process_hci_cmd: already pushed NVRAM id %d\n", id);
            break;
        }
        // allocate buffer for that NVRAM ID and place it to the queue
        if ((p = (mesh_nvram_block_t*)wiced_bt_get_buffer(sizeof(mesh_nvram_block_t) + data_len)) == NULL)
        {
            WICED_BT_TRACE("mesh_application_process_hci_cmd: get_buffer(%d) failed\n", sizeof(mesh_nvram_block_t) + data_len);
            break;
        }
        WICED_BT_TRACE("mesh_application_process_hci_cmd: id:%d len:%d\n", id, data_len);
        p->next = mesh_application_mcu_memory_first_block;
        p->id = id;
        p->len = data_len;
        memcpy(p->data, p_data, data_len);
        mesh_application_mcu_memory_first_block = p;
        break;

    case HCI_CONTROL_MESH_COMMAND_APP_START:
        // Start application and then delete all allocated NVRAM chunks
        mesh_application_init();
        while (mesh_application_mcu_memory_first_block != NULL)
        {

            p = mesh_application_mcu_memory_first_block;
            mesh_application_mcu_memory_first_block = (mesh_nvram_block_t*)p->next;
            wiced_bt_free_buffer(p);
        }
        break;

    case HCI_CONTROL_MESH_COMMAND_GATEWAY_CONN_STATUS:
        WICED_BT_TRACE("HCI_CONTROL_MESH_COMMAND_GATEWAY_CONN_STATUS\n");
        wiced_bt_mesh_core_connection_status(p_data[0] + (p_data[1] << 8), p_data[2], 0, 75);
        break;

    case HCI_CONTROL_MESH_COMMAND_SEND_PROXY_DATA:
        WICED_BT_TRACE("SEND_PROXY_DATA\n");
        wiced_bt_mesh_core_proxy_packet(p_data, data_len);
        break;

    default:
        ret = WICED_FALSE;
    }
    return ret;
}
