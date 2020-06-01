/*
 * Copyright 2018 - 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "board.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "mcmgr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RPMSG_LITE_LINK_ID (RL_PLATFORM_LPC55S69_M33_M33_LINK_ID)
#define MCMGR_USED
#define LOCAL_EPT_ADDR (30)
#define APP_RPMSG_READY_EVENT_DATA (1)
#define APP_RPMSG_EP_READY_EVENT_DATA (2)

typedef struct the_message
{
    uint32_t DATA;
} THE_MESSAGE, *THE_MESSAGE_PTR;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

THE_MESSAGE volatile msg = {0};
uint32_t remote_addr     = 0;

/* Internal functions */
static int32_t my_ept_read_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    int32_t *has_received = priv;

    if (payload_len <= sizeof(THE_MESSAGE))
    {
        memcpy((void *)&msg, payload, payload_len);
        remote_addr   = src;
        *has_received = 1;
    }
    return RL_RELEASE;
}

#ifdef MCMGR_USED
/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    MCMGR_EarlyInit();
}
#endif /* MCMGR_USED */

/*!
 * @brief Main function
 */
int main(void)
{
    volatile int32_t has_received = 0;
    struct rpmsg_lite_ept_static_context my_ept_context;
    struct rpmsg_lite_endpoint *my_ept;
    struct rpmsg_lite_instance rpmsg_ctxt;
    struct rpmsg_lite_instance *my_rpmsg;

    /* Initialize standard SDK demo application pins */
    BOARD_InitPins_Core1();

    /* Check the silicone version */
    if (0x0 == Chip_GetVersion())
    {
        /* The rpmsg_lite_pingpong_tzm example does not work correctly in A0 silicone version */
        assert(false);
    }

#ifdef MCMGR_USED
    uint32_t startupData;
    mcmgr_status_t status;

    /* Initialize MCMGR before calling its API */
    MCMGR_Init();

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    my_rpmsg = rpmsg_lite_remote_init((void *)startupData, RPMSG_LITE_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);

    /* Signal the other core we are ready by triggering the event and passing the APP_RPMSG_READY_EVENT_DATA */
    MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_RPMSG_READY_EVENT_DATA);
#else
    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);
#endif /* MCMGR_USED */

    while (!rpmsg_lite_is_link_up(my_rpmsg))
        ;

    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, my_ept_read_cb, (void *)&has_received, &my_ept_context);

#ifdef MCMGR_USED
    /* Signal the other core the endpoint has been created by triggering the event and passing the
     * APP_RPMSG_READY_EP_EVENT_DATA */
    MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_RPMSG_EP_READY_EVENT_DATA);
#endif

#ifdef RPMSG_LITE_NS_USED
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);
#endif /*RPMSG_LITE_NS_USED*/
    has_received = 0;

    while (msg.DATA <= 1050)
    {
        if (has_received)
        {
            has_received = 0;
            msg.DATA++;
            rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);
        }
    }

    rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
    my_ept = NULL;
    rpmsg_lite_deinit(my_rpmsg);
    msg.DATA = 0;

    /* End of example */
    while (1)
        ;
}
