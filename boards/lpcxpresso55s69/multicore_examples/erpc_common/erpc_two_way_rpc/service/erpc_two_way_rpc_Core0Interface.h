/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Generated by erpcgen 1.7.3 on Mon Sep 23 13:00:45 2019.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#if !defined(_erpc_two_way_rpc_Core0Interface_h_)
#define _erpc_two_way_rpc_Core0Interface_h_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "erpc_version.h"
#include "erpc_two_way_rpc_Core0Interface.h"
#include "erpc_two_way_rpc_Core1Interface.h"

#if 10703 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif

#if !defined(ERPC_TYPE_DEFINITIONS)
#define ERPC_TYPE_DEFINITIONS

// Aliases data types declarations
/*! callback type */
typedef void (*getNumberCallback_t)(uint32_t * param1);

#endif // ERPC_TYPE_DEFINITIONS

/*! @brief Core0Interface identifiers */
enum _Core0Interface_ids
{
    kCore0Interface_service_id = 1,
    kCore0Interface_setGetNumberFunction_id = 1,
    kCore0Interface_getGetNumberFunction_id = 2,
    kCore0Interface_nestedCallGetNumber_id = 3,
    kCore0Interface_getNumberFromCore1_id = 4,
};

#if defined(__cplusplus)
extern "C" {
#endif

/*! Core0 interface */
//! @name Core0Interface
//@{
/*! Sets callback function. To the other side. Implementation on Core1  */
void setGetNumberFunction(const getNumberCallback_t getNumberCallbackParam);

/*! Gets callback function. Implementation on Core1  */
void getGetNumberFunction(getNumberCallback_t * getNumberCallbackParam);

/*! Sets callback function. To the other side. Implementation on Core1  */
void nestedCallGetNumber(const getNumberCallback_t getNumberCallbackParam);

/*! Callback function. Implementation on Core1 */
void getNumberFromCore1(uint32_t * number);
//@} 

#if defined(__cplusplus)
}
#endif

#endif // _erpc_two_way_rpc_Core0Interface_h_
