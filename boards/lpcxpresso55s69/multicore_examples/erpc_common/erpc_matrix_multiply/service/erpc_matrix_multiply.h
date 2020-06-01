/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Generated by erpcgen 1.7.3 on Mon Sep 23 13:00:45 2019.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#if !defined(_erpc_matrix_multiply_h_)
#define _erpc_matrix_multiply_h_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "erpc_version.h"

#if 10703 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif

#if !defined(ERPC_TYPE_DEFINITIONS)
#define ERPC_TYPE_DEFINITIONS

// Aliases data types declarations
/*! This is the matrix array type. The dimension has to be the same as the
    matrix size const. Do not forget to re-generate the erpc code once the
    matrix size is changed in the erpc file */
typedef int32_t Matrix[5][5];

// Constant variable declarations
/*! This const defines the matrix size. The value has to be the same as the
    Matrix array dimension. Do not forget to re-generate the erpc code once the
    matrix size is changed in the erpc file */
extern const int32_t matrix_size;

#endif // ERPC_TYPE_DEFINITIONS

/*! @brief MatrixMultiplyService identifiers */
enum _MatrixMultiplyService_ids
{
    kMatrixMultiplyService_service_id = 1,
    kMatrixMultiplyService_erpcMatrixMultiply_id = 1,
};

#if defined(__cplusplus)
extern "C" {
#endif

//! @name MatrixMultiplyService
//@{
void erpcMatrixMultiply(Matrix matrix1, Matrix matrix2, Matrix result_matrix);
//@} 

#if defined(__cplusplus)
}
#endif

#endif // _erpc_matrix_multiply_h_
