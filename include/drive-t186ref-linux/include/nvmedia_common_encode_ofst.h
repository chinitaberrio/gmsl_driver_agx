/*
 * Copyright (c) 2019 NVIDIA CORPORATION. All rights reserved. All
 * information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */


/**
 * \file
 * \brief <b> NVIDIA Media Interface: Common Types for Video/Image Encode and OFST</b>
 *
 * @b Description: This file contains common types and definitions for image/video
 * encode and OFST operations.
 *
 */

#ifndef NVMEDIA_COMMON_ENCODE_OFST_H
#define NVMEDIA_COMMON_ENCODE_OFST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup common_types_encode_ofst_top Image/Video Encode and OFST
 * \ingroup nvmedia_common_top
 *
 * \brief Defines common types and declarations for image/video encode and
 * OFST operations.
 * @{
 */
/**
 * \brief Specifies the encoder instance ID
 */
typedef enum {
    /** \brief Specifies the encoder instance ID 0 */
    NVMEDIA_ENCODER_INSTANCE_0 = 0,
    /** \brief Specifies the encoder instance ID 1 */
    NVMEDIA_ENCODER_INSTANCE_1,
   /** \brief Specifies that the encoder instance ID
    * can be set dynamically during encode
    */
    NVMEDIA_ENCODER_INSTANCE_AUTO,
} NvMediaEncoderInstanceId;

/** @} <!-- Ends common_types_encode_ofst_top NvMedia Types and Structures common to Video/Image Encode and OFST --> */



#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* NVMEDIA_COMMON_ENCODE_OFST_H */
