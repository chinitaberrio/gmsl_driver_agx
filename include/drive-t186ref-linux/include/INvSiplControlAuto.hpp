/*
   * Copyright (c) 2020 NVIDIA Corporation.  All rights reserved.
   *
   * NVIDIA Corporation and its licensors retain all intellectual property
   * and proprietary rights in and to this software and related documentation
   * and any modifications thereto.  Any use, reproduction, disclosure or
   * distribution of this software and related documentation without an express
   * license agreement from NVIDIA Corporation is strictly prohibited.
   */

/* NVIDIA SIPL Control Auto Interface */

#ifndef NVSIPLCONTROLAUTOINTERFACE_HPP
#define NVSIPLCONTROLAUTOINTERFACE_HPP


#include "NvSIPLCommon.hpp"
#include "NvSiplControlAutoDef.hpp"

/**
 * @file
 *
 * @brief <b> NVIDIA SIPL: Auto Control Interface - @ref NvSIPLAutoControl </b>
 *
 */

/** @defgroup NvSIPLAutoControl SIPL Auto Control
 *
 * @brief Describes interfaces for SIPL Auto Control implementation.
 *
 * @ingroup NvSIPL */

namespace nvsipl
{

/** @ingroup NvSIPLAutoControl
 * @{
 */

/** @class ISiplControlAuto INvSiplControlAuto.hpp
  *
  * @brief Defines SIPL Control Auto Interface Class
  */
class ISiplControlAuto {

public:
    /** @brief Function to process auto (AE/AWB) algorithm
      * @param[in] inParams @ref SiplControlAutoInputParam
      * @param[out] outParams @ref SiplControlAutoOutputParam
      * @returns::SIPLStatus the completion status of the operation.
      */
    virtual SIPLStatus Process(const SiplControlAutoInputParam& inParams,
                               SiplControlAutoOutputParam& outParams) = 0;


    /** @brief Default destructor. */
    virtual ~ISiplControlAuto() = default;

protected:
    /** @brief Default constructor. */
    ISiplControlAuto() = default;

private:

    /** @brief disable copy constructor */
    ISiplControlAuto(const ISiplControlAuto&) = delete;
    /** @brief disable assignment operation */
    ISiplControlAuto& operator= (const ISiplControlAuto&) = delete;
};

/** @} */

} // namespace nvsipl

#endif // NVSIPLCONTROLAUTOINTERFACE_HPP
