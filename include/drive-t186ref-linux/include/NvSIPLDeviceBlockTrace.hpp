/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
#ifndef INVSIPLDEVBLKTRACE_H
#define INVSIPLDEVBLKTRACE_H

/**
 * @file
 *
 * <b> NVIDIA Sensor Input Processing Library: DeviceBlock Trace Interface - @ref NvSIPLDevBlk_API </b>
 *
 */

namespace nvsipl
{

/**@ingroup NvSIPLDevBlk_API
 * @{
 */

/** @class INvSIPLDeviceBlockTrace NvSIPLDeviceBlockTrace.hpp
 *
 * @brief Describes the interfaces of NvSIPLDeviceBlockTrace.
 *
 * Defines the public interfaces to control the logging/tracing
 * of the @ref NvSIPLDevBlk_API for debug purposes.
 */
class INvSIPLDeviceBlockTrace
{
public:

   /** @brief Defines tracing/logging levels. */
   enum TraceLevel
   {
       LevelNone = 0, /**< Indicates logging is turned off. */
       LevelError,    /**< Indicates logging is turned on for errors. */
       LevelWarning,  /**< Indicates logging is turned on for critical warnings. */
       LevelInfo,     /**< Indicates logging is turned on for information level messages. */
       LevelDebug     /**< Indicates logging is turned on for every print statement. */
   };

   using TraceFuncPtr = void(*)(const char*, int);

   /** @brief Gets a handle to INvSIPLDeviceBlockTrace instance.
    *
    * Static function to get a handle to singleton INvSIPLDeviceBlockTrace implementation object.
    *
    * @returns pointer to INvSIPLDeviceBlockTrace. */
   static INvSIPLDeviceBlockTrace* GetInstance(void);

   /** @brief Sets a callable trace hook.
    *
    * Function to set a callable hook to receive the messages from the library.
    * @param[in] traceHook @c std::function object, which could be a functor,
    * function pointer, or a lambda. The function object must include
    * @c const @c char* message and number of chars as arguments.
    *
    * @param[in] bCallDefaultRenderer Boolean flag indicating if the message should be printed
    * to the default renderer (stderr). */
   virtual void SetHook(TraceFuncPtr traceHook,
                        bool bCallDefaultRenderer) = 0;

   /** @brief Sets the log level.
    *
    * Function to set the level of logging.
    * Each trace statement specifies a trace level for that statement, and all traces
    * with a level greater than or equal to the current application trace level will be
    * rendered at runtime.  Traces with a level below the application trace level will
    * be ignored. The application trace level can be changed at any time to render additional
    * or fewer trace statements.
    *
    * @param[in] eLevel Trace level \ref TraceLevel. */
   virtual void SetLevel(TraceLevel eLevel) = 0;

   /** @brief Disable line info (__FUNCTION__ : __LINE__: ) prefix
    *
    * Function to disable line information prefix.
    * Each log/trace is prefixed with function name and the line number.
    * Calling this function will disable the prefix. */
   virtual void DisableLineInfo(void) = 0;

   /** @brief Default destructor. */
   virtual ~INvSIPLDeviceBlockTrace() = default;
};

/** @} */

} // namespace nvsipl


#endif // INVSIPLDEVBLKTRACE_H
