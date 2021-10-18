// -*- C++ -*-
// $Id$

/**
 * Code generated by the The ACE ORB (TAO) IDL Compiler v2.2a_p14
 * TAO and the TAO IDL Compiler have been developed by:
 *       Center for Distributed Object Computing
 *       Washington University
 *       St. Louis, MO
 *       USA
 *       http://www.cs.wustl.edu/~schmidt/doc-center.html
 * and
 *       Distributed Object Computing Laboratory
 *       University of California at Irvine
 *       Irvine, CA
 *       USA
 * and
 *       Institute for Software Integrated Systems
 *       Vanderbilt University
 *       Nashville, TN
 *       USA
 *       http://www.isis.vanderbilt.edu/
 *
 * Information about TAO is available at:
 *     http://www.cs.wustl.edu/~schmidt/TAO.html
 **/

// TAO_IDL - Generated from
// be/be_codegen.cpp:152

#ifndef _TAO_PIDL_ORBINITIALIZERC_LF2QBT_H_
#define _TAO_PIDL_ORBINITIALIZERC_LF2QBT_H_

#include /**/ "ace/pre.h"

#ifndef TAO_PI_SAFE_INCLUDE
#error You should not include ORBInitializerC.h, use tao/PI/PI.h
#endif /* TAO_PI_SAFE_INCLUDE */

#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/PI/pi_export.h"
#include "tao/SystemException.h"
#include "tao/Basic_Types.h"
#include "tao/ORB_Constants.h"
#include "tao/Object.h"
#include "tao/AnyTypeCode/TypeCode.h"
#include "tao/AnyTypeCode/TypeCode_Constants.h"
#include "tao/String_Manager_T.h"
#include "tao/Objref_VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/Object_Argument_T.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"

#include "tao/PI/ORBInitInfoC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_PI_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace PortableInterceptor
{

  // TAO_IDL - Generated from
  // be/be_interface.cpp:751

#if !defined (_PORTABLEINTERCEPTOR_ORBINITIALIZER__VAR_OUT_CH_)
#define _PORTABLEINTERCEPTOR_ORBINITIALIZER__VAR_OUT_CH_

  class ORBInitializer;
  typedef ORBInitializer *ORBInitializer_ptr;

  typedef
    TAO_Objref_Var_T<
        ORBInitializer
      >
    ORBInitializer_var;
  
  typedef
    TAO_Objref_Out_T<
        ORBInitializer
      >
    ORBInitializer_out;

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_interface/interface_ch.cpp:43

  class TAO_PI_Export ORBInitializer
    : public virtual ::CORBA::Object
  {
  public:

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    typedef ORBInitializer_ptr _ptr_type;
    typedef ORBInitializer_var _var_type;
    typedef ORBInitializer_out _out_type;

    // The static operations.
    static ORBInitializer_ptr _duplicate (ORBInitializer_ptr obj);

    static void _tao_release (ORBInitializer_ptr obj);

    static ORBInitializer_ptr _narrow (::CORBA::Object_ptr obj);
    static ORBInitializer_ptr _unchecked_narrow (::CORBA::Object_ptr obj);
    static ORBInitializer_ptr _nil (void);

    virtual void pre_init (
      ::PortableInterceptor::ORBInitInfo_ptr info) = 0;

    virtual void post_init (
      ::PortableInterceptor::ORBInitInfo_ptr info) = 0;

    // TAO_IDL - Generated from
    // be/be_visitor_interface/interface_ch.cpp:140

    virtual ::CORBA::Boolean _is_a (const char *type_id);
    virtual const char* _interface_repository_id (void) const;
    virtual ::CORBA::Boolean marshal (TAO_OutputCDR &cdr);
  
  protected:
    // Abstract or local interface only.
    ORBInitializer (void);

    

    virtual ~ORBInitializer (void);
  
  private:
    // Private and unimplemented for concrete interfaces.
    ORBInitializer (const ORBInitializer &);

    void operator= (const ORBInitializer &);
  };

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module PortableInterceptor

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{
}

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_traits.cpp:62


TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

// Traits specializations.
namespace TAO
{

#if !defined (_PORTABLEINTERCEPTOR_ORBINITIALIZER__TRAITS_)
#define _PORTABLEINTERCEPTOR_ORBINITIALIZER__TRAITS_

  template<>
  struct TAO_PI_Export Objref_Traits< ::PortableInterceptor::ORBInitializer>
  {
    static ::PortableInterceptor::ORBInitializer_ptr duplicate (
        ::PortableInterceptor::ORBInitializer_ptr p);
    static void release (
        ::PortableInterceptor::ORBInitializer_ptr p);
    static ::PortableInterceptor::ORBInitializer_ptr nil (void);
    static ::CORBA::Boolean marshal (
        const ::PortableInterceptor::ORBInitializer_ptr p,
        TAO_OutputCDR & cdr);
  };

#endif /* end #if !defined */
}
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#include /**/ "ace/post.h"

#endif /* ifndef */

