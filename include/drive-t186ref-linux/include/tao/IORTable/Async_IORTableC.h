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

#ifndef _TAO_PIDL_ASYNC_IORTABLEC_8L2OK7_H_
#define _TAO_PIDL_ASYNC_IORTABLEC_8L2OK7_H_

#include /**/ "ace/pre.h"

#ifndef TAO_IORTABLE_SAFE_INCLUDE
#error You should not include Async_IORTableC.h, use tao/IORTable/Async_IORTable.h
#endif /* TAO_IORTABLE_SAFE_INCLUDE */

#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/IORTable/async_iortable_export.h"
#include "tao/SystemException.h"
#include "tao/UserException.h"
#include "tao/Basic_Types.h"
#include "tao/ORB_Constants.h"
#include "tao/Object.h"
#include "tao/Objref_VarOut_T.h"
#include "tao/VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/UB_String_Arguments.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"

#include "tao/IORTable/IORTableC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_Async_IORTable_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace IORTable
{

  // TAO_IDL - Generated from
  // be/be_visitor_native/native_ch.cpp:46

  

  // TAO_IDL - Generated from
  // be/be_interface.cpp:751

#if !defined (_IORTABLE_ASYNCLOCATOR__VAR_OUT_CH_)
#define _IORTABLE_ASYNCLOCATOR__VAR_OUT_CH_

  class AsyncLocator;
  typedef AsyncLocator *AsyncLocator_ptr;

  typedef
    TAO_Objref_Var_T<
        AsyncLocator
      >
    AsyncLocator_var;
  
  typedef
    TAO_Objref_Out_T<
        AsyncLocator
      >
    AsyncLocator_out;

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_interface/interface_ch.cpp:43

  class TAO_Async_IORTable_Export AsyncLocator
    : public virtual ::IORTable::Locator
  
  {
  public:

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    typedef AsyncLocator_ptr _ptr_type;
    typedef AsyncLocator_var _var_type;
    typedef AsyncLocator_out _out_type;

    // The static operations.
    static AsyncLocator_ptr _duplicate (AsyncLocator_ptr obj);

    static void _tao_release (AsyncLocator_ptr obj);

    static AsyncLocator_ptr _narrow (::CORBA::Object_ptr obj);
    static AsyncLocator_ptr _unchecked_narrow (::CORBA::Object_ptr obj);
    static AsyncLocator_ptr _nil (void);

    virtual void async_locate (
      ::IORTable::Locate_ResponseHandler rh,
      const char * object_key) = 0;

    // TAO_IDL - Generated from
    // be/be_visitor_interface/interface_ch.cpp:140

    virtual ::CORBA::Boolean _is_a (const char *type_id);
    virtual const char* _interface_repository_id (void) const;
    virtual ::CORBA::Boolean marshal (TAO_OutputCDR &cdr);
  
  protected:
    // Abstract or local interface only.
    AsyncLocator (void);

    

    virtual ~AsyncLocator (void);
  
  private:
    // Private and unimplemented for concrete interfaces.
    AsyncLocator (const AsyncLocator &);

    void operator= (const AsyncLocator &);
  };

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module IORTable

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

#if !defined (_IORTABLE_ASYNCLOCATOR__TRAITS_)
#define _IORTABLE_ASYNCLOCATOR__TRAITS_

  template<>
  struct TAO_Async_IORTable_Export Objref_Traits< ::IORTable::AsyncLocator>
  {
    static ::IORTable::AsyncLocator_ptr duplicate (
        ::IORTable::AsyncLocator_ptr p);
    static void release (
        ::IORTable::AsyncLocator_ptr p);
    static ::IORTable::AsyncLocator_ptr nil (void);
    static ::CORBA::Boolean marshal (
        const ::IORTable::AsyncLocator_ptr p,
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

