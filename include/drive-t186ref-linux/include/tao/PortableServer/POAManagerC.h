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

#ifndef _TAO_PIDL_POAMANAGERC_HBVOTR_H_
#define _TAO_PIDL_POAMANAGERC_HBVOTR_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/PortableServer/portableserver_export.h"
#include "tao/AnyTypeCode/AnyTypeCode_methods.h"
#include "tao/AnyTypeCode/Any.h"
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
#include "tao/Basic_Argument_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/UB_String_Arguments.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_PortableServer_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace PortableServer
{

  // TAO_IDL - Generated from
  // be/be_interface.cpp:751

#if !defined (_PORTABLESERVER_POAMANAGER__VAR_OUT_CH_)
#define _PORTABLESERVER_POAMANAGER__VAR_OUT_CH_

  class POAManager;
  typedef POAManager *POAManager_ptr;

  typedef
    TAO_Objref_Var_T<
        POAManager
      >
    POAManager_var;
  
  typedef
    TAO_Objref_Out_T<
        POAManager
      >
    POAManager_out;

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_interface/interface_ch.cpp:43

  class TAO_PortableServer_Export POAManager
    : public virtual ::CORBA::Object
  {
  public:

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    typedef POAManager_ptr _ptr_type;
    typedef POAManager_var _var_type;
    typedef POAManager_out _out_type;

    static void _tao_any_destructor (void *);

    // The static operations.
    static POAManager_ptr _duplicate (POAManager_ptr obj);

    static void _tao_release (POAManager_ptr obj);

    static POAManager_ptr _narrow (::CORBA::Object_ptr obj);
    static POAManager_ptr _unchecked_narrow (::CORBA::Object_ptr obj);
    static POAManager_ptr _nil (void);

    // TAO_IDL - Generated from
    // be/be_visitor_exception/exception_ch.cpp:44

    class TAO_PortableServer_Export AdapterInactive : public ::CORBA::UserException
    {
    public:

      AdapterInactive (void);
      AdapterInactive (const AdapterInactive &);
      ~AdapterInactive (void);

      AdapterInactive &operator= (const AdapterInactive &);

      static void _tao_any_destructor (void *);

      static AdapterInactive *_downcast ( ::CORBA::Exception *);
      static const AdapterInactive *_downcast ( ::CORBA::Exception const *);

      static ::CORBA::Exception *_alloc (void);

      virtual ::CORBA::Exception *_tao_duplicate (void) const;

      virtual void _raise (void) const;

      virtual void _tao_encode (TAO_OutputCDR &cdr) const;
      virtual void _tao_decode (TAO_InputCDR &cdr);

      virtual ::CORBA::TypeCode_ptr _tao_type (void) const;
    };

    // TAO_IDL - Generated from
    // be/be_visitor_typecode/typecode_decl.cpp:37

    static ::CORBA::TypeCode_ptr const _tc_AdapterInactive;

    // TAO_IDL - Generated from
    // be/be_visitor_enum/enum_ch.cpp:47

    enum State
    {
      HOLDING,
      ACTIVE,
      DISCARDING,
      INACTIVE
    };

    typedef State &State_out;

    // TAO_IDL - Generated from
    // be/be_visitor_typecode/typecode_decl.cpp:37

    static ::CORBA::TypeCode_ptr const _tc_State;

    virtual void activate (
      void) = 0;

    virtual void hold_requests (
      ::CORBA::Boolean wait_for_completion) = 0;

    virtual void discard_requests (
      ::CORBA::Boolean wait_for_completion) = 0;

    virtual void deactivate (
      ::CORBA::Boolean etherealize_objects,
      ::CORBA::Boolean wait_for_completion) = 0;

    virtual ::PortableServer::POAManager::State get_state (
      void) = 0;

    virtual char * get_id (
      void) = 0;

    // TAO_IDL - Generated from
    // be/be_visitor_interface/interface_ch.cpp:140

    virtual ::CORBA::Boolean _is_a (const char *type_id);
    virtual const char* _interface_repository_id (void) const;
    virtual ::CORBA::Boolean marshal (TAO_OutputCDR &cdr);
  
  protected:
    // Abstract or local interface only.
    POAManager (void);

    

    virtual ~POAManager (void);
  
  private:
    // Private and unimplemented for concrete interfaces.
    POAManager (const POAManager &);

    void operator= (const POAManager &);
  };

  // TAO_IDL - Generated from
  // be/be_visitor_typecode/typecode_decl.cpp:37

  extern TAO_PortableServer_Export ::CORBA::TypeCode_ptr const _tc_POAManager;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module PortableServer

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

#if !defined (_PORTABLESERVER_POAMANAGER__TRAITS_)
#define _PORTABLESERVER_POAMANAGER__TRAITS_

  template<>
  struct TAO_PortableServer_Export Objref_Traits< ::PortableServer::POAManager>
  {
    static ::PortableServer::POAManager_ptr duplicate (
        ::PortableServer::POAManager_ptr p);
    static void release (
        ::PortableServer::POAManager_ptr p);
    static ::PortableServer::POAManager_ptr nil (void);
    static ::CORBA::Boolean marshal (
        const ::PortableServer::POAManager_ptr p,
        TAO_OutputCDR & cdr);
  };

#endif /* end #if !defined */
}
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_interface/any_op_ch.cpp:44



#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace PortableServer
{
  TAO_PortableServer_Export void operator<<= ( ::CORBA::Any &, POAManager_ptr); // copying
  TAO_PortableServer_Export void operator<<= ( ::CORBA::Any &, POAManager_ptr *); // non-copying
  TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, POAManager_ptr &);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_PortableServer_Export void operator<<= (::CORBA::Any &, PortableServer::POAManager_ptr); // copying
TAO_PortableServer_Export void operator<<= (::CORBA::Any &, PortableServer::POAManager_ptr *); // non-copying
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, PortableServer::POAManager_ptr &);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

// TAO_IDL - Generated from
// be/be_visitor_exception/any_op_ch.cpp:41

#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace PortableServer
{
  

  TAO_PortableServer_Export void operator<<= (::CORBA::Any &, const ::PortableServer::POAManager::AdapterInactive &); // copying version
  TAO_PortableServer_Export void operator<<= (::CORBA::Any &, ::PortableServer::POAManager::AdapterInactive*); // noncopying version
  TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, ::PortableServer::POAManager::AdapterInactive *&); // deprecated
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const ::PortableServer::POAManager::AdapterInactive *&);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



TAO_PortableServer_Export void operator<<= (::CORBA::Any &, const PortableServer::POAManager::AdapterInactive &); // copying version
TAO_PortableServer_Export void operator<<= (::CORBA::Any &, PortableServer::POAManager::AdapterInactive*); // noncopying version
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, PortableServer::POAManager::AdapterInactive *&); // deprecated
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const PortableServer::POAManager::AdapterInactive *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

// TAO_IDL - Generated from
// be/be_visitor_enum/any_op_ch.cpp:39

#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace PortableServer
{
  

  TAO_PortableServer_Export void operator<<= (::CORBA::Any &, ::PortableServer::POAManager::State);
  TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, ::PortableServer::POAManager::State &);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



TAO_PortableServer_Export void operator<<= (::CORBA::Any &, PortableServer::POAManager::State);
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, PortableServer::POAManager::State &);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#include /**/ "ace/post.h"

#endif /* ifndef */

