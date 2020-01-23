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

#ifndef _TAO_IDL_DATAWRITERREMOTEC_FALOMJ_H_
#define _TAO_IDL_DATAWRITERREMOTEC_FALOMJ_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "InfoRepoDiscovery_Export.h"
#include "tao/ORB.h"
#include "tao/SystemException.h"
#include "tao/Basic_Types.h"
#include "tao/ORB_Constants.h"
#include "dds/DCPS/ZeroCopyInfoSeq_T.h"
#include "tao/Object.h"
#include "tao/String_Manager_T.h"
#include "tao/Objref_VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/Object_Argument_T.h"
#include "tao/Special_Basic_Arguments.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"
#include /**/ "dds/Versioned_Namespace.h"

#include "dds/DdsDcpsInfoUtilsC.h"
#include "dds/DdsDcpsPublicationC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO OpenDDS_InfoRepoDiscovery_Export

OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_root/root_ch.cpp:160

OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



namespace TAO
{
  template<typename T> class Narrow_Utils;
}
TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace OpenDDS
{

  // TAO_IDL - Generated from
  // be/be_visitor_module/module_ch.cpp:38

  namespace DCPS
  {

    // TAO_IDL - Generated from
    // be/be_interface.cpp:751

#if !defined (_OPENDDS_DCPS_DATAWRITERREMOTE__VAR_OUT_CH_)
#define _OPENDDS_DCPS_DATAWRITERREMOTE__VAR_OUT_CH_

    class DataWriterRemote;
    typedef DataWriterRemote *DataWriterRemote_ptr;

    typedef
      TAO_Objref_Var_T<
          DataWriterRemote
        >
      DataWriterRemote_var;
    
    typedef
      TAO_Objref_Out_T<
          DataWriterRemote
        >
      DataWriterRemote_out;

#endif /* end #if !defined */

    // TAO_IDL - Generated from
    // be/be_visitor_interface/interface_ch.cpp:43

    class OpenDDS_InfoRepoDiscovery_Export DataWriterRemote
      : public virtual ::CORBA::Object
    {
    public:
      friend class TAO::Narrow_Utils<DataWriterRemote>;

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      typedef DataWriterRemote_ptr _ptr_type;
      typedef DataWriterRemote_var _var_type;
      typedef DataWriterRemote_out _out_type;

      // The static operations.
      static DataWriterRemote_ptr _duplicate (DataWriterRemote_ptr obj);

      static void _tao_release (DataWriterRemote_ptr obj);

      static DataWriterRemote_ptr _narrow (::CORBA::Object_ptr obj);
      static DataWriterRemote_ptr _unchecked_narrow (::CORBA::Object_ptr obj);
      static DataWriterRemote_ptr _nil (void);

      virtual void add_association (
        const ::OpenDDS::DCPS::RepoId & yourId,
        const ::OpenDDS::DCPS::ReaderAssociation & reader,
        ::CORBA::Boolean active);

      virtual void association_complete (
        const ::OpenDDS::DCPS::RepoId & remote_id);

      virtual void remove_associations (
        const ::OpenDDS::DCPS::ReaderIdSeq & readers,
        ::CORBA::Boolean notify_lost);

      virtual void update_incompatible_qos (
        const ::OpenDDS::DCPS::IncompatibleQosStatus & status);

      virtual void update_subscription_params (
        const ::OpenDDS::DCPS::RepoId & readerId,
        const ::DDS::StringSeq & exprParams);

      // TAO_IDL - Generated from
      // be/be_visitor_interface/interface_ch.cpp:140

      virtual ::CORBA::Boolean _is_a (const char *type_id);
      virtual const char* _interface_repository_id (void) const;
      virtual ::CORBA::Boolean marshal (TAO_OutputCDR &cdr);
    
    protected:
      // Concrete interface only.
      DataWriterRemote (void);

      // Concrete non-local interface only.
      DataWriterRemote (
          ::IOP::IOR *ior,
          TAO_ORB_Core *orb_core);
      
      // Non-local interface only.
      DataWriterRemote (
          TAO_Stub *objref,
          ::CORBA::Boolean _tao_collocated = false,
          TAO_Abstract_ServantBase *servant = 0,
          TAO_ORB_Core *orb_core = 0);

      virtual ~DataWriterRemote (void);
    
    private:
      // Private and unimplemented for concrete interfaces.
      DataWriterRemote (const DataWriterRemote &);

      void operator= (const DataWriterRemote &);
    };
  
  // TAO_IDL - Generated from
  // be/be_visitor_module/module_ch.cpp:67
  
  } // module OpenDDS::DCPS

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module OpenDDS

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:147

#if !defined (_OPENDDS_DCPS_DATAWRITERREMOTE__ARG_TRAITS_)
#define _OPENDDS_DCPS_DATAWRITERREMOTE__ARG_TRAITS_

  template<>
  class  Arg_Traits< ::OpenDDS::DCPS::DataWriterRemote>
    : public
        Object_Arg_Traits_T<
            ::OpenDDS::DCPS::DataWriterRemote_ptr,
            ::OpenDDS::DCPS::DataWriterRemote_var,
            ::OpenDDS::DCPS::DataWriterRemote_out,
            TAO::Objref_Traits<OpenDDS::DCPS::DataWriterRemote>,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

#endif /* end #if !defined */
}

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_traits.cpp:62


OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

// Traits specializations.
namespace TAO
{

#if !defined (_OPENDDS_DCPS_DATAWRITERREMOTE__TRAITS_)
#define _OPENDDS_DCPS_DATAWRITERREMOTE__TRAITS_

  template<>
  struct OpenDDS_InfoRepoDiscovery_Export Objref_Traits< ::OpenDDS::DCPS::DataWriterRemote>
  {
    static ::OpenDDS::DCPS::DataWriterRemote_ptr duplicate (
        ::OpenDDS::DCPS::DataWriterRemote_ptr p);
    static void release (
        ::OpenDDS::DCPS::DataWriterRemote_ptr p);
    static ::OpenDDS::DCPS::DataWriterRemote_ptr nil (void);
    static ::CORBA::Boolean marshal (
        const ::OpenDDS::DCPS::DataWriterRemote_ptr p,
        TAO_OutputCDR & cdr);
  };

#endif /* end #if !defined */
}
TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_interface/cdr_op_ch.cpp:44


OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_InfoRepoDiscovery_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::DataWriterRemote_ptr );
OpenDDS_InfoRepoDiscovery_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::DataWriterRemote_ptr &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


OPENDDS_END_VERSIONED_NAMESPACE_DECL

#if defined (__ACE_INLINE__)
#include "DataWriterRemoteC.inl"
#endif /* defined INLINE */

#include /**/ "ace/post.h"

#endif /* ifndef */

