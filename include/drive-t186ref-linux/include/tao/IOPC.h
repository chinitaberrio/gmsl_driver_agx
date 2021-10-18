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

#ifndef _TAO_PIDL_IOPC_FDTWOQ_H_
#define _TAO_PIDL_IOPC_FDTWOQ_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/TAO_Export.h"
#include "tao/Basic_Types.h"
#include "tao/String_Manager_T.h"
#include "tao/Sequence_T.h"
#include "tao/Seq_Var_T.h"
#include "tao/Seq_Out_T.h"
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

#include "tao/OctetSeqC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace IOP
{

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:373

  typedef ::CORBA::ULong ProfileId;
  typedef ::CORBA::ULong_out ProfileId_out;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_INTERNET_IOP = 0U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_MULTIPLE_COMPONENTS = 1U;

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct TaggedProfile;

  typedef
    ::TAO_Var_Var_T<
        TaggedProfile
      >
    TaggedProfile_var;

  typedef
    ::TAO_Out_T<
        TaggedProfile
      >
    TaggedProfile_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export TaggedProfile
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef TaggedProfile_var _var_type;
    typedef TaggedProfile_out _out_type;

    static void _tao_any_destructor (void *);
    
    IOP::ProfileId tag;
    CORBA::OctetSeq profile_data;
  };

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_IOP_TAGGEDPROFILESEQ_CH_)
#define _IOP_TAGGEDPROFILESEQ_CH_

  class TaggedProfileSeq;

  typedef
    ::TAO_VarSeq_Var_T<
        TaggedProfileSeq
      >
    TaggedProfileSeq_var;

  typedef
    ::TAO_Seq_Out_T<
        TaggedProfileSeq
      >
    TaggedProfileSeq_out;

  class TAO_Export TaggedProfileSeq
    : public
        ::TAO::unbounded_value_sequence<
            TaggedProfile
          >
  {
  public:
    TaggedProfileSeq (void);
    TaggedProfileSeq ( ::CORBA::ULong max);
    TaggedProfileSeq (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      TaggedProfile* buffer,
      ::CORBA::Boolean release = false);
    TaggedProfileSeq (const TaggedProfileSeq &);
    virtual ~TaggedProfileSeq (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef TaggedProfileSeq_var _var_type;
    typedef TaggedProfileSeq_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct IOR;

  typedef
    ::TAO_Var_Var_T<
        IOR
      >
    IOR_var;

  typedef
    ::TAO_Out_T<
        IOR
      >
    IOR_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export IOR
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef IOR_var _var_type;
    typedef IOR_out _out_type;

    static void _tao_any_destructor (void *);
    
    ::TAO::String_Manager type_id;
    IOP::TaggedProfileSeq profiles;
  };

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:373

  typedef ::CORBA::ULong ComponentId;
  typedef ::CORBA::ULong_out ComponentId_out;

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct TaggedComponent;

  typedef
    ::TAO_Var_Var_T<
        TaggedComponent
      >
    TaggedComponent_var;

  typedef
    ::TAO_Out_T<
        TaggedComponent
      >
    TaggedComponent_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export TaggedComponent
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef TaggedComponent_var _var_type;
    typedef TaggedComponent_out _out_type;

    static void _tao_any_destructor (void *);
    
    IOP::ComponentId tag;
    CORBA::OctetSeq component_data;
  };

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_IOP_MULTIPLECOMPONENTPROFILE_CH_)
#define _IOP_MULTIPLECOMPONENTPROFILE_CH_

  class MultipleComponentProfile;

  typedef
    ::TAO_VarSeq_Var_T<
        MultipleComponentProfile
      >
    MultipleComponentProfile_var;

  typedef
    ::TAO_Seq_Out_T<
        MultipleComponentProfile
      >
    MultipleComponentProfile_out;

  class TAO_Export MultipleComponentProfile
    : public
        ::TAO::unbounded_value_sequence<
            TaggedComponent
          >
  {
  public:
    MultipleComponentProfile (void);
    MultipleComponentProfile ( ::CORBA::ULong max);
    MultipleComponentProfile (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      TaggedComponent* buffer,
      ::CORBA::Boolean release = false);
    MultipleComponentProfile (const MultipleComponentProfile &);
    virtual ~MultipleComponentProfile (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef MultipleComponentProfile_var _var_type;
    typedef MultipleComponentProfile_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_IOP_TAGGEDCOMPONENTSEQ_CH_)
#define _IOP_TAGGEDCOMPONENTSEQ_CH_

  class TaggedComponentSeq;

  typedef
    ::TAO_VarSeq_Var_T<
        TaggedComponentSeq
      >
    TaggedComponentSeq_var;

  typedef
    ::TAO_Seq_Out_T<
        TaggedComponentSeq
      >
    TaggedComponentSeq_out;

  class TAO_Export TaggedComponentSeq
    : public
        ::TAO::unbounded_value_sequence<
            TaggedComponent
          >
  {
  public:
    TaggedComponentSeq (void);
    TaggedComponentSeq ( ::CORBA::ULong max);
    TaggedComponentSeq (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      TaggedComponent* buffer,
      ::CORBA::Boolean release = false);
    TaggedComponentSeq (const TaggedComponentSeq &);
    virtual ~TaggedComponentSeq (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef TaggedComponentSeq_var _var_type;
    typedef TaggedComponentSeq_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_ORB_TYPE = 0U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_CODE_SETS = 1U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_POLICIES = 2U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_ALTERNATE_IIOP_ADDRESS = 3U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_COMPLETE_OBJECT_KEY = 5U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_ENDPOINT_ID_POSITION = 6U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_LOCATION_POLICY = 12U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_DCE_STRING_BINDING = 100U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_DCE_BINDING_NAME = 101U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_DCE_NO_PIPES = 102U;

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:373

  typedef ::CORBA::ULong ServiceId;
  typedef ::CORBA::ULong_out ServiceId_out;

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct ServiceContext;

  typedef
    ::TAO_Var_Var_T<
        ServiceContext
      >
    ServiceContext_var;

  typedef
    ::TAO_Out_T<
        ServiceContext
      >
    ServiceContext_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export ServiceContext
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef ServiceContext_var _var_type;
    typedef ServiceContext_out _out_type;

    static void _tao_any_destructor (void *);
    
    IOP::ServiceId context_id;
    CORBA::OctetSeq context_data;
  };

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_IOP_SERVICECONTEXTLIST_CH_)
#define _IOP_SERVICECONTEXTLIST_CH_

  class ServiceContextList;

  typedef
    ::TAO_VarSeq_Var_T<
        ServiceContextList
      >
    ServiceContextList_var;

  typedef
    ::TAO_Seq_Out_T<
        ServiceContextList
      >
    ServiceContextList_out;

  class TAO_Export ServiceContextList
    : public
        ::TAO::unbounded_value_sequence<
            ServiceContext
          >
  {
  public:
    ServiceContextList (void);
    ServiceContextList ( ::CORBA::ULong max);
    ServiceContextList (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      ServiceContext* buffer,
      ::CORBA::Boolean release = false);
    ServiceContextList (const ServiceContextList &);
    virtual ~ServiceContextList (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef ServiceContextList_var _var_type;
    typedef ServiceContextList_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TransactionService = 0U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong CodeSets = 1U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong ChainBypassCheck = 2U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong ChainBypassInfo = 3U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong LogicalThreadId = 4U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong BI_DIR_IIOP = 5U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong SendingContextRunTime = 6U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong INVOCATION_POLICIES = 7U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong UnknownExceptionInfo = 9U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong RTCorbaPriority = 10U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong RTCorbaPriorityRange = 11U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_FT_GROUP = 27U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_FT_PRIMARY = 28U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_FT_HEARTBEAT_ENABLED = 29U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong FT_GROUP_VERSION = 12U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong FT_REQUEST = 13U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong REP_NWPRIORITY = 1413545989U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_UIPMC = 3U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_GROUP = 39U;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::ULong TAG_GROUP_IIOP = 40U;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module IOP

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::IOP::TaggedProfile>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::TaggedProfile,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::IOP::TaggedProfileSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::TaggedProfileSeq,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::IOP::IOR>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::IOR,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::IOP::TaggedComponent>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::TaggedComponent,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::IOP::MultipleComponentProfile>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::MultipleComponentProfile,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::IOP::TaggedComponentSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::TaggedComponentSeq,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::IOP::ServiceContext>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::ServiceContext,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::IOP::ServiceContextList>
    : public
        Var_Size_Arg_Traits_T<
            ::IOP::ServiceContextList,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };
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
}
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const IOP::TaggedProfile &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, IOP::TaggedProfile &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_IOP_TaggedProfileSeq_H_
#define _TAO_CDR_OP_IOP_TaggedProfileSeq_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const IOP::TaggedProfileSeq &_tao_sequence);
TAO_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    IOP::TaggedProfileSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_IOP_TaggedProfileSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const IOP::IOR &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, IOP::IOR &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const IOP::TaggedComponent &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, IOP::TaggedComponent &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_IOP_MultipleComponentProfile_H_
#define _TAO_CDR_OP_IOP_MultipleComponentProfile_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const IOP::MultipleComponentProfile &_tao_sequence);
TAO_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    IOP::MultipleComponentProfile &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_IOP_MultipleComponentProfile_H_ */

// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_IOP_TaggedComponentSeq_H_
#define _TAO_CDR_OP_IOP_TaggedComponentSeq_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const IOP::TaggedComponentSeq &_tao_sequence);
TAO_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    IOP::TaggedComponentSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_IOP_TaggedComponentSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const IOP::ServiceContext &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, IOP::ServiceContext &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_IOP_ServiceContextList_H_
#define _TAO_CDR_OP_IOP_ServiceContextList_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const IOP::ServiceContextList &_tao_sequence);
TAO_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    IOP::ServiceContextList &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_IOP_ServiceContextList_H_ */

// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#include /**/ "ace/post.h"

#endif /* ifndef */

