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

#ifndef _TAO_IDL_DDSDCPSINFOUTILSC_0BWGVG_H_
#define _TAO_IDL_DDSDCPSINFOUTILSC_0BWGVG_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "dds/DCPS/dcps_export.h"
#include "tao/ORB.h"
#include "tao/Basic_Types.h"
#include "dds/DCPS/ZeroCopyInfoSeq_T.h"
#include "tao/String_Manager_T.h"
#include "tao/Sequence_T.h"
#include "tao/Seq_Var_T.h"
#include "tao/Seq_Out_T.h"
#include "tao/VarOut_T.h"
#include "tao/Array_VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Basic_Argument_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/BD_String_Argument_T.h"
#include "tao/UB_String_Arguments.h"
#include "tao/Fixed_Array_Argument_T.h"
#include "tao/Var_Array_Argument_T.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"
#include /**/ "dds/Versioned_Namespace.h"

#include "dds/DdsDcpsCoreC.h"
#include "dds/DdsDcpsGuidC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO OpenDDS_Dcps_Export

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
    // be/be_visitor_typedef/typedef_ch.cpp:513

    typedef GUID_t RepoId;
    typedef GUID_t_var RepoId_var;
    typedef GUID_t_out RepoId_out;

    // TAO_IDL - Generated from
    // be/be_visitor_typedef/typedef_ch.cpp:466

    typedef DDS::OctetSeq TransportBLOB;
    typedef DDS::OctetSeq_var TransportBLOB_var;
    typedef DDS::OctetSeq_out TransportBLOB_out;

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct TransportLocator;

    typedef
      ::TAO_Var_Var_T<
          TransportLocator
        >
      TransportLocator_var;

    typedef
      ::TAO_Out_T<
          TransportLocator
        >
      TransportLocator_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export TransportLocator
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef TransportLocator_var _var_type;
      typedef TransportLocator_out _out_type;
      
      ::TAO::String_Manager transport_type;
      OpenDDS::DCPS::TransportBLOB data;
    };

    // TAO_IDL - Generated from
    // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_OPENDDS_DCPS_TRANSPORTLOCATORSEQ_CH_)
#define _OPENDDS_DCPS_TRANSPORTLOCATORSEQ_CH_

    class TransportLocatorSeq;

    typedef
      ::TAO_VarSeq_Var_T<
          TransportLocatorSeq
        >
      TransportLocatorSeq_var;

    typedef
      ::TAO_Seq_Out_T<
          TransportLocatorSeq
        >
      TransportLocatorSeq_out;

    class OpenDDS_Dcps_Export TransportLocatorSeq
      : public
          ::TAO::unbounded_value_sequence<
              TransportLocator
            >
    {
    public:
      TransportLocatorSeq (void);
      TransportLocatorSeq ( ::CORBA::ULong max);
      TransportLocatorSeq (
        ::CORBA::ULong max,
        ::CORBA::ULong length,
        TransportLocator* buffer,
        ::CORBA::Boolean release = false);
      TransportLocatorSeq (const TransportLocatorSeq &);
      virtual ~TransportLocatorSeq (void);
      

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef TransportLocatorSeq_var _var_type;
      typedef TransportLocatorSeq_out _out_type;
    };

#endif /* end #if !defined */

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct IncompatibleQosStatus;

    typedef
      ::TAO_Var_Var_T<
          IncompatibleQosStatus
        >
      IncompatibleQosStatus_var;

    typedef
      ::TAO_Out_T<
          IncompatibleQosStatus
        >
      IncompatibleQosStatus_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export IncompatibleQosStatus
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef IncompatibleQosStatus_var _var_type;
      typedef IncompatibleQosStatus_out _out_type;
      
      ::CORBA::Long total_count;
      ::CORBA::Long count_since_last_send;
      DDS::QosPolicyId_t last_policy_id;
      DDS::QosPolicyCountSeq policies;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct AddDomainStatus;

    typedef
      ::TAO_Fixed_Var_T<
          AddDomainStatus
        >
      AddDomainStatus_var;

    typedef
      AddDomainStatus &
      AddDomainStatus_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export AddDomainStatus
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef AddDomainStatus_var _var_type;
      typedef AddDomainStatus_out _out_type;
      
      OpenDDS::DCPS::RepoId id;
      ::CORBA::Boolean federated;
    };

    // TAO_IDL - Generated from
    // be/be_visitor_enum/enum_ch.cpp:47

    enum TopicStatus
    {
      CREATED,
      ENABLED,
      FOUND,
      NOT_FOUND,
      REMOVED,
      CONFLICTING_TYPENAME,
      PRECONDITION_NOT_MET,
      INTERNAL_ERROR,
      TOPIC_DISABLED
    };

    typedef TopicStatus &TopicStatus_out;

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct WriterAssociation;

    typedef
      ::TAO_Var_Var_T<
          WriterAssociation
        >
      WriterAssociation_var;

    typedef
      ::TAO_Out_T<
          WriterAssociation
        >
      WriterAssociation_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export WriterAssociation
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef WriterAssociation_var _var_type;
      typedef WriterAssociation_out _out_type;
      
      OpenDDS::DCPS::TransportLocatorSeq writerTransInfo;
      OpenDDS::DCPS::RepoId writerId;
      DDS::PublisherQos pubQos;
      DDS::DataWriterQos writerQos;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct ReaderAssociation;

    typedef
      ::TAO_Var_Var_T<
          ReaderAssociation
        >
      ReaderAssociation_var;

    typedef
      ::TAO_Out_T<
          ReaderAssociation
        >
      ReaderAssociation_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export ReaderAssociation
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef ReaderAssociation_var _var_type;
      typedef ReaderAssociation_out _out_type;
      
      OpenDDS::DCPS::TransportLocatorSeq readerTransInfo;
      OpenDDS::DCPS::RepoId readerId;
      DDS::SubscriberQos subQos;
      DDS::DataReaderQos readerQos;
      ::TAO::String_Manager filterClassName;
      ::TAO::String_Manager filterExpression;
      DDS::StringSeq exprParams;
    };

    // TAO_IDL - Generated from
    // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_OPENDDS_DCPS_WRITERIDSEQ_CH_)
#define _OPENDDS_DCPS_WRITERIDSEQ_CH_

    class WriterIdSeq;

    typedef
      ::TAO_FixedSeq_Var_T<
          WriterIdSeq
        >
      WriterIdSeq_var;

    typedef
      ::TAO_Seq_Out_T<
          WriterIdSeq
        >
      WriterIdSeq_out;

    class OpenDDS_Dcps_Export WriterIdSeq
      : public
          ::TAO::unbounded_value_sequence<
              RepoId
            >
    {
    public:
      WriterIdSeq (void);
      WriterIdSeq ( ::CORBA::ULong max);
      WriterIdSeq (
        ::CORBA::ULong max,
        ::CORBA::ULong length,
        RepoId* buffer,
        ::CORBA::Boolean release = false);
      WriterIdSeq (const WriterIdSeq &);
      virtual ~WriterIdSeq (void);
      

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef WriterIdSeq_var _var_type;
      typedef WriterIdSeq_out _out_type;
    };

#endif /* end #if !defined */

    // TAO_IDL - Generated from
    // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_OPENDDS_DCPS_READERIDSEQ_CH_)
#define _OPENDDS_DCPS_READERIDSEQ_CH_

    class ReaderIdSeq;

    typedef
      ::TAO_FixedSeq_Var_T<
          ReaderIdSeq
        >
      ReaderIdSeq_var;

    typedef
      ::TAO_Seq_Out_T<
          ReaderIdSeq
        >
      ReaderIdSeq_out;

    class OpenDDS_Dcps_Export ReaderIdSeq
      : public
          ::TAO::unbounded_value_sequence<
              RepoId
            >
    {
    public:
      ReaderIdSeq (void);
      ReaderIdSeq ( ::CORBA::ULong max);
      ReaderIdSeq (
        ::CORBA::ULong max,
        ::CORBA::ULong length,
        RepoId* buffer,
        ::CORBA::Boolean release = false);
      ReaderIdSeq (const ReaderIdSeq &);
      virtual ~ReaderIdSeq (void);
      

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef ReaderIdSeq_var _var_type;
      typedef ReaderIdSeq_out _out_type;
    };

#endif /* end #if !defined */

    // TAO_IDL - Generated from
    // be/be_visitor_array/array_ch.cpp:54

    typedef ::CORBA::Octet OctetArray16[16];
    typedef ::CORBA::Octet OctetArray16_slice;
    struct OctetArray16_tag {};
    

    typedef
      TAO_FixedArray_Var_T<
          OctetArray16,
          OctetArray16_slice,
          OctetArray16_tag
        >
      OctetArray16_var;

    typedef
      OctetArray16
      OctetArray16_out;

    typedef
      TAO_Array_Forany_T<
          OctetArray16,
          OctetArray16_slice,
          OctetArray16_tag
        >
      OctetArray16_forany;

    TAO_NAMESPACE_STORAGE_CLASS OctetArray16_slice *
    OctetArray16_alloc (void);

    TAO_NAMESPACE_STORAGE_CLASS void
    OctetArray16_free (
        OctetArray16_slice *_tao_slice);
    
    TAO_NAMESPACE_STORAGE_CLASS OctetArray16_slice *
    OctetArray16_dup (
        const OctetArray16_slice *_tao_slice);
    
    TAO_NAMESPACE_STORAGE_CLASS void
    OctetArray16_copy (
        OctetArray16_slice *_tao_to,
        const OctetArray16_slice *_tao_from);

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct Locator_t;

    typedef
      ::TAO_Fixed_Var_T<
          Locator_t
        >
      Locator_t_var;

    typedef
      Locator_t &
      Locator_t_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export Locator_t
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef Locator_t_var _var_type;
      typedef Locator_t_out _out_type;
      
      ::CORBA::Long kind;
      ::CORBA::ULong port;
      OpenDDS::DCPS::OctetArray16 address;
    };

    // TAO_IDL - Generated from
    // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_OPENDDS_DCPS_LOCATORSEQ_CH_)
#define _OPENDDS_DCPS_LOCATORSEQ_CH_

    class LocatorSeq;

    typedef
      ::TAO_FixedSeq_Var_T<
          LocatorSeq
        >
      LocatorSeq_var;

    typedef
      ::TAO_Seq_Out_T<
          LocatorSeq
        >
      LocatorSeq_out;

    class OpenDDS_Dcps_Export LocatorSeq
      : public
          ::TAO::unbounded_value_sequence<
              Locator_t
            >
    {
    public:
      LocatorSeq (void);
      LocatorSeq ( ::CORBA::ULong max);
      LocatorSeq (
        ::CORBA::ULong max,
        ::CORBA::ULong length,
        Locator_t* buffer,
        ::CORBA::Boolean release = false);
      LocatorSeq (const LocatorSeq &);
      virtual ~LocatorSeq (void);
      

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef LocatorSeq_var _var_type;
      typedef LocatorSeq_out _out_type;
    };

#endif /* end #if !defined */

    // TAO_IDL - Generated from
    // be/be_visitor_typedef/typedef_ch.cpp:407

    typedef char * String256;
    typedef ::CORBA::String_var String256_var;
    typedef ::CORBA::String_out String256_out;

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct ContentFilterProperty_t;

    typedef
      ::TAO_Var_Var_T<
          ContentFilterProperty_t
        >
      ContentFilterProperty_t_var;

    typedef
      ::TAO_Out_T<
          ContentFilterProperty_t
        >
      ContentFilterProperty_t_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export ContentFilterProperty_t
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef ContentFilterProperty_t_var _var_type;
      typedef ContentFilterProperty_t_out _out_type;
      
      ::TAO::String_Manager contentFilteredTopicName;
      ::TAO::String_Manager relatedTopicName;
      ::TAO::String_Manager filterClassName;
      ::TAO::String_Manager filterExpression;
      DDS::StringSeq expressionParameters;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct ReaderProxy_t;

    typedef
      ::TAO_Var_Var_T<
          ReaderProxy_t
        >
      ReaderProxy_t_var;

    typedef
      ::TAO_Out_T<
          ReaderProxy_t
        >
      ReaderProxy_t_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export ReaderProxy_t
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef ReaderProxy_t_var _var_type;
      typedef ReaderProxy_t_out _out_type;
      
      OpenDDS::DCPS::GUID_t remoteReaderGuid;
      ::CORBA::Boolean expectsInlineQos;
      OpenDDS::DCPS::LocatorSeq unicastLocatorList;
      OpenDDS::DCPS::LocatorSeq multicastLocatorList;
      OpenDDS::DCPS::TransportLocatorSeq allLocators;
      OpenDDS::DCPS::GUIDSeq associatedWriters;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct DiscoveredReaderData;

    typedef
      ::TAO_Var_Var_T<
          DiscoveredReaderData
        >
      DiscoveredReaderData_var;

    typedef
      ::TAO_Out_T<
          DiscoveredReaderData
        >
      DiscoveredReaderData_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export DiscoveredReaderData
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef DiscoveredReaderData_var _var_type;
      typedef DiscoveredReaderData_out _out_type;
      
      DDS::SubscriptionBuiltinTopicData ddsSubscriptionData;
      OpenDDS::DCPS::ReaderProxy_t readerProxy;
      OpenDDS::DCPS::ContentFilterProperty_t contentFilterProperty;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct WriterProxy_t;

    typedef
      ::TAO_Var_Var_T<
          WriterProxy_t
        >
      WriterProxy_t_var;

    typedef
      ::TAO_Out_T<
          WriterProxy_t
        >
      WriterProxy_t_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export WriterProxy_t
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef WriterProxy_t_var _var_type;
      typedef WriterProxy_t_out _out_type;
      
      OpenDDS::DCPS::GUID_t remoteWriterGuid;
      OpenDDS::DCPS::LocatorSeq unicastLocatorList;
      OpenDDS::DCPS::LocatorSeq multicastLocatorList;
      OpenDDS::DCPS::TransportLocatorSeq allLocators;
    };

    // TAO_IDL - Generated from
    // be/be_type.cpp:261

    struct DiscoveredWriterData;

    typedef
      ::TAO_Var_Var_T<
          DiscoveredWriterData
        >
      DiscoveredWriterData_var;

    typedef
      ::TAO_Out_T<
          DiscoveredWriterData
        >
      DiscoveredWriterData_out;

    // TAO_IDL - Generated from
    // be/be_visitor_structure/structure_ch.cpp:51

    struct OpenDDS_Dcps_Export DiscoveredWriterData
    {

      // TAO_IDL - Generated from
      // be/be_type.cpp:307

      
      typedef DiscoveredWriterData_var _var_type;
      typedef DiscoveredWriterData_out _out_type;
      
      DDS::PublicationBuiltinTopicData ddsPublicationData;
      OpenDDS::DCPS::WriterProxy_t writerProxy;
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
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::TransportLocator>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::TransportLocator,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::TransportLocatorSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::TransportLocatorSeq,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::IncompatibleQosStatus>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::IncompatibleQosStatus,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::AddDomainStatus>
    : public
        Fixed_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::AddDomainStatus,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:904

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::TopicStatus>
    : public
        Basic_Arg_Traits_T<
            ::OpenDDS::DCPS::TopicStatus,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::WriterAssociation>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::WriterAssociation,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::ReaderAssociation>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::ReaderAssociation,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::WriterIdSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::WriterIdSeq,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::ReaderIdSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::ReaderIdSeq,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::OctetArray16_tag>
    : public
        Fixed_Array_Arg_Traits_T<
            ::OpenDDS::DCPS::OctetArray16_var,
            ::OpenDDS::DCPS::OctetArray16_forany,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::Locator_t>
    : public
        Fixed_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::Locator_t,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::LocatorSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::LocatorSeq,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

#if !defined (_STRING256256__ARG_TRAITS_)
#define _STRING256256__ARG_TRAITS_

  struct String256_256 {};

  template<>
  class Arg_Traits<String256_256>
    : public
        BD_String_Arg_Traits_T<
            CORBA::String_var,
            256,
            TAO::Any_Insert_Policy_Noop
        >
  {
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::ContentFilterProperty_t>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::ContentFilterProperty_t,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::ReaderProxy_t>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::ReaderProxy_t,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::DiscoveredReaderData>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::DiscoveredReaderData,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::WriterProxy_t>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::WriterProxy_t,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::OpenDDS::DCPS::DiscoveredWriterData>
    : public
        Var_Size_Arg_Traits_T<
            ::OpenDDS::DCPS::DiscoveredWriterData,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };
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
  template<>
  struct OpenDDS_Dcps_Export Array_Traits<
      OpenDDS::DCPS::OctetArray16_forany
    >
  {
    static void free (
        OpenDDS::DCPS::OctetArray16_slice * _tao_slice);
    static OpenDDS::DCPS::OctetArray16_slice * dup (
        const OpenDDS::DCPS::OctetArray16_slice * _tao_slice);
    static void copy (
        OpenDDS::DCPS::OctetArray16_slice * _tao_to,
        const OpenDDS::DCPS::OctetArray16_slice * _tao_from);
    static OpenDDS::DCPS::OctetArray16_slice * alloc (void);
    static void zero (
        OpenDDS::DCPS::OctetArray16_slice * _tao_slice);
  };
}
TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::TransportLocator &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::TransportLocator &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_OpenDDS_DCPS_TransportLocatorSeq_H_
#define _TAO_CDR_OP_OpenDDS_DCPS_TransportLocatorSeq_H_

OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const OpenDDS::DCPS::TransportLocatorSeq &_tao_sequence);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    OpenDDS::DCPS::TransportLocatorSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_OpenDDS_DCPS_TransportLocatorSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::IncompatibleQosStatus &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::IncompatibleQosStatus &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::AddDomainStatus &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::AddDomainStatus &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_enum/cdr_op_ch.cpp:37



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &strm, OpenDDS::DCPS::TopicStatus _tao_enumerator);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &strm, OpenDDS::DCPS::TopicStatus &_tao_enumerator);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::WriterAssociation &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::WriterAssociation &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::ReaderAssociation &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::ReaderAssociation &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_OpenDDS_DCPS_WriterIdSeq_H_
#define _TAO_CDR_OP_OpenDDS_DCPS_WriterIdSeq_H_

OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const OpenDDS::DCPS::WriterIdSeq &_tao_sequence);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    OpenDDS::DCPS::WriterIdSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_OpenDDS_DCPS_WriterIdSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_OpenDDS_DCPS_ReaderIdSeq_H_
#define _TAO_CDR_OP_OpenDDS_DCPS_ReaderIdSeq_H_

OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const OpenDDS::DCPS::ReaderIdSeq &_tao_sequence);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    OpenDDS::DCPS::ReaderIdSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_OpenDDS_DCPS_ReaderIdSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_array/cdr_op_ch.cpp:102



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export CORBA::Boolean operator<< (TAO_OutputCDR &strm, const OpenDDS::DCPS::OctetArray16_forany &_tao_array);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::OctetArray16_forany &_tao_array);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::Locator_t &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::Locator_t &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_OpenDDS_DCPS_LocatorSeq_H_
#define _TAO_CDR_OP_OpenDDS_DCPS_LocatorSeq_H_

OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const OpenDDS::DCPS::LocatorSeq &_tao_sequence);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    OpenDDS::DCPS::LocatorSeq &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_OpenDDS_DCPS_LocatorSeq_H_ */

// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::ContentFilterProperty_t &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::ContentFilterProperty_t &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::ReaderProxy_t &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::ReaderProxy_t &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::DiscoveredReaderData &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::DiscoveredReaderData &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::WriterProxy_t &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::WriterProxy_t &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

OpenDDS_Dcps_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const OpenDDS::DCPS::DiscoveredWriterData &);
OpenDDS_Dcps_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, OpenDDS::DCPS::DiscoveredWriterData &);

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


OPENDDS_END_VERSIONED_NAMESPACE_DECL

#if defined (__ACE_INLINE__)
#include "DdsDcpsInfoUtilsC.inl"
#endif /* defined INLINE */

#include /**/ "ace/post.h"

#endif /* ifndef */

