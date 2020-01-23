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

#ifndef _TAO_IDL_DDSDCPSDATAREADERSEQC_WDO3YA_H_
#define _TAO_IDL_DDSDCPSDATAREADERSEQC_WDO3YA_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "dds/DCPS/dcps_export.h"
#include "tao/ORB.h"
#include "tao/Basic_Types.h"
#include "tao/Sequence_T.h"
#include "tao/Objref_VarOut_T.h"
#include "tao/Seq_Var_T.h"
#include "tao/Seq_Out_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"
#include /**/ "dds/Versioned_Namespace.h"

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

namespace DDS
{

  // TAO_IDL - Generated from
  // be/be_interface.cpp:751

#if !defined (_DDS_DATAREADER__VAR_OUT_CH_)
#define _DDS_DATAREADER__VAR_OUT_CH_

  class DataReader;
  typedef DataReader *DataReader_ptr;

  typedef
    TAO_Objref_Var_T<
        DataReader
      >
    DataReader_var;
  
  typedef
    TAO_Objref_Out_T<
        DataReader
      >
    DataReader_out;

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_DDS_DATAREADERSEQ_CH_)
#define _DDS_DATAREADERSEQ_CH_

  class DataReaderSeq;

  typedef
    ::TAO_VarSeq_Var_T<
        DataReaderSeq
      >
    DataReaderSeq_var;

  typedef
    ::TAO_Seq_Out_T<
        DataReaderSeq
      >
    DataReaderSeq_out;

  class OpenDDS_Dcps_Export DataReaderSeq
    : public
        ::TAO::unbounded_object_reference_sequence<
            DataReader,
            DataReader_var
          >
  {
  public:
    DataReaderSeq (void);
    DataReaderSeq ( ::CORBA::ULong max);
    DataReaderSeq (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      DataReader_ptr* buffer,
      ::CORBA::Boolean release = false);
    DataReaderSeq (const DataReaderSeq &);
    virtual ~DataReaderSeq (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef DataReaderSeq_var _var_type;
    typedef DataReaderSeq_out _out_type;
  };

#endif /* end #if !defined */

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module DDS

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::DDS::DataReaderSeq>
    : public
        Var_Size_Arg_Traits_T<
            ::DDS::DataReaderSeq,
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

#if !defined (_DDS_DATAREADER__TRAITS_)
#define _DDS_DATAREADER__TRAITS_

  template<>
  struct OpenDDS_Dcps_Export Objref_Traits< ::DDS::DataReader>
  {
    static ::DDS::DataReader_ptr duplicate (
        ::DDS::DataReader_ptr p);
    static void release (
        ::DDS::DataReader_ptr p);
    static ::DDS::DataReader_ptr nil (void);
    static ::CORBA::Boolean marshal (
        const ::DDS::DataReader_ptr p,
        TAO_OutputCDR & cdr);
  };

#endif /* end #if !defined */
}
TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


OPENDDS_END_VERSIONED_NAMESPACE_DECL

#if defined (__ACE_INLINE__)
#include "DdsDcpsDataReaderSeqC.inl"
#endif /* defined INLINE */

#include /**/ "ace/post.h"

#endif /* ifndef */

