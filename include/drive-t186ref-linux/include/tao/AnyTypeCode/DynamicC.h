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

#ifndef _TAO_PIDL_DYNAMICC_D5NKNT_H_
#define _TAO_PIDL_DYNAMICC_D5NKNT_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/AnyTypeCode/TAO_AnyTypeCode_Export.h"
#include "tao/Basic_Types.h"
#include "tao/AnyTypeCode/TypeCode.h"
#include "tao/AnyTypeCode/TypeCode_Constants.h"
#include "tao/Sequence_T.h"
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

#include "tao/AnyTypeCode/Dynamic_ParameterC.h"
#include "tao/Typecode_typesC.h"
#include "tao/StringSeqC.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_AnyTypeCode_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace Dynamic
{

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_DYNAMIC_PARAMETERLIST_CH_)
#define _DYNAMIC_PARAMETERLIST_CH_

  class ParameterList;

  typedef
    ::TAO_VarSeq_Var_T<
        ParameterList
      >
    ParameterList_var;

  typedef
    ::TAO_Seq_Out_T<
        ParameterList
      >
    ParameterList_out;

  class TAO_AnyTypeCode_Export ParameterList
    : public
        ::TAO::unbounded_value_sequence<
            Parameter
          >
  {
  public:
    ParameterList (void);
    ParameterList ( ::CORBA::ULong max);
    ParameterList (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      Parameter* buffer,
      ::CORBA::Boolean release = false);
    ParameterList (const ParameterList &);
    virtual ~ParameterList (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef ParameterList_var _var_type;
    typedef ParameterList_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:466

  typedef CORBA::StringSeq ContextList;
  typedef CORBA::StringSeq_var ContextList_var;
  typedef CORBA::StringSeq_out ContextList_out;

  // TAO_IDL - Generated from
  // be/be_visitor_sequence/sequence_ch.cpp:102

#if !defined (_DYNAMIC_EXCEPTIONLIST_CH_)
#define _DYNAMIC_EXCEPTIONLIST_CH_

  class ExceptionList;

  typedef
    ::TAO_VarSeq_Var_T<
        ExceptionList
      >
    ExceptionList_var;

  typedef
    ::TAO_Seq_Out_T<
        ExceptionList
      >
    ExceptionList_out;

  class TAO_AnyTypeCode_Export ExceptionList
    : public
        ::TAO::unbounded_object_reference_sequence<
            ::CORBA::TypeCode,
            ::CORBA::TypeCode_var
          >
  {
  public:
    ExceptionList (void);
    ExceptionList ( ::CORBA::ULong max);
    ExceptionList (
      ::CORBA::ULong max,
      ::CORBA::ULong length,
      ::CORBA::TypeCode_ptr* buffer,
      ::CORBA::Boolean release = false);
    ExceptionList (const ExceptionList &);
    virtual ~ExceptionList (void);
    

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef ExceptionList_var _var_type;
    typedef ExceptionList_out _out_type;

    static void _tao_any_destructor (void *);
  };

#endif /* end #if !defined */

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:466

  typedef CORBA::StringSeq RequestContext;
  typedef CORBA::StringSeq_var RequestContext_var;
  typedef CORBA::StringSeq_out RequestContext_out;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module Dynamic

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::Dynamic::ParameterList>
    : public
        Var_Size_Arg_Traits_T<
            ::Dynamic::ParameterList,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:685

  template<>
  class Arg_Traits< ::Dynamic::ExceptionList>
    : public
        Var_Size_Arg_Traits_T<
            ::Dynamic::ExceptionList,
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
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_Dynamic_ParameterList_H_
#define _TAO_CDR_OP_Dynamic_ParameterList_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_AnyTypeCode_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const Dynamic::ParameterList &_tao_sequence);
TAO_AnyTypeCode_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    Dynamic::ParameterList &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_Dynamic_ParameterList_H_ */

// TAO_IDL - Generated from
// be/be_visitor_sequence/cdr_op_ch.cpp:68

#if !defined _TAO_CDR_OP_Dynamic_ExceptionList_H_
#define _TAO_CDR_OP_Dynamic_ExceptionList_H_

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


TAO_AnyTypeCode_Export ::CORBA::Boolean operator<< (
    TAO_OutputCDR &strm,
    const Dynamic::ExceptionList &_tao_sequence);
TAO_AnyTypeCode_Export ::CORBA::Boolean operator>> (
    TAO_InputCDR &strm,
    Dynamic::ExceptionList &_tao_sequence);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif /* _TAO_CDR_OP_Dynamic_ExceptionList_H_ */

// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#include /**/ "ace/post.h"

#endif /* ifndef */

