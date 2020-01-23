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
// be/be_codegen.cpp:794

#ifndef _TAO_IDL_ANYTYPECODE_CHARSEQA_XFGDDU_H_
#define _TAO_IDL_ANYTYPECODE_CHARSEQA_XFGDDU_H_

#include /**/ "ace/pre.h"

#include /**/ "tao/AnyTypeCode/TAO_AnyTypeCode_Export.h"
#include "tao/AnyTypeCode/Any.h"

#include "tao/CharSeqC.h"


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:48

namespace CORBA
{

  // TAO_IDL - Generated from
  // be/be_visitor_typecode/typecode_decl.cpp:37

  extern TAO_AnyTypeCode_Export ::CORBA::TypeCode_ptr const _tc_CharSeq;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:75

} // module CORBA


// TAO_IDL - Generated from
// be/be_visitor_sequence/any_op_ch.cpp:50



#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace CORBA
{
  TAO_AnyTypeCode_Export void operator<<= ( ::CORBA::Any &, const ::CORBA::CharSeq &); // copying version
  TAO_AnyTypeCode_Export void operator<<= ( ::CORBA::Any &, ::CORBA::CharSeq*); // noncopying version
  TAO_AnyTypeCode_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, ::CORBA::CharSeq *&); // deprecated
  TAO_AnyTypeCode_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const ::CORBA::CharSeq *&);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_AnyTypeCode_Export void operator<<= ( ::CORBA::Any &, const CORBA::CharSeq &); // copying version
TAO_AnyTypeCode_Export void operator<<= ( ::CORBA::Any &, CORBA::CharSeq*); // noncopying version
TAO_AnyTypeCode_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, CORBA::CharSeq *&); // deprecated
TAO_AnyTypeCode_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const CORBA::CharSeq *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

TAO_END_VERSIONED_NAMESPACE_DECL



#include /**/ "ace/post.h"

#endif /* ifndef */
