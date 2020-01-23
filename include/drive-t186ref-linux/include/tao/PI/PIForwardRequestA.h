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

#ifndef _TAO_IDL_PIFORWARDREQUESTA_3YNUZC_H_
#define _TAO_IDL_PIFORWARDREQUESTA_3YNUZC_H_

#include /**/ "ace/pre.h"

#include /**/ "tao/PI/pi_export.h"
#include "tao/AnyTypeCode/Any.h"

#include "tao/PI/PI.h"


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:48

namespace PortableInterceptor
{

  // TAO_IDL - Generated from
  // be/be_visitor_typecode/typecode_decl.cpp:37

  extern TAO_PI_Export ::CORBA::TypeCode_ptr const _tc_ForwardRequest;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:75

} // module PortableInterceptor


// TAO_IDL - Generated from
// be/be_visitor_exception/any_op_ch.cpp:41

#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace PortableInterceptor
{
  

  TAO_PI_Export void operator<<= (::CORBA::Any &, const ::PortableInterceptor::ForwardRequest &); // copying version
  TAO_PI_Export void operator<<= (::CORBA::Any &, ::PortableInterceptor::ForwardRequest*); // noncopying version
  TAO_PI_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, ::PortableInterceptor::ForwardRequest *&); // deprecated
TAO_PI_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const ::PortableInterceptor::ForwardRequest *&);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



TAO_PI_Export void operator<<= (::CORBA::Any &, const PortableInterceptor::ForwardRequest &); // copying version
TAO_PI_Export void operator<<= (::CORBA::Any &, PortableInterceptor::ForwardRequest*); // noncopying version
TAO_PI_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, PortableInterceptor::ForwardRequest *&); // deprecated
TAO_PI_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const PortableInterceptor::ForwardRequest *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

TAO_END_VERSIONED_NAMESPACE_DECL



#include /**/ "ace/post.h"

#endif /* ifndef */
