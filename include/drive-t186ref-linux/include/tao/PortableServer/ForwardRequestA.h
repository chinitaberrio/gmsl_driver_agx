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

#ifndef _TAO_IDL_FORWARDREQUESTA_S9RAH2_H_
#define _TAO_IDL_FORWARDREQUESTA_S9RAH2_H_

#include /**/ "ace/pre.h"

#include /**/ "tao/PortableServer/portableserver_export.h"
#include "tao/AnyTypeCode/Any.h"

#include "tao/PortableServer/ForwardRequestC.h"


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:48

namespace PortableServer
{

  // TAO_IDL - Generated from
  // be/be_visitor_typecode/typecode_decl.cpp:37

  extern TAO_PortableServer_Export ::CORBA::TypeCode_ptr const _tc_ForwardRequest;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:75

} // module PortableServer


// TAO_IDL - Generated from
// be/be_visitor_exception/any_op_ch.cpp:41

#if defined (ACE_ANY_OPS_USE_NAMESPACE)

namespace PortableServer
{
  

  TAO_PortableServer_Export void operator<<= (::CORBA::Any &, const ::PortableServer::ForwardRequest &); // copying version
  TAO_PortableServer_Export void operator<<= (::CORBA::Any &, ::PortableServer::ForwardRequest*); // noncopying version
  TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, ::PortableServer::ForwardRequest *&); // deprecated
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const ::PortableServer::ForwardRequest *&);
}

#else



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL



TAO_PortableServer_Export void operator<<= (::CORBA::Any &, const PortableServer::ForwardRequest &); // copying version
TAO_PortableServer_Export void operator<<= (::CORBA::Any &, PortableServer::ForwardRequest*); // noncopying version
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, PortableServer::ForwardRequest *&); // deprecated
TAO_PortableServer_Export ::CORBA::Boolean operator>>= (const ::CORBA::Any &, const PortableServer::ForwardRequest *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




#endif

TAO_END_VERSIONED_NAMESPACE_DECL



#include /**/ "ace/post.h"

#endif /* ifndef */
