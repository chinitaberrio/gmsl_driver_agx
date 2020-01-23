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
// be/be_codegen.cpp:461

#ifndef _TAO_IDL_POAS_NE0WPO_H_
#define _TAO_IDL_POAS_NE0WPO_H_

#include /**/ "ace/pre.h"

#include "POAC.h"
#include "tao/Policy_ForwardS.h"
#include "tao/OctetSeqS.h"
#include "tao/orb_typesS.h"
#include "tao/PortableServer/PS_ForwardS.h"
#include "tao/PortableServer/IdAssignmentPolicyS.h"
#include "tao/PortableServer/IdUniquenessPolicyS.h"
#include "tao/PortableServer/ImplicitActivationPolicyS.h"
#include "tao/PortableServer/LifespanPolicyS.h"
#include "tao/PortableServer/RequestProcessingPolicyS.h"
#include "tao/PortableServer/ServantRetentionPolicyS.h"
#include "tao/PortableServer/ThreadPolicyS.h"
#include "tao/PortableServer/Basic_SArguments.h"
#include "tao/PortableServer/Special_Basic_SArguments.h"
#include "tao/PortableServer/Fixed_Size_SArgument_T.h"
#include "tao/PortableServer/Var_Size_SArgument_T.h"
#include "tao/PortableServer/Object_SArg_Traits.h"
#include "tao/PortableServer/UB_String_SArguments.h"
#include "tao/PortableServer/Object_SArg_Traits.h"

// Skeleton file generation suppressed with command line option -SS

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



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
  class SArg_Traits< ::PortableServer::POAList>
    : public
        Var_Size_SArg_Traits_T<
            ::PortableServer::POAList,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };
}

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




TAO_END_VERSIONED_NAMESPACE_DECL



#include /**/ "ace/post.h"

#endif /* ifndef */

