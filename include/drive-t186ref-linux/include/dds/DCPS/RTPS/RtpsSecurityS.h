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

#ifndef _TAO_IDL_RTPSSECURITYS_FWZHWF_H_
#define _TAO_IDL_RTPSSECURITYS_FWZHWF_H_

#include /**/ "ace/pre.h"

#include "RtpsSecurityC.h"
#include "dds/DdsSecurityCoreS.h"
#include "dds/DCPS/RTPS/RtpsCoreS.h"
#include "tao/PortableServer/Basic_SArguments.h"
#include "tao/PortableServer/Special_Basic_SArguments.h"
#include "tao/PortableServer/Fixed_Size_SArgument_T.h"
#include "tao/PortableServer/Var_Size_SArgument_T.h"

// Skeleton file generation suppressed with command line option -SS

OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


OPENDDS_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:904

  template<>
  class SArg_Traits< ::OpenDDS::Security::DiscoveredParticipantDataKind>
    : public
        Basic_SArg_Traits_T<
            ::OpenDDS::Security::DiscoveredParticipantDataKind,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class SArg_Traits< ::OpenDDS::Security::SPDPdiscoveredParticipantData>
    : public
        Var_Size_SArg_Traits_T<
            ::OpenDDS::Security::SPDPdiscoveredParticipantData,
            TAO::Any_Insert_Policy_Noop
          >
  {
  };
}

TAO_END_VERSIONED_NAMESPACE_DECL


OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL




OPENDDS_END_VERSIONED_NAMESPACE_DECL



#include /**/ "ace/post.h"

#endif /* ifndef */

