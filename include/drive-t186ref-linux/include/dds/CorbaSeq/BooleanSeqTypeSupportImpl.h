/* -*- C++ -*- */
/* Generated by /dvs/git/dirty/git-master_autodds/middleware/dds/opendds/build/host/bin/opendds_idl version 3.13 (ACE version 6.2a_p14) running on input file CorbaSeq/BooleanSeq.idl */
#ifndef OPENDDS_IDL_GENERATED_BOOLEANSEQTYPESUPPORTIMPL_H_AGBME3
#define OPENDDS_IDL_GENERATED_BOOLEANSEQTYPESUPPORTIMPL_H_AGBME3
#include "tao/BooleanSeqC.h"
#include "dds/DCPS/Definitions.h"
#include "dds/DdsDcpsC.h"
#include "dds/DCPS/Serializer.h"
#include "dds/DCPS/dcps_export.h"


/* Begin MODULE: CORBA */


/* End MODULE: CORBA */


/* Begin MODULE: CORBA */



/* Begin TYPEDEF: BooleanSeq */

OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL
namespace OpenDDS { namespace DCPS {

OpenDDS_Dcps_Export
void gen_find_size(const CORBA::BooleanSeq& seq, size_t& size, size_t& padding);

OpenDDS_Dcps_Export
bool operator<<(Serializer& strm, const CORBA::BooleanSeq& seq);

OpenDDS_Dcps_Export
bool operator>>(Serializer& strm, CORBA::BooleanSeq& seq);

}  }
OPENDDS_END_VERSIONED_NAMESPACE_DECL

#ifndef OPENDDS_NO_CONTENT_SUBSCRIPTION_PROFILE
OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL
namespace OpenDDS { namespace DCPS {

OpenDDS_Dcps_Export
bool gen_skip_over(Serializer& ser, CORBA::BooleanSeq*);

}  }
OPENDDS_END_VERSIONED_NAMESPACE_DECL

#endif

/* End TYPEDEF: BooleanSeq */

/* End MODULE: CORBA */
#endif /* OPENDDS_IDL_GENERATED_BOOLEANSEQTYPESUPPORTIMPL_H_AGBME3 */
