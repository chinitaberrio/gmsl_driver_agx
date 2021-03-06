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

#ifndef _TAO_PIDL_GIOPC_RBEIR8_H_
#define _TAO_PIDL_GIOPC_RBEIR8_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/TAO_Export.h"
#include "tao/Basic_Types.h"
#include "tao/String_Manager_T.h"
#include "tao/VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Basic_Argument_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"

#include "tao/IOPC.h"

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

namespace GIOP
{

  // TAO_IDL - Generated from
  // be/be_visitor_typedef/typedef_ch.cpp:373

  typedef ::CORBA::Short AddressingDisposition;
  typedef ::CORBA::Short_out AddressingDisposition_out;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::Short KeyAddr = 0;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::Short ProfileAddr = 1;

  // TAO_IDL - Generated from
  // be/be_visitor_constant/constant_ch.cpp:38

  const CORBA::Short ReferenceAddr = 2;

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct Version;

  typedef
    ::TAO_Fixed_Var_T<
        Version
      >
    Version_var;

  typedef
    Version &
    Version_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export Version
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef Version_var _var_type;
    typedef Version_out _out_type;

    static void _tao_any_destructor (void *);
    
    ::CORBA::Octet major;
    ::CORBA::Octet minor;
  };

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  struct IORAddressingInfo;

  typedef
    ::TAO_Var_Var_T<
        IORAddressingInfo
      >
    IORAddressingInfo_var;

  typedef
    ::TAO_Out_T<
        IORAddressingInfo
      >
    IORAddressingInfo_out;

  // TAO_IDL - Generated from
  // be/be_visitor_structure/structure_ch.cpp:51

  struct TAO_Export IORAddressingInfo
  {

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef IORAddressingInfo_var _var_type;
    typedef IORAddressingInfo_out _out_type;

    static void _tao_any_destructor (void *);
    
    ::CORBA::ULong selected_profile_index;
    IOP::IOR ior;
  };

  // TAO_IDL - Generated from
  // be/be_type.cpp:261

  class TargetAddress;

  typedef
    ::TAO_Var_Var_T<
        TargetAddress
      >
    TargetAddress_var;

  typedef
    ::TAO_Out_T<
        TargetAddress
      >
    TargetAddress_out;

  class TAO_Export TargetAddress
  {
  public:
    TargetAddress (void);
    TargetAddress (const TargetAddress &);
    ~TargetAddress (void);

    TargetAddress &operator= (const TargetAddress &);

    // TAO_IDL - Generated from
    // be/be_visitor_union/discriminant_ch.cpp:103

    void _d ( ::CORBA::Short);
    ::CORBA::Short _d (void) const;

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef TargetAddress_var _var_type;
    typedef TargetAddress_out _out_type;

    static void _tao_any_destructor (void *);

    // TAO_IDL - Generated from
    // be/be_visitor_union_branch/public_ch.cpp:551

    void object_key (const CORBA::OctetSeq &);
    const CORBA::OctetSeq &object_key (void) const;
    CORBA::OctetSeq &object_key (void);

    // TAO_IDL - Generated from
    // be/be_visitor_union_branch/public_ch.cpp:659

    void profile (const IOP::TaggedProfile &);
    const IOP::TaggedProfile &profile (void) const;
    IOP::TaggedProfile &profile (void);

    // TAO_IDL - Generated from
    // be/be_visitor_union_branch/public_ch.cpp:659

    void ior (const GIOP::IORAddressingInfo &);
    const GIOP::IORAddressingInfo &ior (void) const;
    GIOP::IORAddressingInfo &ior (void);

    // TAO_IDL - Generated from
    // be/be_visitor_union/union_ch.cpp:124

    void _default (void);
  private:
    ::CORBA::Short disc_;

    union
    {
      // TAO_IDL - Generated from
      // be/be_visitor_union_branch/private_ch.cpp:447
      CORBA::OctetSeq *object_key_;
      // TAO_IDL - Generated from
      // be/be_visitor_union_branch/private_ch.cpp:522
      IOP::TaggedProfile *profile_;
      // TAO_IDL - Generated from
      // be/be_visitor_union_branch/private_ch.cpp:522
      GIOP::IORAddressingInfo *ior_;
    } u_;

    /// TAO extension - frees any allocated storage.
    void _reset (void);
  };

  // TAO_IDL - Generated from
  // be/be_visitor_enum/enum_ch.cpp:47

  enum MsgType
  {
    Request,
    Reply,
    CancelRequest,
    LocateRequest,
    LocateReply,
    CloseConnection,
    MessageError,
    Fragment
  };

  typedef MsgType &MsgType_out;

  // TAO_IDL - Generated from
  // be/be_visitor_enum/enum_ch.cpp:47

  enum ReplyStatusType
  {
    NO_EXCEPTION,
    USER_EXCEPTION,
    SYSTEM_EXCEPTION,
    LOCATION_FORWARD,
    LOCATION_FORWARD_PERM,
    NEEDS_ADDRESSING_MODE
  };

  typedef ReplyStatusType &ReplyStatusType_out;

  // TAO_IDL - Generated from
  // be/be_visitor_enum/enum_ch.cpp:47

  enum LocateStatusType
  {
    UNKNOWN_OBJECT,
    OBJECT_HERE,
    OBJECT_FORWARD,
    OBJECT_FORWARD_PERM,
    LOC_SYSTEM_EXCEPTION,
    LOC_NEEDS_ADDRESSING_MODE
  };

  typedef LocateStatusType &LocateStatusType_out;

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module GIOP

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
  class Arg_Traits< ::GIOP::Version>
    : public
        Fixed_Size_Arg_Traits_T<
            ::GIOP::Version,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:947

  template<>
  class Arg_Traits< ::GIOP::IORAddressingInfo>
    : public
        Var_Size_Arg_Traits_T<
            ::GIOP::IORAddressingInfo,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:1058

  template<>
  class Arg_Traits< ::GIOP::TargetAddress>
    : public
        Var_Size_Arg_Traits_T<
            ::GIOP::TargetAddress,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:904

  template<>
  class Arg_Traits< ::GIOP::MsgType>
    : public
        Basic_Arg_Traits_T<
            ::GIOP::MsgType,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:904

  template<>
  class Arg_Traits< ::GIOP::ReplyStatusType>
    : public
        Basic_Arg_Traits_T<
            ::GIOP::ReplyStatusType,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:904

  template<>
  class Arg_Traits< ::GIOP::LocateStatusType>
    : public
        Basic_Arg_Traits_T<
            ::GIOP::LocateStatusType,
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

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const GIOP::Version &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, GIOP::Version &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_structure/cdr_op_ch.cpp:46



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const GIOP::IORAddressingInfo &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, GIOP::IORAddressingInfo &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_union/cdr_op_ch.cpp:41



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const GIOP::TargetAddress &);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, GIOP::TargetAddress &);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_enum/cdr_op_ch.cpp:37



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &strm, GIOP::MsgType _tao_enumerator);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &strm, GIOP::MsgType &_tao_enumerator);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_enum/cdr_op_ch.cpp:37



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &strm, GIOP::ReplyStatusType _tao_enumerator);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &strm, GIOP::ReplyStatusType &_tao_enumerator);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_enum/cdr_op_ch.cpp:37



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &strm, GIOP::LocateStatusType _tao_enumerator);
TAO_Export ::CORBA::Boolean operator>> (TAO_InputCDR &strm, GIOP::LocateStatusType &_tao_enumerator);

TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#if defined (__ACE_INLINE__)
#include "GIOPC.inl"
#endif /* defined INLINE */

#include /**/ "ace/post.h"

#endif /* ifndef */

