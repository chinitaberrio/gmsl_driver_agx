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

#ifndef _TAO_PIDL_STRINGVALUEC_HNKNGY_H_
#define _TAO_PIDL_STRINGVALUEC_HNKNGY_H_

#include /**/ "ace/pre.h"


#include /**/ "ace/config-all.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */


#include /**/ "tao/Valuetype/valuetype_export.h"
#include "tao/Valuetype/ValueBase.h"
#include "tao/CDR.h"
#include "tao/Valuetype/Valuetype_Adapter_Factory_Impl.h"
#include "tao/SystemException.h"
#include "tao/Basic_Types.h"
#include "tao/ORB_Constants.h"
#include "tao/Valuetype/Value_VarOut_T.h"
#include "tao/Arg_Traits_T.h"
#include "tao/Basic_Arguments.h"
#include "tao/Special_Basic_Arguments.h"
#include "tao/Any_Insert_Policy_T.h"
#include "tao/Fixed_Size_Argument_T.h"
#include "tao/Var_Size_Argument_T.h"
#include "tao/UB_String_Arguments.h"
#include /**/ "tao/Version.h"
#include /**/ "tao/Versioned_Namespace.h"

#if TAO_MAJOR_VERSION != 2 || TAO_MINOR_VERSION != 2 || TAO_BETA_VERSION != 0
#error This file should be regenerated with TAO_IDL
#endif

#if defined (TAO_EXPORT_MACRO)
#undef TAO_EXPORT_MACRO
#endif
#define TAO_EXPORT_MACRO TAO_Valuetype_Export

TAO_BEGIN_VERSIONED_NAMESPACE_DECL



// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:38

namespace CORBA
{

  // TAO_IDL - Generated from
  // be/be_visitor_valuebox/valuebox_ch.cpp:44

  class StringValue;

  typedef
    TAO_Value_Var_T<
        StringValue
      >
    StringValue_var;
  
  typedef
    TAO_Value_Out_T<
        StringValue
      >
    StringValue_out;

  class TAO_Valuetype_Export StringValue
    : public virtual ::CORBA::DefaultValueRefCountBase
  {
  public:

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef StringValue_var _var_type;
    typedef StringValue_out _out_type;

    static void _tao_any_destructor (void *);

    static StringValue* _downcast ( ::CORBA::ValueBase *);
    ::CORBA::ValueBase * _copy_value (void);

    virtual const char* _tao_obv_repository_id (void) const;

    virtual void _tao_obv_truncatable_repo_ids (Repository_Id_List &ids) const;

    static const char* _tao_obv_static_repository_id (void);

    static ::CORBA::Boolean _tao_unmarshal (
        TAO_InputCDR &,
        StringValue *&
      );
    
    virtual ::CORBA::TypeCode_ptr _tao_type (void) const;

    

    // TAO_IDL - Generated from
    // be/be_visitor_valuebox/valuebox_ch.cpp:409

    // Constructors
    StringValue (void);
    StringValue (char * val);
    StringValue (const char * val);
    StringValue (const ::CORBA::String_var& var);
    StringValue (const StringValue& val);
    // assignment operators
    StringValue& operator= (char * val);

    StringValue& operator= (const char * val);

    StringValue& operator= (const ::CORBA::String_var& var);

    // Accessor
    const char * _value (void) const;

    // Modifiers
    void _value (char * val);
    void _value (const char * val);
    void _value (const ::CORBA::String_var& var);

    // Access to the boxed value for method signatures
    const char * _boxed_in (void) const;
    char *& _boxed_inout (void);
    char *& _boxed_out (void);
    // Allows access and modification using a slot.
    char & operator[] ( ::CORBA::ULong slot);

    // Allows only accessing thru a slot.
    char operator[] ( ::CORBA::ULong slot) const;
    
  private:
    ::CORBA::String_var _pd_value;
    

    // TAO_IDL - Generated from
    // be/be_visitor_valuebox/valuebox_ch.cpp:117
  
  protected:
    virtual ~StringValue (void);
    virtual ::CORBA::Boolean _tao_marshal_v (TAO_OutputCDR &) const;
    virtual ::CORBA::Boolean _tao_unmarshal_v (TAO_InputCDR &);
    virtual ::CORBA::Boolean _tao_match_formal_type (ptrdiff_t ) const;
    
  private:
    void operator= (const StringValue & val);
    
  };

  // TAO_IDL - Generated from
  // be/be_visitor_valuebox/valuebox_ch.cpp:44

  class WStringValue;

  typedef
    TAO_Value_Var_T<
        WStringValue
      >
    WStringValue_var;
  
  typedef
    TAO_Value_Out_T<
        WStringValue
      >
    WStringValue_out;

  class TAO_Valuetype_Export WStringValue
    : public virtual ::CORBA::DefaultValueRefCountBase
  {
  public:

    // TAO_IDL - Generated from
    // be/be_type.cpp:307

    
    typedef WStringValue_var _var_type;
    typedef WStringValue_out _out_type;

    static void _tao_any_destructor (void *);

    static WStringValue* _downcast ( ::CORBA::ValueBase *);
    ::CORBA::ValueBase * _copy_value (void);

    virtual const char* _tao_obv_repository_id (void) const;

    virtual void _tao_obv_truncatable_repo_ids (Repository_Id_List &ids) const;

    static const char* _tao_obv_static_repository_id (void);

    static ::CORBA::Boolean _tao_unmarshal (
        TAO_InputCDR &,
        WStringValue *&
      );
    
    virtual ::CORBA::TypeCode_ptr _tao_type (void) const;

    

    // TAO_IDL - Generated from
    // be/be_visitor_valuebox/valuebox_ch.cpp:409

    // Constructors
    WStringValue (void);
    WStringValue (CORBA::WChar * val);
    WStringValue (const CORBA::WChar * val);
    WStringValue (const ::CORBA::WString_var& var);
    WStringValue (const WStringValue& val);
    // assignment operators
    WStringValue& operator= (CORBA::WChar * val);

    WStringValue& operator= (const CORBA::WChar * val);

    WStringValue& operator= (const ::CORBA::WString_var& var);

    // Accessor
    const CORBA::WChar * _value (void) const;

    // Modifiers
    void _value (CORBA::WChar * val);
    void _value (const CORBA::WChar * val);
    void _value (const ::CORBA::WString_var& var);

    // Access to the boxed value for method signatures
    const CORBA::WChar * _boxed_in (void) const;
    CORBA::WChar *& _boxed_inout (void);
    CORBA::WChar *& _boxed_out (void);
    // Allows access and modification using a slot.
    ::CORBA::WChar & operator[] ( ::CORBA::ULong slot);

    // Allows only accessing thru a slot.
    ::CORBA::WChar operator[] ( ::CORBA::ULong slot) const;
    
  private:
    ::CORBA::WString_var _pd_value;
    

    // TAO_IDL - Generated from
    // be/be_visitor_valuebox/valuebox_ch.cpp:117
  
  protected:
    virtual ~WStringValue (void);
    virtual ::CORBA::Boolean _tao_marshal_v (TAO_OutputCDR &) const;
    virtual ::CORBA::Boolean _tao_unmarshal_v (TAO_InputCDR &);
    virtual ::CORBA::Boolean _tao_match_formal_type (ptrdiff_t ) const;
    
  private:
    void operator= (const WStringValue & val);
    
  };

// TAO_IDL - Generated from
// be/be_visitor_module/module_ch.cpp:67

} // module CORBA

// TAO_IDL - Generated from
// be/be_visitor_arg_traits.cpp:68


TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL


// Arg traits specializations.
namespace TAO
{

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:252

  template<>
  class Arg_Traits< ::CORBA::StringValue>
    : public
        Object_Arg_Traits_T<
            ::CORBA::StringValue *,
            ::CORBA::StringValue_var,
            ::CORBA::StringValue_out,
            TAO::Value_Traits<CORBA::StringValue>,
            TAO::Any_Insert_Policy_Stream
          >
  {
  };

  // TAO_IDL - Generated from
  // be/be_visitor_arg_traits.cpp:252

  template<>
  class Arg_Traits< ::CORBA::WStringValue>
    : public
        Object_Arg_Traits_T<
            ::CORBA::WStringValue *,
            ::CORBA::WStringValue_var,
            ::CORBA::WStringValue_out,
            TAO::Value_Traits<CORBA::WStringValue>,
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

#if !defined (_CORBA_STRINGVALUE__TRAITS_)
#define _CORBA_STRINGVALUE__TRAITS_

  template<>
  struct TAO_Valuetype_Export Value_Traits<CORBA::StringValue>
  {
    static void add_ref (CORBA::StringValue *);
    static void remove_ref (CORBA::StringValue *);
    static void release (CORBA::StringValue *);
  };

#endif /* end #if !defined */

#if !defined (_CORBA_WSTRINGVALUE__TRAITS_)
#define _CORBA_WSTRINGVALUE__TRAITS_

  template<>
  struct TAO_Valuetype_Export Value_Traits<CORBA::WStringValue>
  {
    static void add_ref (CORBA::WStringValue *);
    static void remove_ref (CORBA::WStringValue *);
    static void release (CORBA::WStringValue *);
  };

#endif /* end #if !defined */
}
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_valuebox/cdr_op_ch.cpp:42



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Valuetype_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const CORBA::StringValue *);
TAO_Valuetype_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, CORBA::StringValue *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_visitor_valuebox/cdr_op_ch.cpp:42



TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

TAO_Valuetype_Export ::CORBA::Boolean operator<< (TAO_OutputCDR &, const CORBA::WStringValue *);
TAO_Valuetype_Export ::CORBA::Boolean operator>> (TAO_InputCDR &, CORBA::WStringValue *&);
TAO_END_VERSIONED_NAMESPACE_DECL


TAO_BEGIN_VERSIONED_NAMESPACE_DECL




// TAO_IDL - Generated from
// be/be_codegen.cpp:1703


TAO_END_VERSIONED_NAMESPACE_DECL

#if defined (__ACE_INLINE__)
#include "StringValueC.inl"
#endif /* defined INLINE */

#include /**/ "ace/post.h"

#endif /* ifndef */

