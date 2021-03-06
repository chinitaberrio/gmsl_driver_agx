// $Id: Intrusive_Auto_Ptr.cpp 2622 2015-08-13 18:30:00Z mitza $

#ifndef ACE_INTRUSIVE_AUTO_PTR_CPP
#define ACE_INTRUSIVE_AUTO_PTR_CPP

#include "ace/Intrusive_Auto_Ptr.h"

#if !defined (__ACE_INLINE__)
#include "ace/Intrusive_Auto_Ptr.inl"
#endif /* __ACE_INLINE __ */

ACE_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_ALLOC_HOOK_DEFINE_Tc(ACE_Intrusive_Auto_Ptr)

template <class X>
ACE_Intrusive_Auto_Ptr<X>::~ACE_Intrusive_Auto_Ptr (void)
{
  reset (0);
}

ACE_END_VERSIONED_NAMESPACE_DECL

#endif  /* !ACE_INTRUSIVE_AUTO_PTR_CPP */
