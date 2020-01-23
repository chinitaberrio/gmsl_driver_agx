// -*- C++ -*-
//
// $Id: ORB_Time_Policy.inl 2179 2013-05-28 22:16:51Z mesnierp $

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

namespace TAO
{

  ACE_INLINE ACE_Time_Value
  ORB_Time_Policy::operator() () const
  {
    return (*active_policy_) ();
  }

}

TAO_END_VERSIONED_NAMESPACE_DECL
