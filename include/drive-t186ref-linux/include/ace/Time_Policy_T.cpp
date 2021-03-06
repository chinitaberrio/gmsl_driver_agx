// $Id: Time_Policy_T.cpp 2179 2013-05-28 22:16:51Z mesnierp $

#ifndef ACE_TIME_POLICY_T_CPP
#define ACE_TIME_POLICY_T_CPP

#include "ace/Time_Policy_T.h"

#if !defined (__ACE_INLINE__)
#include "ace/Time_Policy_T.inl"
#endif /* __ACE_INLINE__ */

ACE_BEGIN_VERSIONED_NAMESPACE_DECL

template <typename TIME_POLICY>
ACE_Time_Policy_T<TIME_POLICY>::~ACE_Time_Policy_T ()
{
}

template <typename TIME_POLICY> ACE_Time_Value_T<ACE_Delegating_Time_Policy>
ACE_Time_Policy_T<TIME_POLICY>::gettimeofday () const
{
  return ACE_Time_Value_T<ACE_Delegating_Time_Policy> (this->time_policy_ (), ACE_Delegating_Time_Policy (this));
}

ACE_END_VERSIONED_NAMESPACE_DECL

#endif /* ACE_TIME_POLICY_T_CPP */
