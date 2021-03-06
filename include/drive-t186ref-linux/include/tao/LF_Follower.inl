// -*- C++ -*-
//
// $Id: LF_Follower.inl 14 2007-02-01 15:49:12Z mitza $

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE TAO_Leader_Follower &
TAO_LF_Follower::leader_follower (void)
{
  return this->leader_follower_;
}

ACE_INLINE int
TAO_LF_Follower::wait (ACE_Time_Value *tv)
{
  return this->condition_.wait (tv);
}

TAO_END_VERSIONED_NAMESPACE_DECL
