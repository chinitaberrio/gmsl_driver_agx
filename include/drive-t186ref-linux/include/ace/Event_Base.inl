// -*- C++ -*-
// $Id: Event_Base.inl 2179 2013-05-28 22:16:51Z mesnierp $

ACE_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE ACE_event_t
ACE_Event_Base::handle (void) const
{
  return this->handle_;
}

ACE_INLINE void
ACE_Event_Base::handle (ACE_event_t new_handle)
{
  this->handle_ = new_handle;
}

ACE_END_VERSIONED_NAMESPACE_DECL
