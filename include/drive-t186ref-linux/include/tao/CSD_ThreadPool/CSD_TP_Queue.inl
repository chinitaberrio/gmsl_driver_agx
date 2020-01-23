// -*- C++ -*-
//
// $Id: CSD_TP_Queue.inl 14 2007-02-01 15:49:12Z mitza $


TAO_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE
TAO::CSD::TP_Queue::TP_Queue()
  : head_(0),
    tail_(0)
{
}


ACE_INLINE
TAO::CSD::TP_Queue::~TP_Queue()
{
}


ACE_INLINE
bool
TAO::CSD::TP_Queue::is_empty() const
{
  return (this->head_ == 0);
}

TAO_END_VERSIONED_NAMESPACE_DECL
