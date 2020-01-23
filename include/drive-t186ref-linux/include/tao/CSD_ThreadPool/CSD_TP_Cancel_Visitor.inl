// -*- C++ -*-
//
// $Id: CSD_TP_Cancel_Visitor.inl 935 2008-12-10 21:47:27Z mitza $

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE
TAO::CSD::TP_Cancel_Visitor::TP_Cancel_Visitor()
{
}


ACE_INLINE
TAO::CSD::TP_Cancel_Visitor::TP_Cancel_Visitor(PortableServer::Servant servant)
  : servant_(servant)
{
  this->servant_->_add_ref ();
}

TAO_END_VERSIONED_NAMESPACE_DECL
