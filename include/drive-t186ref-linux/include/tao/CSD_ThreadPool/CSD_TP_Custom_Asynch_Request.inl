// -*- C++ -*-
//
// $Id: CSD_TP_Custom_Asynch_Request.inl 14 2007-02-01 15:49:12Z mitza $

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE
TAO::CSD::TP_Custom_Asynch_Request::TP_Custom_Asynch_Request
                             (TP_Custom_Request_Operation* op,
                              TP_Servant_State*            servant_state)
  : TP_Custom_Request(op,servant_state)
{
}

TAO_END_VERSIONED_NAMESPACE_DECL
