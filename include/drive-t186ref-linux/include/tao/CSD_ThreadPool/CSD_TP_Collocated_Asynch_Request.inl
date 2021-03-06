// -*- C++ -*-
//
// $Id: CSD_TP_Collocated_Asynch_Request.inl 14 2007-02-01 15:49:12Z mitza $

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

ACE_INLINE
TAO::CSD::TP_Collocated_Asynch_Request::TP_Collocated_Asynch_Request
                             (TAO_ServerRequest&              server_request,
                              const PortableServer::ObjectId& object_id,
                              PortableServer::POA_ptr         poa,
                              const char*                     operation,
                              PortableServer::Servant         servant,
                              TP_Servant_State*               servant_state)
  : TP_Corba_Request(object_id,
                     poa,
                     operation,
                     servant,
                     servant_state,
                     server_request)
{
}

TAO_END_VERSIONED_NAMESPACE_DECL
