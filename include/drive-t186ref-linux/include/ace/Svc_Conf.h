// -*- C++ -*-

//=============================================================================
/**
 *  @file    Svc_Conf.h
 *
 *  $Id: Svc_Conf.h 935 2008-12-10 21:47:27Z mitza $
 *
 *  @author Doug Schmidt
 */
//=============================================================================


#ifndef ACE_SVC_CONF_H
#define ACE_SVC_CONF_H

#include /**/ "ace/pre.h"

#include "ace/Service_Config.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

#include "ace/Parse_Node.h"
#include "ace/Svc_Conf_Param.h"

ACE_BEGIN_VERSIONED_NAMESPACE_DECL

/// Factory that creates a new ACE_Service_Type_Impl.
extern ACE_Service_Type_Impl *
ace_create_service_type (ACE_TCHAR const *,
                         int,
                         void *,
                         unsigned int,
                         ACE_Service_Object_Exterminator = 0);


ACE_END_VERSIONED_NAMESPACE_DECL

#include /**/ "ace/post.h"

#endif /* ACE_SVC_CONF_H */
