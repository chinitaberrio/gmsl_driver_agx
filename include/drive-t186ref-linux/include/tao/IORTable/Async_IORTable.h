// -*- C++ -*-


//=============================================================================
/**
 *  @file    Async_IORTable.h
 *
 *  $Id: Async_IORTable.h 2179 2013-05-28 22:16:51Z mesnierp $
 *
 *  @author Phil Mesnier <mesnier_p@ociweb.com>
 */
//=============================================================================


#ifndef TAO_ASYNC_IORTABLE_H
#define TAO_ASYNC_IORTABLE_H

#include /**/ "ace/pre.h"

#include "tao/IORTable/async_iortable_export.h"

#if !defined (ACE_LACKS_PRAGMA_ONCE)
# pragma once
#endif /* ACE_LACKS_PRAGMA_ONCE */

#include "tao/Versioned_Namespace.h"

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

class TAO_Async_IORTable_Export TAO_Async_IORTable_Initializer
{
public:
  /// Used to force the initialization of the ORB code.
  static int init (void);
};

static int
TAO_Requires_Async_IORTable_Initializer = TAO_Async_IORTable_Initializer::init ();

TAO_END_VERSIONED_NAMESPACE_DECL

#define TAO_IORTABLE_SAFE_INCLUDE
#include "tao/IORTable/Locate_ResponseHandler.h"
#include "tao/IORTable/Async_IORTableC.h"
#undef TAO_IORTABLE_SAFE_INCLUDE

#include /**/ "ace/post.h"

#endif /* TAO_ASYNC_IORTABLE_H */
