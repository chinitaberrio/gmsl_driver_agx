// $Id: config-macosx-snowleopard.h 2179 2013-05-28 22:16:51Z mesnierp $
#ifndef ACE_CONFIG_MACOSX_SNOWLEOPARD_H
#define ACE_CONFIG_MACOSX_SNOWLEOPARD_H

#include "ace/config-macosx-leopard.h"

#ifdef __clang__
#ifdef ACE_HAS_GCC_ATOMIC_BUILTINS
#undef ACE_HAS_GCC_ATOMIC_BUILTINS
#endif

#define ACE_ANY_OPS_USE_NAMESPACE

#endif

#define ACE_LACKS_UCONTEXT_H

#endif // ACE_CONFIG_MACOSX_SNOWLEOPARD_H
