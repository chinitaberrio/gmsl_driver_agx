//* -*- C++ -*- */
// $Id: config-vxworks6.9.h 2179 2013-05-28 22:16:51Z mesnierp $

// The following configuration file is designed to work for VxWorks
// 6.9 platforms using one of these compilers:
// 1) The GNU g++ compiler that is shipped with VxWorks 6.9
// 2) The Diab compiler that is shipped with VxWorks 6.9

#ifndef ACE_CONFIG_VXWORKS_6_9_H
#define ACE_CONFIG_VXWORKS_6_9_H
#include /**/ "ace/pre.h"

#if !defined (ACE_VXWORKS)
# define ACE_VXWORKS 0x690
#endif /* ! ACE_VXWORKS */

#define ACE_HAS_SSIZE_T

#include "ace/config-vxworks6.8.h"

#ifndef ACE_HAS_2_PARAM_ASCTIME_R_AND_CTIME_R
// already defined for earlier RTP versions
# define ACE_HAS_2_PARAM_ASCTIME_R_AND_CTIME_R 1
#endif

#if defined(__RTP__)
// bzero is in strings.h
# define ACE_HAS_STRINGS 1
# if defined ACE_HAS_PTHREADS
#  define ACE_HAS_RECURSIVE_THR_EXIT_SEMANTICS
# endif
#endif

#include /**/ "ace/post.h"
#endif /* ACE_CONFIG_VXWORKS_6_9_H */

