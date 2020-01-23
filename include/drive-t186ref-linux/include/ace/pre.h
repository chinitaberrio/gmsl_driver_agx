// -*- C++ -*-

//=============================================================================
/**
 *  @file    pre.h
 *
 *  $Id: pre.h 2595 2015-07-06 17:31:25Z mesnierp $
 *
 *  @author Christopher Kohlhoff <chris@kohlhoff.com>
 *
 *  This file saves the original alignment rules and changes the alignment
 *  boundary to ACE's default.
 */
//=============================================================================

// No header guard
#if defined (_MSC_VER)
# pragma warning (disable:4103)
# pragma warning (disable:4244)
# pragma warning (disable:4267)
# pragma pack (push, 8)
#elif defined (__BORLANDC__)
# pragma option push -a8 -b -Ve- -Vx- -w-rvl -w-rch -w-ccc -w-obs -w-aus -w-pia -w-inl -w-sig
# pragma nopushoptwarn
# pragma nopackwarning
#endif
