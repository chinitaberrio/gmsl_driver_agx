#ifndef guard_bounded_reference_allocation_traits_hpp
#define guard_bounded_reference_allocation_traits_hpp
/**
 * @file
 *
 * @brief Details can be found in the documentation for
 * TAO::details::generic_sequence
 *
 * $Id: Bounded_Reference_Allocation_Traits_T.h 1861 2011-08-31 16:18:08Z mesnierp $
 *
 * @author Carlos O'Ryan
 */

#include "tao/Basic_Types.h"

TAO_BEGIN_VERSIONED_NAMESPACE_DECL

namespace TAO
{
namespace details
{

template<typename T, class ref_traits, CORBA::ULong MAX, bool dummy>
struct bounded_reference_allocation_traits
{
  typedef T value_type;
  typedef ref_traits reference_traits;

  inline static CORBA::ULong default_maximum()
  {
    return MAX;
  }

  inline static value_type * default_buffer_allocation()
  {
    return 0;
  }

  inline static value_type * allocbuf(CORBA::ULong /* maximum */)
  {
    value_type * buffer = new value_type[MAX];
    // no throw
    reference_traits::initialize_range(buffer, buffer + MAX);

    return buffer;
  }

  inline static value_type * allocbuf_noinit(CORBA::ULong /* maximum */)
  {
    value_type * buffer = new value_type[MAX];
    // no throw
    reference_traits::zero_range(buffer, buffer + MAX);

    return buffer;
  }

  inline static void freebuf(value_type * buffer)
  {
    reference_traits::release_range(buffer, buffer + MAX);
    delete[] buffer;
  }

  inline static CORBA::ULong maximum()
  {
    return MAX;
  }
};

} // namespace details
} // namespace TAO

TAO_END_VERSIONED_NAMESPACE_DECL

#endif // guard_bounded_reference_allocation_traits_hpp
