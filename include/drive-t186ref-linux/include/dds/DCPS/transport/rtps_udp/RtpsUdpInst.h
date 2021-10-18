/*
 *
 *
 * Distributed under the OpenDDS License.
 * See: http://www.opendds.org/license.html
 */

#ifndef DCPS_RTPSUDPINST_H
#define DCPS_RTPSUDPINST_H

#include "Rtps_Udp_Export.h"

#include "dds/DCPS/transport/framework/TransportInst.h"
#include "dds/DCPS/SafetyProfileStreams.h"

OPENDDS_BEGIN_VERSIONED_NAMESPACE_DECL

namespace OpenDDS {
namespace RTPS {
  class Sedp;
}

namespace DCPS {
class RtpsUdpTransport;
class TransportReceiveListener;
typedef RcHandle<TransportReceiveListener> TransportReceiveListener_rch;

class OpenDDS_Rtps_Udp_Export RtpsUdpInst : public TransportInst {
public:
  ACE_INT32 send_buffer_size_;
  ACE_INT32 rcv_buffer_size_;

  bool use_multicast_;
  unsigned char ttl_;
  ACE_INET_Addr multicast_group_address_;
  OPENDDS_STRING multicast_interface_;

  size_t nak_depth_;
  ACE_Time_Value nak_response_delay_, heartbeat_period_,
    heartbeat_response_delay_, handshake_timeout_, durable_data_timeout_;

  virtual int load(ACE_Configuration_Heap& cf,
                   ACE_Configuration_Section_Key& sect);

  /// Diagnostic aid.
  virtual OPENDDS_STRING dump_to_str() const;

  bool is_reliable() const { return true; }
  bool requires_cdr() const { return true; }

  virtual size_t populate_locator(OpenDDS::DCPS::TransportLocator& trans_info) const;
  const TransportBLOB* get_blob(const OpenDDS::DCPS::TransportLocatorSeq& trans_info) const;

  OPENDDS_STRING local_address_string() const { return local_address_config_str_; }
  ACE_INET_Addr local_address() const { return local_address_; }
  void local_address(const char* str)
  {
    local_address_config_str_ = str;
    local_address_.set(str);
  }
  void local_address(u_short port_number, const char* host_name)
  {
    local_address_config_str_ = host_name;
    local_address_config_str_ += ":" + to_dds_string(port_number);
    local_address_.set(port_number, host_name);
  }
  void local_address_set_port(u_short port_number) {
    local_address_.set_port_number(port_number);
    set_port_in_addr_string(local_address_config_str_, port_number);
  }

private:
  friend class RtpsUdpType;
  template <typename T, typename U>
  friend RcHandle<T> OpenDDS::DCPS::make_rch(U const&);
  explicit RtpsUdpInst(const OPENDDS_STRING& name);

  TransportImpl_rch new_impl();

  friend class RTPS::Sedp;
  friend class RtpsUdpTransport;
  TransportReceiveListener_rch opendds_discovery_default_listener_;
  RepoId opendds_discovery_guid_;

  ACE_INET_Addr local_address_;
  OPENDDS_STRING local_address_config_str_;
};

} // namespace DCPS
} // namespace OpenDDS

OPENDDS_END_VERSIONED_NAMESPACE_DECL

#endif  /* DCPS_RTPSUDPINST_H */
