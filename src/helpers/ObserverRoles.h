#pragma once
//
// Role → capability map for the observer-backhaul fork.
//
// Each build env declares exactly ONE role via `-D ROLE_<name>` and pulls this header
// in globally with `-include helpers/ObserverRoles.h` (see variants/*/platformio.ini).
// Everything downstream gates on the derived WITH_* capability flags below — never on a
// raw hardware flag. To add a node role, add a ROLE_* block here and a matching env;
// the command/feature gating then falls out automatically.
//
// Capability flags:
//   WITH_OBSERVER             node observes RF: owns the packet feed + `log*` commands,
//                             boots with the dump on (overridable via /cliext_cfg).
//   WITH_BACKHAUL_PERIPHERAL  serves its console+feed over a BLE NUS peripheral (the
//                             feed source; a central connects in to drain it).
//   WITH_BACKHAUL_CENTRAL     BLE central: connects out to a peripheral's NUS, pulls
//                             its feed, and relays it onward (e.g. to TCP :5001).
//   WITH_RAK13800_ETHERNET    W5100S ethernet hardware present: `eth`, `start tcpota`,
//                             TCP feeds. A genuine hardware flag (not role-semantic).
//   WITH_NET_BRIDGE           bridges the observer feed to a network: `get/set mqtt.*`
//                             config + the Phase-B1 on-node MQTT publisher. Transport-
//                             agnostic, so a future Wi-Fi uplink sets it without ethernet.
//
// NOTE: PACKET_LOG_STREAM (+ _HEADER) is NOT derived here — it names a concrete
// transport object (BleConsole vs EthConsole) and stays in the env so this header
// remains pure capability flags with no transport-header coupling.

#ifdef ROLE_OBSERVER_MAST
  // RAK3401 1W repeater, outdoor. The "real" RF observer; no IP path. Serves its
  // console + packet feed over a BLE NUS peripheral for the relay gateway to drain.
  #define WITH_OBSERVER            1
  #define WITH_BACKHAUL_PERIPHERAL 1
#endif

// ROLE_RELAY_GATEWAY is a superset of ROLE_ETH_GATEWAY: an eth gateway that ALSO runs
// the BLE central relay. Promote it to the base role (so the shared caps below are
// defined in exactly one place) and add the relay capability. The relay's source
// (BleNusRelay.cpp) is compiled only by the _blerelay env, which is why backhaul lives
// on this role and not on the bare eth gateway — a base eth_gw with WITH_BACKHAUL_CENTRAL
// but no relay source would fail to link.
#ifdef ROLE_RELAY_GATEWAY
  #define WITH_BACKHAUL_CENTRAL    1
  #ifndef ROLE_ETH_GATEWAY
    #define ROLE_ETH_GATEWAY
  #endif
#endif

#ifdef ROLE_ETH_GATEWAY
  // RAK4631 wired room-server. Observes its own RF, serves its feed over ethernet (TCP),
  // and uplinks to MQTT. Add the BLE relay by selecting ROLE_RELAY_GATEWAY instead.
  #define WITH_OBSERVER            1
  #define WITH_RAK13800_ETHERNET   1
  #define WITH_NET_BRIDGE          1
#endif
