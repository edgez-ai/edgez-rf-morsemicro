# edgez-rf-morsemicro

Apache-2.0-licensed Morse Micro backend for the vendor-neutral
[`edgez-rf-api`](https://github.com/edgez-ai/edgez-rf-api) interface.

## Layered architecture

![EdgeZ Mesh four-layer architecture](docs/edgez-mesh-four-layer-architecture.png)

`edgez-rf-morsemicro` is the layer-3 adapter. The EdgeZ Mesh core reaches it
only through the Apache-2.0 `edgez-rf-api` contract; Morse packet types,
headers, callbacks, and driver policy stay on the adapter/vendor side of that
boundary. The adapter is Apache-2.0, but that does not relicense the separate
Morse Micro SDK or driver in layer 4.

The Morse hosted-library sources currently identify themselves as
`GPL-3.0-or-later OR LicenseRef-MorseMicroCommercial`. Anyone distributing a
firmware image that incorporates those objects must use a valid Morse Micro
commercial license or independently satisfy the applicable GPL requirements.
Always verify the exact vendor version and its file-level license notices.

This repository contains only the adapter boundary:

- `edgez_umac_morse_adapter.c` maps `edgez_rf_backend_ops` to `mmdrv`.
- `edgez_umac_rf_binding.c` connects Morse RF callbacks to the EdgeZ UMAC.
- `edgez_mmdrv_host.c` supplies the RF-only host callback surface required by
  the Morse lower-MAC/driver archive.
- Beacon templates remain raw EdgeZ-owned bytes; this adapter copies them and
  allocates a native Morse packet only when the driver requests a beacon.
- RX callbacks translate Morse packet metadata only. Frame classification and
  management/data parsing are owned by the vendor-neutral EdgeZ UMAC RX
  pipeline.

The mesh protocol, peer state machines, routing, security policy, and raw
802.11 frame construction remain in `edgez-halow-mesh`. That repository must not
include Morse headers or native packet types. A different radio vendor can be
supported by implementing the same `edgez_rf_backend_ops` ABI in another
adapter repository.

## Consumer contract

The build must provide:

- `edgez-rf-api/include` for the vendor-neutral radio contracts;
- `edgez-halow-mesh/include` for `edgez_umac.h`;
- the Morse Micro `mmdrv`/`mmpkt` headers and implementation;
- the platform MMOSAL implementation used by the Morse driver.

Include `component.cmake` to obtain the source and include-directory variables
used by both ESP-IDF and Zephyr integrations.

## On-demand MAC diagnostics

`edgez_morse_diagnostics.h` exposes `edgez_morse_read_mac_diagnostics()`.
Call it from an application task with the radio running, using caller-owned
storage of at least `EDGEZ_MORSE_MAC_DIAGNOSTICS_CAPACITY` bytes. It returns
one bounded, validated MAC statistics command response, including the native
header/status, for offline decoding against matching firmware metadata.
Zero means success; positive values are native driver errors; negative values
indicate invalid arguments or a malformed response. Always check the result
before interpreting the response as statistics.

The helper installs no timer and does not reset counters, transmit a test
frame, or change the station, power-save or TWT configuration. Reading the
statistics temporarily wakes the host transport. Counters and state flags
are protocol diagnostics, not measurements of RF sleep power.
The portable mesh core continues to use only vendor-neutral RF interfaces;
this backend-specific helper is for platform diagnostics.
