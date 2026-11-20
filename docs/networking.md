# Networking and camera discovery

**Read this before debugging "no cameras found".**

This unit is a **network camera** whose factory address is `192.168.3.99`. Established by
inspecting the SDK, not by guessing:

- `queryCameras()` discovers over **UDP broadcast** — `lib3DCamera.so` contains
  `ossCreateBroadcastSocket` and `"create broadcast socket failed"`. Broadcast is link-local, so
  **a router will not carry it**. Being able to `ping` the camera proves nothing about whether
  the SDK can find it; reachability and discoverability are separate problems here.
- **There is no connect-by-IP API.** `ICamera::connect()` takes only a `CameraInfo`, which holds
  `name`, `serial`, `uniqueId`, `firmwareVersion`, `algorithmVersion` — no address field. You
  cannot bypass discovery.
- `CameraIpSetting` documents `ipBytes[4]` as literally `{192,168,3,99}`, and `IpSetting` exposes
  only a DHCP flag plus `ipFourthByte`. The SDK's addressing model assumes **the host is on the
  camera's /24**. That is the intended topology.

So the fix is a host interface on the camera's L2 segment (direct-attach with e.g.
`192.168.3.10/24`, or a secondary address on the interface facing it). Routing between subnets is
not sufficient; if the camera must stay routed, a UDP broadcast relay is required.
`PROPERTY_EXT_IP_SETTING` / `PROPERTY_EXT_CAMERA_IP` can move the camera onto another subnet
permanently, but only after one successful connection — chicken-and-egg.

Broadcast **discovery** does not survive container bridge networking, a colima VM, or a router.
The **direct-connect fallback does**, and that is the path this unit actually uses, so the
practical rule is narrower than "never use Docker or a VM":

- The robot host is itself a VM (virtio NICs) with `enp6s19` on `192.168.3.98/24`. `queryCameras()`
  returns 0 there, every time, and direct connect succeeds in ~1.2 s.
- `docker run --network host` works too - measured 11-12 fps at 800x600 with 0 errors and 0 dropped
  clouds, plus clean shutdown, from inside a `ros:jazzy-ros-base` container (`make docker-run`).
  Host networking is required; bridge networking does not put you on the camera's L2 segment.

So a `queryCameras` count of 0 is normal here and not the thing to debug. What matters is that the
host has an interface on the camera's /24. On genuine failure the node spends ~20 s in the fallback
before logging `Direct connect failed (ret=2)`; wait it out before concluding anything.
