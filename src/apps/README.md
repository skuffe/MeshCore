# Vendored role apps

Fork-owned copies of MeshCore role applications (the `examples/<role>/` entry
points) that we modify. Vendoring keeps the upstream `examples/` tree pristine,
so `make rebase` never conflicts there. New files under `src/apps/` never
conflict on rebase.

| Dir                  | Vendored from                  | Scope                                   |
|----------------------|--------------------------------|-----------------------------------------|
| `repeater_ble_gw/`   | `examples/simple_repeater/`    | `main.cpp` only — the rest of the role is pristine upstream and still compiled from the example via `-I examples/simple_repeater`. |
| `room_server/`       | `examples/simple_room_server/` | **Whole role** (`main.cpp` + `MyMesh.{cpp,h}` + `UITask.{cpp,h}`) — the example is fully unused, so it stays 100% pristine. |

The env's `build_src_filter` subtracts the upstream source and adds the vendored
copy; see `variants/rak4631/platformio.ini`.

## Drift

Vendoring trades rebase conflicts for silent drift: upstream fixes to these
files won't reach the vendored copies automatically. After every rebase, run:

```
make vendor-diff FROM=<prev-upstream-ref> TO=<new-upstream-ref>
```

and port anything relevant. `make rebase` prints this reminder.

`room_server/` carries the larger drift risk (`MyMesh.cpp` is actively developed
upstream) — review it carefully on each bump.
