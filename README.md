# khantae

An experimental orthogonal-routing layout engine for [D2](https://d2lang.com).

Khantae plugs into D2 as a layout option (`d2 --layout=khantae`) and produces
diagrams with strictly orthogonal edges and perpendicular port entry, implemented
independently under MIT license.

## Status: experimental

Khantae is at an early stage. It produces valid, viewable diagrams with
clean orthogonal routes for small graphs. Node placement uses graph-aware
layered layout: nodes are grouped into horizontal layers by edge topology,
with in-layer edges rendered as clean horizontal connectors.

What this means in practice:

- ✅ All edges are strictly orthogonal (no diagonals)
- ✅ Edges enter and exit nodes perpendicularly
- ✅ Multiple edges leaving the same side fan out to distinct port offsets
- ✅ Routes detour around obstacles instead of crossing them
- ✅ Graph-aware layered placement with compact grouping
- ✅ Labels centred inside their boxes
- ⚠️ Containers are flattened (children don't visually nest inside their parent)
- ⚠️ Performance: ~1s on 50-node graphs; needs work before large-diagram use
- ⚠️ Many D2 features (`near` to objects, container dimensions, descendant edges) are not yet supported and will be rejected by D2 with a clear error

If you want production-quality layouts today, use `dagre` or `elk`. If you
want to hack on a small, readable orthogonal router and contribute to it,
read on.

## Install

Requires Go 1.24 or later.

```sh
go install github.com/AnuchitO/khantae/cmd/d2plugin-khantae@latest
```

This builds and installs `d2plugin-khantae` into `$GOPATH/bin` (typically
`~/go/bin`). Make sure that directory is on your `$PATH`. D2 discovers
layout plugins by looking for binaries named `d2plugin-<name>` on `$PATH`.

Verify D2 sees it:

```sh
d2 layout
```

You should see `khantae` listed alongside `dagre` and `elk`.

## Use

```sh
d2 --layout=khantae input.d2 output.svg
```

Plugin-specific flags:

```sh
d2 --layout=khantae --khantae-seeds=44 --khantae-max-iter=200 input.d2 out.svg
```

Currently `--khantae-seeds` and `--khantae-max-iter` are accepted but only
used by the engine's optimizer, which is not yet active in the D2 path.
They are wired up so the protocol contract is correct; activating them
is part of the next phase.

## How it works

The repo contains two concerns:

- **`engine/`** — the layout engine itself, usable as a Go library
  independent of D2. It does:
  - Constraint-based box layout (`engine.Layout`)
  - A* orthogonal edge routing on a sparse visibility mesh (`engine/router`)
  - Simulated-annealing optimization over multiple seeds (`engine/optimizer`)

- **`cmd/d2plugin-khantae/`** — the D2 plugin binary. It speaks D2's
  external-plugin protocol, translates D2's graph to the engine's domain,
  runs layout + routing, and writes positions and routes back.

The layout algorithm assigns layers using a fanout-aware topological sort:
edges from nodes with out-degree > 1 carry span=0 so immediate successors
land in the same layer as their source, producing compact horizontal grouping.
Within each layer, nodes are ordered by barycenter of parent positions to
minimise edge crossings.

The router uses a multi-factor cost function: `C = 50·bends + 200·crossings + length + 10·staircases`.
Crossing weight is dominant by design — visually, edges crossing each other
is the worst thing the router can do, so it's penalized heavily enough that
the search prefers even ugly detours.

## Examples

See `examples/` for sample `.d2` inputs. To try them:

```sh
d2 --layout=khantae examples/simple.d2 simple.svg
d2 --layout=khantae examples/branchy.d2 branchy.svg
d2 --layout=khantae examples/architecture.d2 architecture.svg
```

## Development

```sh
go test ./...      # run the test suite
go build ./...     # build everything
```

The engine has its own test suite covering layout, routing, the optimizer,
and Phase 4 features (port fanout, staircase detection, weight invariants).
The plugin binary doesn't currently have integration tests; that's tracked
under "next steps" below.

## Roadmap

In rough order of leverage:

1. **Honor D2 nesting.** Containers should visually contain their children.
   Requires reconciling D2's pre-computed container sizes with the engine's
   bottom-up sizing.
2. **Performance.** Get the 50-node stress test under 500ms (currently ~1s).
   The next wins from profiling are pooling A* search nodes and replacing
   the `usedSegments` map with a slice.
3. **Activate the optimizer in the plugin path.** Once placement is good
   enough that the optimizer has room to improve, wire it in behind a flag
   so users can opt into longer-but-better layouts.
4. **Visual benchmark suite.** Render a fixed set of canonical D2 examples
   with `dagre`, `elk`, and `khantae` side by side. Track quality across
   versions.

## Contributing

Contributions welcome. The engine is small and the code is heavily
commented — `engine/router/route.go` and `engine/router/astar.go` are
the most algorithmically dense files; everything else is mechanical.

When opening a PR, please include either a test that demonstrates the
fix or a before/after rendered SVG of an example diagram.

## License

MIT. See [LICENSE](LICENSE).

## Acknowledgements

Built on D2's external plugin protocol — see
[D2's plugin documentation](https://d2lang.com/tour/layouts) and the
`d2plugin` package in the [D2 source](https://github.com/terrastruct/d2)
for protocol details.
