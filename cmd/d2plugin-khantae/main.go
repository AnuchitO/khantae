// Command d2plugin-khantae is the d2 layout-engine plugin binary for
// the khantae layout engine.
//
// PHASE A SCOPE: this build is a *passthrough*. It:
//
//   - implements the d2plugin.Plugin interface,
//   - returns the graph from Layout() unchanged (so D2 falls back to
//     whatever positions the graph already has, which for an
//     uncharted graph means everything is at the origin — that's fine
//     for this phase, we want to learn the schema, not produce
//     usable output yet),
//   - logs every received graph to a file under $HOME/.khantae/logs/
//     so we can inspect the real on-the-wire JSON shape.
//
// Once Phase A confirms the protocol works end-to-end (D2 invokes
// us, our binary parses the input, and D2 accepts our reply), Phase
// B will plug in the real layout engine from the boxlayout module.
//
// Install: go build -o d2plugin-khantae ./cmd/d2plugin-khantae
//          mv d2plugin-khantae ~/go/bin/   # or anywhere on PATH
// Verify:  d2 layout            # khantae should appear in the list
//          d2 --layout=khantae example.d2 out.svg
package main

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"

	"oss.terrastruct.com/d2/d2graph"
	"oss.terrastruct.com/d2/d2plugin"
	"oss.terrastruct.com/util-go/xmain"
)

// pluginVersion is the version of this plugin binary itself, separate
// from the engine version (which, in Phase A, doesn't really exist yet).
const pluginVersion = "0.1.0-phase-a"

// khantaePlugin implements d2plugin.Plugin.
//
// The mutex guards `opts`. d2plugin.Serve happens to call HydrateOpts
// from the same goroutine as Layout, so racing isn't actually possible
// today — but plugin_dagre.go uses the same pattern defensively, and
// matching it costs nothing.
type khantaePlugin struct {
	mu   sync.Mutex
	opts *khantaeOpts
}

// khantaeOpts mirrors the JSON shape that D2 sends to HydrateOpts.
// The JSON field names must match the `Tag` fields returned by Flags().
type khantaeOpts struct {
	Seeds   int64 `json:"khantae-seeds"`
	MaxIter int64 `json:"khantae-max-iter"`
}

func defaultOpts() khantaeOpts {
	return khantaeOpts{Seeds: 5, MaxIter: 500}
}

func (p *khantaePlugin) Flags(context.Context) ([]d2plugin.PluginSpecificFlag, error) {
	d := defaultOpts()
	// IMPORTANT: for external (binary) plugins, `Name` and `Tag` must
	// match. D2 turns each opt into a CLI arg as `--<Tag> <value>`
	// when re-invoking the plugin (see d2plugin/exec.go Layout()),
	// while the plugin itself parses CLI args by `Name` (see Serve()
	// in d2plugin/serve.go). If Tag != Name, the plugin sees an
	// unknown flag and aborts with "bad usage". Bundled plugins
	// don't have this constraint because they receive opts as JSON.
	return []d2plugin.PluginSpecificFlag{
		{
			Name:    "khantae-seeds",
			Type:    "int64",
			Default: d.Seeds,
			Usage:   "number of parallel simulated-annealing seeds to run.",
			Tag:     "khantae-seeds",
		},
		{
			Name:    "khantae-max-iter",
			Type:    "int64",
			Default: d.MaxIter,
			Usage:   "max iterations per seed.",
			Tag:     "khantae-max-iter",
		},
	}, nil
}

func (p *khantaePlugin) HydrateOpts(raw []byte) error {
	p.mu.Lock()
	defer p.mu.Unlock()
	o := defaultOpts()
	if len(raw) > 0 {
		if err := json.Unmarshal(raw, &o); err != nil {
			return xmain.UsageErrorf("invalid options for khantae: %v", err)
		}
	}
	p.opts = &o
	return nil
}

func (p *khantaePlugin) Info(ctx context.Context) (*d2plugin.PluginInfo, error) {
	flags, err := p.Flags(ctx)
	if err != nil {
		return nil, err
	}
	opts := xmain.NewOpts(nil, nil)
	for _, f := range flags {
		f.AddToOpts(opts)
	}
	return &d2plugin.PluginInfo{
		Name:      "khantae",
		Type:      "binary",
		ShortHelp: "Khantae: an orthogonal-routing layout engine inspired by TALA's aesthetic (PHASE A — passthrough).",
		LongHelp: fmt.Sprintf(`Khantae is an experimental layout engine for D2 focusing on
clean orthogonal routing with global energy minimization.

This binary is at PHASE A: it speaks the d2 plugin protocol but does
not yet position nodes. It is a passthrough used to validate the
protocol integration. Do not rely on its output for real diagrams.

Version: %s

Flags:
%s
`, pluginVersion, opts.Defaults()),
		// Claim no features for Phase A. As real support lands we
		// flip these on one by one with corresponding tests.
		Features: []d2plugin.PluginFeature{},
	}, nil
}

// Layout: PHASE B real layout via the boxlayout engine.
//
// The graph is logged for debugging, then translated to the engine's
// domain via runEngine() (in adapter.go), which writes positions and
// routes back onto the graph in place.
func (p *khantaePlugin) Layout(ctx context.Context, g *d2graph.Graph) error {
	if err := dumpGraph(g); err != nil {
		fmt.Fprintf(os.Stderr, "khantae: dumpGraph failed: %v\n", err)
	}
	return runEngine(g)
}

func (p *khantaePlugin) PostProcess(ctx context.Context, in []byte) ([]byte, error) {
	return in, nil
}

// dumpGraph writes a JSON serialization of the received graph to
// $HOME/.khantae/logs/<timestamp>.json. We use d2graph.SerializeGraph
// rather than encoding/json directly because Graph contains many
// pointer cycles (parent <-> child <-> parent...) that the standard
// encoder can't handle.
func dumpGraph(g *d2graph.Graph) error {
	home, err := os.UserHomeDir()
	if err != nil {
		return err
	}
	dir := filepath.Join(home, ".khantae", "logs")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return err
	}
	b, err := d2graph.SerializeGraph(g)
	if err != nil {
		return err
	}
	name := fmt.Sprintf("%s.json", time.Now().UTC().Format("20060102T150405.000000"))
	return os.WriteFile(filepath.Join(dir, name), b, 0o644)
}

func main() {
	xmain.Main(d2plugin.Serve(&khantaePlugin{}))
}
