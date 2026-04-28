// Package optimizer uses simulated annealing to find a tree + edge
// configuration that minimizes a diagram-quality energy function.
//
// The energy is
//
//	E = Wedge * Σ edgeCost + Warea * canvasArea
//
// where edgeCost is the router's weighted cost per edge and canvasArea
// is the width × height of the root rect. Perturbations operate on
// elements a constraint-based layout treats as free:
//
//  1. Sibling reorder: swap two unconstrained children of the same
//     parent. This changes the default stacking order without
//     violating any align/near constraint.
//  2. Nudge: apply a small offset to an unconstrained leaf node's
//     position (only if the caller enables Nudging).
//
// Sibling reorder is the default because it preserves the constraint
// model exactly. Nudges are a lever for finer-grained optimization
// at the cost of inventing a post-solve offset that wasn't in the
// original layout contract.
//
// Five seeds run in parallel; the best result (lowest final energy)
// is returned.
package optimizer

import (
	"math"
	"math/rand"
	"sync"

	"github.com/AnuchitO/khantae/engine"
	"github.com/AnuchitO/khantae/engine/router"
)

// Config tunes the SA run.
type Config struct {
	Seeds       int     // number of parallel SA runs (default 5)
	MaxIter     int     // hard stop per seed (default 500)
	StartTemp   float64 // starting temperature (default 100)
	Alpha       float64 // cooling factor per iteration (default 0.95)
	StagnantLim int     // stop after this many non-improving iters (default 80)

	// Energy term weights. Defaults put canvas area and edge cost on
	// roughly the same order of magnitude for small diagrams.
	Warea float64
	Wedge float64

	// Router weights forwarded to every energy evaluation.
	RouteWeights router.Weights

	// EnableNudge turns on leaf-offset perturbations in addition to
	// sibling reorders. Off by default because it departs slightly
	// from the pure-constraint model.
	EnableNudge bool

	// NudgeMagnitude in user units. Ignored unless EnableNudge is set.
	NudgeMagnitude float64

	// RoutingMargin is extra space around the root's bounding box
	// that's included in the routing mesh. Without margin, a tight
	// stack of nodes leaves no room for edges that need to leave and
	// re-enter from the same side. Default 40 (set by Optimize when 0).
	RoutingMargin float64
}

// DefaultConfig returns the configuration described in the design
// brief.
func DefaultConfig() Config {
	return Config{
		Seeds:          5,
		MaxIter:        500,
		StartTemp:      100,
		Alpha:          0.95,
		StagnantLim:    80,
		Warea:          0.01, // 10000 px² contributes 100 energy units
		Wedge:          1.0,
		RouteWeights:   router.DefaultWeights(),
		EnableNudge:    false,
		NudgeMagnitude: 4,
		RoutingMargin:  40,
	}
}

// Problem is the input to optimization: the initial tree and the set
// of edges to route. Obstacles are derived from the tree (every node
// is a hard obstacle by default).
type Problem struct {
	Root  *engine.Node
	Edges []router.EdgeRequest

	// LabelRects are optional soft-obstacle rects (e.g., text labels
	// attached to nodes). Keyed by an arbitrary label ID.
	LabelRects map[string]engine.Rect
}

// Result carries the best layout found plus metrics.
type Result struct {
	Tree      *engine.Node                // perturbed tree (deep copy of input)
	Rects     map[string]engine.Rect      // final positions
	Routes    []router.Route                 // final routes
	Energy    float64                        // final total energy
	EdgeCost  float64                        // Σ edge costs at final state
	Area      float64                        // canvas area at final state
	Seed      int64                          // rng seed that produced this
	Iters     int                            // iterations consumed
	Trace     []float64                      // per-iter energy (for debugging)
}

// Optimize runs SA with Seeds parallel goroutines and returns the best
// Result. An error surfaces only if the initial configuration cannot
// even be evaluated (i.e. the layout or router fails on turn zero).
func Optimize(p Problem, cfg Config) (*Result, error) {
	if cfg.Seeds <= 0 {
		cfg.Seeds = 1
	}

	if cfg.RoutingMargin == 0 {
		cfg.RoutingMargin = 40
	}

	// Sanity-check that the initial config is solvable. If this
	// fails, no amount of perturbation will help — bubble the error.
	if _, err := evaluate(p.Root, nil, p.Edges, p.LabelRects, cfg); err != nil {
		return nil, err
	}

	type seedResult struct {
		res *Result
		err error
	}
	out := make(chan seedResult, cfg.Seeds)

	var wg sync.WaitGroup
	for i := 0; i < cfg.Seeds; i++ {
		wg.Add(1)
		seed := int64(i*1000 + 1)
		go func() {
			defer wg.Done()
			r, err := runSeed(p, cfg, seed)
			out <- seedResult{r, err}
		}()
	}
	wg.Wait()
	close(out)

	var best *Result
	for sr := range out {
		if sr.err != nil || sr.res == nil {
			continue
		}
		if best == nil || sr.res.Energy < best.Energy {
			best = sr.res
		}
	}
	if best == nil {
		return nil, errAllSeedsFailed
	}
	return best, nil
}

var errAllSeedsFailed = &optError{"all seeds failed"}

type optError struct{ msg string }

func (e *optError) Error() string { return e.msg }

// runSeed executes one SA chain.
func runSeed(p Problem, cfg Config, seed int64) (*Result, error) {
	rng := rand.New(rand.NewSource(seed))

	// Deep-copy the tree so this seed can mutate freely. Nudges live
	// alongside the tree as a per-ID delta applied after engine.Layout.
	tree := deepCopy(p.Root)
	nudges := map[string][2]float64{}

	curEval, err := evaluate(tree, nudges, p.Edges, p.LabelRects, cfg)
	if err != nil {
		return nil, err
	}

	best := snapshot(tree, curEval)
	trace := []float64{curEval.energy}

	temp := cfg.StartTemp
	stagnant := 0

	for iter := 1; iter <= cfg.MaxIter; iter++ {
		// Propose a perturbation on a copy so we can revert cheaply.
		candidate := deepCopy(tree)
		candNudges := copyNudges(nudges)
		changed := perturb(candidate, candNudges, rng, cfg)
		if !changed {
			// No free perturbation available; we're stuck.
			break
		}

		candEval, err := evaluate(candidate, candNudges, p.Edges, p.LabelRects, cfg)
		if err != nil {
			// Infeasible candidate (e.g. constraint error after
			// reorder, which shouldn't happen but is defended).
			// Skip and cool down.
			temp *= cfg.Alpha
			continue
		}

		dE := candEval.energy - curEval.energy
		accept := dE < 0 || rng.Float64() < math.Exp(-dE/temp)
		if accept {
			tree = candidate
			nudges = candNudges
			curEval = candEval
			if candEval.energy < best.Energy {
				best = snapshot(tree, candEval)
				stagnant = 0
			} else {
				stagnant++
			}
		} else {
			stagnant++
		}

		trace = append(trace, curEval.energy)
		temp *= cfg.Alpha

		if stagnant >= cfg.StagnantLim {
			break
		}
	}

	best.Seed = seed
	best.Iters = len(trace) - 1
	best.Trace = trace
	return best, nil
}

func copyNudges(m map[string][2]float64) map[string][2]float64 {
	out := make(map[string][2]float64, len(m))
	for k, v := range m {
		out[k] = v
	}
	return out
}

// evaluation bundles the intermediate results so a snapshot can record
// them without re-running the pipeline.
type evaluation struct {
	rects    map[string]engine.Rect
	routes   []router.Route
	edgeCost float64
	area     float64
	energy   float64
}

// evaluate runs the full pipeline (layout + route) and computes energy.
// nudges is a map of leaf-ID -> [dx, dy] applied after layout but
// before routing. It realizes the Phase 4 "Jitter Pass" — small
// random offsets on mesh nodes to escape local optima.
func evaluate(tree *engine.Node, nudges map[string][2]float64,
	edges []router.EdgeRequest,
	labelRects map[string]engine.Rect, cfg Config) (*evaluation, error) {

	rects, err := engine.Layout(tree)
	if err != nil {
		return nil, err
	}

	// Apply nudges. We copy the rects map so the layout cache (if any
	// in future) and the returned map are separate.
	if len(nudges) > 0 {
		nudged := make(map[string]engine.Rect, len(rects))
		for id, r := range rects {
			if off, ok := nudges[id]; ok {
				r.X += off[0]
				r.Y += off[1]
			}
			nudged[id] = r
		}
		rects = nudged
	}

	// Build obstacles: every LEAF rect becomes a node obstacle. Interior
	// containers are organizational and their rects fully enclose their
	// children, so including them would block every edge endpoint (ports
	// on a child's boundary are strictly inside the parent's rect).
	leafIDs := collectLeafIDs(tree)
	obs := make([]router.Obstacle, 0, len(leafIDs)+len(labelRects))
	for _, id := range leafIDs {
		r, ok := rects[id]
		if !ok {
			continue
		}
		obs = append(obs, router.Obstacle{ID: id, Rect: r, Kind: router.KindNode})
	}
	// leafIDs already comes back in deterministic pre-order so that
	// repeated evaluations of the same tree produce identical obstacle
	// lists. No further sorting needed.
	for id, r := range labelRects {
		obs = append(obs, router.Obstacle{ID: id, Rect: r, Kind: router.KindLabel})
	}

	// Frame points extend the mesh past the root rect so edges have
	// room to maneuver when ports are tight against a boundary.
	rootRect := rects[tree.ID]
	m := cfg.RoutingMargin
	framePts := []router.Point{
		{X: rootRect.X - m, Y: rootRect.Y - m},
		{X: rootRect.X + rootRect.W + m, Y: rootRect.Y + rootRect.H + m},
		{X: rootRect.X - m, Y: rootRect.Y + rootRect.H + m},
		{X: rootRect.X + rootRect.W + m, Y: rootRect.Y - m},
	}

	routes, err := router.RouteAll(rects, obs, edges, cfg.RouteWeights, framePts...)
	if err != nil {
		return nil, err
	}

	var edgeCost float64
	for _, r := range routes {
		edgeCost += r.Cost
	}
	root := tree.ID
	area := rects[root].W * rects[root].H
	energy := cfg.Wedge*edgeCost + cfg.Warea*area
	return &evaluation{
		rects: rects, routes: routes,
		edgeCost: edgeCost, area: area, energy: energy,
	}, nil
}

func snapshot(tree *engine.Node, e *evaluation) *Result {
	return &Result{
		Tree:     deepCopy(tree),
		Rects:    e.rects,
		Routes:   e.routes,
		Energy:   e.energy,
		EdgeCost: e.edgeCost,
		Area:     e.area,
	}
}
