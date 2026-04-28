package optimizer

import (
	"math"
	"testing"

	"github.com/AnuchitO/khantae/engine"
	"github.com/AnuchitO/khantae/engine/router"
)

// buildTestProblem constructs a 4-leaf root with two edges that share
// the right-hand channel, so reordering siblings should help.
func buildTestProblem() Problem {
	mk := func(id string) *engine.Node {
		return &engine.Node{ID: id, MinW: 60, MinH: 30}
	}
	root := &engine.Node{
		ID:      "root",
		Padding: engine.Padding{Top: 10, Right: 10, Bottom: 10, Left: 10},
		Children: []*engine.Node{mk("A"), mk("B"), mk("C"), mk("D")},
	}
	edges := []router.EdgeRequest{
		{ID: "AD", From: router.Endpoint{NodeID: "A", Side: router.SideRight},
			To: router.Endpoint{NodeID: "D", Side: router.SideRight}},
		{ID: "BC", From: router.Endpoint{NodeID: "B", Side: router.SideRight},
			To: router.Endpoint{NodeID: "C", Side: router.SideRight}},
	}
	return Problem{Root: root, Edges: edges}
}

// Energy should not increase (best-ever tracking) and should typically
// decrease relative to the initial configuration.
func TestEnergyNonIncreasing(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 3
	cfg.MaxIter = 100

	// Initial energy.
	initial, err := evaluate(prob.Root, nil, prob.Edges, prob.LabelRects, cfg)
	if err != nil {
		t.Fatalf("initial evaluation: %v", err)
	}

	res, err := Optimize(prob, cfg)
	if err != nil {
		t.Fatalf("Optimize: %v", err)
	}
	if res.Energy > initial.energy+1e-9 {
		t.Errorf("SA worsened energy: initial %v -> final %v", initial.energy, res.Energy)
	}
	t.Logf("initial energy %.2f -> best energy %.2f (%.1f%% reduction)",
		initial.energy, res.Energy,
		100*(initial.energy-res.Energy)/initial.energy)
}

// Fixed seed + single seed must produce deterministic output.
func TestDeterministicSingleSeed(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 1
	cfg.MaxIter = 50

	r1, err := runSeed(prob, cfg, 42)
	if err != nil {
		t.Fatal(err)
	}
	r2, err := runSeed(prob, cfg, 42)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(r1.Energy-r2.Energy) > 1e-9 {
		t.Errorf("same seed produced different energy: %v vs %v", r1.Energy, r2.Energy)
	}
	if len(r1.Trace) != len(r2.Trace) {
		t.Errorf("same seed produced different trace length: %d vs %d", len(r1.Trace), len(r2.Trace))
	}
}

// Different seeds produce different chains (sanity: the SA loop
// actually depends on the RNG).
func TestDifferentSeedsDiffer(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 1
	cfg.MaxIter = 80

	r1, err := runSeed(prob, cfg, 1)
	if err != nil {
		t.Fatalf("seed 1: %v", err)
	}
	r2, err := runSeed(prob, cfg, 2)
	if err != nil {
		t.Fatalf("seed 2: %v", err)
	}
	// The final energies might coincidentally match, but traces
	// should differ on at least one iteration.
	same := len(r1.Trace) == len(r2.Trace)
	if same {
		for i := range r1.Trace {
			if math.Abs(r1.Trace[i]-r2.Trace[i]) > 1e-9 {
				same = false
				break
			}
		}
	}
	if same {
		t.Error("different seeds produced identical traces (RNG not being used?)")
	}
}

// If no free siblings exist, the optimizer should return immediately
// with the initial result rather than error or loop forever.
func TestFullyConstrainedTree(t *testing.T) {
	root := &engine.Node{
		ID: "root",
		Children: []*engine.Node{
			{ID: "A", MinW: 30, MinH: 20},
			{ID: "B", MinW: 30, MinH: 20,
				Align: []engine.AlignConstraint{{To: "A", Edge: engine.EdgeLeft}},
				Near:  []engine.NearConstraint{{To: "A", Side: engine.SideBottom, Gap: 5}}},
		},
	}
	prob := Problem{Root: root, Edges: nil}
	cfg := DefaultConfig()
	cfg.Seeds = 1
	cfg.MaxIter = 50

	res, err := Optimize(prob, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// Should have produced a valid result with iters == 0 or small,
	// since perturb() returned false on turn 1.
	if res.Iters > 0 {
		// With B constrained but A free, no swap is possible (only one
		// unconstrained sibling). So iters == 0 is expected.
		t.Errorf("expected 0 iters on fully-constrained tree, got %d", res.Iters)
	}
}

// Optimize must surface an error if the initial config is infeasible.
func TestInfeasibleInitialErrors(t *testing.T) {
	// Bad constraint: self-reference.
	root := &engine.Node{ID: "root", Children: []*engine.Node{
		{ID: "A", MinW: 10, MinH: 10,
			Align: []engine.AlignConstraint{{To: "A", Edge: engine.EdgeLeft}}},
	}}
	_, err := Optimize(Problem{Root: root}, DefaultConfig())
	if err == nil {
		t.Fatal("expected error on infeasible initial")
	}
}

// Parallel seeds: run the full optimizer and verify it actually spawns
// multiple trajectories (different seeds map to different Result.Seed).
func TestParallelSeedsRun(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 5
	cfg.MaxIter = 30

	res, err := Optimize(prob, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// Result.Seed must be one of the seeds we spawned.
	validSeeds := map[int64]bool{1: true, 1001: true, 2001: true, 3001: true, 4001: true}
	if !validSeeds[res.Seed] {
		t.Errorf("best result has unexpected seed %d", res.Seed)
	}
}

// Swap preserves constraint validity: after any sequence of swaps the
// tree still lays out without error.
func TestSwapPreservesValidity(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 1
	cfg.MaxIter = 200

	res, err := Optimize(prob, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// Re-running Layout on the best tree must not error.
	if _, err := engine.Layout(res.Tree); err != nil {
		t.Errorf("optimized tree is invalid: %v", err)
	}
}
