package optimizer

import (
	"math/rand"

	"github.com/AnuchitO/khantae/engine"
)

// perturb mutates `tree` and `nudges` in place and returns true if
// any change was made. Chooses between two perturbation types:
//
//  1. Swap two unconstrained siblings under some parent (reorders
//     the default stack without violating align/near).
//  2. Nudge a leaf by ±cfg.NudgeMagnitude on either axis (only if
//     cfg.EnableNudge).
//
// Nudges accumulate in the `nudges` map (keyed by leaf ID) and are
// applied after layout solving in evaluate(). They realize the Phase
// 4 "Jitter Pass" — per-node random ±5px offsets that let the search
// escape layouts that are stuck in a local optimum with respect to
// sibling reordering alone.
func perturb(tree *engine.Node, nudges map[string][2]float64,
	rng *rand.Rand, cfg Config) bool {

	// If nudges are enabled and there's at least one leaf, flip a coin
	// to decide between swap and nudge. When only one is available,
	// use the one that works.
	canSwap := len(collectSwappableGroups(tree)) > 0
	canNudge := cfg.EnableNudge && cfg.NudgeMagnitude > 0 &&
		len(collectLeafIDs(tree)) > 0

	switch {
	case canSwap && canNudge:
		// 50/50
		if rng.Intn(2) == 0 {
			return doSwap(tree, rng)
		}
		return doNudge(tree, nudges, rng, cfg.NudgeMagnitude)
	case canSwap:
		return doSwap(tree, rng)
	case canNudge:
		return doNudge(tree, nudges, rng, cfg.NudgeMagnitude)
	default:
		return false
	}
}

func doSwap(tree *engine.Node, rng *rand.Rand) bool {
	candidates := collectSwappableGroups(tree)
	if len(candidates) == 0 {
		return false
	}

	// Pick a parent uniformly at random, then two distinct
	// unconstrained siblings to swap.
	parent := candidates[rng.Intn(len(candidates))]
	freeIdxs := make([]int, 0, len(parent.Children))
	for i, c := range parent.Children {
		if isFree(c) {
			freeIdxs = append(freeIdxs, i)
		}
	}
	if len(freeIdxs) < 2 {
		return false
	}
	i := freeIdxs[rng.Intn(len(freeIdxs))]
	j := freeIdxs[rng.Intn(len(freeIdxs))]
	for j == i {
		j = freeIdxs[rng.Intn(len(freeIdxs))]
	}
	parent.Children[i], parent.Children[j] = parent.Children[j], parent.Children[i]
	return true
}

func doNudge(tree *engine.Node, nudges map[string][2]float64,
	rng *rand.Rand, mag float64) bool {

	leaves := collectLeafIDs(tree)
	if len(leaves) == 0 {
		return false
	}
	id := leaves[rng.Intn(len(leaves))]
	cur := nudges[id] // zero value if absent

	// Pick axis and direction.
	axis := rng.Intn(2)
	sign := 1.0
	if rng.Intn(2) == 0 {
		sign = -1.0
	}
	delta := sign * mag
	if axis == 0 {
		cur[0] += delta
	} else {
		cur[1] += delta
	}
	nudges[id] = cur
	return true
}

// collectSwappableGroups returns every parent that has at least two
// unconstrained children (i.e. a legal swap candidate).
func collectSwappableGroups(n *engine.Node) []*engine.Node {
	var out []*engine.Node
	var walk func(*engine.Node)
	walk = func(p *engine.Node) {
		if p == nil {
			return
		}
		free := 0
		for _, c := range p.Children {
			if isFree(c) {
				free++
				if free >= 2 {
					break
				}
			}
		}
		if free >= 2 {
			out = append(out, p)
		}
		for _, c := range p.Children {
			walk(c)
		}
	}
	walk(n)
	return out
}

func isFree(n *engine.Node) bool {
	return len(n.Align) == 0 && len(n.Near) == 0
}

// collectLeafIDs returns every leaf node's ID in pre-order traversal.
// Used by the optimizer to build the obstacle list for routing —
// interior container nodes are not obstacles because their rects
// fully contain their children and would block every port.
func collectLeafIDs(n *engine.Node) []string {
	var out []string
	var walk func(*engine.Node)
	walk = func(p *engine.Node) {
		if p == nil {
			return
		}
		if len(p.Children) == 0 {
			out = append(out, p.ID)
			return
		}
		for _, c := range p.Children {
			walk(c)
		}
	}
	walk(n)
	return out
}

// deepCopy produces an independent copy of the tree. Constraint slices
// are shallow-copied (AlignConstraint/NearConstraint are value types
// with no pointers, so that's safe).
func deepCopy(n *engine.Node) *engine.Node {
	if n == nil {
		return nil
	}
	cp := &engine.Node{
		ID:      n.ID,
		MinW:    n.MinW,
		MinH:    n.MinH,
		Padding: n.Padding,
	}
	if len(n.Align) > 0 {
		cp.Align = append([]engine.AlignConstraint(nil), n.Align...)
	}
	if len(n.Near) > 0 {
		cp.Near = append([]engine.NearConstraint(nil), n.Near...)
	}
	if len(n.Children) > 0 {
		cp.Children = make([]*engine.Node, len(n.Children))
		for i, c := range n.Children {
			cp.Children[i] = deepCopy(c)
		}
	}
	return cp
}
