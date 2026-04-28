package engine

import "fmt"

// Solver design notes.
//
// A full Cassowary incremental simplex solver is the general tool for
// mixing equalities, inequalities, and strength-weighted preferences.
// For the constraint vocabulary exposed by this package — `align` and
// `near` between siblings, plus a default vertical stack for
// unconstrained nodes — every constraint reduces to a *linear
// equality* of the form
//
//	x_i = x_j + k        or        y_i = y_j + k
//
// where k is a known constant derived from already-computed sizes,
// gaps, and padding. A system of such equalities over an acyclic
// dependency graph can be solved in linear time by topological
// propagation from fixed anchors — this is the specialization of
// Cassowary/LP to the equality-only case. The result is identical to
// what a general solver would produce for these inputs. Cycles (an
// unsatisfiable system under equalities) are detected and returned as
// errors rather than silently producing garbage.
//
// If future constraint types are added (inequalities, preferred-stays,
// soft-minimum gaps), this solver can be swapped for a general
// Cassowary implementation without changing the public API.

// relEdge is a linear-equality edge in the constraint DAG: mine = from + k.
type relEdge struct {
	from string
	k    float64
}

// solveLocal lays out a group of sibling nodes in a local frame
// anchored at (0,0) and returns each sibling's Rect in that frame.
// The caller adds parent padding / global offsets when composing up.
func solveLocal(siblings []*Node, szs sizes) (map[string]Rect, error) {
	idx := make(map[string]*Node, len(siblings))
	for _, s := range siblings {
		idx[s.ID] = s
	}

	xs, err := solveAxis(siblings, idx, szs, axisX)
	if err != nil {
		return nil, err
	}
	ys, err := solveAxis(siblings, idx, szs, axisY)
	if err != nil {
		return nil, err
	}

	out := make(map[string]Rect, len(siblings))
	for _, s := range siblings {
		sz := szs[s.ID]
		out[s.ID] = Rect{X: xs[s.ID], Y: ys[s.ID], W: sz.W, H: sz.H}
	}
	return out, nil
}

type axis int

const (
	axisX axis = iota
	axisY
)

// solveAxis resolves one axis for a sibling group using topological
// propagation.
func solveAxis(siblings []*Node, idx map[string]*Node, szs sizes, ax axis) (map[string]float64, error) {
	deps := make(map[string]*relEdge, len(siblings))
	indeg := make(map[string]int, len(siblings))
	for _, s := range siblings {
		indeg[s.ID] = 0
	}

	// Build one equality edge per node per axis from its constraints.
	// Precedence: near beats align on the same axis (near is the more
	// specific positional statement).
	for _, s := range siblings {
		var e *relEdge
		for _, n := range s.Near {
			if e = nearEdge(n, ax, szs, s.ID); e != nil {
				break
			}
		}
		if e == nil {
			for _, a := range s.Align {
				if e = alignEdge(a, ax, szs, s.ID); e != nil {
					break
				}
			}
		}
		if e == nil {
			continue
		}
		if _, ok := idx[e.from]; !ok {
			return nil, fmt.Errorf("node %q references unknown sibling %q", s.ID, e.from)
		}
		if e.from == s.ID {
			return nil, fmt.Errorf("node %q cannot constrain itself", s.ID)
		}
		deps[s.ID] = e
		indeg[s.ID]++
	}

	// Reverse adjacency: from -> [dependents].
	revAdj := make(map[string][]string, len(siblings))
	for id, e := range deps {
		revAdj[e.from] = append(revAdj[e.from], id)
	}

	// Seed the Kahn queue with roots (indeg 0), preserving input order
	// for determinism across independent roots.
	queue := make([]string, 0, len(siblings))
	for _, s := range siblings {
		if indeg[s.ID] == 0 {
			queue = append(queue, s.ID)
		}
	}

	coords := make(map[string]float64, len(siblings))

	// Default placement for roots. Stacking is vertical: roots cascade
	// down the Y axis; on X, roots start at 0.
	if ax == axisY {
		var cursor float64
		for _, id := range queue {
			coords[id] = cursor
			cursor += szs[id].H
		}
	} else {
		for _, id := range queue {
			coords[id] = 0
		}
	}

	// Kahn propagation.
	processed := 0
	for head := 0; head < len(queue); head++ {
		id := queue[head]
		processed++
		for _, dep := range revAdj[id] {
			indeg[dep]--
			if indeg[dep] == 0 {
				e := deps[dep]
				coords[dep] = coords[e.from] + e.k
				queue = append(queue, dep)
			}
		}
	}
	if processed != len(siblings) {
		axisName := "x"
		if ax == axisY {
			axisName = "y"
		}
		return nil, fmt.Errorf("constraint cycle detected on %s axis", axisName)
	}
	return coords, nil
}

// nearEdge converts a near constraint to an axis edge, or nil if it
// does not act on the requested axis.
//
//	near{to:B, side:right,  gap:g} (x): self.x = B.x + B.W + g
//	near{to:B, side:left,   gap:g} (x): self.x = B.x - g - self.W
//	near{to:B, side:bottom, gap:g} (y): self.y = B.y + B.H + g
//	near{to:B, side:top,    gap:g} (y): self.y = B.y - g - self.H
func nearEdge(n NearConstraint, ax axis, szs sizes, selfID string) *relEdge {
	target, ok := szs[n.To]
	if !ok {
		return nil
	}
	self := szs[selfID]
	switch n.Side {
	case SideRight:
		if ax == axisX {
			return &relEdge{from: n.To, k: target.W + n.Gap}
		}
	case SideLeft:
		if ax == axisX {
			return &relEdge{from: n.To, k: -n.Gap - self.W}
		}
	case SideBottom:
		if ax == axisY {
			return &relEdge{from: n.To, k: target.H + n.Gap}
		}
	case SideTop:
		if ax == axisY {
			return &relEdge{from: n.To, k: -n.Gap - self.H}
		}
	}
	return nil
}

// alignEdge converts an align constraint to an axis edge.
//
//	align{to:B, edge:left}     (x): self.x = B.x
//	align{to:B, edge:right}    (x): self.x = B.x + B.W - self.W
//	align{to:B, edge:center_x} (x): self.x = B.x + (B.W - self.W)/2
//	align{to:B, edge:top}      (y): self.y = B.y
//	align{to:B, edge:bottom}   (y): self.y = B.y + B.H - self.H
//	align{to:B, edge:center_y} (y): self.y = B.y + (B.H - self.H)/2
func alignEdge(a AlignConstraint, ax axis, szs sizes, selfID string) *relEdge {
	target, ok := szs[a.To]
	if !ok {
		return nil
	}
	self := szs[selfID]
	switch a.Edge {
	case EdgeLeft:
		if ax == axisX {
			return &relEdge{from: a.To, k: 0}
		}
	case EdgeRight:
		if ax == axisX {
			return &relEdge{from: a.To, k: target.W - self.W}
		}
	case EdgeCenterX:
		if ax == axisX {
			return &relEdge{from: a.To, k: (target.W - self.W) / 2}
		}
	case EdgeTop:
		if ax == axisY {
			return &relEdge{from: a.To, k: 0}
		}
	case EdgeBottom:
		if ax == axisY {
			return &relEdge{from: a.To, k: target.H - self.H}
		}
	case EdgeCenterY:
		if ax == axisY {
			return &relEdge{from: a.To, k: (target.H - self.H) / 2}
		}
	}
	return nil
}
