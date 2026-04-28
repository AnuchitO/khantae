package engine

import "math"

// sizes holds the computed intrinsic width/height for each node ID after
// the bottom-up size pass. Positions are filled in later by the solver.
type sizes map[string]struct{ W, H float64 }

// computeSizes walks the tree post-order and fills in each node's
// intrinsic size.
//
// Rule: a parent's size is max(MinW/MinH, childrenBBox + padding).
//
// The children's bounding box is computed assuming they have already
// been positioned by a *tentative* layout inside the parent's local
// coordinate space. That tentative layout is the same one the solver
// will produce, which creates a chicken-and-egg: we need sizes to
// solve positions, and we need positions to size parents.
//
// We break the cycle by running a tentative local solve during the
// size pass (with padding-relative anchors) to get children's local
// positions; those local positions, combined with the children's
// already-known sizes, give us the tight bounding box.
func computeSizes(n *Node, out sizes) error {
	if n == nil {
		return nil
	}

	// Post-order: size children first.
	for _, c := range n.Children {
		if err := computeSizes(c, out); err != nil {
			return err
		}
	}

	if len(n.Children) == 0 {
		out[n.ID] = struct{ W, H float64 }{W: n.MinW, H: n.MinH}
		return nil
	}

	// Tentative local layout of children to find their bounding box.
	// We pass a "local" frame anchored at (0,0) with unbounded size;
	// the solver returns child-local Rects, which we then union.
	local, err := solveLocal(n.Children, out)
	if err != nil {
		return err
	}

	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	for _, c := range n.Children {
		r := local[c.ID]
		if r.X < minX {
			minX = r.X
		}
		if r.Y < minY {
			minY = r.Y
		}
		if r.X+r.W > maxX {
			maxX = r.X + r.W
		}
		if r.Y+r.H > maxY {
			maxY = r.Y + r.H
		}
	}

	// The local solve starts children at (0,0), so minX/minY will be 0
	// for the first-placed child. The content width is maxX-minX.
	contentW := maxX - minX
	contentH := maxY - minY

	w := contentW + n.Padding.Left + n.Padding.Right
	h := contentH + n.Padding.Top + n.Padding.Bottom
	if n.MinW > w {
		w = n.MinW
	}
	if n.MinH > h {
		h = n.MinH
	}

	out[n.ID] = struct{ W, H float64 }{W: w, H: h}
	return nil
}
