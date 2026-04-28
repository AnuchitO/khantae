package router

import (
	"sort"

	"github.com/AnuchitO/khantae/engine"
)

// portOffset is the distance outside a node's edge at which we place
// a port vertex. Using a small offset ensures the port is strictly
// outside the node rect (which is a hard obstacle) while still being
// visually attached. It must be small relative to typical node sizes
// but not zero, otherwise the port sits on the obstacle boundary and
// gets filtered out.
const portOffset = 0.5

// mesh is the routing graph. Vertices are indexed by (ix, iy) into the
// sorted xs/ys arrays, so we can address them compactly.
type mesh struct {
	xs []float64 // sorted distinct x-coordinates
	ys []float64 // sorted distinct y-coordinates

	// blocked[iy*nx+ix] is true if vertex (ix,iy) lies strictly inside
	// a hard (node) obstacle. Blocked vertices are not traversable.
	blocked []bool

	// softAt[iy*nx+ix] is the soft obstacle cost associated with the
	// vertex (0 if none). An edge between two vertices picks up the
	// average of the two vertices' soft costs times edge length.
	softAt []float64

	// For convenient coordinate lookups.
	xIndex map[float64]int
	yIndex map[float64]int

	// hardObs is the list of hard-obstacle rects. Used to reject grid
	// edges that pass through an obstacle's interior even though
	// neither endpoint is strictly inside it (possible when the
	// obstacle has no mesh vertex within its span on one axis).
	hardObs []engine.Rect

	// hBlocked[iy*nx + ix] indicates the horizontal segment from
	// (ix,iy) to (ix+1,iy) passes through a hard obstacle's interior.
	// vBlocked is analogous for vertical segments (ix,iy) -> (ix,iy+1).
	// Precomputed once in buildMesh so A* expansion avoids a per-step
	// scan of hardObs — this is the dominant hotspot in profiles of
	// dense graphs.
	hBlocked []bool
	vBlocked []bool
}

func (m *mesh) idx(ix, iy int) int { return iy*len(m.xs) + ix }

// buildMesh constructs the routing graph for a set of obstacles and
// required port locations.
//
// The x-coordinate set is {left, right, left-offset, right-offset}
// for every obstacle's node, plus every port x. The y set is analogous.
// After dedup+sort, the Cartesian product gives the vertex grid.
func buildMesh(obstacles []Obstacle, portPoints []Point) *mesh {
	xSet := map[float64]struct{}{}
	ySet := map[float64]struct{}{}

	add := func(x, y float64) {
		xSet[x] = struct{}{}
		ySet[y] = struct{}{}
	}

	for _, o := range obstacles {
		r := o.Rect
		add(r.X, r.Y)
		add(r.X+r.W, r.Y+r.H)
		// Offset lanes outside node edges — these give the router
		// clear channels immediately adjacent to obstacles.
		if o.Kind == KindNode {
			add(r.X-portOffset, r.Y-portOffset)
			add(r.X+r.W+portOffset, r.Y+r.H+portOffset)
		}
	}
	for _, p := range portPoints {
		add(p.X, p.Y)
	}

	m := &mesh{
		xs:      sortedKeys(xSet),
		ys:      sortedKeys(ySet),
		xIndex:  map[float64]int{},
		yIndex:  map[float64]int{},
	}
	for i, x := range m.xs {
		m.xIndex[x] = i
	}
	for i, y := range m.ys {
		m.yIndex[y] = i
	}

	nv := len(m.xs) * len(m.ys)
	m.blocked = make([]bool, nv)
	m.softAt = make([]float64, nv)

	// Both nodes and labels are hard obstacles: their rects contribute
	// to hardObs (used for grid-segment blocking) and interior grid
	// vertices are marked blocked. The only difference is semantic —
	// labels can be identified by kind if a caller wants to grow
	// containers in response to label-induced routing failures.
	for _, o := range obstacles {
		m.hardObs = append(m.hardObs, o.Rect)
		r := o.Rect
		for iy, y := range m.ys {
			if y <= r.Y || y >= r.Y+r.H {
				continue
			}
			for ix, x := range m.xs {
				if x <= r.X || x >= r.X+r.W {
					continue
				}
				m.blocked[m.idx(ix, iy)] = true
			}
		}
	}

	// Precompute per-edge blocked flags so A* expansion avoids a
	// scan over hardObs per step. For hBlocked, each entry at
	// (iy*nx + ix) covers the segment (ix,iy)->(ix+1,iy).
	nx := len(m.xs)
	ny := len(m.ys)
	m.hBlocked = make([]bool, nx*ny)
	m.vBlocked = make([]bool, nx*ny)
	for _, r := range m.hardObs {
		// Horizontal segments: at each row y, check x ranges that
		// overlap the obstacle's x-span strictly.
		for iy, y := range m.ys {
			if y <= r.Y || y >= r.Y+r.H {
				continue
			}
			for ix := 0; ix < nx-1; ix++ {
				x1, x2 := m.xs[ix], m.xs[ix+1]
				lo, hi := x1, x2
				if lo > hi {
					lo, hi = hi, lo
				}
				if hi <= r.X || lo >= r.X+r.W {
					continue
				}
				m.hBlocked[iy*nx+ix] = true
			}
		}
		// Vertical segments: at each column x, check y ranges.
		for ix, x := range m.xs {
			if x <= r.X || x >= r.X+r.W {
				continue
			}
			for iy := 0; iy < ny-1; iy++ {
				y1, y2 := m.ys[iy], m.ys[iy+1]
				lo, hi := y1, y2
				if lo > hi {
					lo, hi = hi, lo
				}
				if hi <= r.Y || lo >= r.Y+r.H {
					continue
				}
				m.vBlocked[iy*nx+ix] = true
			}
		}
	}

	return m
}

// segmentBlocked reports whether the grid edge from (ax,ay) to (bx,by)
// (assumed horizontal or vertical and spanning exactly one step on
// the grid) passes through the interior of any hard obstacle.
// O(1) — uses the hBlocked/vBlocked tables precomputed in buildMesh.
func (m *mesh) segmentBlocked(ax, ay, bx, by int) bool {
	nx := len(m.xs)
	if ax == bx {
		// vertical: normalize so (ax,ay)->(ax,ay+1)
		lo := ay
		if by < lo {
			lo = by
		}
		return m.vBlocked[lo*nx+ax]
	}
	// horizontal
	lo := ax
	if bx < lo {
		lo = bx
	}
	return m.hBlocked[ay*nx+lo]
}

func sortedKeys(s map[float64]struct{}) []float64 {
	out := make([]float64, 0, len(s))
	for k := range s {
		out = append(out, k)
	}
	sort.Float64s(out)
	return out
}

// portPoint returns the coordinate of a port on a given side of a node.
// Ports sit at the midpoint of the side by default, offset outward by
// portOffset so they lie strictly outside the (blocked) interior.
//
// When subOffset is non-zero, it overrides the coordinate along the
// side (Y for left/right; X for top/bottom) — used by SQL tables to
// attach edges to a specific row rather than the overall midpoint,
// and by the port-fanout pass to distribute multiple edges along a
// side instead of stacking them all on the midpoint.
func portPoint(r engine.Rect, side Side, subOffset float64) Point {
	cx := r.X + r.W/2
	cy := r.Y + r.H/2
	switch side {
	case SideLeft:
		y := cy
		if subOffset != 0 {
			y = subOffset
		}
		return Point{X: r.X - portOffset, Y: y}
	case SideRight:
		y := cy
		if subOffset != 0 {
			y = subOffset
		}
		return Point{X: r.X + r.W + portOffset, Y: y}
	case SideTop:
		x := cx
		if subOffset != 0 {
			x = subOffset
		}
		return Point{X: x, Y: r.Y - portOffset}
	case SideBottom:
		x := cx
		if subOffset != 0 {
			x = subOffset
		}
		return Point{X: x, Y: r.Y + r.H + portOffset}
	}
	return Point{X: cx, Y: cy}
}

// initialDirection gives the "outward" direction from a port on the
// given side — this is the direction A* must take on its first move
// so the edge leaves the node perpendicularly.
func initialDirection(side Side) direction {
	switch side {
	case SideLeft:
		return dirLeft
	case SideRight:
		return dirRight
	case SideTop:
		return dirUp
	case SideBottom:
		return dirDown
	}
	return dirRight
}
