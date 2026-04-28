package router

import (
	"fmt"

	"github.com/AnuchitO/khantae/engine"
)

// RouteAll routes a batch of edges against the given obstacles using
// the weighted cost function. Edges are routed in the order given;
// each edge's route occupies grid segments that incur crossing cost
// for later edges.
//
// nodeRects must contain every NodeID referenced by an EdgeRequest.
// framePoints are extra coordinates injected into the mesh but not
// treated as obstacles or routable anchors — useful for ensuring the
// mesh extends past the tight bounding box of the obstacles when
// edges need room to maneuver (e.g. same-side ↔ same-side routes).
// Returned routes are in the same order as the input requests.
func RouteAll(
	nodeRects map[string]engine.Rect,
	obstacles []Obstacle,
	edges []EdgeRequest,
	weights Weights,
	framePoints ...Point,
) ([]Route, error) {

	// --- Port fanout (Kandinsky enforcement) ---
	//
	// When multiple edges share the same (node, side) without a
	// caller-supplied SubOffset, they should NOT stack at the midpoint.
	// Instead, distribute them across N+1 evenly-spaced segments along
	// the side. This is the "Kandinsky model" from the Phase 4 brief.
	//
	// Callers that supplied their own SubOffset (e.g. SQL column
	// routing) are left untouched — they explicitly know which row to
	// attach to.
	//
	// We mutate a local copy of edges so the caller's slice is untouched.
	edges = assignPortOffsets(edges, nodeRects)

	// Port fanout moves SubOffsets to non-zero values; any endpoint
	// still at SubOffset=0 is singleton and uses midpoint.

	// Collect all port points so the mesh includes their coordinates.
	// Also add "stub guidance points" minStubLength away from each
	// port in the outward direction. These contribute mesh vertices
	// at the preferred bend location; A*'s bend cost then makes those
	// the natural place to turn. Without them, the nearest bend
	// vertex could be at an arbitrary distance and we can't control
	// where the first bend happens — violating the Kandinsky rule
	// "parallel for at least 20px before first bend". With them, the
	// rule holds whenever the mesh geometry doesn't force otherwise
	// (e.g. an obstacle at 15px out).
	portPts := make([]Point, 0, 2*len(edges))
	stubPts := make([]Point, 0, 2*len(edges))
	type epInfo struct {
		pt  Point
		dir direction // outward direction from the port
	}
	endpoints := make(map[string]epInfo, 2*len(edges))
	key := func(ep Endpoint) string {
		// SubOffset is part of the key because two endpoints on the
		// same (node, side) but different offsets (SQL columns, or
		// fanout slots) are distinct ports.
		return fmt.Sprintf("%s@%d#%g", ep.NodeID, ep.Side, ep.SubOffset)
	}
	for _, e := range edges {
		for _, ep := range []Endpoint{e.From, e.To} {
			k := key(ep)
			if _, ok := endpoints[k]; ok {
				continue
			}
			r, ok := nodeRects[ep.NodeID]
			if !ok {
				return nil, fmt.Errorf("edge %q references unknown node %q", e.ID, ep.NodeID)
			}
			pt := portPoint(r, ep.Side, ep.SubOffset)
			endpoints[k] = epInfo{pt: pt, dir: initialDirection(ep.Side)}
			portPts = append(portPts, pt)

			// Stub guidance point: minStubLength outward along the
			// port's direction. Only included if it doesn't land
			// inside the same rect (wouldn't, for a rect with
			// portOffset < minStubLength away from the edge).
			stub := stubPoint(pt, ep.Side, minStubLength)
			stubPts = append(stubPts, stub)
		}
	}

	// Concatenate port points, stub guidance points, and frame points
	// — all three only contribute mesh coordinates; they don't create
	// obstacles.
	meshPoints := append([]Point(nil), portPts...)
	meshPoints = append(meshPoints, stubPts...)
	meshPoints = append(meshPoints, framePoints...)
	m := buildMesh(obstacles, meshPoints)

	// A vertex lookup for the exact port coordinates. Every port
	// coordinate should be present by construction of the mesh.
	vertexAt := func(p Point) (int, int, error) {
		ix, xok := m.xIndex[p.X]
		iy, yok := m.yIndex[p.Y]
		if !xok || !yok {
			return 0, 0, fmt.Errorf("port %v missing from mesh", p)
		}
		return ix, iy, nil
	}

	usedSegments := map[segment]int{} // segment -> count of prior uses
	routes := make([]Route, 0, len(edges))

	for _, e := range edges {
		fromInfo := endpoints[key(e.From)]
		toInfo := endpoints[key(e.To)]

		sx, sy, err := vertexAt(fromInfo.pt)
		if err != nil {
			return nil, fmt.Errorf("edge %q: %w", e.ID, err)
		}
		tx, ty, err := vertexAt(toInfo.pt)
		if err != nil {
			return nil, fmt.Errorf("edge %q: %w", e.ID, err)
		}

		// Start direction: first step must leave the source node
		// perpendicularly, i.e. in the outward direction of the port.
		startDir := fromInfo.dir
		// Target approach direction: final step must enter the target
		// node perpendicularly, i.e. OPPOSITE of the target's outward.
		targetApproach := toInfo.dir.opposite()

		end, err := findRoute(m, sx, sy, startDir, tx, ty, targetApproach, weights, usedSegments)
		if err != nil {
			return nil, fmt.Errorf("edge %q: %w", e.ID, err)
		}

		rt := reconstruct(m, end, e.ID, weights, usedSegments)
		if err := verifyOrthogonal(rt); err != nil {
			return nil, fmt.Errorf("edge %q: %w", e.ID, err)
		}
		// Record segments used by this route so future edges pay
		// crossing cost over them.
		for cur := end; cur != nil && cur.hasSeg; cur = cur.parent {
			usedSegments[cur.seg]++
		}
		routes = append(routes, rt)
	}

	return routes, nil
}

// reconstruct walks the parent chain to build the polyline and tally
// the component costs. Consecutive waypoints on the same line are
// collapsed so the output is a minimal polyline (bend points only).
func reconstruct(m *mesh, end *searchNode, edgeID string, w Weights, used map[segment]int) Route {
	var pts []Point
	for cur := end; cur != nil; cur = cur.parent {
		pts = append(pts, Point{X: m.xs[cur.ix], Y: m.ys[cur.iy]})
	}
	// Reverse.
	for i, j := 0, len(pts)-1; i < j; i, j = i+1, j-1 {
		pts[i], pts[j] = pts[j], pts[i]
	}

	// Collapse collinear runs.
	collapsed := make([]Point, 0, len(pts))
	for i, p := range pts {
		if i > 0 && i < len(pts)-1 {
			a := collapsed[len(collapsed)-1]
			c := pts[i+1]
			if (a.X == p.X && p.X == c.X) || (a.Y == p.Y && p.Y == c.Y) {
				continue // p is on the line between a and c
			}
		}
		collapsed = append(collapsed, p)
	}

	// Count bends (interior vertices where direction changes) and
	// length (sum of segment lengths). Crossings counted from the
	// used-segments map before this edge was added.
	bends := 0
	var length float64
	for i := 1; i < len(collapsed); i++ {
		length += abs(collapsed[i].X-collapsed[i-1].X) + abs(collapsed[i].Y-collapsed[i-1].Y)
		if i >= 2 {
			bends++
		}
	}
	crossings := 0
	for cur := end; cur != nil && cur.hasSeg; cur = cur.parent {
		if _, ok := used[cur.seg]; ok {
			crossings++
		}
	}

	stairs := countStairs(collapsed, shortRunThreshold)

	cost := float64(bends)*w.Wbend +
		float64(crossings)*w.Wcross +
		length*w.Wlength +
		float64(stairs)*w.Wstair
	return Route{
		EdgeID:    edgeID,
		Waypoints: collapsed,
		Bends:     bends,
		Crossings: crossings,
		Stairs:    stairs,
		Length:    length,
		Cost:      cost,
	}
}

// shortRunThreshold is the length below which a straight run between
// two bends is considered "staircase-short." Chosen to match the
// Kandinsky minimum-straight-stub rule (also 20px) so the two
// constraints reinforce each other: lines that bend too quickly get
// penalized AND get their stub extended.
const shortRunThreshold = 20.0

// countStairs counts "staircase units" in a polyline. A stair is a
// bend whose two adjacent straight runs are both below the threshold.
// We count one stair per qualifying interior bend; a long zig-zag of
// N small runs yields roughly N-2 stairs.
//
// First and last runs are NOT penalized for being short — those are
// the port stubs, which we want short & perpendicular by design.
func countStairs(pts []Point, threshold float64) int {
	if len(pts) < 4 {
		return 0
	}
	stairs := 0
	for i := 2; i < len(pts)-1; i++ {
		// The bend at pts[i] connects run (pts[i-1]->pts[i]) and
		// (pts[i]->pts[i+1]).
		runA := abs(pts[i].X-pts[i-1].X) + abs(pts[i].Y-pts[i-1].Y)
		runB := abs(pts[i+1].X-pts[i].X) + abs(pts[i+1].Y-pts[i].Y)
		if runA < threshold && runB < threshold {
			stairs++
		}
	}
	return stairs
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// minStubLength is the Kandinsky "parallel before first bend" distance.
// The router plants mesh vertices this far from each port along the
// outward direction so A*'s bend cost makes them the natural first
// bend location. A hard enforcement would require threading stub
// distance through the search state; this cheap variant instead just
// makes the preferred bend location discoverable.
const minStubLength = 20.0

// verifyOrthogonal returns an error if any segment in rt has a slope
// that isn't 0 or ∞ (i.e., not horizontal and not vertical). This is
// an invariant of the router: the final check that TALA's "strictly
// orthogonal" rule holds. A failure here indicates a bug in the
// neighbor-expansion code, not a user-supplied input problem.
func verifyOrthogonal(rt Route) error {
	for i := 1; i < len(rt.Waypoints); i++ {
		a, b := rt.Waypoints[i-1], rt.Waypoints[i]
		if a.X != b.X && a.Y != b.Y {
			return fmt.Errorf("orthogonal invariant violated: segment %v -> %v is diagonal", a, b)
		}
	}
	return nil
}

// stubPoint helpers moved below.
func stubPoint(port Point, side Side, d float64) Point {
	switch side {
	case SideLeft:
		return Point{X: port.X - d, Y: port.Y}
	case SideRight:
		return Point{X: port.X + d, Y: port.Y}
	case SideTop:
		return Point{X: port.X, Y: port.Y - d}
	case SideBottom:
		return Point{X: port.X, Y: port.Y + d}
	}
	return port
}
