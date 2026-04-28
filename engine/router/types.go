// Package router computes orthogonal (Manhattan) routes between
// anchor points on the boundaries of rectangular nodes, avoiding node
// obstacles and minimizing a weighted combination of bends, crossings,
// and path length.
//
// The algorithm builds a sparse visibility graph — a "navigation mesh"
// whose vertices are the Cartesian product of the distinct x- and
// y-coordinates of all node edges (plus small port offsets), and whose
// edges connect orthogonal neighbors. Routes are found by A* over
// states (vertex, incoming_direction), which lets us charge a cost
// every time the direction changes (a 90° bend).
//
// Edges are routed sequentially. Each routed edge records the grid
// segments it used; subsequent edges pay a crossing penalty for each
// shared segment. The routing order therefore matters — earlier edges
// get cleaner paths. Callers that want balanced routing can sort
// their edge list before calling (e.g. by descending straight-line
// distance) or loop with different orders.
package router

import (
	"github.com/AnuchitO/khantae/engine"
)

// Weights controls the multi-factor cost function
//
//	C = B*Wbend + X*Wcross + L*Wlength
//
// Defaults match the prompt: 15 / 100 / 1. Crossing cost is dominant
// Weights controls the multi-factor cost function
//
//	C = B*Wbend + X*Wcross + L*Wlength + S*Wstair
//
// The Phase 4 "course correction" defaults are 50 / 200 / 1 / 10.
// Wbend=50 with Wlength=1 implements the invariant "a bend is only
// worth it if it saves ≥50px of path length" — by arithmetic, a
// detour via a bend only wins if Δlength > Wbend.
//
// Crossing cost remains dominant (Wcross=200 vs Wbend=50) so the
// router prefers even ugly detours over crossing another edge.
// Staircasing is charged per detected zig-zag unit; see countStairs.
type Weights struct {
	Wbend   float64 // per 90° turn
	Wcross  float64 // per crossing of a previously-routed edge segment
	Wlength float64 // per unit of Manhattan distance travelled
	Wstair  float64 // per staircase unit — added post-hoc to Route.Cost
}

// DefaultWeights returns the weights specified in the Phase 4 design brief.
func DefaultWeights() Weights {
	return Weights{Wbend: 50, Wcross: 200, Wlength: 1, Wstair: 10}
}

// Side of a node a port attaches to.
type Side int

const (
	SideLeft Side = iota
	SideRight
	SideTop
	SideBottom
)

// Endpoint describes one end of an edge: a node ID and which side of
// the node the edge connects to. The precise port coordinate is
// computed by the router at the midpoint of the requested side unless
// a sub-anchor override is provided via SubOffset.
//
// SubOffset, when non-zero, overrides the *perpendicular-to-side*
// coordinate of the port. For a left/right side, SubOffset is an
// absolute Y (not relative), interpreted as the row's vertical
// midpoint; for a top/bottom side, SubOffset is an absolute X.
// This is the mechanism SQL tables use to connect edges to specific
// rows/columns instead of the table's overall midpoint.
//
// If SubOffset is 0 (default), midpoint behavior is used.
type Endpoint struct {
	NodeID    string
	Side      Side
	SubOffset float64
}

// EdgeRequest is one edge to route.
type EdgeRequest struct {
	ID   string
	From Endpoint
	To   Endpoint
}

// Point is a grid coordinate.
type Point struct {
	X, Y float64
}

// Route is the computed path for one edge, returned as a polyline.
// Waypoints always alternate orientation (a result guaranteed by
// orthogonal routing) and always start and end perpendicular to the
// attached node's side.
type Route struct {
	EdgeID    string
	Waypoints []Point

	// Reporting: raw component counts and total weighted cost, so
	// callers can log / tune / debug.
	Bends     int
	Crossings int
	Stairs    int
	Length    float64
	Cost      float64
}

// Obstacle classifies a rect as a node or label. Both are hard
// obstacles that block routing entirely. Labels are distinguished
// because a caller (e.g. the layout engine) may want to respond to
// label-caused routing failures by growing the parent container,
// whereas a node-caused failure is a true topology problem.
type ObstacleKind int

const (
	KindNode ObstacleKind = iota
	// KindLabel — text labels are hard obstacles. The Phase 4 design
	// requires that lines never cross labels; instead, the layout
	// engine should grow containers so routes can pass around them.
	// The grow-on-failure feedback loop is the caller's responsibility;
	// the router simply reports "no route found" if a label is the
	// only path. See (*TODO*) in the layout package for the
	// iterate-until-feasible outer loop — currently unimplemented.
	KindLabel
)

// Obstacle is an axis-aligned rectangle with a kind.
type Obstacle struct {
	ID   string
	Rect engine.Rect
	Kind ObstacleKind
}

// LabelSoftCost is retained for backward compatibility with any
// caller that might refer to it, but is no longer applied — labels
// are hard obstacles in the Phase 4 design. Kept at its former
// value so callers linking against it still compile.
const LabelSoftCost = 50
