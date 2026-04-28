// Package engine computes absolute positions and sizes for a tree of
// nested 2D bounding boxes.
//
// The layout runs in two deterministic phases:
//
//  1. Size pass (bottom-up, recursive): every node's width and height is
//     derived from the minimum bounding box of its children plus the
//     node's padding. Leaf nodes use their intrinsic (MinW, MinH) size.
//
//  2. Position pass (constraint solve): positions are resolved from
//     relative constraints (`align`, `near`) between siblings, with
//     unconstrained children stacked vertically inside their parent's
//     padding. See solver.go for the solver design rationale.
//
// The public entry point is Layout(root) which returns a map from node
// ID to the absolute Rect of every node in the tree.
package engine

// Rect is an axis-aligned bounding box in absolute coordinates.
// X,Y is the top-left corner; W,H are width and height. Units are
// caller-defined (pixels, points, etc.).
type Rect struct {
	X, Y, W, H float64
}

// Padding is the internal spacing a parent reserves around its children.
type Padding struct {
	Top, Right, Bottom, Left float64
}

// Edge identifies one side (or center axis) of a box. Used by align.
type Edge string

const (
	EdgeLeft    Edge = "left"
	EdgeRight   Edge = "right"
	EdgeTop     Edge = "top"
	EdgeBottom  Edge = "bottom"
	EdgeCenterX Edge = "center_x"
	EdgeCenterY Edge = "center_y"
)

// Side identifies which side of the reference node this node should sit
// against. Used by near.
type Side string

const (
	SideLeft   Side = "left"
	SideRight  Side = "right"
	SideTop    Side = "top"
	SideBottom Side = "bottom"
)

// AlignConstraint pins an edge of this node to the same edge of another
// sibling. Example: {To: "B", Edge: "left"} means "my left x == B's left x".
type AlignConstraint struct {
	To   string `json:"to"`
	Edge Edge   `json:"edge"`
}

// NearConstraint places this node adjacent to a sibling on the given
// side, separated by Gap. Example: {To: "B", Side: "right", Gap: 10}
// means "my left x == B's right x + 10".
type NearConstraint struct {
	To   string  `json:"to"`
	Side Side    `json:"side"`
	Gap  float64 `json:"gap"`
}

// Node is one entry in the input JSON tree.
//
// A leaf node should set MinW and MinH. An interior node's size is
// computed from its children and may leave MinW/MinH at zero (they act
// as a floor if non-zero).
//
// Align and Near apply to this node's position relative to a *sibling*.
// At most one of each axis' worth of constraints should be supplied;
// conflicts are resolved in solver.go with a documented precedence.
type Node struct {
	ID       string  `json:"id"`
	MinW     float64 `json:"min_w,omitempty"`
	MinH     float64 `json:"min_h,omitempty"`
	Padding  Padding `json:"padding,omitempty"`
	Children []*Node `json:"children,omitempty"`

	Align []AlignConstraint `json:"align,omitempty"`
	Near  []NearConstraint  `json:"near,omitempty"`
}
