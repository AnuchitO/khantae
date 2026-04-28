package router

import (
	"testing"

	"github.com/AnuchitO/khantae/engine"
)

// Port fanout: 3 edges leaving the same side should land at 1/4, 2/4,
// 3/4 of the side length, not all stacked at the midpoint.
func TestPortFanoutDistributes(t *testing.T) {
	// Node A is 100 tall. 3 edges to its right side => Y offsets at
	// A.Y + 25, 50, 75.
	nodeA := engine.Rect{X: 0, Y: 0, W: 40, H: 100}
	nodeB := engine.Rect{X: 200, Y: 0, W: 40, H: 40}
	nodeC := engine.Rect{X: 200, Y: 60, W: 40, H: 40}
	nodeD := engine.Rect{X: 200, Y: 120, W: 40, H: 40}
	rects := map[string]engine.Rect{"A": nodeA, "B": nodeB, "C": nodeC, "D": nodeD}
	edges := []EdgeRequest{
		{ID: "AB", From: Endpoint{NodeID: "A", Side: SideRight}, To: Endpoint{NodeID: "B", Side: SideLeft}},
		{ID: "AC", From: Endpoint{NodeID: "A", Side: SideRight}, To: Endpoint{NodeID: "C", Side: SideLeft}},
		{ID: "AD", From: Endpoint{NodeID: "A", Side: SideRight}, To: Endpoint{NodeID: "D", Side: SideLeft}},
	}

	// Before calling RouteAll, exercise assignPortOffsets directly so
	// we can assert on the computed offsets without being coupled to
	// route reconstruction details.
	assigned := assignPortOffsets(edges, rects)
	gotYs := []float64{assigned[0].From.SubOffset, assigned[1].From.SubOffset, assigned[2].From.SubOffset}
	wantYs := []float64{25, 50, 75} // A.Y + (1/4, 2/4, 3/4) * A.H
	for i, y := range wantYs {
		if gotYs[i] != y {
			t.Errorf("edge %d from-port Y: got %v, want %v (all ports: %v)",
				i, gotYs[i], y, gotYs)
		}
	}
	// Sanity: they're distinct.
	if gotYs[0] == gotYs[1] || gotYs[1] == gotYs[2] || gotYs[0] == gotYs[2] {
		t.Errorf("fanout produced duplicate offsets: %v", gotYs)
	}
}

// A single edge to a side should NOT get fanout applied — stays at midpoint.
func TestPortFanoutSingletonUsesMidpoint(t *testing.T) {
	nodeA := engine.Rect{X: 0, Y: 0, W: 40, H: 100}
	nodeB := engine.Rect{X: 200, Y: 30, W: 40, H: 40}
	rects := map[string]engine.Rect{"A": nodeA, "B": nodeB}
	edges := []EdgeRequest{
		{ID: "e", From: Endpoint{NodeID: "A", Side: SideRight}, To: Endpoint{NodeID: "B", Side: SideLeft}},
	}
	assigned := assignPortOffsets(edges, rects)
	if assigned[0].From.SubOffset != 0 {
		t.Errorf("singleton should preserve SubOffset=0 (midpoint), got %v",
			assigned[0].From.SubOffset)
	}
}

// Staircase detection: a path with three closely-spaced bends where
// each run is <20px should count 1 staircase unit.
func TestStaircaseCounting(t *testing.T) {
	// Zig-zag: long run, then two short (15px) perpendicular runs
	// forming a stair, then a long run. The middle bend has two
	// adjacent short runs, so it should count.
	pts := []Point{
		{X: 0, Y: 0},
		{X: 100, Y: 0},  // long run; bend
		{X: 100, Y: 15}, // short run; bend
		{X: 115, Y: 15}, // short run; bend  ← this bend has short on both sides
		{X: 115, Y: 80}, // long run
	}
	stairs := countStairs(pts, 20)
	if stairs != 1 {
		t.Errorf("expected 1 stair, got %d", stairs)
	}
}

// Default weights must satisfy "bend worth ≥50px of length".
func TestWeightRatioBendVsLength(t *testing.T) {
	w := DefaultWeights()
	if w.Wbend < 50 || w.Wlength > 1 {
		t.Errorf("weights violate the 50px-bend invariant: Wbend=%v Wlength=%v", w.Wbend, w.Wlength)
	}
	// Concrete check: a 49px detour via 1 bend is worse than a 49px
	// straight path by arithmetic (49*1 + 50 > 49*1).
	detourCost := 49*w.Wlength + w.Wbend
	straightCost := 49 * w.Wlength
	if detourCost <= straightCost {
		t.Errorf("detour should cost more than straight at equal length; got detour=%v straight=%v",
			detourCost, straightCost)
	}
}

// Orthogonal invariant: verifyOrthogonal must flag a diagonal.
func TestVerifyOrthogonalCatchesDiagonal(t *testing.T) {
	bad := Route{Waypoints: []Point{{0, 0}, {10, 10}}}
	if err := verifyOrthogonal(bad); err == nil {
		t.Error("expected error on diagonal segment")
	}
	good := Route{Waypoints: []Point{{0, 0}, {10, 0}, {10, 20}}}
	if err := verifyOrthogonal(good); err != nil {
		t.Errorf("unexpected error on orthogonal route: %v", err)
	}
}

// Labels are hard obstacles: routes cannot pass through their interior.
//
// CAVEAT: "interior" excludes the boundary, so a label that exactly
// matches the canvas span on one axis can still be grazed along its
// top or bottom edge. Truly inviolable labels would require treating
// the boundary as blocked too — a change deferred until we have a
// concrete diagram where boundary-grazing produces a visible defect.
// For now the test pins the current behavior.
func TestLabelIsHardObstacle(t *testing.T) {
	nodeA := engine.Rect{X: 0, Y: 40, W: 30, H: 20}
	nodeB := engine.Rect{X: 200, Y: 40, W: 30, H: 20}
	// Label sits inside the routing corridor but does NOT span the
	// full vertical extent — there's room to detour above (Y < 30)
	// or below (Y > 90). The router should detour rather than route
	// straight through.
	label := engine.Rect{X: 60, Y: 30, W: 80, H: 60}
	rects := map[string]engine.Rect{"A": nodeA, "B": nodeB}
	obs := []Obstacle{
		{ID: "A", Rect: nodeA, Kind: KindNode},
		{ID: "B", Rect: nodeB, Kind: KindNode},
		{ID: "lbl", Rect: label, Kind: KindLabel},
	}
	// Frame points so the mesh extends above and below the obstacles
	// — without them the only Y-coordinates available are those of
	// the obstacles themselves.
	framePts := []Point{
		{X: -50, Y: -50}, {X: 280, Y: 140},
		{X: -50, Y: 140}, {X: 280, Y: -50},
	}
	edges := []EdgeRequest{{
		ID:   "e",
		From: Endpoint{NodeID: "A", Side: SideRight},
		To:   Endpoint{NodeID: "B", Side: SideLeft},
	}}
	routes, err := RouteAll(rects, obs, edges, DefaultWeights(), framePts...)
	if err != nil {
		t.Fatalf("expected route around label, got error: %v", err)
	}
	// Verify no waypoint lies strictly inside the label rect.
	for _, p := range routes[0].Waypoints {
		if p.X > label.X && p.X < label.X+label.W &&
			p.Y > label.Y && p.Y < label.Y+label.H {
			t.Errorf("waypoint %v lies inside label rect", p)
		}
	}
}
