package router

import (
	"github.com/AnuchitO/khantae/engine"
)

// assignPortOffsets implements the Kandinsky port-fanout rule.
//
// For each (nodeID, side) pair, we look at all endpoints that did NOT
// specify a SubOffset. If there are N such endpoints, we distribute
// them across N+1 evenly-spaced slots along the side. For left/right,
// the side length is the node's height; for top/bottom, it's width.
//
// Example: 3 edges attaching to A's right side, node A 40px tall.
// Slots 1..3 of 4 segments => Y offsets at 1/4, 2/4, 3/4 of the
// height, so ports at A.y + 10, +20, +30. (For a node at Y=100,
// that's Y=110, 120, 130.)
//
// Endpoints with a caller-supplied SubOffset are left untouched.
// Assignment order within a fanout group follows the order edges
// appear in the input slice — this is deterministic and keeps edges
// that appear early "closer to the top" of the side.
func assignPortOffsets(edges []EdgeRequest, rects map[string]engine.Rect) []EdgeRequest {
	// Key: "nodeID@side" (only for endpoints without SubOffset).
	type portKey struct {
		node string
		side Side
	}
	// Count how many auto-slot endpoints target each (node, side).
	counts := map[portKey]int{}
	// Collect per-key endpoint references so we can rewrite them.
	// Each entry is (edgeIndex, isFrom): which endpoint of which edge.
	type ref struct {
		edgeIdx int
		isFrom  bool
	}
	groups := map[portKey][]ref{}

	for i, e := range edges {
		if e.From.SubOffset == 0 {
			k := portKey{e.From.NodeID, e.From.Side}
			counts[k]++
			groups[k] = append(groups[k], ref{i, true})
		}
		if e.To.SubOffset == 0 {
			k := portKey{e.To.NodeID, e.To.Side}
			counts[k]++
			groups[k] = append(groups[k], ref{i, false})
		}
	}

	// Copy edges so we can mutate endpoints without touching caller's input.
	out := make([]EdgeRequest, len(edges))
	copy(out, edges)

	for k, refs := range groups {
		if len(refs) <= 1 {
			// Single endpoint: keep midpoint behaviour (SubOffset=0).
			// This preserves the existing single-edge tests unchanged.
			continue
		}
		r, ok := rects[k.node]
		if !ok {
			continue // will fail later with a clearer error
		}
		n := len(refs)
		for i, rf := range refs {
			slot := float64(i+1) / float64(n+1) // in (0, 1)
			var sub float64
			switch k.side {
			case SideLeft, SideRight:
				sub = r.Y + slot*r.H
			case SideTop, SideBottom:
				sub = r.X + slot*r.W
			}
			if rf.isFrom {
				out[rf.edgeIdx].From.SubOffset = sub
			} else {
				out[rf.edgeIdx].To.SubOffset = sub
			}
		}
	}
	return out
}
