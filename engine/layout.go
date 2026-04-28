package engine

import (
	"encoding/json"
	"fmt"
)

// Layout computes the absolute Rect of every node in the tree rooted
// at root. The returned map is keyed by Node.ID.
//
// The root is placed at (0,0). Children are positioned inside their
// parent by the solver, then translated into the parent's global
// frame plus the parent's padding offset.
//
// Returns an error if:
//   - two nodes share the same ID,
//   - a constraint targets a non-sibling or unknown node,
//   - the constraint graph contains a cycle.
func Layout(root *Node) (map[string]Rect, error) {
	if root == nil {
		return map[string]Rect{}, nil
	}
	if err := validateUniqueIDs(root, map[string]struct{}{}); err != nil {
		return nil, err
	}
	allIDs := map[string]struct{}{}
	collectIDs(root, allIDs)
	if err := validateConstraints(root, allIDs); err != nil {
		return nil, err
	}

	szs := sizes{}
	if err := computeSizes(root, szs); err != nil {
		return nil, err
	}

	out := make(map[string]Rect)
	rootSz := szs[root.ID]
	out[root.ID] = Rect{X: 0, Y: 0, W: rootSz.W, H: rootSz.H}

	if err := placeChildren(root, out, szs); err != nil {
		return nil, err
	}
	return out, nil
}

// LayoutJSON is the convenience entry point for callers who have the
// tree as a JSON document.
func LayoutJSON(data []byte) (map[string]Rect, error) {
	var root Node
	if err := json.Unmarshal(data, &root); err != nil {
		return nil, fmt.Errorf("parse tree: %w", err)
	}
	return Layout(&root)
}

// placeChildren recursively composes local sibling layouts into the
// global frame, offset by the parent's global position and padding.
func placeChildren(parent *Node, out map[string]Rect, szs sizes) error {
	if len(parent.Children) == 0 {
		return nil
	}
	local, err := solveLocal(parent.Children, szs)
	if err != nil {
		return fmt.Errorf("layout %q: %w", parent.ID, err)
	}

	// Normalise: shift the sibling group so its bounding box starts
	// at (padding.Left, padding.Top) in the parent's local frame. The
	// size pass already grew the parent to fit, so this just anchors.
	minX, minY := 0.0, 0.0
	first := true
	for _, c := range parent.Children {
		r := local[c.ID]
		if first || r.X < minX {
			minX = r.X
		}
		if first || r.Y < minY {
			minY = r.Y
		}
		first = false
	}

	parentRect := out[parent.ID]
	offX := parentRect.X + parent.Padding.Left - minX
	offY := parentRect.Y + parent.Padding.Top - minY

	for _, c := range parent.Children {
		r := local[c.ID]
		out[c.ID] = Rect{
			X: r.X + offX,
			Y: r.Y + offY,
			W: r.W,
			H: r.H,
		}
		if err := placeChildren(c, out, szs); err != nil {
			return err
		}
	}
	return nil
}

func validateUniqueIDs(n *Node, seen map[string]struct{}) error {
	if n == nil {
		return nil
	}
	if n.ID == "" {
		return fmt.Errorf("node with empty ID")
	}
	if _, dup := seen[n.ID]; dup {
		return fmt.Errorf("duplicate node ID %q", n.ID)
	}
	seen[n.ID] = struct{}{}
	for _, c := range n.Children {
		if err := validateUniqueIDs(c, seen); err != nil {
			return err
		}
	}
	return nil
}

// validateConstraints walks the tree and verifies that every align/near
// constraint targets a *sibling* (a node under the same parent). It
// also rejects over-determined axes (e.g., two align constraints on
// the x axis from the same node).
//
// allIDs is the full set of node IDs in the tree, used to distinguish
// "references an unknown ID" from "references a non-sibling".
func validateConstraints(n *Node, allIDs map[string]struct{}) error {
	if n == nil {
		return nil
	}
	siblingIDs := make(map[string]struct{}, len(n.Children))
	for _, c := range n.Children {
		siblingIDs[c.ID] = struct{}{}
	}
	for _, c := range n.Children {
		if err := checkNodeConstraints(c, siblingIDs, allIDs); err != nil {
			return err
		}
	}
	for _, c := range n.Children {
		if err := validateConstraints(c, allIDs); err != nil {
			return err
		}
	}
	return nil
}

func checkNodeConstraints(c *Node, siblingIDs, allIDs map[string]struct{}) error {
	check := func(targetID, kind string) error {
		if targetID == c.ID {
			return fmt.Errorf("node %q cannot %s to itself", c.ID, kind)
		}
		if _, ok := siblingIDs[targetID]; ok {
			return nil
		}
		if _, ok := allIDs[targetID]; ok {
			return fmt.Errorf("node %q %s references %q, which exists but is not a sibling", c.ID, kind, targetID)
		}
		return fmt.Errorf("node %q %s references unknown node %q", c.ID, kind, targetID)
	}
	for _, a := range c.Align {
		if err := check(a.To, "align"); err != nil {
			return err
		}
	}
	for _, n := range c.Near {
		if err := check(n.To, "near"); err != nil {
			return err
		}
	}
	// Over-determination check: at most one axis-affecting constraint per axis.
	xCount, yCount := 0, 0
	for _, n := range c.Near {
		switch n.Side {
		case SideLeft, SideRight:
			xCount++
		case SideTop, SideBottom:
			yCount++
		}
	}
	for _, a := range c.Align {
		switch a.Edge {
		case EdgeLeft, EdgeRight, EdgeCenterX:
			xCount++
		case EdgeTop, EdgeBottom, EdgeCenterY:
			yCount++
		}
	}
	if xCount > 1 {
		return fmt.Errorf("node %q has %d constraints on x axis (max 1)", c.ID, xCount)
	}
	if yCount > 1 {
		return fmt.Errorf("node %q has %d constraints on y axis (max 1)", c.ID, yCount)
	}
	return nil
}

func collectIDs(n *Node, out map[string]struct{}) {
	if n == nil {
		return
	}
	out[n.ID] = struct{}{}
	for _, c := range n.Children {
		collectIDs(c, out)
	}
}
