package engine

import (
	"fmt"
	"testing"
)

// Five levels deep. Each level wraps its single child with padding 2.
// Leaf is 10x10. After N levels of padding 2, outermost = 10 + 2*2*N.
func TestFiveLevelsDeep(t *testing.T) {
	const levels = 5
	var node *Node = &Node{ID: "leaf", MinW: 10, MinH: 10}
	for i := levels - 1; i >= 0; i-- {
		node = &Node{
			ID:       fmt.Sprintf("L%d", i),
			Padding:  Padding{Top: 2, Right: 2, Bottom: 2, Left: 2},
			Children: []*Node{node},
		}
	}
	out, err := Layout(node)
	if err != nil {
		t.Fatal(err)
	}
	wantSide := 10.0 + 2.0*2.0*float64(levels)
	assertRect(t, out, "L0", Rect{0, 0, wantSide, wantSide})
	// Leaf sits at cumulative padding on both axes.
	cumPad := 2.0 * float64(levels)
	assertRect(t, out, "leaf", Rect{cumPad, cumPad, 10, 10})
}

// Chain of near-right constraints: a <- b <- c <- d, each gap 5.
// Widths 10, 20, 30, 40. b.x = 0+10+5 = 15; c.x = 15+20+5 = 40; d.x = 40+30+5 = 75.
func TestNearChain(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "a", MinW: 10, MinH: 10},
		{ID: "b", MinW: 20, MinH: 10,
			Near:  []NearConstraint{{To: "a", Side: SideRight, Gap: 5}},
			Align: []AlignConstraint{{To: "a", Edge: EdgeTop}}},
		{ID: "c", MinW: 30, MinH: 10,
			Near:  []NearConstraint{{To: "b", Side: SideRight, Gap: 5}},
			Align: []AlignConstraint{{To: "b", Edge: EdgeTop}}},
		{ID: "d", MinW: 40, MinH: 10,
			Near:  []NearConstraint{{To: "c", Side: SideRight, Gap: 5}},
			Align: []AlignConstraint{{To: "c", Edge: EdgeTop}}},
	}}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	assertRect(t, out, "a", Rect{0, 0, 10, 10})
	assertRect(t, out, "b", Rect{15, 0, 20, 10})
	assertRect(t, out, "c", Rect{40, 0, 30, 10})
	assertRect(t, out, "d", Rect{75, 0, 40, 10})
}

// A parent with no children and no MinW/MinH should size to 0x0 plus padding.
func TestEmptyChildrenParent(t *testing.T) {
	root := &Node{
		ID:      "r",
		Padding: Padding{Top: 3, Right: 3, Bottom: 3, Left: 3},
	}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	// Leaf with MinW=MinH=0 has 0x0 size; padding doesn't apply to leaves.
	assertRect(t, out, "r", Rect{0, 0, 0, 0})
}

// Nil root must not panic.
func TestNilRoot(t *testing.T) {
	out, err := Layout(nil)
	if err != nil {
		t.Fatal(err)
	}
	if len(out) != 0 {
		t.Errorf("expected empty map, got %d entries", len(out))
	}
}

// --- Benchmarks ---------------------------------------------------------

// Wide tree: one root with N leaf children.
func benchmarkWide(b *testing.B, n int) {
	b.ReportAllocs()
	children := make([]*Node, n)
	for i := range children {
		children[i] = &Node{ID: fmt.Sprintf("c%d", i), MinW: 10, MinH: 10}
	}
	root := &Node{ID: "r",
		Padding:  Padding{Top: 1, Right: 1, Bottom: 1, Left: 1},
		Children: children,
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := Layout(root); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkWide100(b *testing.B)  { benchmarkWide(b, 100) }
func BenchmarkWide1000(b *testing.B) { benchmarkWide(b, 1000) }

// Deep tree: N levels of single-child nesting.
func benchmarkDeep(b *testing.B, depth int) {
	b.ReportAllocs()
	var node *Node = &Node{ID: "leaf", MinW: 5, MinH: 5}
	for i := depth - 1; i >= 0; i-- {
		node = &Node{
			ID:       fmt.Sprintf("L%d", i),
			Padding:  Padding{Top: 1, Right: 1, Bottom: 1, Left: 1},
			Children: []*Node{node},
		}
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := Layout(node); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkDeep50(b *testing.B)  { benchmarkDeep(b, 50) }
func BenchmarkDeep200(b *testing.B) { benchmarkDeep(b, 200) }

// Chain: a single sibling group of N nodes where each `near`s the previous.
// Exercises the topological propagation path on a long chain.
func BenchmarkNearChain500(b *testing.B) {
	b.ReportAllocs()
	n := 500
	children := make([]*Node, n)
	children[0] = &Node{ID: "n0", MinW: 4, MinH: 4}
	for i := 1; i < n; i++ {
		children[i] = &Node{
			ID:   fmt.Sprintf("n%d", i),
			MinW: 4, MinH: 4,
			Near:  []NearConstraint{{To: fmt.Sprintf("n%d", i-1), Side: SideRight, Gap: 1}},
			Align: []AlignConstraint{{To: fmt.Sprintf("n%d", i-1), Edge: EdgeTop}},
		}
	}
	root := &Node{ID: "r", Children: children}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := Layout(root); err != nil {
			b.Fatal(err)
		}
	}
}
