package engine

import (
	"math"
	"testing"
)

const eps = 1e-9

func approxEq(a, b float64) bool { return math.Abs(a-b) < eps }

func assertRect(t *testing.T, got map[string]Rect, id string, want Rect) {
	t.Helper()
	r, ok := got[id]
	if !ok {
		t.Fatalf("no rect for %q", id)
	}
	if !approxEq(r.X, want.X) || !approxEq(r.Y, want.Y) ||
		!approxEq(r.W, want.W) || !approxEq(r.H, want.H) {
		t.Errorf("%s: got %+v, want %+v", id, r, want)
	}
}

// A single leaf takes its MinW/MinH and sits at the origin.
func TestLeafOnly(t *testing.T) {
	root := &Node{ID: "leaf", MinW: 40, MinH: 20}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	assertRect(t, out, "leaf", Rect{0, 0, 40, 20})
}

// A parent sized bottom-up should equal children bbox + padding.
// Two 50x30 leaves stacked vertically, padding 10 all sides
// => parent W = 50 + 20 = 70, H = 30+30 + 20 = 80.
func TestBottomUpStacking(t *testing.T) {
	root := &Node{
		ID:      "root",
		Padding: Padding{Top: 10, Right: 10, Bottom: 10, Left: 10},
		Children: []*Node{
			{ID: "a", MinW: 50, MinH: 30},
			{ID: "b", MinW: 50, MinH: 30},
		},
	}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	assertRect(t, out, "root", Rect{0, 0, 70, 80})
	assertRect(t, out, "a", Rect{10, 10, 50, 30})
	assertRect(t, out, "b", Rect{10, 40, 50, 30})
}

// Align center_x: child a is 50 wide, b is 30 wide and aligned to a's
// center_x. Then b.x = a.x + (50-30)/2 = a.x + 10.
func TestAlignCenterX(t *testing.T) {
	root := &Node{
		ID:      "root",
		Padding: Padding{Top: 5, Bottom: 5, Left: 5, Right: 5},
		Children: []*Node{
			{ID: "a", MinW: 50, MinH: 20},
			{ID: "b", MinW: 30, MinH: 20, Align: []AlignConstraint{{To: "a", Edge: EdgeCenterX}}},
		},
	}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	// a at padding (5,5); b centered on a horizontally, stacked below.
	assertRect(t, out, "a", Rect{5, 5, 50, 20})
	assertRect(t, out, "b", Rect{15, 25, 30, 20})
	// root should wrap both: W = 50+10 = 60, H = 40+10 = 50.
	assertRect(t, out, "root", Rect{0, 0, 60, 50})
}

// Near right with gap: b sits to the right of a with a 12 gap.
// a is 40x20 at (pad.L, pad.T); b.x = a.x + 40 + 12.
func TestNearRight(t *testing.T) {
	root := &Node{
		ID:      "root",
		Padding: Padding{Top: 4, Right: 4, Bottom: 4, Left: 4},
		Children: []*Node{
			{ID: "a", MinW: 40, MinH: 20},
			{ID: "b", MinW: 25, MinH: 20,
				Near:  []NearConstraint{{To: "a", Side: SideRight, Gap: 12}},
				Align: []AlignConstraint{{To: "a", Edge: EdgeTop}},
			},
		},
	}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	assertRect(t, out, "a", Rect{4, 4, 40, 20})
	assertRect(t, out, "b", Rect{4 + 40 + 12, 4, 25, 20})
	// Content width = 40 + 12 + 25 = 77, + pad 8 = 85. Height 20 + 8 = 28.
	assertRect(t, out, "root", Rect{0, 0, 85, 28})
}

// Near left: b sits to the left of a. a is still placed by its own
// default (root at 0 on X axis), so a.x = 0 and b.x = 0 - gap - b.W,
// which is negative. We then shift everything so the bbox left edge
// lands on padding.Left.
func TestNearLeftShifts(t *testing.T) {
	root := &Node{
		ID:      "root",
		Padding: Padding{Top: 0, Right: 0, Bottom: 0, Left: 10},
		Children: []*Node{
			{ID: "a", MinW: 30, MinH: 20},
			{ID: "b", MinW: 15, MinH: 20,
				Near:  []NearConstraint{{To: "a", Side: SideLeft, Gap: 5}},
				Align: []AlignConstraint{{To: "a", Edge: EdgeTop}},
			},
		},
	}
	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	// Local layout: a.x=0, b.x=0-5-15=-20. After shift so minX=pad.Left=10:
	// offX = 10 - (-20) = 30. So b.x=10, a.x=30.
	assertRect(t, out, "b", Rect{10, 0, 15, 20})
	assertRect(t, out, "a", Rect{30, 0, 30, 20})
}

// 3-level nested tree, as requested by the prompt.
//
// root
//   ├── header (leaf, 200x40)
//   └── body (container, padding 8)
//         ├── sidebar (leaf, 60x120)
//         └── content (container, padding 4)
//               ├── title (leaf, 80x16)
//               └── para  (leaf, 80x40)  aligned left to title
//
// Expected sizes (bottom-up):
//   title 80x16; para 80x40 -> stacked, content content-bbox = 80x56;
//   content = 80 + 8 = 88 wide, 56 + 8 = 64 tall.
//   sidebar 60x120; content 88x64 -> body stacks them vertically:
//   content-bbox width = max(60,88) = 88, height = 120+64=184;
//   body = 88 + 16 = 104 wide, 184 + 16 = 200 tall.
//   header 200x40; body 104x200 -> root default-stack (no padding):
//   content-bbox = 200 wide, 40+200=240 tall.
func TestThreeLevelNesting(t *testing.T) {
	title := &Node{ID: "title", MinW: 80, MinH: 16}
	para := &Node{ID: "para", MinW: 80, MinH: 40,
		Align: []AlignConstraint{{To: "title", Edge: EdgeLeft}}}
	content := &Node{ID: "content",
		Padding:  Padding{Top: 4, Right: 4, Bottom: 4, Left: 4},
		Children: []*Node{title, para}}
	sidebar := &Node{ID: "sidebar", MinW: 60, MinH: 120}
	body := &Node{ID: "body",
		Padding:  Padding{Top: 8, Right: 8, Bottom: 8, Left: 8},
		Children: []*Node{sidebar, content}}
	header := &Node{ID: "header", MinW: 200, MinH: 40}
	root := &Node{ID: "root", Children: []*Node{header, body}}

	out, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}

	// Sizes first (check W/H independent of positions).
	checkSize := func(id string, w, h float64) {
		t.Helper()
		r := out[id]
		if !approxEq(r.W, w) || !approxEq(r.H, h) {
			t.Errorf("%s size: got (%v, %v), want (%v, %v)", id, r.W, r.H, w, h)
		}
	}
	checkSize("title", 80, 16)
	checkSize("para", 80, 40)
	checkSize("content", 88, 64)
	checkSize("sidebar", 60, 120)
	checkSize("body", 104, 200)
	checkSize("header", 200, 40)
	checkSize("root", 200, 240)

	// Positions (global).
	// root at (0,0). header at (0,0). body at (0,40) because header is
	// 40 tall and stacking is vertical.
	assertRect(t, out, "root", Rect{0, 0, 200, 240})
	assertRect(t, out, "header", Rect{0, 0, 200, 40})
	assertRect(t, out, "body", Rect{0, 40, 104, 200})

	// Inside body: padding 8, sidebar at (8, 48), content at (8, 168).
	assertRect(t, out, "sidebar", Rect{8, 48, 60, 120})
	assertRect(t, out, "content", Rect{8, 168, 88, 64})

	// Inside content: padding 4, title at (12, 172), para at (12, 188).
	assertRect(t, out, "title", Rect{12, 172, 80, 16})
	assertRect(t, out, "para", Rect{12, 188, 80, 40})
}

func TestJSONInput(t *testing.T) {
	js := []byte(`{
	  "id": "r",
	  "padding": {"top": 2, "right": 2, "bottom": 2, "left": 2},
	  "children": [
	    {"id": "a", "min_w": 10, "min_h": 10},
	    {"id": "b", "min_w": 10, "min_h": 10,
	     "near": [{"to": "a", "side": "right", "gap": 3}],
	     "align": [{"to": "a", "edge": "top"}]}
	  ]
	}`)
	out, err := LayoutJSON(js)
	if err != nil {
		t.Fatal(err)
	}
	assertRect(t, out, "a", Rect{2, 2, 10, 10})
	assertRect(t, out, "b", Rect{15, 2, 10, 10})
	assertRect(t, out, "r", Rect{0, 0, 27, 14})
}

func TestCycleDetection(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "a", MinW: 10, MinH: 10, Align: []AlignConstraint{{To: "b", Edge: EdgeLeft}}},
		{ID: "b", MinW: 10, MinH: 10, Align: []AlignConstraint{{To: "a", Edge: EdgeLeft}}},
	}}
	if _, err := Layout(root); err == nil {
		t.Fatal("expected cycle error, got nil")
	}
}

func TestUnknownSibling(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "a", MinW: 10, MinH: 10,
			Align: []AlignConstraint{{To: "ghost", Edge: EdgeLeft}}},
	}}
	if _, err := Layout(root); err == nil {
		t.Fatal("expected error for unknown sibling reference")
	}
}

// Reference exists in the tree but is not a sibling — must error.
func TestNonSiblingReferenceRejected(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "outer", MinW: 20, MinH: 20, Children: []*Node{
			{ID: "inner", MinW: 5, MinH: 5},
		}},
		{ID: "b", MinW: 10, MinH: 10,
			// "inner" exists but is a cousin, not a sibling.
			Align: []AlignConstraint{{To: "inner", Edge: EdgeLeft}}},
	}}
	if _, err := Layout(root); err == nil {
		t.Fatal("expected error for non-sibling reference")
	}
}

// Two constraints affecting the same axis must be rejected.
func TestAxisConflictRejected(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "a", MinW: 10, MinH: 10},
		{ID: "b", MinW: 10, MinH: 10,
			Align: []AlignConstraint{
				{To: "a", Edge: EdgeLeft},
				{To: "a", Edge: EdgeRight}, // also x-axis — conflict
			}},
	}}
	if _, err := Layout(root); err == nil {
		t.Fatal("expected error for conflicting x-axis constraints")
	}
}

// A near on x and an align on y is fine — one per axis.
func TestOrthogonalConstraintsAllowed(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "a", MinW: 20, MinH: 20},
		{ID: "b", MinW: 10, MinH: 10,
			Near:  []NearConstraint{{To: "a", Side: SideRight, Gap: 2}},
			Align: []AlignConstraint{{To: "a", Edge: EdgeTop}},
		},
	}}
	if _, err := Layout(root); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestDuplicateIDRejected(t *testing.T) {
	root := &Node{ID: "r", Children: []*Node{
		{ID: "dup", MinW: 5, MinH: 5},
		{ID: "dup", MinW: 5, MinH: 5},
	}}
	if _, err := Layout(root); err == nil {
		t.Fatal("expected duplicate-ID error")
	}
}
