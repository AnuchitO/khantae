package engine

import (
	"bytes"
	"os"
	"strings"
	"testing"
)

func TestRenderSVGBasicShape(t *testing.T) {
	root := &Node{
		ID:      "root",
		Padding: Padding{Top: 10, Right: 10, Bottom: 10, Left: 10},
		Children: []*Node{
			{ID: "a", MinW: 50, MinH: 30},
			{ID: "b", MinW: 50, MinH: 30},
		},
	}
	rects, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	var buf bytes.Buffer
	if err := RenderSVG(&buf, root, rects, DefaultRenderOptions()); err != nil {
		t.Fatal(err)
	}
	out := buf.String()

	// Basic structural checks — the SVG should contain a root element,
	// three rects (root + 2 children), and the IDs as text labels.
	for _, want := range []string{
		`<svg`,
		`</svg>`,
		`>root<`,
		`>a<`,
		`>b<`,
	} {
		if !strings.Contains(out, want) {
			t.Errorf("missing %q in SVG output:\n%s", want, out)
		}
	}

	// Three rects: root + a + b.
	if n := strings.Count(out, "<rect "); n != 3 {
		t.Errorf("want 3 rects, got %d", n)
	}
}

func TestRenderSVGEscapesSpecialChars(t *testing.T) {
	root := &Node{ID: `<weird "id">`, MinW: 80, MinH: 40}
	rects, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}
	var buf bytes.Buffer
	if err := RenderSVG(&buf, root, rects, DefaultRenderOptions()); err != nil {
		t.Fatal(err)
	}
	out := buf.String()
	if strings.Contains(out, `<weird "id">`) {
		t.Error("unescaped ID leaked into SVG")
	}
	if !strings.Contains(out, "&lt;weird &quot;id&quot;&gt;") {
		t.Errorf("expected escaped ID, got:\n%s", out)
	}
}

// Integration test: render the 3-level nested tree from the prompt
// and write it to a file that a human can actually open.
func TestRenderSVGThreeLevelToFile(t *testing.T) {
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

	rects, err := Layout(root)
	if err != nil {
		t.Fatal(err)
	}

	// Emit to a well-known path that the outer harness can pick up.
	path := os.Getenv("BOXLAYOUT_DEMO_SVG")
	if path == "" {
		path = "/tmp/engine_three_level.svg"
	}
	f, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	opts := DefaultRenderOptions()
	opts.Padding = 30
	if err := RenderSVG(f, root, rects, opts); err != nil {
		t.Fatal(err)
	}
	t.Logf("wrote demo SVG to %s", path)
}
