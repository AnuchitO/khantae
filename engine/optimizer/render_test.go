package optimizer

import (
	"bytes"
	"fmt"
	"os"
	"testing"

	"github.com/AnuchitO/khantae/engine"
	"github.com/AnuchitO/khantae/engine/router"
)

// Renders a before/after comparison of SA optimization to an SVG file
// for visual inspection. Uses the same 4-leaf test problem where
// reordering the stack should produce noticeably cleaner routes.
func TestRenderBeforeAfterSA(t *testing.T) {
	prob := buildTestProblem()
	cfg := DefaultConfig()
	cfg.Seeds = 5
	cfg.MaxIter = 200

	// Before.
	beforeEval, err := evaluate(prob.Root, nil, prob.Edges, prob.LabelRects, cfg)
	if err != nil {
		t.Fatal(err)
	}

	// After.
	res, err := Optimize(prob, cfg)
	if err != nil {
		t.Fatal(err)
	}

	t.Logf("before: energy=%.1f  edges=%.1f  area=%.0f",
		beforeEval.energy, beforeEval.edgeCost, beforeEval.area)
	t.Logf("after:  energy=%.1f  edges=%.1f  area=%.0f (seed %d, %d iters)",
		res.Energy, res.EdgeCost, res.Area, res.Seed, res.Iters)

	var buf bytes.Buffer
	const panelW, panelH, gap, pad = 400.0, 300.0, 60.0, 20.0
	totalW := panelW*2 + gap + 2*pad
	totalH := panelH + 2*pad
	fmt.Fprintf(&buf, `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.0f %.0f" width="%.0f" height="%.0f" font-family="monospace">`+"\n",
		totalW, totalH, totalW, totalH)
	fmt.Fprintf(&buf, `  <text x="%.0f" y="24" font-size="14" fill="#222">BEFORE (E=%.0f)</text>`+"\n",
		pad+10, beforeEval.energy)
	fmt.Fprintf(&buf, `  <text x="%.0f" y="24" font-size="14" fill="#222">AFTER (E=%.0f)</text>`+"\n",
		pad+panelW+gap+10, res.Energy)

	drawPanel(&buf, pad, 40, beforeEval.rects, beforeEval.routes, prob.Root)
	drawPanel(&buf, pad+panelW+gap, 40, res.Rects, res.Routes, res.Tree)

	buf.WriteString("</svg>\n")

	path := os.Getenv("OPTIMIZER_DEMO_SVG")
	if path == "" {
		path = "/tmp/optimizer_before_after.svg"
	}
	if err := os.WriteFile(path, buf.Bytes(), 0o644); err != nil {
		t.Fatal(err)
	}
	t.Logf("wrote demo to %s", path)
}

// drawPanel writes a single panel (one configuration's boxes+routes)
// into the SVG buffer at (offX, offY).
func drawPanel(buf *bytes.Buffer, offX, offY float64,
	rects map[string]engine.Rect, routes []router.Route, tree *engine.Node) {

	colors := []string{"#4c72b0", "#dd8452", "#55a868", "#c44e52", "#8172b3"}
	// Deterministic color assignment by leaf ID.
	ids := collectLeafIDs(tree)
	colorFor := map[string]string{}
	for i, id := range ids {
		colorFor[id] = colors[i%len(colors)]
	}

	for _, id := range ids {
		r := rects[id]
		fmt.Fprintf(buf,
			`  <rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" fill-opacity="0.35" stroke="%s" stroke-width="1.5"/>`+"\n",
			r.X+offX, r.Y+offY, r.W, r.H, colorFor[id], colorFor[id])
		fmt.Fprintf(buf, `  <text x="%.2f" y="%.2f" font-size="12" fill="#222">%s</text>`+"\n",
			r.X+offX+4, r.Y+offY+14, id)
	}
	// Routes.
	for _, r := range routes {
		if len(r.Waypoints) < 2 {
			continue
		}
		buf.WriteString(`  <polyline fill="none" stroke="#222" stroke-width="1.5" points="`)
		for i, p := range r.Waypoints {
			if i > 0 {
				buf.WriteByte(' ')
			}
			fmt.Fprintf(buf, "%.2f,%.2f", p.X+offX, p.Y+offY)
		}
		buf.WriteString(`"/>` + "\n")
	}
}
