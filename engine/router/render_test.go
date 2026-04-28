package router

import (
	"bytes"
	"fmt"
	"os"
	"strings"
	"testing"

	"github.com/AnuchitO/khantae/engine"
)

// Produces a visual artifact for eyeballing routing quality.
// Four nodes arranged roughly in a diamond with two crossing
// connections and one obstacle-avoiding connection.
func TestRenderRoutedDemo(t *testing.T) {
	rects := map[string]engine.Rect{
		"A": {X: 20, Y: 20, W: 60, H: 40},
		"B": {X: 220, Y: 20, W: 60, H: 40},
		"C": {X: 220, Y: 160, W: 60, H: 40},
		"D": {X: 20, Y: 160, W: 60, H: 40},
		// Obstacle in the middle:
		"X": {X: 130, Y: 90, W: 40, H: 40},
	}
	obs := []Obstacle{
		{ID: "A", Rect: rects["A"], Kind: KindNode},
		{ID: "B", Rect: rects["B"], Kind: KindNode},
		{ID: "C", Rect: rects["C"], Kind: KindNode},
		{ID: "D", Rect: rects["D"], Kind: KindNode},
		{ID: "X", Rect: rects["X"], Kind: KindNode},
	}
	edges := []EdgeRequest{
		{ID: "AC", From: Endpoint{NodeID: "A", Side: SideBottom}, To: Endpoint{NodeID: "C", Side: SideLeft}},
		{ID: "BD", From: Endpoint{NodeID: "B", Side: SideBottom}, To: Endpoint{NodeID: "D", Side: SideRight}},
		{ID: "AB", From: Endpoint{NodeID: "A", Side: SideRight}, To: Endpoint{NodeID: "B", Side: SideLeft}},
	}
	routes, err := RouteAll(rects, obs, edges, DefaultWeights())
	if err != nil {
		t.Fatal(err)
	}
	for _, r := range routes {
		t.Logf("route %s: %d bends, %d crossings, length %.1f, cost %.1f",
			r.EdgeID, r.Bends, r.Crossings, r.Length, r.Cost)
	}

	// Build a standalone SVG: boxes + overlay.
	var buf bytes.Buffer
	const pad = 20.0
	const w, h = 300.0, 220.0
	fmt.Fprintf(&buf, `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.0f %.0f" width="%.0f" height="%.0f" font-family="monospace">`+"\n",
		w+2*pad, h+2*pad, w+2*pad, h+2*pad)
	// Box palette.
	colors := map[string]string{"A": "#4c72b0", "B": "#dd8452", "C": "#55a868", "D": "#c44e52", "X": "#8c8c8c"}
	for id, r := range rects {
		fill := colors[id]
		fmt.Fprintf(&buf,
			`  <rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" fill-opacity="0.35" stroke="%s" stroke-width="1.5"/>`+"\n",
			r.X+pad, r.Y+pad, r.W, r.H, fill, fill)
		fmt.Fprintf(&buf, `  <text x="%.2f" y="%.2f" font-size="12" fill="#222">%s</text>`+"\n",
			r.X+pad+4, r.Y+pad+14, id)
	}
	if err := RenderSVGOverlay(&buf, routes, pad, pad); err != nil {
		t.Fatal(err)
	}
	buf.WriteString("</svg>\n")

	// Sanity: at least one polyline emitted per edge.
	if n := strings.Count(buf.String(), "<polyline"); n != len(edges) {
		t.Errorf("want %d polylines, got %d", len(edges), n)
	}

	path := os.Getenv("ROUTER_DEMO_SVG")
	if path == "" {
		path = "/tmp/router_demo.svg"
	}
	if err := os.WriteFile(path, buf.Bytes(), 0o644); err != nil {
		t.Fatal(err)
	}
	t.Logf("wrote demo to %s", path)
}
