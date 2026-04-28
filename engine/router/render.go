package router

import (
	"fmt"
	"io"
	"strings"
)

// RenderSVGOverlay emits an SVG fragment (not a full document) that
// draws the given routes as orthogonal polylines. Intended to be
// written into the same file an engine SVG renderer produced, so
// the routes overlay the boxes.
//
// If you need a standalone SVG, see RenderSVGDocument.
func RenderSVGOverlay(w io.Writer, routes []Route, offsetX, offsetY float64) error {
	var b strings.Builder
	for _, r := range routes {
		if len(r.Waypoints) < 2 {
			continue
		}
		b.WriteString(`  <polyline fill="none" stroke="#222" stroke-width="1.5" points="`)
		for i, p := range r.Waypoints {
			if i > 0 {
				b.WriteByte(' ')
			}
			fmt.Fprintf(&b, "%.2f,%.2f", p.X+offsetX, p.Y+offsetY)
		}
		b.WriteString(`"/>` + "\n")
		// Small arrowhead at the terminal point — direction inferred
		// from the last segment.
		n := len(r.Waypoints)
		last := r.Waypoints[n-1]
		prev := r.Waypoints[n-2]
		drawArrowhead(&b, last, prev, offsetX, offsetY)
	}
	_, err := io.WriteString(w, b.String())
	return err
}

func drawArrowhead(b *strings.Builder, tip, prev Point, offX, offY float64) {
	const size = 4.0
	tx, ty := tip.X+offX, tip.Y+offY
	var p1x, p1y, p2x, p2y float64
	switch {
	case prev.Y == tip.Y && prev.X < tip.X: // arrow points right
		p1x, p1y = tx-size, ty-size*0.6
		p2x, p2y = tx-size, ty+size*0.6
	case prev.Y == tip.Y && prev.X > tip.X: // left
		p1x, p1y = tx+size, ty-size*0.6
		p2x, p2y = tx+size, ty+size*0.6
	case prev.X == tip.X && prev.Y < tip.Y: // down
		p1x, p1y = tx-size*0.6, ty-size
		p2x, p2y = tx+size*0.6, ty-size
	case prev.X == tip.X && prev.Y > tip.Y: // up
		p1x, p1y = tx-size*0.6, ty+size
		p2x, p2y = tx+size*0.6, ty+size
	default:
		return
	}
	fmt.Fprintf(b, `  <polygon fill="#222" points="%.2f,%.2f %.2f,%.2f %.2f,%.2f"/>`+"\n",
		tx, ty, p1x, p1y, p2x, p2y)
}
