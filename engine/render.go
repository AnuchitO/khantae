package engine

import (
	"fmt"
	"io"
	"sort"
	"strings"
)

// RenderOptions controls the appearance of SVG debug output.
type RenderOptions struct {
	// Padding is extra space around the content bounding box, in user units.
	Padding float64
	// ShowLabels toggles the ID label drawn on each box.
	ShowLabels bool
	// ShowDimensions toggles the "WxH" sub-label under each ID.
	ShowDimensions bool
	// StrokeWidth in user units.
	StrokeWidth float64
	// FontSize in user units.
	FontSize float64
}

// DefaultRenderOptions returns sensible defaults for debug rendering.
func DefaultRenderOptions() RenderOptions {
	return RenderOptions{
		Padding:        20,
		ShowLabels:     true,
		ShowDimensions: true,
		StrokeWidth:    1.5,
		FontSize:       11,
	}
}

// RenderSVG writes an SVG representation of the laid-out tree to w.
// It draws every node as a rectangle, colored by a stable hash of its
// depth so parents, children, and grandchildren are visually distinct.
// Deeper nodes are drawn later so they render on top of their ancestors.
func RenderSVG(w io.Writer, root *Node, rects map[string]Rect, opts RenderOptions) error {
	if root == nil {
		return fmt.Errorf("nil root")
	}

	// Compute the overall drawing bounds from the root rect.
	rootRect, ok := rects[root.ID]
	if !ok {
		return fmt.Errorf("no rect for root %q", root.ID)
	}

	canvasW := rootRect.W + 2*opts.Padding
	canvasH := rootRect.H + 2*opts.Padding

	// Collect nodes with their depth so we can draw shallow-first.
	type entry struct {
		node  *Node
		depth int
	}
	var entries []entry
	var walk func(n *Node, d int)
	walk = func(n *Node, d int) {
		if n == nil {
			return
		}
		entries = append(entries, entry{n, d})
		for _, c := range n.Children {
			walk(c, d+1)
		}
	}
	walk(root, 0)

	// Stable order: by depth asc, then by ID for determinism.
	sort.SliceStable(entries, func(i, j int) bool {
		if entries[i].depth != entries[j].depth {
			return entries[i].depth < entries[j].depth
		}
		return entries[i].node.ID < entries[j].node.ID
	})

	var b strings.Builder
	fmt.Fprintf(&b, `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.2f %.2f" width="%.2f" height="%.2f" font-family="monospace">`,
		canvasW, canvasH, canvasW, canvasH)
	fmt.Fprintf(&b, "\n")

	for _, e := range entries {
		r, ok := rects[e.node.ID]
		if !ok {
			continue
		}
		fill := depthColor(e.depth)
		x := r.X + opts.Padding
		y := r.Y + opts.Padding

		fmt.Fprintf(&b,
			`  <rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" fill-opacity="0.35" stroke="%s" stroke-width="%.2f"/>`+"\n",
			x, y, r.W, r.H, fill, darken(fill), opts.StrokeWidth)

		if opts.ShowLabels {
			// Skip labels that won't fit. Rule of thumb: need box width
			// at least a few characters worth of font, and height for
			// one or two lines of text.
			minLabelW := opts.FontSize * 2.0
			if r.W < minLabelW || r.H < opts.FontSize+4 {
				continue
			}

			// Offset label by depth so sibling labels don't land on top
			// of each other when one box is nested inside another that
			// starts at the same corner.
			labelX := x + 4
			labelY := y + opts.FontSize + 2 + float64(e.depth)*2

			fmt.Fprintf(&b,
				`  <text x="%.2f" y="%.2f" font-size="%.2f" fill="%s">%s</text>`+"\n",
				labelX, labelY, opts.FontSize, darken(fill), escapeText(e.node.ID))

			if opts.ShowDimensions && r.H >= opts.FontSize*2+6 {
				dimY := labelY + opts.FontSize + 1
				fmt.Fprintf(&b,
					`  <text x="%.2f" y="%.2f" font-size="%.2f" fill="%s" fill-opacity="0.7">%.0f×%.0f</text>`+"\n",
					labelX, dimY, opts.FontSize*0.85, darken(fill), r.W, r.H)
			}
		}
	}

	b.WriteString("</svg>\n")
	_, err := io.WriteString(w, b.String())
	return err
}

// depthColor returns a stable hex color per depth. Colors cycle through
// a small palette so nested levels are distinguishable.
func depthColor(depth int) string {
	palette := []string{
		"#4c72b0", // blue
		"#dd8452", // orange
		"#55a868", // green
		"#c44e52", // red
		"#8172b3", // purple
		"#937860", // brown
		"#da8bc3", // pink
		"#8c8c8c", // gray
	}
	return palette[depth%len(palette)]
}

// darken converts a "#rrggbb" color into a darker variant for strokes
// and text. Uses a simple 60% luminance reduction in sRGB space — not
// colorimetrically correct, but fine for debug output.
func darken(hex string) string {
	if len(hex) != 7 || hex[0] != '#' {
		return hex
	}
	var r, g, bl uint8
	fmt.Sscanf(hex[1:], "%02x%02x%02x", &r, &g, &bl)
	r = uint8(float64(r) * 0.6)
	g = uint8(float64(g) * 0.6)
	bl = uint8(float64(bl) * 0.6)
	return fmt.Sprintf("#%02x%02x%02x", r, g, bl)
}

// escapeText XML-escapes a label. Belt-and-suspenders against IDs
// containing characters that would break the SVG.
func escapeText(s string) string {
	r := strings.NewReplacer(
		"&", "&amp;",
		"<", "&lt;",
		">", "&gt;",
		`"`, "&quot;",
		"'", "&apos;",
	)
	return r.Replace(s)
}
