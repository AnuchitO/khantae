package main

import (
	"math"
	"sort"

	"oss.terrastruct.com/d2/d2graph"
	"oss.terrastruct.com/d2/lib/geo"

	"github.com/AnuchitO/khantae/engine"
)

const (
	rowGap = 60.0
	colGap = 60.0
)

// runEngine is the layout pipeline. It translates a D2 graph to a
// layered layout, writes positions back, and routes edges.
func runEngine(g *d2graph.Graph) error {
	if g.Root == nil {
		return nil
	}

	objects := collectObjects(g)
	if len(objects) == 0 {
		return nil
	}

	// Build object map for quick lookup.
	objMap := make(map[string]*d2graph.Object, len(objects))
	for _, obj := range objects {
		objMap[obj.AbsID()] = obj
	}

	// Compute layered layout from graph edges.
	layers := computeLayers(objects, g.Edges, objMap)
	rects := computeLayeredPositions(layers, objMap)

	// Write positions back to D2 objects and centre labels in their boxes.
	labelCenter := "INSIDE_MIDDLE_CENTER"
	for _, obj := range objects {
		r, ok := rects[obj.AbsID()]
		if !ok {
			continue
		}
		obj.TopLeft = geo.NewPoint(r.X, r.Y)
		// Do NOT overwrite Width/Height — D2 set them from label metrics.
		obj.LabelPosition = &labelCenter
	}

	if len(g.Edges) == 0 {
		return nil
	}

	// Build a layer-lookup map for edge routing decisions.
	nodeLayer := make(map[string]int, len(objects))
	for l, layer := range layers {
		for _, id := range layer {
			nodeLayer[id] = l
		}
	}

	// Route edges directly using clean orthogonal paths based on layer
	// positions rather than the A* router, which over-bends on dense graphs.
	for _, e := range g.Edges {
		if e.Src == nil || e.Dst == nil {
			continue
		}
		src, srcOK := rects[e.Src.AbsID()]
		dst, dstOK := rects[e.Dst.AbsID()]
		if !srcOK || !dstOK {
			continue
		}
		srcL := nodeLayer[e.Src.AbsID()]
		dstL := nodeLayer[e.Dst.AbsID()]
		e.Route = routeEdge(src, dst, srcL, dstL)
	}
	return nil
}

// computeLayers assigns each node to a layer using a modified topological
// sort that replicates tala's grouping: when a node fans out (out-degree > 1)
// its edges carry span=0, so all immediate successors land on the same layer
// as their source. Only single-output edges carry span=1.
//
// Within each layer, nodes are ordered so that the "source" of in-layer edges
// sits in the middle, with its in-layer successors distributed left/right in
// the order they appear in the original edge list relative to any cross-layer
// successors.
func computeLayers(objects []*d2graph.Object, edges []*d2graph.Edge, _ map[string]*d2graph.Object) [][]string {
	objIDs := make(map[string]bool, len(objects))
	for _, obj := range objects {
		objIDs[obj.AbsID()] = true
	}

	// Build adjacency lists, preserving original edge order (D2 file order).
	outEdges := make(map[string][]string, len(objects))
	inEdges := make(map[string][]string, len(objects))
	for _, obj := range objects {
		id := obj.AbsID()
		outEdges[id] = nil
		inEdges[id] = nil
	}
	for _, e := range edges {
		if e.Src == nil || e.Dst == nil {
			continue
		}
		src, dst := e.Src.AbsID(), e.Dst.AbsID()
		if !objIDs[src] || !objIDs[dst] || src == dst {
			continue
		}
		outEdges[src] = append(outEdges[src], dst)
		inEdges[dst] = append(inEdges[dst], src)
	}

	// Compute in-degrees for Kahn's algorithm.
	inDeg := make(map[string]int, len(objects))
	for _, obj := range objects {
		for _, dst := range outEdges[obj.AbsID()] {
			inDeg[dst]++
		}
	}

	// Modified Kahn's: span is 0 when the current node fans out (out-degree>1),
	// otherwise 1. This matches tala's "compact" layer grouping.
	layer := make(map[string]int, len(objects))
	for _, obj := range objects {
		layer[obj.AbsID()] = -1
	}
	queue := make([]string, 0, len(objects))
	for _, obj := range objects {
		id := obj.AbsID()
		if inDeg[id] == 0 {
			layer[id] = 0
			queue = append(queue, id)
		}
	}
	processed := 0
	for len(queue) > 0 {
		curr := queue[0]
		queue = queue[1:]
		processed++
		span := 1
		if len(outEdges[curr]) > 1 {
			span = 0
		}
		for _, next := range outEdges[curr] {
			if layer[curr]+span > layer[next] {
				layer[next] = layer[curr] + span
			}
			inDeg[next]--
			if inDeg[next] == 0 {
				queue = append(queue, next)
			}
		}
	}

	// Handle cycles: assign remaining nodes after the last assigned layer.
	if processed < len(objects) {
		maxL := 0
		for _, l := range layer {
			if l > maxL {
				maxL = l
			}
		}
		for _, obj := range objects {
			id := obj.AbsID()
			if layer[id] < 0 {
				layer[id] = maxL + 1
			}
		}
	}

	// Group nodes by layer, preserving input order within each group.
	maxLayer := 0
	for _, l := range layer {
		if l > maxLayer {
			maxLayer = l
		}
	}
	layerNodes := make([][]string, maxLayer+1)
	for _, obj := range objects {
		id := obj.AbsID()
		layerNodes[layer[id]] = append(layerNodes[layer[id]], id)
	}

	// Order within each layer.
	// Layer 0: keep input order; track position for barycenter of next layers.
	posInLayer := make(map[string]float64, len(objects))
	for i, id := range layerNodes[0] {
		posInLayer[id] = float64(i)
	}

	for l := 1; l <= maxLayer; l++ {
		layerNodes[l] = orderLayer(layerNodes[l], l, layer, inEdges, outEdges, posInLayer, objIDs)
		for i, id := range layerNodes[l] {
			posInLayer[id] = float64(i)
		}
	}

	return layerNodes
}

// orderLayer orders nodes within a single layer.
//
// "Source" nodes (no in-layer predecessors) are sorted by their cross-layer
// barycenter. For each source that has in-layer successors, those successors
// are split left/right: successors appearing before the first cross-layer
// successor in the original edge list go LEFT of the source; those appearing
// after go RIGHT. The final order is [left..., source, right...] per group.
func orderLayer(
	nodes []string,
	l int,
	layer map[string]int,
	inEdges, outEdges map[string][]string,
	posInLayer map[string]float64,
	objIDs map[string]bool,
) []string {
	// Partition into sources (no in-layer predecessor) and side nodes.
	isSource := make(map[string]bool, len(nodes))
	for _, id := range nodes {
		src := true
		for _, pred := range inEdges[id] {
			if layer[pred] == l {
				src = false
				break
			}
		}
		isSource[id] = src
	}

	// Sort sources by barycenter of their cross-layer predecessors.
	type srcEntry struct {
		id string
		bc float64
	}
	sources := make([]srcEntry, 0, len(nodes))
	for _, id := range nodes {
		if !isSource[id] {
			continue
		}
		sum, count := 0.0, 0
		for _, pred := range inEdges[id] {
			if layer[pred] < l {
				if pos, ok := posInLayer[pred]; ok {
					sum += pos
					count++
				}
			}
		}
		bc := float64(len(sources)) // default: preserve input order
		if count > 0 {
			bc = sum / float64(count)
		}
		sources = append(sources, srcEntry{id, bc})
	}
	sort.SliceStable(sources, func(a, b int) bool {
		return sources[a].bc < sources[b].bc
	})

	// Build the ordered result group by group.
	used := make(map[string]bool, len(nodes))
	result := make([]string, 0, len(nodes))

	for _, s := range sources {
		src := s.id
		if used[src] {
			continue
		}
		// Walk the source's out-edges in original order, splitting in-layer
		// successors into left (before first cross-layer) and right (after).
		var left, right []string
		seenCross := false
		for _, succ := range outEdges[src] {
			if !objIDs[succ] || layer[succ] != l || used[succ] {
				if objIDs[succ] && layer[succ] != l {
					seenCross = true
				}
				continue
			}
			if !seenCross {
				left = append(left, succ)
			} else {
				right = append(right, succ)
			}
		}
		for _, id := range left {
			result = append(result, id)
			used[id] = true
		}
		result = append(result, src)
		used[src] = true
		for _, id := range right {
			result = append(result, id)
			used[id] = true
		}
	}

	// Append any remaining nodes (e.g. in cycles or disconnected in-layer).
	for _, id := range nodes {
		if !used[id] {
			result = append(result, id)
		}
	}
	return result
}

// computeLayeredPositions assigns absolute pixel positions to each node.
// Each layer is centered horizontally relative to the widest layer.
func computeLayeredPositions(layers [][]string, objMap map[string]*d2graph.Object) map[string]engine.Rect {
	rects := make(map[string]engine.Rect, len(objMap))
	if len(layers) == 0 {
		return rects
	}

	// Compute each layer's total width.
	layerW := make([]float64, len(layers))
	for l, layer := range layers {
		w := 0.0
		for i, id := range layer {
			if i > 0 {
				w += colGap
			}
			w += objMap[id].Width
		}
		layerW[l] = w
	}

	// Widest layer sets the canvas width for centering.
	maxW := 0.0
	for _, w := range layerW {
		if w > maxW {
			maxW = w
		}
	}

	y := 0.0
	for l, layer := range layers {
		// Advance Y past the previous layer's tallest node + gap.
		if l > 0 {
			prevH := 0.0
			for _, id := range layers[l-1] {
				if h := objMap[id].Height; h > prevH {
					prevH = h
				}
			}
			y += prevH + rowGap
		}

		// Center this layer horizontally.
		x := (maxW - layerW[l]) / 2.0
		for i, id := range layer {
			if i > 0 {
				x += colGap
			}
			obj := objMap[id]
			rects[id] = engine.Rect{X: x, Y: y, W: obj.Width, H: obj.Height}
			x += obj.Width
		}
	}

	return rects
}

// routeEdge produces clean orthogonal waypoints for a single edge.
//
//   - Same layer:  straight horizontal line (left↔right sides at center Y).
//   - Cross-layer, vertically aligned:  straight vertical line (top/bottom centers).
//   - Cross-layer, offset:  L-shape — exit top/bottom center, travel to dst center Y,
//     then enter dst from the nearer horizontal side.
func routeEdge(src, dst engine.Rect, srcL, dstL int) []*geo.Point {
	srcCX := src.X + src.W/2
	srcCY := src.Y + src.H/2
	dstCX := dst.X + dst.W/2
	dstCY := dst.Y + dst.H/2

	if srcL == dstL {
		// Horizontal: exit the side that faces the destination.
		if srcCX <= dstCX {
			return []*geo.Point{
				geo.NewPoint(src.X+src.W, srcCY),
				geo.NewPoint(dst.X, dstCY),
			}
		}
		return []*geo.Point{
			geo.NewPoint(src.X, srcCY),
			geo.NewPoint(dst.X+dst.W, dstCY),
		}
	}

	// Determine vertical exit/entry points.
	var srcY, dstY float64
	if dstL > srcL {
		srcY = src.Y + src.H // exit bottom
		dstY = dst.Y         // enter top
	} else {
		srcY = src.Y         // exit top
		dstY = dst.Y + dst.H // enter bottom
	}

	// If roughly horizontally aligned (within half a colGap), emit a true
	// vertical segment.  Use the average X so D2's border-trim lands on the
	// same column for both endpoints, preventing a subtle diagonal.
	if math.Abs(srcCX-dstCX) < colGap/2 {
		midX := (srcCX + dstCX) / 2
		return []*geo.Point{
			geo.NewPoint(midX, srcY),
			geo.NewPoint(midX, dstY),
		}
	}

	// L-shape: go vertically to dst's center Y, then horizontally into dst side.
	var dstSideX float64
	if srcCX < dstCX {
		dstSideX = dst.X // enter from left
	} else {
		dstSideX = dst.X + dst.W // enter from right
	}
	return []*geo.Point{
		geo.NewPoint(srcCX, srcY),
		geo.NewPoint(srcCX, dstCY),
		geo.NewPoint(dstSideX, dstCY),
	}
}

// collectObjects returns every non-root, non-zero-size object in stable order.
func collectObjects(g *d2graph.Graph) []*d2graph.Object {
	out := make([]*d2graph.Object, 0, len(g.Objects))
	for _, obj := range g.Objects {
		if obj == nil || obj == g.Root {
			continue
		}
		if obj.Width <= 0 || obj.Height <= 0 {
			continue
		}
		out = append(out, obj)
	}
	return out
}

