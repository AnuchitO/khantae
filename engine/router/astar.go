package router

import (
	"container/heap"
	"fmt"
	"math"
)

// direction encodes the direction of the last move into a vertex.
// A* state is (vertex, direction); charging bend cost requires
// knowing the incoming direction so we can detect a turn.
type direction int

const (
	dirNone direction = iota
	dirLeft
	dirRight
	dirUp
	dirDown
)

func (d direction) opposite() direction {
	switch d {
	case dirLeft:
		return dirRight
	case dirRight:
		return dirLeft
	case dirUp:
		return dirDown
	case dirDown:
		return dirUp
	}
	return dirNone
}

// segment is an undirected grid edge between two neighboring vertices,
// keyed canonically (smaller index first) so crossings can be counted
// regardless of traversal direction.
type segment struct {
	a, b int // vertex linear indices; a < b
}

func makeSeg(a, b int) segment {
	if a > b {
		a, b = b, a
	}
	return segment{a, b}
}

// searchNode is an entry in the A* open set.
type searchNode struct {
	ix, iy int
	dir    direction
	g, f   float64 // g = cost from start; f = g + heuristic
	parent *searchNode
	seg    segment // segment used to arrive here (for crossing accounting)
	hasSeg bool
	idx    int // index in the heap, maintained by container/heap
}

type openSet []*searchNode

func (o openSet) Len() int            { return len(o) }
func (o openSet) Less(i, j int) bool  { return o[i].f < o[j].f }
func (o openSet) Swap(i, j int)       { o[i], o[j] = o[j], o[i]; o[i].idx = i; o[j].idx = j }
func (o *openSet) Push(x interface{}) { n := x.(*searchNode); n.idx = len(*o); *o = append(*o, n) }
func (o *openSet) Pop() interface{} {
	old := *o
	n := old[len(old)-1]
	*o = old[:len(old)-1]
	n.idx = -1
	return n
}

// (stateKey was used when the 'best' map was map-based; the slice
// representation now encodes state as v*5 + int(dir).)

// findRoute runs A* from startIx,startIy (entering in startDir) to the
// target vertex targetIx,targetIy (which must be approached going
// targetApproachDir, i.e. the direction INTO the target's node).
//
// usedSegments is the set of segments already occupied by previously
// routed edges; traversing one of these adds a crossing penalty.
func findRoute(m *mesh, startIx, startIy int, startDir direction,
	targetIx, targetIy int, targetApproachDir direction,
	weights Weights, usedSegments map[segment]int,
) (*searchNode, error) {

	targetVertex := m.idx(targetIx, targetIy)
	startVertex := m.idx(startIx, startIy)

	if m.blocked[startVertex] {
		return nil, fmt.Errorf("start vertex is inside a hard obstacle")
	}
	if m.blocked[targetVertex] {
		return nil, fmt.Errorf("target vertex is inside a hard obstacle")
	}

	open := &openSet{}
	heap.Init(open)
	// best[v*5 + dir] = lowest g-cost seen for state (vertex v,
	// direction dir). math.MaxFloat64 sentinel means "not yet seen".
	// Using a slice instead of a map eliminates the dominant mapaccess
	// hotspot in profiles of large graphs.
	nv := len(m.xs) * len(m.ys)
	best := make([]float64, nv*5)
	for i := range best {
		best[i] = math.MaxFloat64
	}
	bestKey := func(v int, d direction) int { return v*5 + int(d) }

	start := &searchNode{ix: startIx, iy: startIy, dir: startDir, g: 0}
	start.f = heuristic(m, startIx, startIy, targetIx, targetIy, weights)
	heap.Push(open, start)
	best[bestKey(startVertex, startDir)] = 0

	for open.Len() > 0 {
		cur := heap.Pop(open).(*searchNode)

		if cur.ix == targetIx && cur.iy == targetIy {
			// Target reached. If the incoming direction doesn't match
			// the required perpendicular approach, that's a bend we
			// didn't charge — so we only accept the state if it does.
			// To keep things simple, we allow any direction but add
			// a final bend cost if it doesn't match.
			if cur.dir != targetApproachDir && cur.dir != dirNone {
				// Check: if current direction is perpendicular to the
				// approach direction, this is a real bend that was
				// unavoidable — charge it. If it's opposite, we came
				// in from the wrong side (should not happen since we
				// filter below, but defend).
				if cur.dir.opposite() != targetApproachDir {
					cur.g += weights.Wbend
				}
			}
			return cur, nil
		}

		stateG := best[bestKey(m.idx(cur.ix, cur.iy), cur.dir)]
		if stateG < cur.g {
			continue // a better state has superseded this one
		}

		// Expand in 4 directions; skip the direction we came from
		// (would reverse) — orthogonal routes never double back.
		for _, nd := range []direction{dirLeft, dirRight, dirUp, dirDown} {
			if cur.dir != dirNone && nd == cur.dir.opposite() {
				continue
			}
			nx, ny, ok := stepNeighbor(m, cur.ix, cur.iy, nd)
			if !ok {
				continue
			}
			nv := m.idx(nx, ny)
			if m.blocked[nv] {
				continue
			}
			if m.segmentBlocked(cur.ix, cur.iy, nx, ny) {
				continue
			}

			// Forbid the final approach to the target from the wrong
			// direction: if we're stepping onto the target, the step
			// direction must equal targetApproachDir.
			if nx == targetIx && ny == targetIy && nd != targetApproachDir {
				continue
			}

			seg := makeSeg(m.idx(cur.ix, cur.iy), nv)
			length := segLength(m, cur.ix, cur.iy, nx, ny)

			// Base length cost, plus average soft cost along the edge.
			stepCost := length * weights.Wlength
			softAvg := (m.softAt[m.idx(cur.ix, cur.iy)] + m.softAt[nv]) / 2
			stepCost += softAvg * length

			// Bend if direction changed.
			if cur.dir != dirNone && cur.dir != nd {
				stepCost += weights.Wbend
			}
			// Crossing if segment already used.
			if crossings, used := usedSegments[seg]; used {
				stepCost += weights.Wcross * float64(crossings)
			}

			ng := cur.g + stepCost
			k := bestKey(nv, nd)
			if best[k] <= ng {
				continue
			}
			best[k] = ng

			nn := &searchNode{
				ix: nx, iy: ny, dir: nd,
				g:      ng,
				f:      ng + heuristic(m, nx, ny, targetIx, targetIy, weights),
				parent: cur, seg: seg, hasSeg: true,
			}
			heap.Push(open, nn)
		}
	}
	return nil, fmt.Errorf("no route found")
}

// heuristic is admissible Manhattan distance in length units. It does
// not include bend cost (which would require knowing the minimum
// number of bends, an extra search in itself) and so remains a valid
// lower bound for A*.
func heuristic(m *mesh, ax, ay, bx, by int, w Weights) float64 {
	dx := math.Abs(m.xs[ax] - m.xs[bx])
	dy := math.Abs(m.ys[ay] - m.ys[by])
	return (dx + dy) * w.Wlength
}

func stepNeighbor(m *mesh, ix, iy int, d direction) (int, int, bool) {
	switch d {
	case dirLeft:
		if ix == 0 {
			return 0, 0, false
		}
		return ix - 1, iy, true
	case dirRight:
		if ix == len(m.xs)-1 {
			return 0, 0, false
		}
		return ix + 1, iy, true
	case dirUp:
		if iy == 0 {
			return 0, 0, false
		}
		return ix, iy - 1, true
	case dirDown:
		if iy == len(m.ys)-1 {
			return 0, 0, false
		}
		return ix, iy + 1, true
	}
	return 0, 0, false
}

func segLength(m *mesh, ax, ay, bx, by int) float64 {
	if ax == bx {
		return math.Abs(m.ys[ay] - m.ys[by])
	}
	return math.Abs(m.xs[ax] - m.xs[bx])
}
