package intersekt

import (
	"fmt"
	"github.com/prl900/proj4go"
	"math"
)

//type Point = proj4go.Point
type Point struct {
	X, Y float64
}

type Interval struct {
	Lo, Hi float64
}

func (i Interval) Contains(oi Interval) bool {
	return i.Lo <= oi.Lo && i.Hi >= oi.Hi
}

func (i Interval) Intersects(oi Interval) bool {
	if i.Lo <= oi.Lo {
		return oi.Lo <= i.Hi && oi.Lo <= oi.Hi // oi.Lo ∈ i and oi is not empty
	}
	return i.Lo <= oi.Hi && i.Lo <= i.Hi // i.Lo ∈ oi and i is not empty
}

type Rect struct {
	X, Y Interval
}

func (r Rect) Contains(other Rect) bool {
	return r.X.Contains(other.X) && r.Y.Contains(other.Y)
}

func (r Rect) Intersects(other Rect) bool {
	return r.X.Intersects(other.X) && r.Y.Intersects(other.Y)
}

// Checks if three points are Counter Clockwise
func ccw(a, b, c Point) bool {
	return (c.Y-a.Y)*(b.X-a.X) > (b.Y-a.Y)*(c.X-a.X)
}

// Return true if line segments AB and CD intersect
func SegmentIntersect(a, b, c, d Point) bool {
	return ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d)
}

// Return true if line segments AB and CD intersect
func Distance(a, b Point) float64 {
	return math.Sqrt(math.Pow((a.X-b.X), 2) + math.Pow((a.Y-b.Y), 2))
}

type Loop struct {
	Vertices []Point
	Bound    Rect
	V, H     int
}

func (l *Loop) Segmentize(maxDist float64) *Loop {
	newPts := []Point{}

	fst := l.Vertices[0]
	newPts = append(newPts, fst)
	for _, scd := range l.Vertices[1:] {
		if n := int(Distance(fst, scd) / maxDist); n > 1 {
			for i := 1; i < n; i++ {
				pt := Point{}
				pt.X = fst.X + float64(i)*(scd.X-fst.X)/float64(n)
				pt.Y = fst.Y + float64(i)*(scd.Y-fst.Y)/float64(n)
				newPts = append(newPts, pt)
			}
		}
		newPts = append(newPts, scd)
		fst = scd
	}
	fst = l.Vertices[len(l.Vertices)-1]
	scd := l.Vertices[0]
	if n := int(Distance(fst, scd) / maxDist); n > 1 {
		for i := 1; i < n; i++ {
			pt := Point{}
			pt.X = fst.X + float64(i)*(scd.X-fst.X)/float64(n)
			pt.Y = fst.Y + float64(i)*(scd.Y-fst.Y)/float64(n)
			newPts = append(newPts, pt)
		}
	}

	sl, _ := NewLoop(newPts, l.V, l.H)
	return sl
}

func (l *Loop) Transform(projStr string) *Loop {
	pts := l.Vertices

	ppts := make([]proj4go.Point, len(pts))
	for i, p := range pts {
		ppts[i].X = p.X
		ppts[i].Y = p.Y
	}

	proj4go.Forwards(projStr, ppts)

	for i, p := range ppts {
		pts[i].X = p.X
		pts[i].Y = p.Y
	}

	nl, _ := NewLoop(pts, l.V, l.H)
	return nl
}

func (l *Loop) SharedVertex(o *Loop) bool {
	for _, lv := range l.Vertices {
		for _, ov := range o.Vertices {
			if lv == ov {
				return true
			}
		}
	}

	return false
}

func (l *Loop) EdgesCross(o *Loop) bool {
	lFirst := l.Vertices[len(l.Vertices)-1]
	for _, lSecond := range l.Vertices {
		oFirst := o.Vertices[len(o.Vertices)-1]
		for _, oSecond := range o.Vertices {
			if SegmentIntersect(lFirst, lSecond, oFirst, oSecond) {
				return true
			}

			oFirst = oSecond
		}
		lFirst = lSecond
	}

	return false
}

// Intersects reports whether the region contained by this loop intersects the region
// contained by the other loop.
func (l *Loop) Intersects(o *Loop) bool {
	// Given two loops, A and B, A.Intersects(B) if and only if !A.Complement().Contains(B).
	//
	// This code is similar to Contains, but is optimized for the case
	// where both loops enclose less than half of the sphere.
	// 1. Their Bounds do not intersect
	if !l.Bound.Intersects(o.Bound) {
		return false
	}

	// Check whether there are any edge crossings, and also check the loop
	// relationship at any shared vertices.
	if l.EdgesCross(o) {
		return true
	}

	if l.SharedVertex(o) {
		return true
	}

	// Since there are no edge intersections or shared vertices, the loops
	// intersect only if A contains B, B contains A, or the two loops contain
	// each other's boundaries.  These checks are usually cheap because of the
	// bounding box preconditions.  Note that neither loop is empty (because of
	// the bounding box check above), so it is safe to access vertex(0).

	// Check whether A contains B, or A and B contain each other's boundaries.
	// (Note that A contains all the vertices of B in either case.)
	if l.Bound.Contains(o.Bound) || o.Bound.Contains(l.Bound) {
		return true
	}

	return false
}

func NewLoop(pts []Point, x, y int) (*Loop, error) {

	if len(pts) < 3 || len(pts) < 3 {
		return nil, fmt.Errorf("Loop has to contain at least 3 points")
	}

	rect := Rect{Interval{math.MaxFloat64, -math.MaxFloat64}, Interval{math.MaxFloat64, -math.MaxFloat64}}

	for _, v := range pts {
		if v.X > rect.X.Hi {
			rect.X.Hi = v.X
		}
		if v.X < rect.X.Lo {
			rect.X.Lo = v.X
		}
		if v.Y > rect.Y.Hi {
			rect.Y.Hi = v.Y
		}
		if v.Y < rect.Y.Lo {
			rect.Y.Lo = v.Y
		}
	}

	return &Loop{pts, rect, x, y}, nil
}
