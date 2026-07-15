package algorithms

import (
	"gonum.org/v1/gonum/mat"

	"vostok/pkg/types"
)

// KabschAlign возвращает копию points, совмещённую с reference оптимальным
// поворотом и сдвигом (без масштаба): минимизирует сумму квадратов расстояний
// между парами точек. Отражение исключено поправкой знака детерминанта.
func KabschAlign(points, reference []types.Point) []types.Point {
	n := len(points)
	aligned := make([]types.Point, n)

	if n < 3 {
		copy(aligned, points)
		return aligned
	}

	var pcX, pcY, pcZ, rcX, rcY, rcZ float64
	for i := 0; i < n; i++ {
		pcX += points[i].X
		pcY += points[i].Y
		pcZ += points[i].Z
		rcX += reference[i].X
		rcY += reference[i].Y
		rcZ += reference[i].Z
	}
	fn := float64(n)
	pcX, pcY, pcZ = pcX/fn, pcY/fn, pcZ/fn
	rcX, rcY, rcZ = rcX/fn, rcY/fn, rcZ/fn

	H := mat.NewDense(3, 3, nil)
	for i := 0; i < n; i++ {
		p := [3]float64{points[i].X - pcX, points[i].Y - pcY, points[i].Z - pcZ}
		q := [3]float64{reference[i].X - rcX, reference[i].Y - rcY, reference[i].Z - rcZ}
		for r := 0; r < 3; r++ {
			for c := 0; c < 3; c++ {
				H.Set(r, c, H.At(r, c)+p[r]*q[c])
			}
		}
	}

	var svd mat.SVD
	if ok := svd.Factorize(H, mat.SVDFull); !ok {
		copy(aligned, points)
		return aligned
	}
	var U, V mat.Dense
	svd.UTo(&U)
	svd.VTo(&V)

	var VUt mat.Dense
	VUt.Mul(&V, U.T())
	detSign := 1.0
	if mat.Det(&VUt) < 0 {
		detSign = -1.0
	}

	D := mat.NewDense(3, 3, nil)
	D.Set(0, 0, 1)
	D.Set(1, 1, 1)
	D.Set(2, 2, detSign)

	var VD, R mat.Dense
	VD.Mul(&V, D)
	R.Mul(&VD, U.T())

	for i := 0; i < n; i++ {
		px := points[i].X - pcX
		py := points[i].Y - pcY
		pz := points[i].Z - pcZ
		aligned[i] = types.Point{
			X: R.At(0, 0)*px + R.At(0, 1)*py + R.At(0, 2)*pz + rcX,
			Y: R.At(1, 0)*px + R.At(1, 1)*py + R.At(1, 2)*pz + rcY,
			Z: R.At(2, 0)*px + R.At(2, 1)*py + R.At(2, 2)*pz + rcZ,
		}
	}

	return aligned
}

// AlignMovableToReference совмещает CurrentCoord подвижных узлов с опорными
// координатами (gauge-фиксация после оптимизации: убирает произвольный
// сдвиг/поворот каркаса, внесённый оптимизатором). reference индексируется
// по номерам узлов, анкеры игнорируются.
func AlignMovableToReference(nodes []*types.Node, reference []types.Point) {
	movable := make([]int, 0, len(nodes))
	for i, node := range nodes {
		if !node.IsAnchor {
			movable = append(movable, i)
		}
	}

	points := make([]types.Point, len(movable))
	refs := make([]types.Point, len(movable))
	for k, idx := range movable {
		points[k] = nodes[idx].CurrentCoord
		refs[k] = reference[idx]
	}

	aligned := KabschAlign(points, refs)
	for k, idx := range movable {
		nodes[idx].CurrentCoord = aligned[k]
	}
}
