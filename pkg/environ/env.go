package environ

import (
	"math"
	"math/rand"

	"vostok/pkg/types"
)

// BuildMap генерирует случайную сеть узлов и матрицу расстояний между ними.
// Возвращает список узлов, реальные координаты и матрицу измеренных дальностей.
func BuildMap(n int, sMin, sMax, gErr, dErr float64, anchorsCount int, rng *rand.Rand) ([]*types.Node, []types.Point, [][]float64) {
	// [Генерация координат]
	realCoords := make([]types.Point, n)
	nodes := make([]*types.Node, n)
	for i := 0; i < n; i++ {
		rX := sMin + rng.Float64()*(sMax-sMin)
		rY := sMin + rng.Float64()*(sMax-sMin)
		rZ := sMin + rng.Float64()*(sMax-sMin)
		realCoords[i] = types.Point{X: rX, Y: rY, Z: rZ}

		isAnchor := i < anchorsCount

		dx := (rng.Float64()*2 - 1.0) * gErr
		dy := (rng.Float64()*2 - 1.0) * gErr
		dz := (rng.Float64()*2 - 1.0) * gErr

		if isAnchor {
			// Анкер: координаты точные, без шума GPS
			nodes[i] = &types.Node{
				ID:            i,
				InitialCoord:  realCoords[i],
				CurrentCoord:  realCoords[i],
				RealCoord:     realCoords[i],
				BelievedCoord: realCoords[i],
				Uncertainty:   0.0,
				TickCounter:   0,
				IsAnchor:      true,
			}
			continue
		}

		noisyCoord := types.Point{X: rX + dx, Y: rY + dy, Z: rZ + dz}
		nodes[i] = &types.Node{
			ID:            i,
			InitialCoord:  noisyCoord,
			CurrentCoord:  noisyCoord,
			RealCoord:     realCoords[i],
			BelievedCoord: noisyCoord,
			Uncertainty:   gErr,
			TickCounter:   0,
			IsAnchor:      false,
		}
	}

	distances := buildDistanceMatrix(realCoords, dErr, rng)

	return nodes, realCoords, distances
}

// RecomputeDistances пересчитывает матрицу измеренных расстояний по текущим RealCoord с добавлением шума. Вызывается на каждом тике при временной динамике.
func RecomputeDistances(nodes []*types.Node, distErr float64, rng *rand.Rand) [][]float64 {
	realCoords := make([]types.Point, len(nodes))
	for i, node := range nodes {
		realCoords[i] = node.RealCoord
	}
	return buildDistanceMatrix(realCoords, distErr, rng)
}

func buildDistanceMatrix(coords []types.Point, distErr float64, rng *rand.Rand) [][]float64 {
	n := len(coords)
	distances := make([][]float64, n)
	for i := range distances {
		distances[i] = make([]float64, n)
	}

	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			dx := coords[i].X - coords[j].X
			dy := coords[i].Y - coords[j].Y
			dz := coords[i].Z - coords[j].Z
			trueDist := math.Sqrt(dx*dx + dy*dy + dz*dz)
			noise := (rng.Float64()*2 - 1.0) * distErr
			measDist := trueDist + noise
			if measDist < 0 {
				measDist = 0.001
			}
			distances[i][j] = measDist
			distances[j][i] = measDist
		}
	}

	return distances
}
