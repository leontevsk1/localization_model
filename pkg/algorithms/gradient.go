package algorithms

import (
	"fmt"
	"math"

	"vostok/pkg/io"
	"vostok/pkg/types"
)

// Веса построены на отношении дисперсий источников: невязки дальномеров имеют
// вес 1, привязка узла к InitialCoord — distErr²/Uncertainty². Коррекция сама
// затухает, когда дальномеры хуже текущей оценки узла, и наоборот.
// Возвращает номер итерации, на которой остановился (maxIter при исчерпании лимита).
func RunGradientDescent(nodes []*types.Node, measurements []types.Measurement, alpha, distErr, epsilon float64, maxIter int, appendLog bool) int {
	n := len(nodes)

	rangeVar := math.Max(distErr*distErr, 1e-12)
	priorWeight := make([]float64, n)
	for i, node := range nodes {
		if node.IsAnchor {
			continue
		}
		priorWeight[i] = rangeVar / math.Max(node.Uncertainty*node.Uncertainty, 1e-12)
	}

	cost := func() float64 {
		var c float64
		for _, m := range measurements {
			d := types.Distance(nodes[m.From].CurrentCoord, nodes[m.To].CurrentCoord)
			r := d - m.Value
			c += r * r
		}
		for i := 0; i < n; i++ {
			if nodes[i].IsAnchor {
				continue
			}
			dx := nodes[i].CurrentCoord.X - nodes[i].InitialCoord.X
			dy := nodes[i].CurrentCoord.Y - nodes[i].InitialCoord.Y
			dz := nodes[i].CurrentCoord.Z - nodes[i].InitialCoord.Z
			c += priorWeight[i] * (dx*dx + dy*dy + dz*dz)
		}
		return c
	}

	// Для лога берём первый подвижный узел — Node0 может быть анкером
	// (неподвижен по определению), тогда график сходимости был бы плоским.
	logNode := 0
	for i, node := range nodes {
		if !node.IsAnchor {
			logNode = i
			break
		}
	}

	csvData := [][]string{
		{"Iteration", fmt.Sprintf("Node%d_X", logNode), fmt.Sprintf("Node%d_Y", logNode), fmt.Sprintf("Node%d_Z", logNode), "RealX", "RealY", "RealZ"},
	}

	alphaCur := alpha
	prevCost := cost()
	snapshot := make([]types.Point, n)
	stoppedAt := maxIter

	for iter := 0; iter < maxIter; iter++ {
		gradients := make([]types.Point, n)

		for _, m := range measurements {
			i, j := m.From, m.To

			dCalc := types.Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
			if dCalc < 1e-9 {
				dCalc = 1e-9
			}
			errRatio := 1.0 - (m.Value / dCalc)

			ux := nodes[i].CurrentCoord.X - nodes[j].CurrentCoord.X
			uy := nodes[i].CurrentCoord.Y - nodes[j].CurrentCoord.Y
			uz := nodes[i].CurrentCoord.Z - nodes[j].CurrentCoord.Z

			if !nodes[i].IsAnchor {
				gradients[i].X += errRatio * ux
				gradients[i].Y += errRatio * uy
				gradients[i].Z += errRatio * uz
			}
			if !nodes[j].IsAnchor {
				gradients[j].X -= errRatio * ux
				gradients[j].Y -= errRatio * uy
				gradients[j].Z -= errRatio * uz
			}
		}

		// Привязка к InitialCoord с весом, обратным неопределённости узла
		for i := 0; i < n; i++ {
			if nodes[i].IsAnchor {
				continue
			}

			gradients[i].X += priorWeight[i] * (nodes[i].CurrentCoord.X - nodes[i].InitialCoord.X)
			gradients[i].Y += priorWeight[i] * (nodes[i].CurrentCoord.Y - nodes[i].InitialCoord.Y)
			gradients[i].Z += priorWeight[i] * (nodes[i].CurrentCoord.Z - nodes[i].InitialCoord.Z)
		}

		for i := 0; i < n; i++ {
			snapshot[i] = nodes[i].CurrentCoord
		}

		maxShift := 0.0
		for i := 0; i < n; i++ {
			if nodes[i].IsAnchor {
				continue
			}

			shiftX := alphaCur * gradients[i].X
			shiftY := alphaCur * gradients[i].Y
			shiftZ := alphaCur * gradients[i].Z

			nodes[i].CurrentCoord.X -= shiftX
			nodes[i].CurrentCoord.Y -= shiftY
			nodes[i].CurrentCoord.Z -= shiftZ

			shiftMag := math.Sqrt(shiftX*shiftX + shiftY*shiftY + shiftZ*shiftZ)
			if shiftMag > maxShift {
				maxShift = shiftMag
			}
		}

		// Backtracking: расходящийся или ухудшающий шаг откатываем и уменьшаем
		newCost := cost()
		if math.IsNaN(newCost) || math.IsInf(newCost, 0) || newCost > prevCost {
			for i := 0; i < n; i++ {
				nodes[i].CurrentCoord = snapshot[i]
			}
			alphaCur *= 0.5
			if alphaCur < alpha*1e-12 {
				fmt.Printf("Градиентный спуск остановлен: шаг выродился на итерации %d\n", iter)
				stoppedAt = iter
				break
			}
			continue
		}
		prevCost = newCost
		alphaCur = math.Min(alphaCur*1.1, alpha)

		csvData = append(csvData, []string{
			fmt.Sprintf("%d", iter),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.X),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.Y),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.Z),
			fmt.Sprintf("%.4f", nodes[logNode].RealCoord.X),
			fmt.Sprintf("%.4f", nodes[logNode].RealCoord.Y),
			fmt.Sprintf("%.4f", nodes[logNode].RealCoord.Z),
		})

		if maxShift < epsilon {
			stoppedAt = iter
			break
		}
	}

	// Запись накопленного лога в файл через pkg/io
	if err := io.WriteHistoryToCSV("mod1_history.csv", csvData, appendLog); err != nil {
		fmt.Printf("Не удалось записать CSV: %v\n", err)
	}

	return stoppedAt
}
