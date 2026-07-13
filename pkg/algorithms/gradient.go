package algorithms

import (
	"fmt"
	"math"

	"vostok/pkg/io"
	"vostok/pkg/types"
)

func RunGradientDescent(nodes []*types.Node, measurements []types.Measurement, alpha, lambda, epsilon float64, maxIter int, appendLog bool) {
	n := len(nodes)

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
		{"Iteration", fmt.Sprintf("Node%d_X", logNode), fmt.Sprintf("Node%d_Y", logNode), fmt.Sprintf("Node%d_Z", logNode)},
	}

	for iter := 0; iter < maxIter; iter++ {
		maxShift := 0.0

		// Временный массив для хранения градиентов на текущем шаге
		gradients := make([]types.Point, n)

		for _, m := range measurements {
			i, j := m.From, m.To
			if nodes[i].IsAnchor {
				continue
			}

			dCalc := types.Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
			errRatio := 1.0 - (m.Value / dCalc)

			gradients[i].X += errRatio * (nodes[i].CurrentCoord.X - nodes[j].CurrentCoord.X)
			gradients[i].Y += errRatio * (nodes[i].CurrentCoord.Y - nodes[j].CurrentCoord.Y)
			gradients[i].Z += errRatio * (nodes[i].CurrentCoord.Z - nodes[j].CurrentCoord.Z)
		}

		// Регуляризация к InitialCoord
		for i := 0; i < n; i++ {
			if nodes[i].IsAnchor {
				continue
			}

			gradients[i].X += lambda * (nodes[i].CurrentCoord.X - nodes[i].InitialCoord.X)
			gradients[i].Y += lambda * (nodes[i].CurrentCoord.Y - nodes[i].InitialCoord.Y)
			gradients[i].Z += lambda * (nodes[i].CurrentCoord.Z - nodes[i].InitialCoord.Z)
		}

		// применение шага
		for i := 0; i < n; i++ {
			if nodes[i].IsAnchor {
				continue // анкер не двигаем, в maxShift тоже не учитываем
			}

			shiftX := alpha * gradients[i].X
			shiftY := alpha * gradients[i].Y
			shiftZ := alpha * gradients[i].Z

			nodes[i].CurrentCoord.X -= shiftX
			nodes[i].CurrentCoord.Y -= shiftY
			nodes[i].CurrentCoord.Z -= shiftZ

			// модуль смещения для условия остановки
			shiftMag := math.Sqrt(shiftX*shiftX + shiftY*shiftY + shiftZ*shiftZ)
			if shiftMag > maxShift {
				maxShift = shiftMag
			}
		}

		// Фиксируем шаг в массив для CSV
		csvData = append(csvData, []string{
			fmt.Sprintf("%d", iter),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.X),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.Y),
			fmt.Sprintf("%.4f", nodes[logNode].CurrentCoord.Z),
		})

		// Условие остановки
		if maxShift < epsilon {
			fmt.Printf("Градиентный спуск сошелся на итерации %d\n", iter)
			break
		}
	}

	// Запись накопленного лога в файл через pkg/io
	if err := io.WriteHistoryToCSV("mod1_history.csv", csvData, appendLog); err != nil {
		fmt.Printf("Не удалось записать CSV: %v\n", err)
	} else {
		fmt.Println("Файл mod1_history.csv успешно сгенерирован.")
	}
}
