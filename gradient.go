package main
import (
	"fmt"
	"math"
)

func RunGradientDescent(nodes []*Node, distances [][]float64, alpha, lambda, epsilon float64, maxIter int) {
	n := len(nodes)
	// Подготовка структуры данных для CSV
	csvData := [][]string{
		{"Iteration", "Node0_X", "Node0_Y", "Node0_Z"},
	}
	for iter := 0; iter < maxIter; iter++ {
		maxShift := 0.0

		// Временный массив для хранения градиентов на текущем шаге
		gradients := make([]Point, n)

		for i := 0; i < n; i++ {
			var gx, gy, gz float64

			// градиент от соседей 
			for j := 0; j < n; j++ {
				if i == j {
					continue
				}
				
				dMeas := distances[i][j]
				dCalc := Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
				
				// Коэффициент невязки (1 - d_ij / d_calc)
				errRatio := 1.0 - (dMeas / dCalc)

				gx += errRatio * (nodes[i].CurrentCoord.X - nodes[j].CurrentCoord.X)
				gy += errRatio * (nodes[i].CurrentCoord.Y - nodes[j].CurrentCoord.Y)
				gz += errRatio * (nodes[i].CurrentCoord.Z - nodes[j].CurrentCoord.Z)
			}

			// мягкий штраф за отдаление от мнимых координат
			gx += lambda * (nodes[i].CurrentCoord.X - nodes[i].InitialCoord.X)
			gy += lambda * (nodes[i].CurrentCoord.Y - nodes[i].InitialCoord.Y)
			gz += lambda * (nodes[i].CurrentCoord.Z - nodes[i].InitialCoord.Z)

			gradients[i] = Point{X: gx, Y: gy, Z: gz}
		}

		// проверка сходимости
		for i := 0; i < n; i++ {
			shiftX := alpha * gradients[i].X
			shiftY := alpha * gradients[i].Y
			shiftZ := alpha * gradients[i].Z

			nodes[i].CurrentCoord.X -= shiftX
			nodes[i].CurrentCoord.Y -= shiftY
			nodes[i].CurrentCoord.Z -= shiftZ

			//модуль смещения для условия остановки
			shiftMag := math.Sqrt(shiftX*shiftX + shiftY*shiftY + shiftZ*shiftZ)
			if shiftMag > maxShift {
				maxShift = shiftMag
			}
		}
		// Фиксируем шаг в массив для CSV
		csvData = append(csvData, []string{
			fmt.Sprintf("%d", iter),
			fmt.Sprintf("%.4f", nodes[0].CurrentCoord.X),
			fmt.Sprintf("%.4f", nodes[0].CurrentCoord.Y),
			fmt.Sprintf("%.4f", nodes[0].CurrentCoord.Z),
		})
		// Условие остановки
		if maxShift < epsilon {
			fmt.Printf("Градиентный спуск сошелся на итерации %d\n", iter)
			break
		}
	}
	// Запись накопленного лога в файл через utils.go
	if err := WriteHistoryToCSV("mod1_history.csv", csvData); err != nil {
		fmt.Printf("Не удалось записать CSV: %v\n", err)
	} else {
		fmt.Println("Файл mod1_history.csv успешно сгенерирован.")
	}
}
