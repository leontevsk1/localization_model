package analytics

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"

	"vostok/pkg/types"
)

// PrintGradientMetrics считает и выводит на экран RMSE координат и дисперсию невязок
func PrintGradientMetrics(nodes []*types.Node, realCoords []types.Point, distances [][]float64) {
	n := len(nodes)

	// RMSE рассчетных и реальных координат
	var sumSqCoordDist float64
	for i := 0; i < n; i++ {
		sumSqCoordDist += math.Pow(nodes[i].CurrentCoord.X-realCoords[i].X, 2) +
			math.Pow(nodes[i].CurrentCoord.Y-realCoords[i].Y, 2) +
			math.Pow(nodes[i].CurrentCoord.Z-realCoords[i].Z, 2)
	}
	rmseCoords := math.Sqrt(sumSqCoordDist / float64(n))

	// ошибки внутри текущей геометрии
	var sumSqDistError float64
	var count float64
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			dCalc := types.Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
			dMeas := distances[i][j]
			sumSqDistError += math.Pow(dCalc-dMeas, 2)
			count++
		}
	}
	varianceDist := sumSqDistError / count

	fmt.Printf("Среднеквадратичная ошибка координат (RMSE до Real): %.5f м\n", rmseCoords)
	fmt.Printf("Дисперсия внутренних невязок дальностей:          %.5f м²\n", varianceDist)
}

// AnalyzeEKFHistory читает CSV с историей EKF и выводит статистику сходимости.
// movable — те же индексы узлов, что использовались в RunEKF (порядок важен —
// он определяет, какой колонке CSV соответствует какой узел).
func AnalyzeEKFHistory(filename string, nodes []*types.Node, movable []int, realCoords []types.Point, distances [][]float64) error {
	file, err := os.Open(filename)
	if err != nil {
		return fmt.Errorf("ошибка открытия файла: %v", err)
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return fmt.Errorf("ошибка чтения CSV: %v", err)
	}

	if len(records) < 2 {
		return fmt.Errorf("недостаточно данных в файле для анализа")
	}

	headers := records[0]
	numCols := len(headers)
	numRows := len(records) - 1
	numNodes := len(nodes) // ВСЕ узлы сети, включая анкеров — не только те, что в CSV

	columns := make([][]float64, numCols)
	for i := 0; i < numCols; i++ {
		columns[i] = make([]float64, numRows)
	}

	for r := 0; r < numRows; r++ {
		for c := 0; c < numCols; c++ {
			val, err := strconv.ParseFloat(records[r+1][c], 64)
			if err != nil {
				return fmt.Errorf("ошибка парсинга числа в строке %d, колонке %d: %v", r+1, c, err)
			}
			columns[c][r] = val
		}
	}

	rmseHistory := make([]float64, numRows)
	varianceHistory := make([]float64, numRows)

	for r := 0; r < numRows; r++ {
		// Реконструируем координаты ВСЕХ узлов на итерации r:
		// анкеры — статичны, берём их текущие (=истинные) координаты напрямую;
		// подвижные — берём из CSV по той же карте movable, что использовалась в RunEKF.
		currentCoords := make([]types.Point, numNodes)
		for i, node := range nodes {
			if node.IsAnchor {
				currentCoords[i] = node.CurrentCoord
			}
		}
		for k, nodeIdx := range movable {
			currentCoords[nodeIdx] = types.Point{
				X: columns[1+k*3][r],
				Y: columns[2+k*3][r],
				Z: columns[3+k*3][r],
			}
		}

		var sumSqCoordDist float64
		for i := 0; i < numNodes; i++ {
			sumSqCoordDist += math.Pow(currentCoords[i].X-realCoords[i].X, 2) +
				math.Pow(currentCoords[i].Y-realCoords[i].Y, 2) +
				math.Pow(currentCoords[i].Z-realCoords[i].Z, 2)
		}
		rmseHistory[r] = math.Sqrt(sumSqCoordDist / float64(numNodes))

		var sumSqDistError float64
		var count float64
		for i := 0; i < numNodes; i++ {
			for j := i + 1; j < numNodes; j++ {
				dx := currentCoords[i].X - currentCoords[j].X
				dy := currentCoords[i].Y - currentCoords[j].Y
				dz := currentCoords[i].Z - currentCoords[j].Z
				dCalc := math.Sqrt(dx*dx + dy*dy + dz*dz)
				dMeas := distances[i][j]

				sumSqDistError += math.Pow(dCalc-dMeas, 2)
				count++
			}
		}
		varianceHistory[r] = sumSqDistError / count
	}

	fmt.Println("\n--- Анализ сходимости координат EKF ---")
	fmt.Printf("%-10s | %-12s | %-12s | %-12s | %-12s\n", "Координата", "Старт", "Финиш", "Смещение (Δ)", "Размах (Max-Min)")
	fmt.Println("-------------------------------------------------------------------------")

	for c := 1; c < numCols; c++ {
		colData := columns[c]
		startVal := colData[0]
		finishVal := colData[numRows-1]
		delta := finishVal - startVal

		minVal, maxVal := colData[0], colData[0]
		for _, val := range colData {
			if val < minVal {
				minVal = val
			}
			if val > maxVal {
				maxVal = val
			}
		}
		spread := maxVal - minVal

		fmt.Printf("%-10s | %12.5f | %12.5f | %12.5f | %12.5f\n",
			headers[c], startVal, finishVal, delta, spread)
	}

	fmt.Println("\n--- Глобальные метрики EKF ---")
	fmt.Printf("%-15s | %-12s | %-12s | %-12s\n", "Метрика", "Старт", "Финиш", "Изменение (Δ)")
	fmt.Println("-------------------------------------------------------------------------")

	startRMSE := rmseHistory[0]
	finishRMSE := rmseHistory[numRows-1]
	fmt.Printf("%-15s | %12.5f | %12.5f | %12.5f\n", "RMSE (м)", startRMSE, finishRMSE, finishRMSE-startRMSE)

	startVar := varianceHistory[0]
	finishVar := varianceHistory[numRows-1]
	fmt.Printf("%-15s | %12.5f | %12.5f | %12.5f\n", "Дисперсия (м²)", startVar, finishVar, finishVar-startVar)
	fmt.Println("-------------------------------------------------------------------------")

	return nil
}
