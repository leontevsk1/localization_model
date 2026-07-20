package analytics

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"

	"vostok/pkg/types"
)

// ComputeGlobalMetrics возвращает RMSE расчётных координат относительно
// реальных и дисперсию невязок дальностей внутри текущей геометрии.
func ComputeGlobalMetrics(nodes []*types.Node, realCoords []types.Point, distances [][]float64) (float64, float64) {
	n := len(nodes)

	var sumSqCoordDist float64
	for i := 0; i < n; i++ {
		sumSqCoordDist += math.Pow(nodes[i].CurrentCoord.X-realCoords[i].X, 2) +
			math.Pow(nodes[i].CurrentCoord.Y-realCoords[i].Y, 2) +
			math.Pow(nodes[i].CurrentCoord.Z-realCoords[i].Z, 2)
	}
	rmseCoords := math.Sqrt(sumSqCoordDist / float64(n))

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

	return rmseCoords, varianceDist
}

// SummarizeEKFSpread читает CSV с историей EKF и возвращает максимальный
// размах координаты (max-min по итерациям), её имя из заголовка и средний
// размах по всем координатам.
func SummarizeEKFSpread(filename string) (float64, string, float64, error) {
	file, err := os.Open(filename)
	if err != nil {
		return 0, "", 0, fmt.Errorf("ошибка открытия файла: %v", err)
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return 0, "", 0, fmt.Errorf("ошибка чтения CSV: %v", err)
	}

	if len(records) < 2 {
		return 0, "", 0, fmt.Errorf("недостаточно данных в файле для анализа")
	}

	headers := records[0]
	numCols := len(headers)
	numRows := len(records) - 1

	if numCols < 2 {
		return 0, "", 0, fmt.Errorf("в файле нет координатных колонок")
	}

	maxSpread := 0.0
	maxLabel := headers[1]
	sumSpread := 0.0
	scannedCols := 0

	for c := 1; c < numCols; c++ {
		// RealX/RealY/RealZ — справочные колонки для графиков, не оценки фильтра
		if strings.HasPrefix(headers[c], "Real") {
			continue
		}
		minVal, maxVal := 0.0, 0.0
		for r := 0; r < numRows; r++ {
			val, err := strconv.ParseFloat(records[r+1][c], 64)
			if err != nil {
				return 0, "", 0, fmt.Errorf("ошибка парсинга числа в строке %d, колонке %d: %v", r+1, c, err)
			}
			if r == 0 {
				minVal, maxVal = val, val
				continue
			}
			if val < minVal {
				minVal = val
			}
			if val > maxVal {
				maxVal = val
			}
		}

		spread := maxVal - minVal
		sumSpread += spread
		scannedCols++
		if spread > maxSpread {
			maxSpread = spread
			maxLabel = headers[c]
		}
	}

	if scannedCols == 0 {
		return 0, "", 0, fmt.Errorf("в файле нет координатных колонок")
	}

	meanSpread := sumSpread / float64(scannedCols)
	return maxSpread, maxLabel, meanSpread, nil
}
