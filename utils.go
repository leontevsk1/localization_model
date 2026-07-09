package main

import (
	"encoding/csv"
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"

	"github.com/pelletier/go-toml/v2"
)
// === СТРУКТУРЫ ===

type Config struct {
	Network struct {
		Nodes    int     `toml:"nodes"`
		SpaceMin float64 `toml:"space_min"`
		SpaceMax float64 `toml:"space_max"`
		GpsErr   float64 `toml:"gps_err"`
		DistErr  float64 `toml:"dist_err"`
	} `toml:"network"`

	Hyperparams struct {
		Alpha        float64 `toml:"alpha"`
		Lambda       float64 `toml:"lambda"`
		Eps          float64 `toml:"eps"`
		MaxIter      int     `toml:"max_iter"`
		EkfIter      int     `toml:"ekf_iter"`
		AnchorsCount int     `toml:"anchors_count"`

		Q float64 `toml:"q"`
		R float64 `toml:"r"`
	} `toml:"hyperparams"`
}

// Point описывает вектор
type Point struct {
	X, Y, Z float64
}

// Node описывает объект сети
type Node struct {
	ID           int
	InitialCoord Point // начальные мнимые координаты
	CurrentCoord Point // текущие оптимизированные координаты
	IsAnchor     bool
}

// === ФУНКЦИИ ===

// Distance считает евклидово расстояние между двумя точками
func Distance(p1, p2 Point) float64 {
	return math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2) + math.Pow(p1.Z-p2.Z, 2))
}

// buildMap генерирует случайную сеть узлов и матрицу расстояний между ними.
// Возвращает список узлов, реальные координаты и матрицу измеренных дальностей.
func buildMap(n int, sMin, sMax, gErr, dErr float64, anchorsCount int, rng *rand.Rand) ([]*Node, []Point, [][]float64) {
	// [Генерация координат]
	realCoords := make([]Point, n)
	nodes := make([]*Node, n)
	for i := 0; i < n; i++ {
		rX := sMin + rng.Float64()*(sMax-sMin)
		rY := sMin + rng.Float64()*(sMax-sMin)
		rZ := sMin + rng.Float64()*(sMax-sMin)
		realCoords[i] = Point{X: rX, Y: rY, Z: rZ}

		isAnchor := i < anchorsCount

		if isAnchor {
			// Анкер: координаты точные, без шума GPS
			nodes[i] = &Node{
				ID:           i,
				InitialCoord: realCoords[i],
				CurrentCoord: realCoords[i],
				IsAnchor:     true,
			}
			continue
		}

		dx := (rng.Float64()*2 - 1.0) * gErr
		dy := (rng.Float64()*2 - 1.0) * gErr
		dz := (rng.Float64()*2 - 1.0) * gErr

		nodes[i] = &Node{
			ID:           i,
			InitialCoord: Point{X: rX + dx, Y: rY + dy, Z: rZ + dz},
			CurrentCoord: Point{X: rX + dx, Y: rY + dy, Z: rZ + dz},
			IsAnchor:     false,
		}
	}

	// [Генерация матрицы расстояний]
	distances := make([][]float64, n)
	for i := range distances {
		distances[i] = make([]float64, n)
	}
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			dx := realCoords[i].X - realCoords[j].X
			dy := realCoords[i].Y - realCoords[j].Y
			dz := realCoords[i].Z - realCoords[j].Z
			trueDist := math.Sqrt(dx*dx + dy*dy + dz*dz)
			noise := (rng.Float64()*2 - 1.0) * dErr
			measDist := trueDist + noise
			if measDist < 0 {
				measDist = 0.001
			}
			distances[i][j] = measDist
			distances[j][i] = measDist
		}
	}

	return nodes, realCoords, distances
}

// LoadConfigFromFile читает и парсит TOML-конфиг
func LoadConfigFromFile(filepath string) (*Config, error) {
	file, err := os.ReadFile(filepath)
	if err != nil {
		return nil, err
	}
	var cfg Config
	err = toml.Unmarshal(file, &cfg)
	if err != nil {
		return nil, err
	}
	return &cfg, nil
}

// WriteHistoryToCSV — утилита, которую вы можете вызвать из gradient.go
func WriteHistoryToCSV(filename string, recordFields [][]string) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	for _, row := range recordFields {
		if err := writer.Write(row); err != nil {
			return err
		}
	}
	return nil
}

// SaveEKFHistory сохраняет детальную историю итераций EKF для всех узлов сети.
func SaveEKFHistory(filename string, history [][]float64, nodeCount int) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	// Инициализируем slice строкой "Iteration" до входа в цикл
	header := []string{"Iteration"}
	
	for i := 0; i < nodeCount; i++ {
		header = append(header, 
			fmt.Sprintf("Node%d_X", i), 
			fmt.Sprintf("Node%d_Y", i), 
			fmt.Sprintf("Node%d_Z", i),
		)
	}
	
	if err := writer.Write(header); err != nil {
		return err
	}

	for _, row := range history {
		strRow := make([]string, len(row))
		for i, val := range row {
			if i == 0 {
				strRow[i] = fmt.Sprintf("%.0f", val) // Номер итерации без дробей
			} else {
				strRow[i] = fmt.Sprintf("%.6f", val) // Координаты с высокой точностью
			}
		}
		if err := writer.Write(strRow); err != nil {
			return err
		}
	}
	return nil
}


// printMetrics считает и выводит на экран RMSE координат и дисперсию невязок
func printGradientMetrics(nodes []*Node, realCoords []Point, distances [][]float64) {
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
			dCalc := Distance(nodes[i].CurrentCoord, nodes[j].CurrentCoord)
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
func AnalyzeEKFHistory(filename string, nodes []*Node, movable []int, realCoords []Point, distances [][]float64) error {
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
		currentCoords := make([]Point, numNodes)
		for i, node := range nodes {
			if node.IsAnchor {
				currentCoords[i] = node.CurrentCoord
			}
		}
		for k, nodeIdx := range movable {
			currentCoords[nodeIdx] = Point{
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
