package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"os"
	"time"

	"vostok/pkg/algorithms"
	"vostok/pkg/analytics"
	"vostok/pkg/environ"
	vio "vostok/pkg/io"
	"vostok/pkg/types"
)

func main() {
	configPath := flag.String("config", "", "Путь к TOML-файлу конфигурации сценария")

	// Стандартные CLI флаги (как значения по умолчанию)
	numNodes := flag.Int("nodes", 4, "Количество объектов (узлов) в сети")
	spaceMin := flag.Float64("space-min", 0.0, "Минимальная граница пространства")
	spaceMax := flag.Float64("space-max", 100.0, "Максимальная граница пространства")
	gpsErr := flag.Float64("gps-err", 3.0, "Максимальная абсолютная ошибка GPS (мнимых координат)")
	distErr := flag.Float64("dist-err", 0.5, "Максимальная абсолютная ошибка дальномеров")
	kFlag := flag.Int("k", -1, "Число ближайших целей на узел (дефолт: n-1 для полносвязного графа)")

	alpha := flag.Float64("alpha", 0.02, "Скорость обучения градиентного спуска")
	lambda := flag.Float64("lambda", 0.15, "Вес мягкой регуляризации")
	epsilon := flag.Float64("eps", 1e-5, "Критерий останова градиентного спуска")
	maxIter := flag.Int("max-iter", 2000, "Лимит итераций градиентного спуска")
	ekfIter := flag.Int("ekf-iter", 5, "Количество итераций EKF")
	ekfQFlag := flag.Float64("q", 0.001, "Шум процесса EKF")
	ekfRFlag := flag.Float64("r", 0.1, "Шум измерений EKF")
	anchorsCountFlag := flag.Int("anchors", 0, "Количество анкерных узлов с точными координатами")
	flag.Parse()

	var (
		n int
		sMin, sMax, gErr, dErr float64
		gAlpha, gLambda, gEps float64
		gMaxIter, eIter int
		anchorsCount int
		topologyK int

		ekfQ float64
		ekfR float64
	)
	// Если передан конфиг — читаем его, иначе берем CLI флаги
	if *configPath != "" {
		fmt.Printf("Загрузка сценария из файла: %s\n", *configPath)
		cfg, err := vio.LoadConfigFromFile(*configPath)
		if err != nil {
			fmt.Printf("Ошибка загрузки конфигурации: %v\n", err)
			return
		}
		n = cfg.Network.Nodes
		sMin = cfg.Network.SpaceMin
		sMax = cfg.Network.SpaceMax
		gErr = cfg.Network.GpsErr
		dErr = cfg.Network.DistErr
		anchorsCount = cfg.Hyperparams.AnchorsCount
		gAlpha = cfg.Hyperparams.Alpha
		gLambda = cfg.Hyperparams.Lambda
		gEps = cfg.Hyperparams.Eps
		gMaxIter = cfg.Hyperparams.MaxIter
		eIter = cfg.Hyperparams.EkfIter
		ekfQ = cfg.Hyperparams.Q
		ekfR = cfg.Hyperparams.R
		topologyK = cfg.Topology.K
		if topologyK <= 0 {
			topologyK = -1
		}
	} else {
		n = *numNodes
		sMin = *spaceMin
		sMax = *spaceMax
		gErr = *gpsErr
		dErr = *distErr
		anchorsCount = *anchorsCountFlag
		gAlpha = *alpha
		gLambda = *lambda
		gEps = *epsilon
		gMaxIter = *maxIter
		eIter = *ekfIter
		ekfQ = *ekfQFlag
		ekfR = *ekfRFlag
	}

	if n < 3 {
		fmt.Println("Ошибка: требуется минимум 3 узла.")
		return
	}

	// Определяем k: CLI флаг > конфиг > дефолт (n-1)
	k := topologyK
	if *kFlag >= 0 {
		k = *kFlag
	}
	if k < 0 {
		k = n - 1
	}

	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	fmt.Printf("--- Выполнение сценария ---\n")
	fmt.Printf("Узлов: %d | Пространство: [%.1f, %.1f] | Шум GPS: ±%.2f м | Шум дальномеров: ±%.2f м | k=%d\n\n",
		n, sMin, sMax, gErr, dErr, k)

	nodes, realCoords, distances := environ.BuildMap(n, sMin, sMax, gErr, dErr, anchorsCount, rng)

	// Параметры движения
	ticks := 10                      // количество шагов временной динамики
	motionSigma := 0.5               // σ шага случайного блуждания объектов
	growthRate := 0.01               // коэффициент линейного роста ошибки за шаг
	gdItersPerTick := 50             // итерации GD на каждом тике
	ekfItersPerTick := 3             // итерации EKF на каждом тике

	// [Выполнение алгоритмов]
	fmt.Println("Состояние ДО оптимизации:")
	analytics.PrintGradientMetrics(nodes, realCoords, distances)

	// Начальное решение (один раз)
	fmt.Println("\n--- Запуск Градиентного спуска (начальное) ---")
	measurements := algorithms.BuildMeasurements(nodes, distances, k)
	algorithms.CheckConnectivity(nodes, measurements)
	algorithms.RunGradientDescent(nodes, measurements, gAlpha, gLambda, gEps, gMaxIter, false)
	fmt.Println("\n--- Запуск Расширенного фильтра Калмана (начальное) ---")
	algorithms.RunEKF(nodes, measurements, eIter, ekfQ, ekfR, false)

	fmt.Println("\nСостояние ПОСЛЕ начальной оптимизации:")
	analytics.PrintGradientMetrics(nodes, realCoords, distances)

	// Тиковый цикл — движение и коррекция ошибок
	tickMetrics := [][]interface{}{
		{"Tick", "RawRMSE", "FinalRMSE", "CompensationRatio"},
	}

	fmt.Printf("\n--- Временная динамика: %d тиков ---\n", ticks)
	for tick := 0; tick < ticks; tick++ {
		fmt.Printf("\n[Тик %d] Сдвиг объектов, рост ошибки, переоптимизация\n", tick)

		// 1. Смещение истинных координат на один шаг (случайное блуждание)
		environ.StepTrueMotion(nodes, motionSigma, rng)

		// 2. Рост ошибки (Uncertainty растёт, BelievedCoord размывается)
		environ.GrowUncertainty(nodes, growthRate, rng)

		// 3. Перенос BelievedCoord в InitialCoord для регуляризации GD
		environ.RefreshInitialCoord(nodes)

		// 4. Пересчёт измеренных расстояний по новым истинным координатам
		distances = environ.RecomputeDistances(nodes, dErr, rng)

		// 5. Переоптимизация с warm start (малое число итераций)
		measurements = algorithms.BuildMeasurements(nodes, distances, k)
		fmt.Printf("  GD: %d итераций...", gdItersPerTick)
		algorithms.RunGradientDescent(nodes, measurements, gAlpha, gLambda, gEps, gdItersPerTick, true)
		fmt.Printf(" OK\n")

		fmt.Printf("  EKF: %d итераций...", ekfItersPerTick)
		algorithms.RunEKF(nodes, measurements, ekfItersPerTick, ekfQ, ekfR, true)
		fmt.Printf(" OK\n")

		// 6. Вычисление и логирование метрик на этом тике
		rawRMSE := computeRawRMSE(nodes)
		finalRMSE := computeFinalRMSE(nodes)
		compensationRatio := 0.0
		if rawRMSE > 0 {
			compensationRatio = rawRMSE / finalRMSE
		}

		fmt.Printf("  Метрики на тике %d:\n", tick)
		fmt.Printf("    Raw RMSE (BelievedCoord vs RealCoord): %.5f\n", rawRMSE)
		fmt.Printf("    Final RMSE (CurrentCoord vs RealCoord): %.5f\n", finalRMSE)
		fmt.Printf("    Compensation ratio: %.3f\n", compensationRatio)

		// Добавляем метрики в историю
		tickMetrics = append(tickMetrics, []interface{}{tick, rawRMSE, finalRMSE, compensationRatio})
	}

	// Сохранение истории тиков в CSV
	if err := saveTickMetricsToCSV("tick_history.csv", tickMetrics); err != nil {
		fmt.Printf("Ошибка при записи tick_history.csv: %v\n", err)
	} else {
		fmt.Println("Файл tick_history.csv успешно сгенерирован.")
	}

	fmt.Println("\n--- Итоговое состояние ---")
	analytics.PrintGradientMetrics(nodes, realCoords, distances)

	movable := make([]int, 0, len(nodes))
	for i, node := range nodes {
		if !node.IsAnchor {
			movable = append(movable, i)
		}
	}

	if err := analytics.AnalyzeEKFHistory("ekf_history.csv", nodes, movable, realCoords, distances); err != nil {
		fmt.Printf("Ошибка при анализе статистики: %v\n", err)
	}
}

// Вспомогательные функции для метрик
func computeRawRMSE(nodes []*types.Node) float64 {
	var sumSqDist float64
	count := 0
	for _, node := range nodes {
		if node.IsAnchor {
			continue
		}
		dx := node.BelievedCoord.X - node.RealCoord.X
		dy := node.BelievedCoord.Y - node.RealCoord.Y
		dz := node.BelievedCoord.Z - node.RealCoord.Z
		sumSqDist += dx*dx + dy*dy + dz*dz
		count++
	}
	if count == 0 {
		return 0
	}
	return math.Sqrt(sumSqDist / float64(count))
}

func computeFinalRMSE(nodes []*types.Node) float64 {
	var sumSqDist float64
	count := 0
	for _, node := range nodes {
		if node.IsAnchor {
			continue
		}
		dx := node.CurrentCoord.X - node.RealCoord.X
		dy := node.CurrentCoord.Y - node.RealCoord.Y
		dz := node.CurrentCoord.Z - node.RealCoord.Z
		sumSqDist += dx*dx + dy*dy + dz*dz
		count++
	}
	if count == 0 {
		return 0
	}
	return math.Sqrt(sumSqDist / float64(count))
}

func saveTickMetricsToCSV(filename string, metrics [][]interface{}) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	for _, row := range metrics {
		strRow := make([]string, len(row))
		for i, val := range row {
			strRow[i] = fmt.Sprintf("%v", val)
		}
		if err := writer.Write(strRow); err != nil {
			return err
		}
	}
	return nil
}
