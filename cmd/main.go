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
	epsilon := flag.Float64("eps", 1e-5, "Критерий останова градиентного спуска")
	maxIter := flag.Int("max-iter", 2000, "Лимит итераций градиентного спуска")
	ekfIter := flag.Int("ekf-iter", 5, "Количество итераций EKF")
	ekfQFlag := flag.Float64("q", 0.001, "Шум процесса EKF")
	ekfRFlag := flag.Float64("r", 0.1, "Шум измерений EKF")
	anchorsCountFlag := flag.Int("anchors", 0, "Количество анкерных узлов с точными координатами")

	ticksFlag := flag.Int("ticks", 10, "Количество шагов временной динамики")
	dtFlag := flag.Float64("dt", 1.0, "Длительность тика")
	swarmSpeedFlag := flag.Float64("swarm-speed", 2.0, "Модуль командной скорости роя, м/тик")
	growthRateFlag := flag.Float64("growth-rate", 0.01, "Коэффициент квадратичного роста ошибки")
	gdTickIterFlag := flag.Int("gd-tick-iter", 50, "Итерации GD на каждом тике")
	ekfTickIterFlag := flag.Int("ekf-tick-iter", 3, "Итерации EKF на каждом тике")
	flag.Parse()

	var (
		n                      int
		sMin, sMax, gErr, dErr float64
		gAlpha, gEps           float64
		gMaxIter, eIter        int
		anchorsCount           int
		topologyK              int

		ekfQ float64
		ekfR float64

		ticks                           int
		dt, swarmSpeed, growthRate      float64
		gdItersPerTick, ekfItersPerTick int
	)
	// Если передан конфиг — читаем его, иначе берем CLI флаги
	if *configPath != "" {
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
		gEps = cfg.Hyperparams.Eps
		gMaxIter = cfg.Hyperparams.MaxIter
		eIter = cfg.Hyperparams.EkfIter
		ekfQ = cfg.Hyperparams.Q
		ekfR = cfg.Hyperparams.R
		topologyK = cfg.Topology.K
		if topologyK <= 0 {
			topologyK = -1
		}
		ticks = cfg.Dynamics.Ticks
		dt = cfg.Dynamics.Dt
		swarmSpeed = cfg.Dynamics.SwarmSpeed
		growthRate = cfg.Dynamics.GrowthRate
		gdItersPerTick = cfg.Dynamics.GdItersPerTick
		ekfItersPerTick = cfg.Dynamics.EkfItersPerTick
	} else {
		n = *numNodes
		sMin = *spaceMin
		sMax = *spaceMax
		gErr = *gpsErr
		dErr = *distErr
		anchorsCount = *anchorsCountFlag
		topologyK = -1
		gAlpha = *alpha
		gEps = *epsilon
		gMaxIter = *maxIter
		eIter = *ekfIter
		ekfQ = *ekfQFlag
		ekfR = *ekfRFlag
		ticks = *ticksFlag
		dt = *dtFlag
		swarmSpeed = *swarmSpeedFlag
		growthRate = *growthRateFlag
		gdItersPerTick = *gdTickIterFlag
		ekfItersPerTick = *ekfTickIterFlag
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

	fmt.Printf("N = %d; A = %d; (X, Y, Z) ∈ {%g, %g}; k = %d\n", n, anchorsCount, sMin, sMax, k)
	fmt.Printf("dX, dY, dZ = ±%.2f м; dD = ±%.2f м\n", gErr, dErr)
	fmt.Printf("t ∈ {1..%d}; |v| = %.2f м/тик; рост ошибки %.3f\n", ticks, swarmSpeed, growthRate)

	nodes, realCoords, distances := environ.BuildMap(n, sMin, sMax, gErr, dErr, anchorsCount, rng)

	// Командный вектор скорости — случайное направление, инициализируется один раз
	dirX := rng.Float64()*2 - 1
	dirY := rng.Float64()*2 - 1
	dirZ := rng.Float64()*2 - 1
	dirNorm := math.Sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ)
	velocity := types.Velocity{
		X: dirX / dirNorm * swarmSpeed,
		Y: dirY / dirNorm * swarmSpeed,
		Z: dirZ / dirNorm * swarmSpeed,
	}
	fmt.Println("\nСТАТИЧНАЯ КОРРЕКТИРОВКА:")
	fmt.Println()

	rmseBefore, varBefore := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)

	// Начальное решение (один раз)
	measurements := algorithms.BuildMeasurements(nodes, distances, k)
	if anchorsCount > 0 {
		algorithms.CheckConnectivity(nodes, measurements)
	}
	gdIters := algorithms.RunGradientDescent(nodes, measurements, gAlpha, dErr, gEps, gMaxIter, false)
	ekfP := algorithms.RunEKF(nodes, measurements, eIter, ekfQ, ekfR, nil, false)

	// Без анкеров gauge ненаблюдаем: совмещаем решение с GNSS-кадром (BelievedCoord)
	if anchorsCount == 0 {
		believedRef := make([]types.Point, n)
		for i, node := range nodes {
			believedRef[i] = node.BelievedCoord
		}
		algorithms.AlignMovableToReference(nodes, believedRef)
	}

	rmseAfter, varAfter := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)

	fmt.Printf("Градиентный спуск %d итераций; Фильтр Калмана %d итераций\n", gdIters, eIter)
	fmt.Printf("RMSE:      %.5f м  -> %.5f м  (Δ %+.5f)\n", rmseBefore, rmseAfter, rmseAfter-rmseBefore)
	fmt.Printf("Дисперсия: %.5f м² -> %.5f м² (Δ %+.5f)\n", varBefore, varAfter, varAfter-varBefore)

	if maxSpread, maxLabel, meanSpread, err := analytics.SummarizeEKFSpread("ekf_history.csv"); err != nil {
		fmt.Printf("Ошибка при анализе истории EKF: %v\n", err)
	} else {
		fmt.Printf("Размах координат: max %.4g (%s), среднее %.4g\n", maxSpread, maxLabel, meanSpread)
	}

	// Тиковый цикл — движение и коррекция ошибок
	tickMetrics := [][]interface{}{
		{"Tick", "RawRMSE", "FinalRMSE", "ShapeRMSE", "CompensationRatio"},
	}

	fmt.Println("\nДИНАМИКА ПО ТИКАМ:")
	fmt.Println()
	fmt.Println("Тик |  RawRMSE | FinalRMSE | ShapeRMSE |  Ratio")
	fmt.Println("----+----------+-----------+-----------+-------")
	for tick := 0; tick < ticks; tick++ {
		// 1. Сдвиг роя по командному вектору скорости (счисление пути)
		environ.StepSwarmMotion(nodes, velocity, dt)

		// 2. Рост ошибки (Uncertainty растёт, BelievedCoord размывается)
		environ.GrowUncertainty(nodes, growthRate, rng)

		// 3. Перенос BelievedCoord в InitialCoord для регуляризации GD
		environ.RefreshInitialCoord(nodes)

		// 4. Пересчёт измеренных расстояний по новым истинным координатам
		distances = environ.RecomputeDistances(nodes, dErr, rng)

		// Снимок предсказанных координат — опора для gauge-фиксации после оптимизации
		predicted := make([]types.Point, len(nodes))
		for i, node := range nodes {
			predicted[i] = node.CurrentCoord
		}

		// 5. Переоптимизация с warm start (малое число итераций)
		measurements = algorithms.BuildMeasurements(nodes, distances, k)
		algorithms.RunGradientDescent(nodes, measurements, gAlpha, dErr, gEps, gdItersPerTick, true)
		ekfP = algorithms.RunEKF(nodes, measurements, ekfItersPerTick, ekfQ, ekfR, ekfP, true)

		if anchorsCount == 0 {
			algorithms.AlignMovableToReference(nodes, predicted)
		}

		// 6. Вычисление и логирование метрик на этом тике
		rawRMSE := computeRawRMSE(nodes)
		finalRMSE := computeFinalRMSE(nodes)
		shapeRMSE := computeShapeRMSE(nodes)
		compensationRatio := 0.0
		if rawRMSE > 0 {
			compensationRatio = rawRMSE / finalRMSE
		}

		fmt.Printf("%3d | %8.5f | %9.5f | %9.5f | %6.2f\n", tick, rawRMSE, finalRMSE, shapeRMSE, compensationRatio)

		// Добавляем метрики в историю
		tickMetrics = append(tickMetrics, []interface{}{tick, rawRMSE, finalRMSE, shapeRMSE, compensationRatio})
	}

	// realCoords снят при инициализации — после движения роя он устарел
	for i, node := range nodes {
		realCoords[i] = node.RealCoord
	}

	rmseFinal, varFinal := analytics.ComputeGlobalMetrics(nodes, realCoords, distances)
	fmt.Printf("\nИтог в динамике: RMSE = %.5f м, дисперсия = %.5f м²\n", rmseFinal, varFinal)

	if err := saveTickMetricsToCSV("tick_history.csv", tickMetrics); err != nil {
		fmt.Printf("Ошибка при записи tick_history.csv: %v\n", err)
	} else {
		fmt.Println("Сохранено в: mod1_history.csv, ekf_history.csv, tick_history.csv")
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

// computeShapeRMSE — ошибка формы: RMSE после оптимального совмещения оценки
// с истинными координатами. Показывает качество каркаса без ошибки привязки.
func computeShapeRMSE(nodes []*types.Node) float64 {
	current := make([]types.Point, 0, len(nodes))
	real := make([]types.Point, 0, len(nodes))
	for _, node := range nodes {
		if node.IsAnchor {
			continue
		}
		current = append(current, node.CurrentCoord)
		real = append(real, node.RealCoord)
	}
	if len(current) == 0 {
		return 0
	}

	aligned := algorithms.KabschAlign(current, real)

	var sumSqDist float64
	for i := range aligned {
		dx := aligned[i].X - real[i].X
		dy := aligned[i].Y - real[i].Y
		dz := aligned[i].Z - real[i].Z
		sumSqDist += dx*dx + dy*dy + dz*dz
	}
	return math.Sqrt(sumSqDist / float64(len(aligned)))
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
