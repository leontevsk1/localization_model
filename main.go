package main

import (
	"flag"
	"fmt"
	"math"
	"math/rand"
	"time"
)

func main() {
	// Флаг для пути к файлу конфигурации
	configPath := flag.String("config", "", "Путь к TOML-файлу конфигурации сценария")

	// Стандартные CLI флаги (как значения по умолчанию)
	numNodes := flag.Int("nodes", 4, "Количество объектов (узлов) в сети")
	spaceMin := flag.Float64("space-min", 0.0, "Минимальная граница пространства")
	spaceMax := flag.Float64("space-max", 100.0, "Максимальная граница пространства")
	gpsErr := flag.Float64("gps-err", 3.0, "Максимальная абсолютная ошибка GPS (мнимых координат)")
	distErr := flag.Float64("dist-err", 0.5, "Максимальная абсолютная ошибка дальномеров")

	alpha := flag.Float64("alpha", 0.02, "Скорость обучения градиентного спуска")
	lambda := flag.Float64("lambda", 0.15, "Вес мягкой регуляризации")
	epsilon := flag.Float64("eps", 1e-5, "Критерий останова градиентного спуска")
	maxIter := flag.Int("max-iter", 2000, "Лимит итераций градиентного спуска")
	ekfIter := flag.Int("ekf-iter", 5, "Количество итераций EKF")

	flag.Parse()

	// Переменные, которые пойдут в логику генерации
	var n int
	var sMin, sMax, gErr, dErr float64
	var gAlpha, gLambda, gEps float64
	var gMaxIter, eIter int

	// Если передан конфиг — читаем его, иначе берем CLI флаги
	if *configPath != "" {
		fmt.Printf("Загрузка сценария из файла: %s\n", *configPath)
		cfg, err := LoadConfigFromFile(*configPath)
		if err != nil {
			fmt.Printf("Ошибка загрузки конфигурации: %v\n", err)
			return
		}
		n = cfg.Network.Nodes
		sMin = cfg.Network.SpaceMin
		sMax = cfg.Network.SpaceMax
		gErr = cfg.Network.GpsErr
		dErr = cfg.Network.DistErr

		gAlpha = cfg.Hyperparams.Alpha
		gLambda = cfg.Hyperparams.Lambda
		gEps = cfg.Hyperparams.Eps
		gMaxIter = cfg.Hyperparams.MaxIter
		eIter = cfg.Hyperparams.EkfIter
	} else {
		n = *numNodes
		sMin = *spaceMin
		sMax = *spaceMax
		gErr = *gpsErr
		dErr = *distErr

		gAlpha = *alpha
		gLambda = *lambda
		gEps = *epsilon
		gMaxIter = *maxIter
		eIter = *ekfIter
	}

	if n < 3 {
		fmt.Println("Ошибка: требуется минимум 3 узла.")
		return
	}

	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	fmt.Printf("--- Выполнение сценария ---\n")
	fmt.Printf("Узлов: %d | Пространство: [%.1f, %.1f] | Шум GPS: ±%.2f м | Шум дальномеров: ±%.2f м\n\n",
		n, sMin, sMax, gErr, dErr)

	// [Генерация координат]
	realCoords := make([]Point, n)
	nodes := make([]*Node, n)
	for i := 0; i < n; i++ {
		rX := sMin + rng.Float64()*(sMax-sMin)
		rY := sMin + rng.Float64()*(sMax-sMin)
		rZ := sMin + rng.Float64()*(sMax-sMin)
		realCoords[i] = Point{X: rX, Y: rY, Z: rZ}

		dx := (rng.Float64()*2 - 1.0) * gErr
		dy := (rng.Float64()*2 - 1.0) * gErr
		dz := (rng.Float64()*2 - 1.0) * gErr

		nodes[i] = &Node{
			ID:           i,
			InitialCoord: Point{X: rX + dx, Y: rY + dy, Z: rZ + dz},
			CurrentCoord: Point{X: rX + dx, Y: rY + dy, Z: rZ + dz},
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

	// [Выполнение алгоритмов]
	fmt.Println("Состояние ДО оптимизации:")
	printGradientMetrics(nodes, realCoords, distances)

	fmt.Println("\n--- Запуск Градиентного спуска ---")
	RunGradientDescent(nodes, distances, gAlpha, gLambda, gEps, gMaxIter)

	fmt.Println("\n--- Запуск Расширенного фильтра Калмана (EKF) ---")
	RunEKF(nodes, distances, eIter, 0.001, 0.1)

	fmt.Println("\nСостояние ПОСЛЕ оптимизации:")
	printGradientMetrics(nodes, realCoords, distances)

	fmt.Println("\nСтатистика по EKF: ")
	if err := AnalyzeEKFHistory("ekf_history.csv", realCoords, distances); err != nil {
		fmt.Printf("Ошибка при анализе статистики: %v\n", err)
	}
}
