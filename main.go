package main

import (
	"flag"
	"fmt"
	"math"
	"math/rand"
	"time"
)

func main() {
	// 1. Определение флагов CLI
	numNodes := flag.Int("nodes", 4, "Количество объектов (узлов) в сети")
	spaceMin := flag.Float64("space-min", 0.0, "Минимальная граница пространства для генерации координат")
	spaceMax := flag.Float64("space-max", 100.0, "Максимальная граница пространства для генерации координат")
	gpsErr := flag.Float64("gps-err", 3.0, "Максимальная абсолютная ошибка начальных (мнимых) координат (+/- метры)")
	distErr := flag.Float64("dist-err", 0.5, "Максимальная абсолютная ошибка дальномеров (+/- метры)")

	// Гиперпараметры алгоритмов также можно вынести в флаги для удобства тестирования
	alpha := flag.Float64("alpha", 0.02, "Скорость обучения градиентного спуска")
	lambda := flag.Float64("lambda", 0.15, "Вес мягкой регуляризации")
	epsilon := flag.Float64("eps", 1e-5, "Критерий останова градиентного спуска")
	maxIter := flag.Int("max-iter", 2000, "Лимит итераций градиентного спуска")
	ekfIter := flag.Int("ekf-iter", 5, "Количество итераций EKF")

	flag.Parse()

	if *numNodes < 3 {
		fmt.Println("Ошибка: для трилатерации в 3D требуется минимум 3 узла (желательно 4 и более).")
		return
	}

	// Инициализация генератора случайных чисел
	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	fmt.Printf("--- Генерация тестового сценария ---\n")
	fmt.Printf("Узлов: %d | Пространство: [%.1f, %.1f] | Шум GPS: ±%.2f м | Шум дальномеров: ±%.2f м\n\n",
		*numNodes, *spaceMin, *spaceMax, *gpsErr, *distErr)

	// 2. Генерация истинных координат и мнимых узлов
	realCoords := make([]Point, *numNodes)
	nodes := make([]*Node, *numNodes)

	for i := 0; i < *numNodes; i++ {
		// Истинные случайные координаты
		rX := *spaceMin + rng.Float64()*(*spaceMax-*spaceMin)
		rY := *spaceMin + rng.Float64()*(*spaceMax-*spaceMin)
		rZ := *spaceMin + rng.Float64()*(*spaceMax-*spaceMin)
		realCoords[i] = Point{X: rX, Y: rY, Z: rZ}

		// Смещение (ошибка GPS) от -gpsErr до +gpsErr
		dx := (rng.Float64()*2 - 1.0) * (*gpsErr)
		dy := (rng.Float64()*2 - 1.0) * (*gpsErr)
		dz := (rng.Float64()*2 - 1.0) * (*gpsErr)

		imagX := rX + dx
		imagY := rY + dy
		imagZ := rZ + dz

		nodes[i] = &Node{
			ID:           i,
			InitialCoord: Point{X: imagX, Y: imagY, Z: imagZ},
			CurrentCoord: Point{X: imagX, Y: imagY, Z: imagZ},
		}
	}

	// 3. Генерация симметричной матрицы измеренных расстояний
	distances := make([][]float64, *numNodes)
	for i := range distances {
		distances[i] = make([]float64, *numNodes)
	}

	for i := 0; i < *numNodes; i++ {
		for j := i + 1; j < *numNodes; j++ {
			// Истинное расстояние
			dx := realCoords[i].X - realCoords[j].X
			dy := realCoords[i].Y - realCoords[j].Y
			dz := realCoords[i].Z - realCoords[j].Z
			trueDist := math.Sqrt(dx*dx + dy*dy + dz*dz)

			// Симметричный шум дальномера от -distErr до +distErr
			noise := (rng.Float64()*2 - 1.0) * (*distErr)
			measDist := trueDist + noise

			// Защита от отрицательных дистанций при огромном шуме
			if measDist < 0 {
				measDist = 0.001
			}

			distances[i][j] = measDist
			distances[j][i] = measDist // Матрица должна быть симметричной
		}
	}

	// 4. Запуск пайплайна
	fmt.Println("Состояние ДО оптимизации:")
	printGradientMetrics(nodes, realCoords, distances) // Убедитесь, что функция называется printMetrics или printGradientMetrics согласно вашему utils.go

	fmt.Println("\n--- Запуск Градиентного спуска ---")
	RunGradientDescent(nodes, distances, *alpha, *lambda, *epsilon, *maxIter)

	fmt.Println("\n--- Запуск Расширенного фильтра Калмана (EKF) ---")
	RunEKF(nodes, distances, *ekfIter, 0.001, 0.1)

	fmt.Println("\nСостояние ПОСЛЕ оптимизации:")
	// Используем расширенный вывод, если он есть, либо стандартный
	printGradientMetrics(nodes, realCoords, distances) 

	fmt.Println("\nСтатистика по EKF: ")
	if err := AnalyzeEKFHistory("ekf_history.csv", realCoords, distances); err != nil {
		fmt.Printf("Ошибка при анализе статистики: %v\n", err)
	}
}

// Если printMetrics и printMetricsExt остались в main.go, 
// убедитесь что они добавлены сюда (или находятся в utils.go)
