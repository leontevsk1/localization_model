package main

import (
	"math"
	"gonum.org/v1/gonum/mat"
	"fmt"
)

// RunEKF выполняет Расширенный фильтр Калмана (EKF) с использованием gonum/mat.
// Алгоритм обрабатывает все измерения дальностей за один матричный шаг (Batch Update).
func RunEKF(nodes []*Node, distances [][]float64, iterations int, q, r float64) {
	n := len(nodes)
	stateSize := n * 3 // По 3 координаты (X, Y, Z) на каждый узел
	// Хранилище истории
	var ekfHistory [][]float64
	// Подсчет количества измерений 
	numMeas := 0
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			numMeas++
		}
	}

	// Инициализация вектора состояния X
	xData := make([]float64, stateSize)
	for i, node := range nodes {
		xData[i*3+0] = node.CurrentCoord.X
		xData[i*3+1] = node.CurrentCoord.Y
		xData[i*3+2] = node.CurrentCoord.Z
	}
	X := mat.NewVecDense(stateSize, xData)

	// Инициализация ковариационной матрицы P
	P := mat.NewDense(stateSize, stateSize, nil)
	for i := 0; i < stateSize; i++ {
		P.Set(i, i, 1.0) // Начальная неопределенность
	}

	// Матрицы шума процесса Q и шума измерений R
	Q := mat.NewDense(stateSize, stateSize, nil)
	for i := 0; i < stateSize; i++ {
		Q.Set(i, i, q)
	}

	R := mat.NewDense(numMeas, numMeas, nil)
	for i := 0; i < numMeas; i++ {
		R.Set(i, i, r)
	}

	// Предварительное выделение памяти под рабочие матрицы 

	H := mat.NewDense(numMeas, stateSize, nil)       // Якобиан
	Z := mat.NewVecDense(numMeas, nil)               // Реальные измерения
	Zcalc := mat.NewVecDense(numMeas, nil)           // Расчетные измерения
	Y := mat.NewVecDense(numMeas, nil)               // Инновация (Невязка)
	
	PHt := mat.NewDense(stateSize, numMeas, nil)
	S := mat.NewDense(numMeas, numMeas, nil)
	Sinv := mat.NewDense(numMeas, numMeas, nil)      // Обратная матрица инноваций
	K := mat.NewDense(stateSize, numMeas, nil)       // Коэффициент Калмана
	
	I := mat.NewDense(stateSize, stateSize, nil)     // Единичная матрица
	for i := 0; i < stateSize; i++ {
		I.Set(i, i, 1.0)
	}
	
	KX := mat.NewVecDense(stateSize, nil)
	KH := mat.NewDense(stateSize, stateSize, nil)
	IKH := mat.NewDense(stateSize, stateSize, nil)

	// Основной цикл EKF
	for iter := 0; iter < iterations; iter++ {
		// ЭТАП ПРЕДСКАЗАНИЯ
		// Состояние X остается прежним (объекты неподвижны), но дисперсия растет
		P.Add(P, Q)

		// ЭТАП КОРРЕКЦИИ
		measIdx := 0
		for i := 0; i < n; i++ {
			for j := i + 1; j < n; j++ {
				// Текущие координаты узлов из вектора состояния
				xi, yi, zi := X.AtVec(i*3+0), X.AtVec(i*3+1), X.AtVec(i*3+2)
				xj, yj, zj := X.AtVec(j*3+0), X.AtVec(j*3+1), X.AtVec(j*3+2)

				dx, dy, dz := xi-xj, yi-yj, zi-zj
				dCalc := math.Sqrt(dx*dx + dy*dy + dz*dz)
				if dCalc == 0 {
					dCalc = 1e-6 // Защита от деления на ноль
				}

				// Заполнение векторов измерений
				Z.SetVec(measIdx, distances[i][j])
				Zcalc.SetVec(measIdx, dCalc)

				// Заполнение матрицы Якоби H (производные дистанции по координатам)
				H.Set(measIdx, i*3+0, dx/dCalc)
				H.Set(measIdx, i*3+1, dy/dCalc)
				H.Set(measIdx, i*3+2, dz/dCalc)

				H.Set(measIdx, j*3+0, -dx/dCalc)
				H.Set(measIdx, j*3+1, -dy/dCalc)
				H.Set(measIdx, j*3+2, -dz/dCalc)

				measIdx++
			}
		}

		// Вычисление инновации: Y = Z - Zcalc
		Y.SubVec(Z, Zcalc)

		// Вычисление ковариации инновации: S = H * P * H^T + R
		PHt.Mul(P, H.T())
		S.Mul(H, PHt)
		S.Add(S, R)

		// Обращение матрицы S
		err := Sinv.Inverse(S)
		if err != nil {
			// Если матрица вырождена, прекращаем итерации
			break
		}

		// Вычисление коэффициента Калмана: K = P * H^T * Sinv
		K.Mul(PHt, Sinv)

		// Коррекция состояния: X = X + K * Y
		KX.MulVec(K, Y)
		X.AddVec(X, KX)

		// Обновление ковариации: P = (I - K * H) * P
		KH.Mul(K, H)
		IKH.Sub(I, KH)
		P.Mul(IKH, P)

		// Сохраняем текущее состояние всех узлов в историю
		historyRow := make([]float64, 0, 1+stateSize)
		historyRow = append(historyRow, float64(iter))
		for i := 0; i < stateSize; i++ {
			historyRow = append(historyRow, X.AtVec(i))
		}
		ekfHistory = append(ekfHistory, historyRow)
	}

	// Запись вычисленных координат обратно в структуру узлов
	for i := 0; i < n; i++ {
		nodes[i].CurrentCoord.X = X.AtVec(i*3 + 0)
		nodes[i].CurrentCoord.Y = X.AtVec(i*3 + 1)
		nodes[i].CurrentCoord.Z = X.AtVec(i*3 + 2)
	}

	// Сохранение лога в корень проекта
	if err := SaveEKFHistory("ekf_history.csv", ekfHistory, n); err != nil {
		fmt.Printf("Ошибка при записи лога EKF: %v\n", err)
	} else {
		fmt.Println("Файл ekf_history.csv успешно сгенерирован.")
	}
}
