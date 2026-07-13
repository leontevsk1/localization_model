package io

import (
	"encoding/csv"
	"fmt"
	"os"

	"github.com/pelletier/go-toml/v2"

	"vostok/pkg/types"
)

func LoadConfigFromFile(filepath string) (*types.Config, error) {
	file, err := os.ReadFile(filepath)
	if err != nil {
		return nil, err
	}
	var cfg types.Config
	err = toml.Unmarshal(file, &cfg)
	if err != nil {
		return nil, err
	}
	return &cfg, nil
}

// WriteHistoryToCSV — утилита, которую вы можете вызвать из gradient.go.
// doAppend=false создаёт файл заново (с заголовком), doAppend=true дозаписывает
// строки в конец существующего файла (заголовок не пишется повторно).
func WriteHistoryToCSV(filename string, recordFields [][]string, doAppend bool) error {
	flags := os.O_CREATE | os.O_WRONLY
	rows := recordFields
	if doAppend {
		flags |= os.O_APPEND
		rows = recordFields[1:] // пропускаем заголовок при дозаписи
	} else {
		flags |= os.O_TRUNC
	}

	file, err := os.OpenFile(filename, flags, 0644)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	for _, row := range rows {
		if err := writer.Write(row); err != nil {
			return err
		}
	}
	return nil
}

// SaveEKFHistory сохраняет детальную историю итераций EKF для всех узлов сети.
// doAppend=false создаёт файл заново (с заголовком), doAppend=true дозаписывает
// строки в конец существующего файла (заголовок не пишется повторно).
func SaveEKFHistory(filename string, history [][]float64, nodeCount int, doAppend bool) error {
	flags := os.O_CREATE | os.O_WRONLY
	if doAppend {
		flags |= os.O_APPEND
	} else {
		flags |= os.O_TRUNC
	}

	file, err := os.OpenFile(filename, flags, 0644)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()

	if !doAppend {
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
