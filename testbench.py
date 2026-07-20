#!/usr/bin/env python3
import os
import subprocess
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

CONFIG_DIR = "config"
MD_REPORT_PATH = "report.md"
GRADIENT_CSV = "mod1_history.csv"
EKF_CSV = "ekf_history.csv"
TICK_CSV = "tick_history.csv"
IMG_DIR = "report_images"


def get_git_commit():
    try:
        commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()
        return commit
    except Exception:
        return "Git-репозиторий не найден"


def run_go_simulation(config_path):
    """Запускает симуляцию с указанным конфигом."""
    # Логи GD/EKF накапливаются по тикам (append) — удаляем остатки
    # предыдущего прогона, чтобы график отражал только текущий запуск.
    for f in (GRADIENT_CSV, EKF_CSV, TICK_CSV):
        if os.path.exists(f):
            os.remove(f)

    cmd = ["go", "run", "./cmd/main.go", "-config", config_path]

    print(f"Команда: {' '.join(cmd)}\n")
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=".")

    if result.returncode != 0:
        print("Ошибка выполнения:")
        print(result.stderr)
        return None

    print("Симуляция успешно завершена\n")
    return result.stdout


def plot_metrics(run_id):
    """Строит графики для всех доступных CSV."""
    plots = {}

    # Градиентный спуск — CSV накапливается по всем тикам (append), поэтому
    # колонка Iteration скачет 0..N на каждом вызове. Ось X — сквозной номер
    # строки, иначе график читается как разрыв на каждой границе тика.
    # Имя узла в колонках не всегда Node0 — Go-логгер пишет первый подвижный
    # узел (Node0 может быть анкером и тогда был бы всегда плоским).
    if os.path.exists(GRADIENT_CSV):
        try:
            df = pd.read_csv(GRADIENT_CSV)
            node_label = df.columns[1].rsplit('_', 1)[0]  # напр. "Node2"
            x = range(len(df))
            plt.figure(figsize=(10, 5))
            plt.plot(x, df[f'{node_label}_X'], label='X', color='blue')
            plt.plot(x, df[f'{node_label}_Y'], label='Y', color='green')
            plt.plot(x, df[f'{node_label}_Z'], label='Z', color='red')
            if 'RealX' in df.columns:
                plt.plot(x, df['RealX'], label='X реальная', color='blue', linestyle='--', linewidth=1)
                plt.plot(x, df['RealY'], label='Y реальная', color='green', linestyle='--', linewidth=1)
                plt.plot(x, df['RealZ'], label='Z реальная', color='red', linestyle='--', linewidth=1)
            plt.title(f'Сходимость GD ({node_label}), сквозная история по всем тикам')
            plt.xlabel('Шаг (накопительно)')
            plt.ylabel('Координата')
            plt.grid(True)
            plt.legend()

            img_path = os.path.join(IMG_DIR, f"gd_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['gradient'] = img_path
        except Exception as e:
            print(f"Ошибка при построении графика GD: {e}")

    # EKF — та же логика: сквозной индекс вместо скачущей колонки Iteration.
    if os.path.exists(EKF_CSV):
        try:
            df = pd.read_csv(EKF_CSV)
            x = range(len(df))
            plt.figure(figsize=(10, 5))
            if len(df.columns) > 1:
                node_label = df.columns[1].rsplit('_', 1)[0]
                plt.plot(x, df[f'{node_label}_X'], label='X', color='blue')
                plt.plot(x, df[f'{node_label}_Y'], label='Y', color='green')
                plt.plot(x, df[f'{node_label}_Z'], label='Z', color='red')
                if 'RealX' in df.columns:
                    plt.plot(x, df['RealX'], label='X реальная', color='blue', linestyle='--', linewidth=1)
                    plt.plot(x, df['RealY'], label='Y реальная', color='green', linestyle='--', linewidth=1)
                    plt.plot(x, df['RealZ'], label='Z реальная', color='red', linestyle='--', linewidth=1)
            else:
                node_label = '?'
            plt.title(f'Траектория EKF ({node_label}), сквозная история по всем тикам')
            plt.xlabel('Шаг (накопительно)')
            plt.ylabel('Координата')
            plt.grid(True)
            plt.legend()

            img_path = os.path.join(IMG_DIR, f"ekf_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['ekf'] = img_path
        except Exception as e:
            print(f"Ошибка при построении графика EKF: {e}")

    # Временная динамика (тиков). RawRMSE (инерциальный дрейф без коррекции)
    # и FinalRMSE (после GD+EKF) обычно отличаются на 2-3 порядка — на общей
    # линейной шкале FinalRMSE визуально сливается с нулём, что выглядит как
    # "система не работает", хотя это и есть цель компенсации. Разносим их
    # по отдельным панелям с независимым масштабом.
    if os.path.exists(TICK_CSV):
        try:
            df = pd.read_csv(TICK_CSV)
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11))

            ax1.plot(df['Tick'], df['RawRMSE'], label='Raw RMSE (без коррекции)', marker='o', color='red')
            ax1.set_xlabel('Тик')
            ax1.set_ylabel('RMSE (м)')
            ax1.set_title('Сырая ошибка (инерциальный дрейф BelievedCoord)')
            ax1.grid(True)
            ax1.legend()

            ax2.plot(df['Tick'], df['FinalRMSE'], label='Final RMSE (после GD+EKF)', marker='s', color='green')
            ax2.set_xlabel('Тик')
            ax2.set_ylabel('RMSE (м)')
            ax2.set_title('Ошибка после коррекции дальномерами')
            ax2.grid(True)
            ax2.legend()

            ax3.plot(df['Tick'], df['CompensationRatio'], marker='^', color='blue')
            ax3.set_xlabel('Тик')
            ax3.set_ylabel('Compensation Ratio')
            ax3.set_title('Эффективность компенсации (RawRMSE / FinalRMSE)')
            ax3.grid(True)

            plt.tight_layout()
            img_path = os.path.join(IMG_DIR, f"ticks_{run_id}.png")
            plt.savefig(img_path, dpi=100)
            plt.close()
            plots['ticks'] = img_path
        except Exception as e:
            print(f"Ошибка при построении графика тиков: {e}")

    return plots


def generate_markdown(commit, config_path, output, plots, timestamp):
    """Генерирует запись в report.md."""
    lines = []
    lines.append(f"## Тест ({timestamp})")
    lines.append(f"**Коммит:** `{commit}`")
    lines.append(f"**Конфиг:** `{config_path}`")

    lines.append("\n### Консоль")
    lines.append("```")
    lines.append(output)
    lines.append("```")

    for key, path in plots.items():
        lines.append(f"\n![{key}]({path})")

    lines.append("\n---\n")

    with open(MD_REPORT_PATH, "a", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"Отчёт добавлен в {MD_REPORT_PATH}")


def find_configs():
    """Список конфигов config/*.toml."""
    configs = []
    for f in sorted(os.listdir(CONFIG_DIR)):
        if f.endswith(".toml"):
            configs.append(os.path.join(CONFIG_DIR, f))
    return configs


def run_and_report(config_path):
    """Прогон одного конфига с графиками и записью в отчёт."""
    output = run_go_simulation(config_path)
    if output is None:
        return

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    plots = plot_metrics(run_id)

    generate_markdown(
        get_git_commit(),
        config_path,
        output,
        plots,
        datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )


def menu_single_config():
    """Тест одного конфига."""
    configs = find_configs()
    if not configs:
        print("Конфиги не найдены")
        return

    print("\nДоступные конфиги:")
    for i, cfg in enumerate(configs, 1):
        print(f"  {i}. {cfg}")

    while True:
        try:
            choice = input("\nВыберите номер (или Enter для отмены): ").strip()
            if not choice:
                print("Отменено")
                return
            idx = int(choice) - 1
            if 0 <= idx < len(configs):
                run_and_report(configs[idx])
                return
            print("Неверный выбор")
        except ValueError:
            print("Введите число")


def menu_batch():
    """Пакетный запуск всех конфигов."""
    configs = find_configs()
    if not configs:
        print("Конфиги не найдены")
        return

    print(f"\nНайдено {len(configs)} конфигов")
    for cfg in configs:
        print(f"\n{'='*60}")
        print(f"Тестируем: {cfg}")
        run_and_report(cfg)

    print("\nТестирование завершено")


def main_menu():
    """Главное меню."""
    os.makedirs(IMG_DIR, exist_ok=True)

    while True:
        print(f"""
TESTBENCH - Vostok
Тестирование сети локализации

1. Запуск конфига
2. Пакетный запуск всех конфигов
0. Выход

Текущий коммит: {get_git_commit()}
""")

        choice = input("Выберите действие: ").strip()

        if choice == "1":
            menu_single_config()
        elif choice == "2":
            menu_batch()
        elif choice == "0":
            break
        else:
            print("Неверный выбор")


if __name__ == "__main__":
    main_menu()
