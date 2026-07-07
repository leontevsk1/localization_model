import os
import subprocess
import toml
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# Настройки путей
CONFIG_PATH = "config/test1.toml"
GO_COMMAND = ["go", "run", ".", "-config", CONFIG_PATH]
MD_REPORT_PATH = "report.md"
GRADIENT_CSV = "mod1_history.csv"
EKF_CSV = "ekf_history.csv"
IMG_DIR = "report_images"


def get_git_commit():
    """Получает хеш текущего коммита git."""
    try:
        commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()
        return commit
    except Exception:
        return "Git-репозиторий не найден или нет коммитов"


def run_go_simulation():
    print("🚀 Запуск Go-симуляции...")
    result = subprocess.run(GO_COMMAND, capture_output=True, text=True)
    print(result.stdout)
    if result.returncode != 0:
        print("❌ Ошибка компиляции или выполнения Go:")
        print(result.stderr)
        exit(1)
    print("✅ Симуляция успешно завершена.\n")
    return result.stdout


def plot_gradient_descent(run_id):
    """Строит график сходимости градиентного спуска. run_id делает имя файла уникальным для каждого запуска."""
    if not os.path.exists(GRADIENT_CSV):
        return ""

    df = pd.read_csv(GRADIENT_CSV)

    plt.figure(figsize=(10, 5))
    plt.plot(df['Iteration'], df['Node0_X'], label='Node 0 X-coord', color='blue')
    plt.title('Сходимость Градиентного спуска (Узел 0)')
    plt.xlabel('Итерация')
    plt.ylabel('Координата X')
    plt.grid(True)
    plt.legend()

    img_path = os.path.join(IMG_DIR, f"gradient_plot_{run_id}.png")
    plt.savefig(img_path)
    plt.close()
    return img_path


def plot_ekf(run_id):
    """Строит график стабилизации EKF. run_id делает имя файла уникальным для каждого запуска."""
    if not os.path.exists(EKF_CSV):
        return ""

    df = pd.read_csv(EKF_CSV)

    if 'Iteration' not in df.columns or 'Node0_X' not in df.columns:
        print(f"⚠️ Неожиданные колонки в {EKF_CSV}: {list(df.columns)}")
        return ""

    plt.figure(figsize=(10, 5))
    plt.plot(df['Iteration'], df['Node0_X'], label='Node 0 X (EKF)', color='green')
    plt.title('Сглаживание координат фильтром Калмана')
    plt.xlabel('Итерация EKF')
    plt.ylabel('Координата X')
    plt.grid(True)
    plt.legend()

    img_path = os.path.join(IMG_DIR, f"ekf_plot_{run_id}.png")
    plt.savefig(img_path)
    plt.close()
    return img_path


def generate_markdown(commit_hash, config_data, go_output, grad_img, ekf_img, timestamp):
    """Добавляет запись о новом испытании в конец report.md, не затирая предыдущие."""
    toml_str = toml.dumps(config_data)

    lines = []
    lines.append(f"# Результаты испытания ({timestamp})")
    lines.append(f"**Версия программы (Git Commit):** `{commit_hash}`\n")
    lines.append("## Параметры симуляции")
    lines.append("```toml")
    lines.append(toml_str)
    lines.append("```\n")
    lines.append("## Вывод консоли")
    lines.append("```text")
    lines.append(go_output)
    lines.append("```\n")

    lines.append("## Результаты Градиентного спуска")
    if grad_img:
        lines.append(f"![Градиентный спуск]({grad_img})")
    else:
        lines.append("*Файл CSV градиентного спуска не найден.*")

    lines.append("\n## Результаты Фильтра Калмана (EKF)")
    if ekf_img:
        lines.append(f"![EKF]({ekf_img})")
    else:
        lines.append("*Файл CSV EKF не найден.*")

    lines.append("\n---\n")  # разделитель между испытаниями

    file_exists = os.path.exists(MD_REPORT_PATH)
    with open(MD_REPORT_PATH, "a", encoding="utf-8") as f:
        if file_exists:
            f.write("\n")  # отступ перед новой записью, если файл уже не пуст
        f.write("\n".join(lines))
        f.write("\n")

    print(f"📄 Отчёт дополнен новой записью в {MD_REPORT_PATH}")


if __name__ == "__main__":
    os.makedirs(IMG_DIR, exist_ok=True)

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    commit_hash = get_git_commit()
    config_data = toml.load(CONFIG_PATH)

    go_output = run_go_simulation()

    grad_img = plot_gradient_descent(run_id)
    ekf_img = plot_ekf(run_id)

    generate_markdown(commit_hash, config_data, go_output, grad_img, ekf_img, timestamp)
