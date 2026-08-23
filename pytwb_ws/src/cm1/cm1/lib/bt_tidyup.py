import re
import csv
import sys
from pathlib import Path

def extract_behavior_classes(filename: str, output_csv: str = "behavior_classes.csv"):
    # ファイルを読み込み
    text = Path(filename).read_text(encoding="utf-8")

    # @behavior の直後にあるクラス定義を抽出
    class_pattern = re.compile(
        r"@behavior\s*\n\s*class\s+([A-Za-z_]\w*)\s*\([^)]*\):([\s\S]*?)(?=\n\s*@behavior|\Z)",
        re.MULTILINE
    )

    init_pattern = re.compile(
        r"def\s+__init__\s*\(\s*self\s*,\s*name\s*,\s*node\s*(?:,\s*([^)]*))?\)"
    )

    results = []

    for class_match in class_pattern.finditer(text):
        class_name = class_match.group(1)
        class_body = class_match.group(2)

        init_match = init_pattern.search(class_body)
        if not init_match:
            continue

        extra_args = init_match.group(1)
        if extra_args:
            args = [arg.strip() for arg in extra_args.split(",") if arg.strip()]
        else:
            args = []

        results.append((class_name, args))

    # CSV出力
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Class Name", "Arguments"])
        for cls_name, args in results:
            writer.writerow([cls_name, "; ".join(args)])

    print(f"✅ Extracted {len(results)} @behavior classes from '{filename}' → saved to '{output_csv}'")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_behavior_classes.py <python_file> [output_csv]")
        sys.exit(1)

    filename = sys.argv[1]
    output_csv = sys.argv[2] if len(sys.argv) > 2 else "behavior_classes.csv"

    extract_behavior_classes(filename, output_csv)
