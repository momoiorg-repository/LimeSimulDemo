import re
import csv
import sys
from pathlib import Path

def tidyup(filename: str, output_csv: str = "actor_functions.csv"):
    # ファイルの読み込み
    text = Path(filename).read_text(encoding="utf-8")

    # 正規表現パターン
    # - @actor の直後に def がある関数を抽出
    pattern = re.compile(
        r"@actor\s*\n\s*def\s+([A-Za-z_]\w*)\s*\(([^)]*)\)\s*(?:->\s*([^\s:]+))?:",
        re.MULTILINE
    )

    results = []

    for match in pattern.finditer(text):
        func_name = match.group(1)
        args_str = match.group(2).strip()
        return_type = match.group(3) if match.group(3) else ""

        # 引数をカンマ区切りでリスト化（空の場合は空リスト）
        args = [arg.strip() for arg in args_str.split(",")] if args_str else []

        results.append((func_name, args, return_type))

    # CSV出力
    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Function Name", "Arguments", "Return Type"])
        for func_name, args, return_type in results:
            writer.writerow([func_name, "; ".join(args), return_type])

    print(f"✅ Extracted {len(results)} @actor functions from '{filename}' → saved to '{output_csv}'")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python tidyup.py <python_file> [output_csv]")
        sys.exit(1)

    filename = sys.argv[1]
    output_csv = sys.argv[2] if len(sys.argv) > 2 else "actor_functions.csv"

    tidyup(filename, output_csv)
