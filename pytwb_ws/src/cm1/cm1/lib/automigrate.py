import re
import sys
from pathlib import Path

def generate_behavior_classes(input_file: str, output_file: str = "mig_test.py"):
    text = Path(input_file).read_text(encoding="utf-8")

    # @actor 直後の関数定義を抽出
    pattern = re.compile(
        r"@actor\s*\n\s*def\s+([A-Za-z_]\w*)\s*\(([^)]*)\)",
        re.MULTILINE
    )

    class_definitions = []

    for match in pattern.finditer(text):
        func_name = match.group(1)
        args_str = match.group(2).strip()

        # 引数を整形
        args = [a.strip() for a in args_str.split(",") if a.strip()]
        # self を除外
        args = [a for a in args if not a.startswith("self")]

        # クラス名（関数名の先頭を大文字に）
        class_name = func_name[0].upper() + func_name[1:]

        # 引数リストを出力形式に整形
        arg_list = ", ".join(args)

        # クラス定義テンプレート
        template = f"""@behavior
class {class_name}(ActorBT):
    def __init__(self, name, node{', ' + arg_list if arg_list else ''}):
        super().__init__(name, '{func_name}'{', ' + arg_list if arg_list else ''})

"""

        class_definitions.append(template)

    # 出力
    output_text = (
        "# Auto-generated from @actor functions\n"
        "# Do not edit manually\n\n"
        "from some_module import ActorBT, behavior\n\n"
        + "\n".join(class_definitions)
    )

    Path(output_file).write_text(output_text, encoding="utf-8")

    print(f"✅ Generated {len(class_definitions)} behavior classes → '{output_file}'")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python generate_behavior_classes.py <python_file> [output_file]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "mig_test.py"

    generate_behavior_classes(input_file, output_file)
