"""Writes exercises/NN/dataflow.yml from solutions/NN/dataflow.yml with the wiring commented out.

Timers and env stay. Every other input becomes `# name:`, so the dataflow starts as it is and a
learner uncomments a line and adds <node>/<output>. Run after changing a solution.

    uv run python tools/make_exercises.py
"""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
HINT = "# Wire it: uncomment an input and give it one node's output, written <node>/<output>."


def skeleton(solution: Path) -> str:
    text = solution.read_text()
    flow = yaml.safe_load(text)
    lines = [text.splitlines()[0], HINT, "nodes:"]
    for node in flow["nodes"]:
        lines += [f"  - id: {node['id']}", f"    path: {node['path']}"]
        if "env" in node:
            lines.append("    env:")
            lines += [f"      {key}: {value}" for key, value in node["env"].items()]
        lines.append("    inputs:")
        for name, source in node["inputs"].items():
            lines.append(f"      {name}: {source}" if str(source).startswith("dora/") else f"      # {name}:")
        lines += [f"    outputs: [{', '.join(node['outputs'])}]", ""]
    return "\n".join(lines)


def main() -> None:
    for solution in sorted((ROOT / "solutions").glob("0*/dataflow.yml")):
        target = ROOT / "exercises" / solution.parent.name / "dataflow.yml"
        target.write_text(skeleton(solution))
        print(target.relative_to(ROOT))


if __name__ == "__main__":
    main()
