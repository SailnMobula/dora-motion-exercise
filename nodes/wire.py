"""One JSON document per dora message, carried as a single string in an arrow array."""

from __future__ import annotations

import json

import pyarrow as pa
from dora import Node


def send(node: Node, output: str, message: dict) -> None:
    node.send_output(output, pa.array([json.dumps(message)]))


def read(event: dict) -> dict:
    return json.loads(event["value"][0].as_py())


def wired_outputs(node: Node, node_id: str) -> set[str]:
    """Outputs of this node that another node in the dataflow takes as an input."""
    wired: set[str] = set()
    for other in node.dataflow_descriptor().get("nodes", []):
        for source in (other.get("inputs") or {}).values():
            source = source["source"] if isinstance(source, dict) else source
            producer, _, output = str(source).partition("/")
            if producer == node_id:
                wired.add(output)
    return wired


def wired_inputs(node: Node, node_id: str) -> set[str]:
    for entry in node.dataflow_descriptor().get("nodes", []):
        if entry.get("id") == node_id:
            return set((entry.get("inputs") or {}).keys())
    return set()
