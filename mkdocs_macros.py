import json
import os
import urllib

from tabulate import tabulate

# This file is for defining macros for mkdocs-macros plugin
# Check https://mkdocs-macros-plugin.readthedocs.io/en/latest/macros/ for the details


def format_param_type(param_type):
    if param_type == "number":
        return "float"
    else:
        return param_type


def format_param_range(param):
    list_of_range = []
    if "enum" in param.keys():
        list_of_range.append(param["enum"])
    if "minimum" in param.keys():
        list_of_range.append("≥" + str(param["minimum"]))
    if "exclusiveMinimum" in param.keys():
        list_of_range.append(">" + str(param["exclusiveMinimum"]))
    if "maximum" in param.keys():
        list_of_range.append("≤" + str(param["maximum"]))
    if "exclusiveMaximum" in param.keys():
        list_of_range.append("<" + str(param["exclusiveMaximum"]))
    if "exclusive" in param.keys():
        list_of_range.append("≠" + str(param["exclusive"]))

    if len(list_of_range) == 0:
        return "N/A"
    else:
        range_in_text = ""
        for item in list_of_range:
            if range_in_text != "":
                range_in_text += "<br/>"
            range_in_text += str(item)
        return range_in_text


def resolve_ref(node, doc, base_dir, cache):
    """Resolve a JSON-schema $ref and return ``(resolved_node, doc, base_dir)``.

    Handles both local refs ("#/definitions/foo") and external-file refs
    ("sub/bar.json#/definitions/baz", resolved relative to ``base_dir``). The
    returned ``doc`` / ``base_dir`` describe the context the resolved node lives in,
    so nested refs keep resolving correctly. Sibling keys placed next to the $ref
    (e.g. a per-use "description" or "default") take precedence over the referenced
    definition. The node is returned unchanged when it carries no resolvable $ref.
    """
    seen = set()
    while isinstance(node, dict) and "$ref" in node:
        ref = node["$ref"]
        if not isinstance(ref, str) or ref in seen:
            break
        seen.add(ref)
        file_part, _, fragment = ref.partition("#")
        if file_part:  # external-file ref: load relative to the current base_dir
            path = os.path.normpath(os.path.join(base_dir, file_part))
            if path not in cache:
                try:
                    with open(path) as f:
                        cache[path] = json.load(f)
                except (OSError, ValueError):
                    return node, doc, base_dir
            doc = cache[path]
            base_dir = os.path.dirname(path)
        target = doc
        for part in fragment.split("/"):
            if part == "":
                continue
            if not isinstance(target, dict) or part not in target:
                return node, doc, base_dir
            target = target[part]
        node = {**target, **{k: v for k, v in node.items() if k != "$ref"}}
    return node, doc, base_dir


def extract_parameter_info(parameters, doc, base_dir, cache, namespace="", seen_refs=frozenset()):
    params = []
    for k, v in parameters.items():
        if not isinstance(v, dict):
            continue
        # Resolve "$ref" children so parameters grouped into sub-definitions are
        # documented instead of skipped; guard against cyclic references.
        ref = v.get("$ref")
        if ref and ref in seen_refs:
            continue
        resolved, child_doc, child_base = resolve_ref(v, doc, base_dir, cache)
        child_seen = seen_refs | {ref} if ref else seen_refs
        # Dive into a namespace only when it actually carries nested properties;
        # tolerate entries without an explicit "type" / "description" / "default".
        if resolved.get("type") == "object" and "properties" in resolved:
            params.extend(
                extract_parameter_info(
                    resolved["properties"],
                    child_doc,
                    child_base,
                    cache,
                    namespace + k + ".",
                    child_seen,
                )
            )
        else:
            param = {}
            param["Name"] = namespace + k
            param["Type"] = format_param_type(resolved.get("type", "N/A"))
            param["Description"] = resolved.get("description", "")
            param["Default"] = resolved.get("default", "")
            param["Range"] = format_param_range(resolved)
            params.append(param)
    return params


def format_json(json_data, base_dir=""):
    cache = {}
    # Prefer the parameter set under ".../ros__parameters" (referenced or inlined):
    # a schema may declare helper sub-definitions ahead of the node definition, so
    # picking the first definition with properties would document the wrong
    # (helper) parameter set.
    parameters = None
    param_doc, param_base = json_data, base_dir
    for top in json_data.get("properties", {}).values():
        top_resolved, top_doc, top_base = resolve_ref(top, json_data, base_dir, cache)
        ros_params = top_resolved.get("properties", {}).get("ros__parameters")
        if isinstance(ros_params, dict):
            resolved, r_doc, r_base = resolve_ref(ros_params, top_doc, top_base, cache)
            if "properties" in resolved:
                parameters, param_doc, param_base = resolved["properties"], r_doc, r_base
                break
    if parameters is None:
        # Fallback for schemas that inline their parameters (e.g. component templates):
        # use the first definition that actually exposes parameters (skip enum-only ones).
        definitions = list(json_data.get("definitions", {}).values())
        parameters = next((d["properties"] for d in definitions if "properties" in d), None)
    if parameters is None:
        # Last resort for flat schemas that declare parameters directly at the top
        # level (no "/**"/"ros__parameters" wrapper, no "definitions").
        top_properties = json_data.get("properties", {})
        if "/**" not in top_properties:
            parameters = top_properties
    if parameters is None:
        return ""
    # cspell: ignore tablefmt
    markdown_table = tabulate(
        extract_parameter_info(parameters, param_doc, param_base, cache),
        headers="keys",
        tablefmt="github",
    )
    return markdown_table


def define_env(env):
    @env.macro
    def json_to_markdown(json_schema_file_path):
        with open(json_schema_file_path) as f:
            data = json.load(f)
            return format_json(data, os.path.dirname(json_schema_file_path))

    @env.macro
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"
