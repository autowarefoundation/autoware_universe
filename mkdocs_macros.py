import json
import os
import re
import urllib

from tabulate import tabulate
import yaml

# This file is for defining macros for mkdocs-macros plugin
# Check https://mkdocs-macros-plugin.readthedocs.io/en/latest/macros/ for the details


def format_param_type(param_type):
    if param_type == "number":
        return "float"
    else:
        return param_type


# Identifiers (parameter names, topic paths, message types, file paths) carry no
# space or hyphen: their separators are the only line-break opportunities. A
# separator followed by a digit is kept unbreakable so numeric values stay intact.
SEPARATOR_BREAK_PATTERN = re.compile(r"([._/])(?!\d)")


def add_break_opportunities(text):
    """Mark the separators of an unbreakable identifier as line-break opportunities."""
    return SEPARATOR_BREAK_PATTERN.sub(r"\1<wbr>", text)


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
            param["Name"] = add_break_opportunities(namespace + k)
            param["Type"] = format_param_type(resolved.get("type", "N/A"))
            param["Description"] = resolved.get("description", "")
            param["Default"] = add_break_opportunities(format_cell(resolved.get("default")))
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


# ---------------------------------------------------------------------------
# autoware_system_design_format `*.node.yaml` rendering
#
# Renders the interface and parameter sections of a node design file
# (see schema autoware_system_designer/schema/<version>/node.json) into
# Markdown tables, the same way json_to_markdown renders a JSON schema.
# ---------------------------------------------------------------------------


def format_cell(value):
    """Format a YAML scalar/collection into a single Markdown table cell."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, dict)):
        # render nested qos/structures compactly without breaking the table
        text = json.dumps(value)
    else:
        text = str(value)
    # escape pipe so it does not break the Markdown table, keep on one line
    return text.replace("|", "\\|").replace("\n", "<br/>")


def format_qos(qos):
    """Render a qos mapping as a compact, readable cell."""
    if not qos:
        return ""
    return "<br/>".join(f"{k}: {format_cell(v)}" for k, v in qos.items())


def format_code(value):
    """Format an identifier as inline code, breakable at its separators."""
    text = format_cell(value)
    if not text:
        return ""
    return f"<code>{add_break_opportunities(text)}</code>"


def build_table(rows, columns):
    """Build a GitHub Markdown table for the given column spec.

    ``columns`` is a list of ``(header, key)`` tuples. A column is dropped
    entirely when none of the rows provide a value for it, so optional fields
    (description, qos, global, ...) only appear when actually used.
    """
    used = [(header, key) for header, key in columns if any(r.get(key) for r in rows)]
    table = []
    for row in rows:
        table.append({header: row.get(key, "") for header, key in used})
    if not table:
        return ""
    # cspell: ignore tablefmt
    return tabulate(table, headers="keys", tablefmt="github")


def extract_interface_rows(entries):
    """Normalize subscriber/publisher/server/client entries into table rows."""
    rows = []
    for entry in entries or []:
        rows.append(
            {
                "Name": format_code(entry["name"]),
                "Type": format_code(entry["message_type"]),
                "Description": format_cell(entry.get("description")),
                "Topic": format_code(entry.get("global")),
                "Remap": format_code(entry.get("remap_target")),
                "QoS": format_qos(entry.get("qos")),
            }
        )
    return rows


INTERFACE_COLUMNS = [
    ("Name", "Name"),
    ("Type", "Type"),
    ("Description", "Description"),
    ("Topic (global)", "Topic"),
    ("Remap", "Remap"),
    ("QoS", "QoS"),
]


def format_interface_table(entries):
    return build_table(extract_interface_rows(entries), INTERFACE_COLUMNS)


def format_param_values_table(param_values):
    rows = []
    for param in param_values or []:
        rows.append(
            {
                "Name": format_code(param["name"]),
                "Type": format_param_type(param.get("type", "")),
                "Default": format_code(param.get("default")),
                "Description": format_cell(param.get("description")),
            }
        )
    return build_table(
        rows,
        [
            ("Name", "Name"),
            ("Type", "Type"),
            ("Default", "Default"),
            ("Description", "Description"),
        ],
    )


def format_param_files_table(param_files):
    # param_files may be a mapping (legacy) or a list of entries
    if isinstance(param_files, dict):
        param_files = [{"name": k, "default": v} for k, v in param_files.items()]
    rows = []
    for entry in param_files or []:
        rows.append(
            {
                "Name": format_code(entry["name"]),
                "Default": format_code(entry.get("default")),
                "Schema": format_code(entry.get("schema")),
                "Description": format_cell(entry.get("description")),
            }
        )
    return build_table(
        rows,
        [
            ("Name", "Name"),
            ("Default", "Default"),
            ("Schema", "Schema"),
            ("Description", "Description"),
        ],
    )


def format_node_yaml(data, heading_level=3):
    """Render the interface and parameter sections of a node design file."""
    sections = []
    heading = "#" * heading_level

    def add_section(title, body):
        if body:
            sections.append(f"{heading} {title}\n\n{body}")

    add_section("Subscribers", format_interface_table(data.get("subscribers")))
    add_section("Publishers", format_interface_table(data.get("publishers")))
    add_section("Service servers", format_interface_table(data.get("servers")))
    add_section("Service clients", format_interface_table(data.get("clients")))
    add_section("Parameter files", format_param_files_table(data.get("param_files")))
    add_section("Parameters", format_param_values_table(data.get("param_values")))

    return "\n\n".join(sections)


def define_env(env):
    @env.macro
    def json_to_markdown(json_schema_file_path):
        with open(json_schema_file_path) as f:
            data = json.load(f)
            return format_json(data, os.path.dirname(json_schema_file_path))

    @env.macro
    def node_to_markdown(node_yaml_file_path, heading_level=3):
        """Render all interface + parameter tables of a `*.node.yaml` file."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        return format_node_yaml(data, heading_level)

    @env.macro
    def node_interfaces_to_markdown(node_yaml_file_path, heading_level=3):
        """Render only the subscriber/publisher/server/client tables."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        heading = "#" * heading_level
        sections = []
        for title, key in [
            ("Subscribers", "subscribers"),
            ("Publishers", "publishers"),
            ("Service servers", "servers"),
            ("Service clients", "clients"),
        ]:
            table = format_interface_table(data.get(key))
            if table:
                sections.append(f"{heading} {title}\n\n{table}")
        return "\n\n".join(sections)

    @env.macro
    def node_param_values_to_markdown(node_yaml_file_path):
        """Render only the `param_values` table of a `*.node.yaml` file."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        return format_param_values_table(data.get("param_values"))

    @env.macro
    def node_param_files_to_markdown(node_yaml_file_path):
        """Render only the `param_files` table of a `*.node.yaml` file."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        return format_param_files_table(data.get("param_files"))

    @env.macro
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"
