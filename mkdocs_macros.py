import json
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


def resolve_ref(ref, root):
    """Resolve a local JSON-schema reference like ``#/definitions/foo``."""
    target = root
    for part in ref.lstrip("#/").split("/"):
        target = target[part]
    return target


def extract_parameter_info(parameters, namespace="", root=None, seen=None):
    if seen is None:
        seen = frozenset()
    params = []
    for k, v in parameters.items():
        local_seen = seen
        # Resolve a "$ref" so referenced definitions are documented rather than skipped.
        if "$ref" in v.keys():
            ref = v["$ref"]
            if root is None or ref in seen:  # guard against recursive schemas
                continue
            resolved = dict(resolve_ref(ref, root))
            # keep a default/description supplied at the reference site
            for key in ("default", "description"):
                if key in v:
                    resolved[key] = v[key]
            v = resolved
            local_seen = seen | {ref}
        # Dive into a namespace only when it actually carries nested properties;
        # an "object" without "properties" (e.g. a free-form map) is a leaf.
        if v.get("type") == "object" and "properties" in v:
            params.extend(
                extract_parameter_info(v["properties"], namespace + k + ".", root, local_seen)
            )
        else:
            param = {}
            param["Name"] = namespace + k
            param["Type"] = format_param_type(v.get("type", "N/A"))
            param["Description"] = v.get("description", "")
            param["Default"] = v.get("default", "")
            param["Range"] = format_param_range(v)
            params.append(param)
    return params


def get_parameter_root(json_data):
    """Locate the properties mapping that holds the documented parameters.

    Preferred convention: ``properties -> "/**" -> properties -> ros__parameters``,
    resolving a ``$ref`` to the main definition when present. Falls back to the
    first definition that exposes ``properties`` for older single-definition schemas.
    """
    try:
        ros_params = json_data["properties"]["/**"]["properties"]["ros__parameters"]
        if "$ref" in ros_params:
            return resolve_ref(ros_params["$ref"], json_data)["properties"]
    except (KeyError, TypeError):
        pass
    for definition in json_data.get("definitions", {}).values():
        if "properties" in definition:
            return definition["properties"]
    return {}


def format_json(json_data):
    parameters = get_parameter_root(json_data)
    # cspell: ignore tablefmt
    markdown_table = tabulate(
        extract_parameter_info(parameters, root=json_data), headers="keys", tablefmt="github"
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
                "Name": f"`{entry['name']}`",
                "Type": f"`{entry['message_type']}`",
                "Description": format_cell(entry.get("description")),
                "Topic": f"`{entry['global']}`" if entry.get("global") else "",
                "Remap": f"`{entry['remap_target']}`" if entry.get("remap_target") else "",
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
                "Name": f"`{param['name']}`",
                "Type": format_param_type(param.get("type", "")),
                "Default": format_cell(param.get("default")),
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
                "Name": f"`{entry['name']}`",
                "Default": format_cell(entry.get("default")),
                "Schema": f"`{entry['schema']}`" if entry.get("schema") else "",
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


def format_node_yaml(data):
    """Render the interface and parameter sections of a node design file."""
    sections = []

    def add_section(title, body):
        if body:
            sections.append(f"#### {title}\n\n{body}")

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
            return format_json(data)

    @env.macro
    def node_to_markdown(node_yaml_file_path):
        """Render all interface + parameter tables of a `*.node.yaml` file."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        return format_node_yaml(data)

    @env.macro
    def node_interfaces_to_markdown(node_yaml_file_path):
        """Render only the subscriber/publisher/server/client tables."""
        with open(node_yaml_file_path) as f:
            data = yaml.safe_load(f)
        sections = []
        for title, key in [
            ("Subscribers", "subscribers"),
            ("Publishers", "publishers"),
            ("Service servers", "servers"),
            ("Service clients", "clients"),
        ]:
            table = format_interface_table(data.get(key))
            if table:
                sections.append(f"#### {title}\n\n{table}")
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
