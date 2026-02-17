import json
import re
import subprocess
import urllib
from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

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


def extract_parameter_info(parameters, namespace=""):
    params = []
    for k, v in parameters.items():
        if "$ref" in v.keys():
            continue
        if v["type"] != "object":
            param = {}
            param["Name"] = namespace + k
            param["Type"] = format_param_type(v["type"])
            param["Description"] = v["description"]
            param["Default"] = v["default"]
            param["Range"] = format_param_range(v)
            params.append(param)
        else:  # if the object is namespace, then dive deeper in to json value
            params.extend(extract_parameter_info(v["properties"], k + "."))
    return params


def format_json(json_data):
    parameters = list(json_data["definitions"].values())[0]["properties"]
    # cspell: ignore tablefmt
    markdown_table = tabulate(extract_parameter_info(parameters), headers="keys", tablefmt="github")
    return markdown_table


DOCS_MANAGED_SUFFIXES = (
    ".md",
    ".markdown",
    ".png",
    ".jpg",
    ".jpeg",
    ".gif",
    ".svg",
    ".webp",
)

INLINE_LINK_PATTERN = re.compile(r"(?P<prefix>!?\[[^\]]+\]\()(?P<target>[^)\n]+)(?P<suffix>\))")
REFERENCE_LINK_PATTERN = re.compile(
    r"^(?P<prefix>\s{0,3}\[[^\]]+\]:\s*)(?P<target>\S+(?:\s+\"[^\"]*\")?)(?P<suffix>\s*)$"
)


def split_markdown_link_target(target):
    trimmed = target.strip()
    if not trimmed:
        return "", ""

    # Support markdown angle-bracket links: [label](<path with spaces>)
    if trimmed.startswith("<"):
        end = trimmed.find(">")
        if end != -1:
            return trimmed[1:end], trimmed[end + 1 :]

    parts = trimmed.split(maxsplit=1)
    destination = parts[0]
    suffix = f" {parts[1]}" if len(parts) > 1 else ""
    return destination, suffix


def is_external_or_anchor_link(link):
    parsed = urlparse(link)
    return bool(parsed.scheme or parsed.netloc or link.startswith("#") or link.startswith("/"))


def is_docs_managed_link(link_path):
    return link_path.lower().endswith(DOCS_MANAGED_SUFFIXES)


def to_commit_pinned_repo_link(link_target, page_src_path, repo_root, repo_url, commit):
    destination, suffix = split_markdown_link_target(link_target)
    if not destination or is_external_or_anchor_link(destination):
        return link_target

    path_part, has_fragment, fragment = destination.partition("#")
    if not path_part or is_docs_managed_link(path_part):
        return link_target

    resolved_path = (repo_root / Path(page_src_path).parent / path_part).resolve()
    try:
        repo_relative_path = resolved_path.relative_to(repo_root)
    except ValueError:
        return link_target

    is_directory_link = path_part.endswith("/") or resolved_path.is_dir()
    git_endpoint = "tree" if is_directory_link else "blob"
    normalized_repo_url = repo_url[:-4] if repo_url.endswith(".git") else repo_url

    converted = f"{normalized_repo_url}/{git_endpoint}/{commit}/{PurePosixPath(repo_relative_path).as_posix()}"
    if has_fragment:
        converted += f"#{fragment}"
    return f"{converted}{suffix}"


def rewrite_non_doc_links_to_commit_pinned(markdown, page_src_path, repo_root, repo_url, commit):
    in_fence = False
    fence_marker = None
    output_lines = []

    for line in markdown.splitlines(keepends=True):
        stripped = line.lstrip()
        starts_fence = stripped.startswith("```") or stripped.startswith("~~~")
        if starts_fence:
            current_marker = "```" if stripped.startswith("```") else "~~~"
            if not in_fence:
                in_fence = True
                fence_marker = current_marker
            elif current_marker == fence_marker:
                in_fence = False
                fence_marker = None
            output_lines.append(line)
            continue

        if in_fence:
            output_lines.append(line)
            continue

        def inline_replacer(match):
            updated_target = to_commit_pinned_repo_link(
                link_target=match.group("target"),
                page_src_path=page_src_path,
                repo_root=repo_root,
                repo_url=repo_url,
                commit=commit,
            )
            return f"{match.group('prefix')}{updated_target}{match.group('suffix')}"

        line = INLINE_LINK_PATTERN.sub(inline_replacer, line)

        def reference_replacer(match):
            updated_target = to_commit_pinned_repo_link(
                link_target=match.group("target"),
                page_src_path=page_src_path,
                repo_root=repo_root,
                repo_url=repo_url,
                commit=commit,
            )
            return f"{match.group('prefix')}{updated_target}{match.group('suffix')}"

        line = REFERENCE_LINK_PATTERN.sub(reference_replacer, line)
        output_lines.append(line)

    return "".join(output_lines)


def define_env(env):
    @env.macro
    def json_to_markdown(json_schema_file_path):
        with open(json_schema_file_path) as f:
            data = json.load(f)
            return format_json(data)

    @env.macro
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url']}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"


def on_post_page_macros(env):
    git_info = env.variables["git"]
    repo_url = env.conf.get("repo_url")
    if not repo_url:
        raise RuntimeError("repo_url is not configured; cannot generate commit-pinned links.")

    repo_root = Path(git_info["root_dir"]).resolve()
    try:
        commit = (
            subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
            )
            .stdout.strip()
        )
    except subprocess.CalledProcessError as exc:
        raise RuntimeError("Failed to resolve git HEAD commit for commit-pinned links.") from exc

    if not commit:
        raise RuntimeError("git HEAD commit is empty; cannot generate commit-pinned links.")

    env.markdown = rewrite_non_doc_links_to_commit_pinned(
        markdown=env.markdown,
        page_src_path=env.page.file.src_path,
        repo_root=repo_root,
        repo_url=repo_url,
        commit=commit,
    )
