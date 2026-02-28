import json
import subprocess
import urllib
from pathlib import Path, PurePosixPath
from urllib.parse import quote, unquote, urlparse, urlsplit

import lxml.html
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

_PAGE_SOURCE_BY_URL = {}


def is_external_or_anchor_link(link):
    parsed = urlparse(link)
    return bool(parsed.scheme or parsed.netloc or link.startswith("#") or link.startswith("/"))


def is_docs_managed_link(link_path):
    return link_path.lower().endswith(DOCS_MANAGED_SUFFIXES)


def resolve_git_head_commit(repo_root):
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
    return commit


def register_page_source(page_url, page_src_path):
    normalized = page_url.lstrip("/")
    _PAGE_SOURCE_BY_URL[normalized] = page_src_path
    if normalized.endswith("/"):
        _PAGE_SOURCE_BY_URL[normalized.rstrip("/")] = page_src_path


def page_url_candidates_from_html(site_dir, html_path):
    relative = html_path.relative_to(site_dir).as_posix()
    if relative == "index.html":
        return ("", "/", "index.html")
    if relative.endswith("/index.html"):
        base = relative[: -len("index.html")]
        return (base, base.rstrip("/"), base.lstrip("/"), base.lstrip("/").rstrip("/"))
    return (relative, relative.lstrip("/"))


def to_commit_pinned_href(href, page_src_path, html_path, site_dir, repo_root, repo_url, commit):
    if not href or is_external_or_anchor_link(href):
        return href

    parsed = urlsplit(href)
    path_part = unquote(parsed.path)
    if not path_part or is_docs_managed_link(path_part):
        return href

    site_target_path = (html_path.parent / path_part).resolve()
    try:
        site_target_path.relative_to(site_dir)
        if site_target_path.exists():
            return href
    except ValueError:
        pass

    resolved_path = (repo_root / Path(page_src_path).parent / path_part).resolve()
    try:
        repo_relative_path = resolved_path.relative_to(repo_root)
    except ValueError:
        return href

    if not resolved_path.exists():
        return href

    is_directory_link = path_part.endswith("/") or resolved_path.is_dir()
    git_endpoint = "tree" if is_directory_link else "blob"
    normalized_repo_url = repo_url[:-4] if repo_url.endswith(".git") else repo_url

    converted = (
        f"{normalized_repo_url}/{git_endpoint}/{commit}/"
        f"{quote(PurePosixPath(repo_relative_path).as_posix(), safe='/')}"
    )
    if parsed.query:
        converted += f"?{parsed.query}"
    if parsed.fragment:
        converted += f"#{parsed.fragment}"
    return converted


def rewrite_html_links_to_commit_pinned(html_path, site_dir, repo_root, repo_url, commit):
    page_src_path = None
    for url_key in page_url_candidates_from_html(site_dir, html_path):
        if url_key in _PAGE_SOURCE_BY_URL:
            page_src_path = _PAGE_SOURCE_BY_URL[url_key]
            break
    if not page_src_path:
        return

    document = lxml.html.parse(str(html_path))
    changed = False
    for anchor in document.iter("a"):
        href = anchor.get("href")
        rewritten = to_commit_pinned_href(
            href=href,
            page_src_path=page_src_path,
            html_path=html_path,
            site_dir=site_dir,
            repo_root=repo_root,
            repo_url=repo_url,
            commit=commit,
        )
        if rewritten != href:
            anchor.set("href", rewritten)
            changed = True

    if changed:
        doctype = document.docinfo.doctype or "<!DOCTYPE html>"
        html_path.write_text(
            lxml.html.tostring(document, encoding="unicode", method="html", doctype=doctype),
            encoding="utf-8",
        )


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
    register_page_source(env.page.file.url, env.page.file.src_path)


def on_post_build(env):
    repo_url = env.conf.get("repo_url")
    if not repo_url:
        raise RuntimeError("repo_url is not configured; cannot generate commit-pinned links.")

    repo_root = Path(env.variables["git"]["root_dir"]).resolve()
    site_dir = Path(env.conf["site_dir"]).resolve()
    commit = resolve_git_head_commit(repo_root)

    for html_path in site_dir.rglob("*.html"):
        rewrite_html_links_to_commit_pinned(
            html_path=html_path,
            site_dir=site_dir,
            repo_root=repo_root,
            repo_url=repo_url,
            commit=commit,
        )
