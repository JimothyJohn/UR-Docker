"""The product page (site/, perceptronics.advin.io): what ``site/build.py`` assembles is
the repo's own URCaps and facts, and it satisfies the Content Security Policy the stack
serves it under (``site/cloudformation/static-site.yaml``: no inline script, nothing
loaded from another host) — a violation there is a silently broken page, not an error."""

from __future__ import annotations

import hashlib
import importlib.util
import re
import sys
from html.parser import HTMLParser
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
SITE = REPO / "site"
DIST = REPO / "urcap" / "dist"


def _load_build():
    spec = importlib.util.spec_from_file_location("site_build", SITE / "build.py")
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules["site_build"] = mod
    spec.loader.exec_module(mod)
    return mod


site_build = _load_build()


class _Page(HTMLParser):
    """Every tag with its attributes, in document order."""

    def __init__(self) -> None:
        super().__init__()
        self.tags: list[tuple[str, dict[str, str | None]]] = []

    def handle_starttag(self, tag, attrs):
        self.tags.append((tag, dict(attrs)))

    def all(self, tag: str) -> list[dict[str, str | None]]:
        return [a for t, a in self.tags if t == tag]


@pytest.fixture(scope="module")
def built(tmp_path_factory) -> Path:
    return site_build.build(tmp_path_factory.mktemp("site") / "out")


@pytest.fixture(scope="module")
def pages(built: Path) -> dict[str, _Page]:
    out = {}
    for path in sorted(built.glob("*.html")):
        page = _Page()
        page.feed(path.read_text(encoding="utf-8"))
        out[path.name] = page
    return out


def test_build_writes_both_pages_with_every_placeholder_filled(built: Path):
    assert sorted(p.name for p in built.glob("*.html")) == ["error.html", "index.html"]
    for page in built.glob("*.html"):
        text = page.read_text(encoding="utf-8")
        assert "{{" not in text and "}}" not in text, page.name


def test_downloads_are_the_committed_urcaps_byte_for_byte(built: Path):
    names = sorted(p.name for p in (built / "downloads").iterdir())
    committed = sorted(p.name for p in DIST.iterdir() if p.suffix in (".urcap", ".urcapx"))
    assert names == committed
    for name in names:
        assert (built / "downloads" / name).read_bytes() == (DIST / name).read_bytes()


def test_each_download_link_shows_its_own_file_version_and_checksum(built: Path, pages):
    text = (built / "index.html").read_text(encoding="utf-8")
    links = [a["href"] for a in pages["index.html"].all("a") if "download" in a]
    assert len(links) == 2
    for href in links:
        target = built / href.lstrip("/")
        assert target.is_file(), href
        assert hashlib.sha256(target.read_bytes()).hexdigest() in text
        version = re.search(r"-(\d+\.\d+\.\d+)\.urcapx?$", target.name).group(1)
        assert f"version {version}" in text


def test_supported_ranges_are_the_ci_matrices(built: Path):
    values = site_build.facts()
    assert values["PS5_RANGE"] == "5.4 to 5.26"  # compat.floor .. the newest minor in ps5_matrix
    floor, newest = values["PSX_RANGE"].split(" to ")
    psx = sys.modules["psx_matrix"]
    assert psx.FLOOR.startswith(floor + ".")
    assert psx.RELEASES[0].startswith(newest + ".")
    text = (built / "index.html").read_text(encoding="utf-8")
    assert values["PS5_RANGE"] in text and values["PSX_RANGE"] in text


def test_unknown_placeholder_fails_the_build():
    with pytest.raises(SystemExit, match="NOT_A_FACT"):
        site_build.render("<p>{{NOT_A_FACT}}</p>", site_build.facts())


def test_no_script_and_no_inline_handlers(pages):
    # CSP script-src 'self': an inline <script> or on*= handler is blocked; the page has no JS at all
    for name, page in pages.items():
        assert page.all("script") == [], name
        for tag, attrs in page.tags:
            assert not [k for k in attrs if k.startswith("on")], (name, tag)


def test_nothing_is_loaded_from_another_host(built: Path, pages):
    # CSP default-src 'self', img-src 'self' data:, style-src 'self' 'unsafe-inline'
    for name, page in pages.items():
        loaded = [
            a.get("src") for t, a in page.tags if t in ("img", "iframe", "source", "video", "audio", "embed")
        ]
        loaded += [a.get("href") for a in page.all("link")]
        loaded += [a.get("data") for a in page.all("object")]
        for url in loaded:
            assert url, name
            if url.startswith("data:"):
                continue
            assert url.startswith("/") and not url.startswith("//"), (name, url)
            assert (built / url.lstrip("/")).is_file(), (name, url)
        assert not [a for a in page.all("link") if a.get("rel") == "stylesheet"], name
        text = (built / name).read_text(encoding="utf-8")
        assert "@import" not in text and "@font-face" not in text, name
        assert not re.search(r"url\(\s*['\"]?(?!data:)", text), name


def test_links_leave_only_for_the_repo_and_the_contact_address(pages):
    for name, page in pages.items():
        for a in page.all("a"):
            href = a["href"]
            if href.startswith(("#", "/")) and not href.startswith("//"):
                continue
            assert href.startswith(
                ("https://github.com/JimothyJohn/perceptronics", "mailto:nick@advin.io")
            ), (name, href)


def test_in_page_anchors_exist(pages):
    page = pages["index.html"]
    ids = {a["id"] for _, a in page.tags if a.get("id")}
    for a in page.all("a"):
        if a["href"].startswith("#"):
            assert a["href"][1:] in ids, a["href"]


def test_images_reserve_their_space_and_describe_themselves(pages):
    # explicit width/height: no layout shift; alt: the picture's content for a screen reader
    for img in pages["index.html"].all("img"):
        assert img.get("width") == "1000" and img.get("height") == "560", img
        assert len(img.get("alt") or "") > 20, img


def test_pages_have_title_description_and_viewport(pages):
    for name, page in pages.items():
        metas = {m.get("name"): m.get("content") for m in page.all("meta")}
        assert metas.get("viewport") == "width=device-width, initial-scale=1", name
        assert metas.get("description"), name
        assert page.all("title"), name
