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
    assert sorted(p.name for p in built.glob("*.html")) == [
        "datasheet.html",
        "error.html",
        "index.html",
        "quickstart-ur.html",
    ]
    for page in built.glob("*.html"):
        text = page.read_text(encoding="utf-8")
        assert "{{" not in text and "}}" not in text, page.name


def test_downloads_are_the_committed_urcaps_byte_for_byte(built: Path):
    names = sorted(p.name for p in (built / "downloads").iterdir() if p.suffix != ".pdf")
    committed = sorted(p.name for p in DIST.iterdir() if p.suffix in (".urcap", ".urcapx"))
    assert names == committed
    for name in names:
        assert (built / "downloads" / name).read_bytes() == (DIST / name).read_bytes()


def test_each_download_link_shows_its_own_file_version_and_checksum(built: Path, pages):
    text = (built / "index.html").read_text(encoding="utf-8")
    links = [a["href"] for a in pages["index.html"].all("a") if a["href"].endswith((".urcap", ".urcapx"))]
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
            assert "//" not in url and ":" not in url, (name, url)  # same host: /root or relative
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


@pytest.mark.parametrize("page", sorted(site_build.PRINTS))
def test_each_pdf_is_the_print_of_the_current_page(built: Path, pages, page: str):
    # CI has no browser: each PDF is committed with the sha256 of the page it was printed from.
    # A new URCap version or an edited page changes the page -> `site/site.sh datasheet`.
    stamp = site_build.stamp_path(page).read_text(encoding="utf-8").strip()
    assert stamp == site_build.page_digest(built, page), f"{page} changed: run site/site.sh datasheet"
    name = site_build.PRINTS[page]
    pdf = (site_build.PRINT_DIR / name).read_bytes()
    assert pdf.startswith(b"%PDF")
    assert (built / "downloads" / name).read_bytes() == pdf
    assert f"/downloads/{name}" in [a["href"] for a in pages["index.html"].all("a")]


def test_datasheet_pdf_is_one_page():
    pdf = (site_build.PRINT_DIR / site_build.SHEET_PDF).read_bytes()
    assert len(re.findall(rb"/Type\s*/Page\b", pdf)) == 1


def test_quickstart_names_the_urcaps_it_installs(built: Path):
    text = (built / "quickstart-ur.html").read_text(encoding="utf-8")
    values = site_build.facts()
    for key in ("PS5_FILE", "PSX_FILE", "PS5_VERSION", "PSX_VERSION", "QUICKSTART_DATE"):
        assert values[key] in text, key
    # the address the URCap's Cockpit field defaults to (and the camera computer gives itself)
    java = (
        REPO / "urcap" / "perceptronic-ps5" / "src" / "io" / "advin" / "perceptronic" / "Cockpit.java"
    ).read_text(encoding="utf-8")
    default = re.search(r'DEFAULT_HOST\s*=\s*"([^"]+)"', java).group(1)
    assert f"<code>{default}</code>" in text


def test_index_no_longer_carries_specs_or_install_steps(pages):
    ids = {a["id"] for _, a in pages["index.html"].tags if a.get("id")}
    assert not ids & {"specs", "install"}


def test_datasheet_says_its_figures_are_untested_estimates(built: Path):
    text = (built / "datasheet.html").read_text(encoding="utf-8")
    assert "Planning figures, not guarantees" in text
    assert "have not been measured on a production cell" in text
    assert "Estimate, not a measured or guaranteed value" in text
    values = site_build.facts()
    for key in ("PS5_VERSION", "PSX_VERSION", "PS5_RANGE", "PSX_RANGE", "SHEET_DATE"):
        assert values[key] in text, key


def test_describe_image_measures_both_the_download_and_what_it_writes(tmp_path):
    import lzma

    raw = bytes(range(256)) * 4096 + b"\0" * 12345
    img = tmp_path / "pickpc-test.img.xz"
    img.write_bytes(lzma.compress(raw, format=lzma.FORMAT_XZ))
    info = site_build.describe_image(img, "2026-10-03")
    assert info["file"] == "pickpc-test.img.xz"
    assert info["extract_size"] == len(raw)
    assert info["extract_sha256"] == hashlib.sha256(raw).hexdigest()
    assert info["image_download_size"] == img.stat().st_size
    assert info["image_download_sha256"] == hashlib.sha256(img.read_bytes()).hexdigest()
    with pytest.raises(SystemExit):
        site_build.describe_image(tmp_path / "card.img", "2026-10-03")


def test_imager_list_is_one_pi5_image_with_no_settings(built: Path):
    import json

    image = json.loads(site_build.PICKPC.read_text(encoding="utf-8"))
    repo = json.loads((built / "imager.json").read_text(encoding="utf-8"))
    assert [d["tags"] for d in repo["imager"]["devices"]] == [["pi5-64bit"]]
    (entry,) = repo["os_list"]
    assert entry["devices"] == ["pi5-64bit"]
    # no init_format: Imager offers no settings, so it writes no cloud-init seed of its own
    assert "init_format" not in entry
    assert entry["url"] == f"https://perceptronics.advin.io/images/{image['file']}"
    for key in ("extract_size", "image_download_size"):
        assert isinstance(entry[key], int) and entry[key] > 0
    for key in ("extract_sha256", "image_download_sha256"):
        assert re.fullmatch(r"[0-9a-f]{64}", entry[key])
    assert entry["extract_size"] > entry["image_download_size"]  # it is compressed
    text = (built / "index.html").read_text(encoding="utf-8")
    assert f'href="/images/{image["file"]}"' in text
    assert "--repo https://perceptronics.advin.io/imager.json" in text


def test_sync_never_deletes_the_published_images():
    sync = (SITE / "site.sh").read_text(encoding="utf-8").split("\nsync)", 1)[1].split(";;", 1)[0]
    assert "--delete" in sync and '--exclude "images/*"' in sync
