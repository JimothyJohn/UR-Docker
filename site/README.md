# The product page: perceptronics.advin.io

One static page: 3D picking off flat surfaces for any industrial robot, when to use it, the
connectors (Universal Robots first: the two URCap downloads and the UR Quickstart guide) and
what a cell needs. Specs live on the datasheet, install steps in the Quickstart. Built with the
[statician](https://github.com/JimothyJohn/statician) skill's stack: a private S3 bucket
behind CloudFront, a Route53 record and an ACM certificate, with a strict Content Security
Policy (no inline scripts, nothing loaded from another host).

```
site/public/        index.html and error.html, with {{PLACEHOLDERS}}
site/build.py       fills them from the repo and assembles site/_build/ (gitignored)
site/site.sh        build | preview | validate | deploy | sync | outputs | status
site/cloudformation/static-site.yaml   statician's template, unchanged
```

**The page cannot drift from the repo.** `build.py` takes the versions, sizes and sha256
from the committed `urcap/dist/` files and copies them to `downloads/`, takes the
supported PolyScope ranges from the CI matrices (`urcap/ps5_matrix.py`,
`urcap/psx_matrix.py`) and copies the pendant screens from
`urcap/perceptronic-ps5/screens/`. `tests/test_site.py` builds it and checks the result
against the CSP.

**The PDFs**: the datasheet (`public/datasheet.html`, one US Letter page) and the UR
Quickstart guide (`public/quickstart-ur.html`) are printed to `site/print/` by a local
Chrome: `site/site.sh datasheet`, then commit each PDF and its `<page>.sha256` stamp. CI has
no browser, so the test compares each stamp with the page as built: a new URCap version or
an edit to either page fails it until the PDFs are printed again. Chrome writes a new PDF
even for an unchanged page; restore an unchanged one from git rather than committing the
noise. Its performance figures are conservative
estimates (marked E) drawn from the code's limits and programmed speeds, not measurements;
replace them as cells are tested and move `SHEET_DATE` in `build.py`.

After a new URCap lands in `urcap/dist/`: `site/site.sh datasheet`, commit, then `site/site.sh sync`. Settings are in `site/.env` (copy `.env.example`); `deploy` is only
needed when the template changes.
