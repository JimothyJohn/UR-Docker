# The product page: perceptronics.advin.io

One static page for Universal Robots users: what the Perceptronic URCap does, when to use
it, what it needs, its limits, and the two downloads. Built with the
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

After a new URCap lands in `urcap/dist/`: `site/site.sh sync`. That is the whole release
step for the page. Settings are in `site/.env` (copy `.env.example`); `deploy` is only
needed when the template changes.
