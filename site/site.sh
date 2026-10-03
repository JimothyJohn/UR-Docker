#!/usr/bin/env bash
# The product page at perceptronics.advin.io: build it, look at it, deploy it.
#
#   site/site.sh build      assemble site/_build/ from public/, urcap/dist/ and the screens
#   site/site.sh preview    build, then serve it on http://localhost:8000
#   site/site.sh datasheet  build, then print the datasheet and UR Quickstart PDFs (needs Chrome; commit the results)
#                           Over SSH, Chrome can't print (no display: CVDisplayLink fails, no PDF);
#                           use Playwright's headless shell, which needs none:
#                           CHROME=$(ls -d ~/Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-mac-arm64/chrome-headless-shell | tail -1) site/site.sh datasheet
#   site/site.sh validate   check the CloudFormation template parses
#   site/site.sh deploy     create/update the stack (first run: 5-15 min, certificate + CloudFront)
#   site/site.sh sync       build, upload to the bucket, invalidate CloudFront
#   site/site.sh image F    publish pick PC image F (.img.xz): record it in pickpc-image.json, upload
#                           it to images/, upload imager.json (Raspberry Pi Imager --repo); commit the json
#   site/site.sh outputs    the stack's outputs (SiteURL, bucket, distribution)
#   site/site.sh status     the stack's status
#
# Settings come from site/.env (see .env.example). The stack is statician's
# (github.com/JimothyJohn/statician): private bucket behind CloudFront, strict CSP.
set -euo pipefail

SITE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SITE_DIR}/cloudformation/static-site.yaml"
BUILD_DIR="${SITE_DIR}/_build"

load_env() {
    if [ ! -f "${SITE_DIR}/.env" ]; then
        echo "site/.env not found: cp site/.env.example site/.env and fill it in" >&2
        exit 1
    fi
    set -a
    # shellcheck disable=SC1091
    . "${SITE_DIR}/.env"
    set +a
    # an empty AWS_PROFILE makes the CLI look for a profile named ""
    [ -n "${AWS_PROFILE:-}" ] || unset AWS_PROFILE
    STACK_NAME="${STACK_NAME:-perceptronics-site}"
    if [ -z "${DOMAIN_NAME:-}" ] || [ -z "${HOSTED_ZONE_ID:-}" ]; then
        echo "DOMAIN_NAME and HOSTED_ZONE_ID must be set in site/.env" >&2
        exit 1
    fi
}

build() {
    python3 "${SITE_DIR}/build.py" --out "${BUILD_DIR}"
}

output() {
    aws cloudformation describe-stacks --stack-name "${STACK_NAME}" \
        --query "Stacks[0].Outputs[?OutputKey=='$1'].OutputValue" --output text
}

case "${1:-}" in
build)
    build
    ;;
preview)
    build
    python3 -m http.server 8000 --bind 127.0.0.1 --directory "${BUILD_DIR}"
    ;;
datasheet)
    python3 "${SITE_DIR}/build.py" --out "${BUILD_DIR}" --pdf
    ;;
validate)
    load_env
    aws cloudformation validate-template --template-body "file://${TEMPLATE}" >/dev/null
    echo "template ok"
    ;;
deploy)
    load_env
    aws cloudformation deploy \
        --stack-name "${STACK_NAME}" \
        --template-file "${TEMPLATE}" \
        --parameter-overrides \
        "DomainName=${DOMAIN_NAME}" \
        "Subdomain=${SUBDOMAIN:-}" \
        "HostedZoneId=${HOSTED_ZONE_ID}" \
        --no-fail-on-empty-changeset
    ;;
sync)
    load_env
    build
    BUCKET="$(output BucketName)"
    DISTRIBUTION_ID="$(output DistributionId)"
    echo "Uploading to s3://${BUCKET}..."
    # images/ is published by `image`, never part of _build/: keep --delete away from it
    aws s3 sync "${BUILD_DIR}" "s3://${BUCKET}" --delete --cache-control "max-age=3600" \
        --exclude ".DS_Store" --exclude "images/*"
    echo "Invalidating CloudFront..."
    aws cloudfront create-invalidation --distribution-id "${DISTRIBUTION_ID}" --paths "/*" \
        --query 'Invalidation.Id' --output text
    ;;
image)
    IMAGE_FILE="${2:?usage: site/site.sh image <file>.img.xz}"
    [ -f "${IMAGE_FILE}" ] || { echo "no such file: ${IMAGE_FILE}" >&2; exit 1; }
    load_env
    echo "Measuring ${IMAGE_FILE} (decompresses it once)..."
    python3 "${SITE_DIR}/build.py" --describe-image "${IMAGE_FILE}"
    build
    BUCKET="$(output BucketName)"
    DISTRIBUTION_ID="$(output DistributionId)"
    # the name carries the build, so the object never changes: cache it for good
    aws s3 cp "${IMAGE_FILE}" "s3://${BUCKET}/images/$(basename "${IMAGE_FILE}")" \
        --content-type application/x-xz --cache-control "public, max-age=31536000, immutable"
    aws s3 cp "${BUILD_DIR}/imager.json" "s3://${BUCKET}/imager.json" \
        --content-type application/json --cache-control "max-age=300"
    aws cloudfront create-invalidation --distribution-id "${DISTRIBUTION_ID}" --paths "/imager.json" \
        --query 'Invalidation.Id' --output text
    echo "Published. Commit site/pickpc-image.json; the page links it on the next sync."
    ;;
outputs)
    load_env
    aws cloudformation describe-stacks --stack-name "${STACK_NAME}" --query 'Stacks[0].Outputs' --output table
    ;;
status)
    load_env
    aws cloudformation describe-stacks --stack-name "${STACK_NAME}" --query 'Stacks[0].StackStatus' --output text
    ;;
*)
    sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
