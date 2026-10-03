#!/usr/bin/env bash
# The product page at perceptronics.advin.io: build it, look at it, deploy it.
#
#   site/site.sh build      assemble site/_build/ from public/, urcap/dist/ and the screens
#   site/site.sh preview    build, then serve it on http://localhost:8000
#   site/site.sh datasheet  build, then print the one-page datasheet PDF (needs Chrome; commit the result)
#                           Over SSH, Chrome can't print (no display: CVDisplayLink fails, no PDF);
#                           use Playwright's headless shell, which needs none:
#                           CHROME=$(ls -d ~/Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-mac-arm64/chrome-headless-shell | tail -1) site/site.sh datasheet
#   site/site.sh validate   check the CloudFormation template parses
#   site/site.sh deploy     create/update the stack (first run: 5-15 min, certificate + CloudFront)
#   site/site.sh sync       build, upload to the bucket, invalidate CloudFront
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
    aws s3 sync "${BUILD_DIR}" "s3://${BUCKET}" --delete --cache-control "max-age=3600" --exclude ".DS_Store"
    echo "Invalidating CloudFront..."
    aws cloudfront create-invalidation --distribution-id "${DISTRIBUTION_ID}" --paths "/*" \
        --query 'Invalidation.Id' --output text
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
    sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
